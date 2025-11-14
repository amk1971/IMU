#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xspi.h"
//#include "xgpio.h"
#include <math.h>
#include "ahrs_kalman.h"
#include "xtime_l.h"
#include "Madgwick.h"

static const float PI = 3.14159265358979323846f;

#define WHO_AM_I_REG      0x0F
#define CTRL_REG1_G       0x10
#define CTRL_REG2_G       0x11
#define CTRL_REG3_G       0x12
#define GYRO_OUT_X_L      0x18
#define CTRL_REG4         0x1E
#define GYRO_SENSITIVITY_2000DPS  70.0f

// accel registers
#define CTRL_REG5_XL      0x1F
#define CTRL_REG6_XL      0x20
#define CTRL_REG7_XL      0x21
#define ACCEL_SENSITIVITY_4G  0.000122f

// mag registers
#define CTRL_REG1_M      0x20
#define CTRL_REG2_M      0x21
#define CTRL_REG3_M      0x22
#define CTRL_REG4_M      0x23
#define STATUS_REG_M      0x27
const float MAG_RES_GAUSS = 0.000146f;
#define OUT_X_L_M      0x28


#define READ_BIT          0x80
#define AUTO_INCREMENT    0x40
#define STATUS_REG        0x17
#define OUT_X_XL      0x28

// --- Hard-iron calibration storage ---
#define CALIBRATION_SAMPLES 200
static int16_t mag_x_min = 32767, mag_x_max = -32768;
static int16_t mag_y_min = 32767, mag_y_max = -32768;
static int16_t mag_z_min = 32767, mag_z_max = -32768;
static int16_t mag_offset_x = 0;
static int16_t mag_offset_y = 0;
static int16_t mag_offset_z = 0;
static int calibration_done = 0;


union C2S {
	int16_t s;
	struct CHAR{
		uint8_t c[2];
	}c;
}c2s;


#define SAMPLE_FREQ  100.0f   // Sample frequency in Hz (adjust to your actual rate)
#define BETA         0.1f     // Filter gain (0.01 to 0.5, tune for your application)

float dt = 1.0f / SAMPLE_FREQ; // or compute actual dt




XSpi SpiInstance;
XSpi_Config *ConfigPtr;

int Status;

u8 SendBuffer[80];
u8 ReceiveBuffer[80];


//XGpio Gpio;
//    const u32 GPIO_DEVICE_ID = XPAR_GPIO_0_DEVICE_ID; // check xparameters.h; adjust if different
//    const u32 CHANNEL = 1;
//    const u32 MASK = 0x1; // using bit 0


void readIMU(int16_t* x, int16_t* y, int16_t* z, uint8_t regAddr, uint8_t slave_addr){

    for (int i = 1; i < 7; i++) {
        SendBuffer[i] = 0x00; // Dummy bytes to clock out data
    }

    SendBuffer[0] = regAddr | READ_BIT  ;

    XSpi_SetSlaveSelect(&SpiInstance,slave_addr);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, ReceiveBuffer, 7);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    if (Status != XST_SUCCESS) {
        xil_printf("Read failed\n");
        return XST_FAILURE;
    }

    //c2s.c.c
    *x = (int16_t)((ReceiveBuffer[2] << 8) | ReceiveBuffer[1]);
    *y = (int16_t)((ReceiveBuffer[4] << 8) | ReceiveBuffer[3]);
    *z = (int16_t)(((int16_t)ReceiveBuffer[6] << 8) | ReceiveBuffer[5]);
    //*z = * (int16_t *) &ReceiveBuffer[5];
}


void readIMUMAG(int16_t* x, int16_t* y, int16_t* z, uint8_t regAddr, uint8_t slave_addr){
    SendBuffer[0] = regAddr | READ_BIT | AUTO_INCREMENT ;
    for (int i = 1; i < 7; i++) {
        SendBuffer[i] = 0x00; // Dummy bytes to clock out data
    }

    XSpi_SetSlaveSelect(&SpiInstance,slave_addr);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, ReceiveBuffer, 7);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    if (Status != XST_SUCCESS) {
        xil_printf("Read failed\n");
        return XST_FAILURE;
    }


    *x = (int16_t)((ReceiveBuffer[2] << 8) | ReceiveBuffer[1]);
    *y = (int16_t)((ReceiveBuffer[4] << 8) | ReceiveBuffer[3]);
    *z = (int16_t)((ReceiveBuffer[6] << 8) | ReceiveBuffer[5]);
}

void print_float(float f)
{
    int integer = (int)f;
    int frac = (int)((f - integer) * 100);   // print 2 decimal places
    if (frac < 0) frac = -frac;
    xil_printf("%d.%02d", integer, frac);
}

// --- Magnetometer calibration function ---
void calibrate_magnetometer() {
    xil_printf("\n=== MAGNETOMETER CALIBRATION ===\n");
    xil_printf("Slowly rotate the device in all directions...\n");
    xil_printf("Collecting %d samples...\n", CALIBRATION_SAMPLES);

    int16_t mag_x, mag_y, mag_z;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        readIMUMAG(&mag_x, &mag_y, &mag_z, OUT_X_L_M, 0x02);

        // Update min/max
        if (mag_x < mag_x_min) mag_x_min = mag_x;
        if (mag_x > mag_x_max) mag_x_max = mag_x;
        if (mag_y < mag_y_min) mag_y_min = mag_y;
        if (mag_y > mag_y_max) mag_y_max = mag_y;
        if (mag_z < mag_z_min) mag_z_min = mag_z;
        if (mag_z > mag_z_max) mag_z_max = mag_z;

        // Progress indicator
        if (i % 20 == 0) {
            xil_printf(".");
        }

        usleep(50000); // 50ms between samples
    }

    // Compute hard-iron offsets
    mag_offset_x = (mag_x_max + mag_x_min) / 2;
    mag_offset_y = (mag_y_max + mag_y_min) / 2;
    mag_offset_z = (mag_z_max + mag_z_min) / 2;

    calibration_done = 1;

    xil_printf("\n\n=== CALIBRATION COMPLETE ===\n");
    xil_printf("Offsets (raw counts):\n");
    xil_printf("  X: %d (min=%d, max=%d)\n", mag_offset_x, mag_x_min, mag_x_max);
    xil_printf("  Y: %d (min=%d, max=%d)\n", mag_offset_y, mag_y_min, mag_y_max);
    xil_printf("  Z: %d (min=%d, max=%d)\n", mag_offset_z, mag_z_min, mag_z_max);
    xil_printf("Starting normal operation...\n\n");
}


float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;   // the median
}


int main()
{
    init_platform();
    AHRS_KalmanFilter_init(0.0f, 0.0f, 0.0f);

    XTime t_prev, t_now;
    XTime_GetTime(&t_prev);
//    Status = XGpio_Initialize(&Gpio, GPIO_DEVICE_ID);
//    if (Status != XST_SUCCESS) {
//        xil_printf("GPIO Initialization Failed\r\n");
//        return XST_FAILURE;
//    }

//    XGpio_SetDataDirection(&Gpio, CHANNEL, 0x0); // all bits outputs
    // Gpio for chip select of magnetometer
//    XGpio_DiscreteWrite(&Gpio, CHANNEL, 0x1);

    // Initialize SPI
    ConfigPtr = XSpi_LookupConfig(XPAR_SPI_0_DEVICE_ID);
    if (ConfigPtr == NULL) {
        xil_printf("No config found for SPI device\n");
        return XST_FAILURE;
    }

    Status = XSpi_CfgInitialize(&SpiInstance, ConfigPtr, ConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI initialization failed\n");
        return XST_FAILURE;
    }

    // Set SPI options: master mode and manual slave select
    Status = XSpi_SetOptions(&SpiInstance, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI SetOptions failed\n");
        return XST_FAILURE;
    }

    XSpi_Start(&SpiInstance);
    XSpi_IntrGlobalDisable(&SpiInstance);

    xil_printf("SPI initialized!\r\n");

    // --- Test communication by reading WHO_AM_I ---


    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    SendBuffer[0] = WHO_AM_I_REG | READ_BIT;
    SendBuffer[1] = 0x00;
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, ReceiveBuffer, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    if (Status != XST_SUCCESS) {
        xil_printf("SPI Transfer failed\n");
        return XST_FAILURE;
    }

    xil_printf("WHO_AM_I: 0x%02X (Expected: 0x68)\r\n", ReceiveBuffer[1]);






    // --- Configure gyro registers ---
    SendBuffer[0] = CTRL_REG1_G;
    // Lower ODR from 119 Hz (0x78) to 59.5 Hz (0x58). Keep FS=2000 dps (bits 4:3 = 10).
    SendBuffer[1] = 0x58;  // ODR=59.5 Hz, FS=2000 dps

    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    // CTRL_REG2_G (unchanged)
    SendBuffer[0] = CTRL_REG2_G;
    SendBuffer[1] = 0x00;
    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    // CTRL_REG3_G (unchanged)
    SendBuffer[0] = CTRL_REG3_G;
    SendBuffer[1] = 0x47;
    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    // CTRL_REG4 - Enable X, Y, Z axes (unchanged)
    SendBuffer[0] = CTRL_REG4;
    SendBuffer[1] = 0x38;
    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    xil_printf("Gyro configured!\r\n");

    // --- Configure accel registers ---

    // CTRL_REG5_XL (leave as you have it, unless you intend to use decimation/axis enables there)
    SendBuffer[0] = CTRL_REG5_XL;
    SendBuffer[1] = 0x78;
    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    // CTRL_REG6_XL: reduce ODR from 952 Hz (0xD2) to 50 Hz (0x52), keep ±4 g FS and BW=50 Hz
    SendBuffer[0] = CTRL_REG6_XL;
    SendBuffer[1] = 0x52;  // ODR=50 Hz (010<<5), FS=±4 g (10<<3), BW=50 Hz (01<<1)
    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);

    // CTRL_REG7_XL (unchanged)
    SendBuffer[0] = CTRL_REG7_XL;
    SendBuffer[1] = 0x00;
    XSpi_SetSlaveSelect(&SpiInstance, 0x01);
    Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
    XSpi_SetSlaveSelect(&SpiInstance, 0x00);


      // --- Configure mag registers ---
                SendBuffer[0] = WHO_AM_I_REG | READ_BIT;
            SendBuffer[1] = 0x58;

            XSpi_SetSlaveSelect(&SpiInstance, 0x02);
            Status = XSpi_Transfer(&SpiInstance, SendBuffer, ReceiveBuffer, 2);
            XSpi_SetSlaveSelect(&SpiInstance, 0x00);

            xil_printf("WHO_AM_I_M: 0x%02X (Expected: 0x68)\r\n", ReceiveBuffer[1]);


//            SendBuffer[0] = CTRL_REG1_M;
//                       SendBuffer[1] = 0x58;
//
//                       XSpi_SetSlaveSelect(&SpiInstance, 0x02);
//                       Status = XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
//                       XSpi_SetSlaveSelect(&SpiInstance, 0x02);
            // CTRL_REG1_M (0x20) – Ultra-high performance, 80Hz, Temp compensation
            SendBuffer[0] = 0x20;
            SendBuffer[1] = 0xD4;   // 0b01110000
            XSpi_SetSlaveSelect(&SpiInstance, 0x02);
            XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
            XSpi_SetSlaveSelect(&SpiInstance,0x00);

            // CTRL_REG2_M (0x21) – Magnetic full-scale = ±4 gauss
            SendBuffer[0] = 0x21;
            SendBuffer[1] = 0x04;
            XSpi_SetSlaveSelect(&SpiInstance, 0x02);
            XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
            XSpi_SetSlaveSelect(&SpiInstance, 0x00);

            // CTRL_REG3_M (0x22) – **CONTINUOUS CONVERSION MODE**
            SendBuffer[0] = 0x22;
            SendBuffer[1] = 0x00;  // THIS IS THE FIX
            XSpi_SetSlaveSelect(&SpiInstance, 0x02);
            XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
            XSpi_SetSlaveSelect(&SpiInstance,0x00);

            // CTRL_REG4_M (0x23) – Z-axis performance mode
            SendBuffer[0] = 0x23;
            SendBuffer[1] = 0x0C;  // Ultra-high performance on Z axis
            XSpi_SetSlaveSelect(&SpiInstance, 0x02);
            XSpi_Transfer(&SpiInstance, SendBuffer, NULL, 2);
            XSpi_SetSlaveSelect(&SpiInstance, 0x00);





    // Small delay to let gyro stabilize
    usleep(100000);

    // --- bias correction ---

    int16_t gyroX[50], gyroY[50], gyroZ[50];
    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;

    for (int i = 0; i < 50; ++i) {
        readIMU(&gyroX[i], &gyroY[i], &gyroZ[i], GYRO_OUT_X_L, 0x01);
        sumX += (int32_t)gyroX[i];
        sumY += (int32_t)gyroY[i];
        sumZ += (int32_t)gyroZ[i];
    }
    int32_t biasX = sumX / 50;
    int32_t biasY = sumY / 50;
    int32_t biasZ = sumZ / 50;


    float pitch_deg, roll_deg, yaw_deg;

    // --- HARD IRON CALIBRATION ---
    calibrate_magnetometer();

    Quaternion q = {1,0,0,0};
    // --- Main read loop ---
    while (1) {

    	XTime_GetTime(&t_now);
    	double dt_seconds = (double)(t_now - t_prev) / (double)COUNTS_PER_SECOND;
    	if (dt_seconds <= 0.0) dt_seconds = 1.0 / SAMPLE_FREQ; // fallback
    	t_prev = t_now;

    	// optionally clamp dt to reasonable range:
    	if (dt_seconds > 1.0) dt_seconds = 1.0;
    	if (dt_seconds < 0.001) dt_seconds = 0.001;


        // Check status register
        XSpi_SetSlaveSelect(&SpiInstance, 0x01);
        SendBuffer[0] = STATUS_REG | READ_BIT;
        SendBuffer[1] = 0x00;
        Status = XSpi_Transfer(&SpiInstance, SendBuffer, ReceiveBuffer, 2);
        XSpi_SetSlaveSelect(&SpiInstance, 0x00);

//        xil_printf("Status: 0x%02X \n", ReceiveBuffer[1]);

        // Check if gyro data is available (bit 1)
        if ((ReceiveBuffer[1] & 0x02) == 0) {
            xil_printf("(Gyro data not ready)\r\n");
            usleep(10000);
            continue;
        }






        // --- Reading gyro registers ---
        // Read all 6 bytes of gyro data (X, Y, Z - low and high bytes)

        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        readIMU( &gyro_x, &gyro_y, &gyro_z,GYRO_OUT_X_L , 0x01);

        float gyro_x_dps = ((float)gyro_x - (float)biasX) * GYRO_SENSITIVITY_2000DPS / 1000.0f;
        float gyro_y_dps = ((float)gyro_y - (float)biasY) * GYRO_SENSITIVITY_2000DPS / 1000.0f;
        float gyro_z_dps = ((float)gyro_z - (float)biasZ) * GYRO_SENSITIVITY_2000DPS / 1000.0f;

//        xil_printf("Gyro values: X: ");
//        print_float(gyro_x_dps);
//        xil_printf(" Y: ");
//        print_float(gyro_y_dps);
//        xil_printf(" Z: ");
//        print_float(gyro_z_dps);
//        xil_printf("\n");







        // --- Reading accel registers ---
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;

        readIMU( &accel_x, &accel_y, &accel_z, OUT_X_XL , 0x01);

        float accel_x_g = (float)accel_x * ACCEL_SENSITIVITY_4G ;  // Convert mg to g
        float accel_y_g = (float)accel_y * ACCEL_SENSITIVITY_4G ;
        float accel_z_g = ((float)accel_z * ACCEL_SENSITIVITY_4G);

        // Optional: Convert to m/s²
        float accel_x_ms2 = accel_x_g * 9.81f;
        float accel_y_ms2 = accel_y_g * 9.81f;
        float accel_z_ms2 = accel_z_g * 9.81f;







        // --- Reading mag registers ---
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;

        readIMUMAG( &mag_x, &mag_y, &mag_z, OUT_X_L_M , 0x02);

        // --- APPLY HARD-IRON CALIBRATION ---
        int16_t mag_x_cal = mag_x - 1964.00;
        int16_t mag_y_cal = mag_y - 2993.00;
        int16_t mag_z_cal = mag_z +3728.00;

        // Convert calibrated values to µT
        float mag_x_T = (float)mag_x_cal * MAG_RES_GAUSS * 100.0f;
        float mag_y_T = (float)mag_y_cal * MAG_RES_GAUSS * 100.0f;
        float mag_z_T = (float)mag_z_cal * MAG_RES_GAUSS * 100.0f;

        float yaw_degree = atan2_approx(mag_x_T, mag_y_T) * 180.0f/PI;
        if (yaw_degree < 0.0f) yaw_degree += 360.0f;

//        print_float(mag_offset_x);
//        xil_printf(" ");
//        print_float(mag_offset_y);
//        xil_printf(" ");
//        print_float(mag_offset_z);
//        xil_printf(" ");
//        print_float(yaw_degree);
//        xil_printf("\n");

//
//                print_float(accel_x_g);
//                xil_printf(" ");
//                print_float(accel_y_g);
//                xil_printf(" ");
//                print_float(accel_z_g);
//                xil_printf(" ");
//        		print_float(gyro_x_dps);
//        		xil_printf(" ");
//                print_float(gyro_y_dps);
//                xil_printf(" ");
//                print_float(gyro_z_dps);
//                xil_printf(" ");
//        		print_float(mag_x_T);
//        		xil_printf(" ");
//                print_float(mag_y_T);
//                xil_printf(" ");
//                print_float(mag_z_T);
//                xil_printf("\n");

        // Debug: raw accel (g) and gyro (dps)
//        xil_printf("RAW a(g): ");
//        print_float(accel_x_g); xil_printf(" ");
//        print_float(accel_y_g); xil_printf(" ");
//        print_float(accel_z_g); xil_printf(" | g(dps): ");
//        print_float(gyro_x_dps); xil_printf(" ");
//        print_float(gyro_y_dps); xil_printf(" ");
//        print_float(gyro_z_dps); xil_printf("\n");

        float beta = 0.23;
        float dt = dt_seconds ;
        Vector3 gyroV  = {gyro_x_dps, gyro_y_dps, gyro_z_dps};
        Vector3 accelV =  {accel_x_g , accel_y_g, accel_z_g};


        q = madgwick_update(q, gyroV , accelV , dt,  beta);

        quat_to_euler_fast(q.w, q.x, q.y, q.z, &roll_deg, &pitch_deg, &yaw_deg);

//                AHRS_KalmanFilter_update(
//                    accel_x_g, accel_y_g, accel_z_g,
//                    gyro_x_dps, gyro_y_dps, gyro_z_dps,
//                    mag_x_T, mag_y_T, mag_z_T,
//					(float)dt_seconds, // dt in seconds (or pass 0 to use SAMPLE_FREQ fallback)
//                    &pitch_deg, &roll_deg, &yaw_deg
//                );

//                                print_float(dt);
//                                xil_printf(" ");
//                                print_float(q.w);
//                                xil_printf(" ");
//                                print_float(q.x);
//                                xil_printf(" ");
//                                print_float(q.y);
//                                xil_printf(" ");
//                                print_float(q.z);
//                                xil_printf("\n");

                xil_printf("ROLL=");
                print_float(roll_deg);
                xil_printf(" PITCH=");
                print_float(-pitch_deg);
                xil_printf(" YAW=");
                print_float(yaw_deg);
                xil_printf("\n");

//                xil_printf(" ");
//                print_float(pitch_deg);
//                xil_printf(" ");
//                print_float(roll_deg);
//                xil_printf(" ");
//                print_float(yaw_degree);
//                xil_printf("\n");

//                float heading1 = atan2_approx( mag_y_T,  mag_x_T) * (180.0f/PI);
//                float heading2 = atan2_approx(-mag_y_T,  mag_x_T) * (180.0f/PI);
//                float heading3 = atan2_approx( mag_y_T, -mag_x_T) * (180.0f/PI);
//                float heading4 = atan2_approx(-mag_y_T, -mag_x_T) * (180.0f/PI);
//                xil_printf("h1=%7.2f h2=%7.2f h3=%7.2f h4=%7.2f\n",
//                           heading1, heading2, heading3, heading4);



        usleep(100000);
    }

    cleanup_platform();
    return 0;
}
