#define MY_SSID       "YOUR_SSID"
#define MY_PASS       "YOUR_PASSWORD"
#ifndef CONFIG_H
#define CONFIG_H
#define OTA_HOSTNAME  "stasis-arm"
 
// Bus 1: GPIO1 (TX) / GPIO2 (RX) — drivers 1-4 (addresses 0-3)
// Bus 2: GPIO17 (TX) / GPIO18 (RX) — driver 5 (address 0)
#define TMC_BUS1_TX   1
#define TMC_BUS1_RX   2
#define TMC_BUS2_TX   17
#define TMC_BUS2_RX   18
#define TMC_BAUD      115200
#define R_SENSE       0.10f
 
#define STEP_1  5
#define DIR_1   4
#define STEP_2  6
#define DIR_2   7
#define STEP_3  9
#define DIR_3   10
#define STEP_4  11
#define DIR_4   12
#define STEP_5  13
#define DIR_5   8
 
#define DIAG_1  21
#define DIAG_2  38
#define DIAG_3  39
#define DIAG_4  44
#define DIAG_5  43
 
#define SERVO_PIN  47
 
#define DEFAULT_CURRENT_MA   800
#define DEFAULT_MICROSTEPS   16
#define DEFAULT_SPEED        400
#define DEFAULT_ACCEL        800
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500
 
#define SERIAL_BUF_SIZE  128
#define NUM_JOINTS       5
 
#endif
