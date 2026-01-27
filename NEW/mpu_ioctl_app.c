#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <math.h>

// I2C Addresses
#define MPU_ADDR 0x68

/* Structure to hold sensor data - MUST match driver struct sensor
 * Driver defines:
 *   s16 ax, ay, az;    // Accelerometer X, Y, Z data (raw 16-bit)
 *   s16 gx, gy, gz;    // Gyroscope X, Y, Z data (raw 16-bit)
 *   s16 temp;          // Temperature data (raw 16-bit)
 */
struct sensor {
    int16_t ax;    // Accelerometer X-axis raw value (matches driver s16)
    int16_t ay;    // Accelerometer Y-axis raw value
    int16_t az;    // Accelerometer Z-axis raw value
    int16_t gx;    // Gyroscope X-axis raw value
    int16_t gy;    // Gyroscope Y-axis raw value
    int16_t gz;    // Gyroscope Z-axis raw value
    int16_t temp;  // Temperature raw value
};

struct sensor s;

// IOCTL command - MUST match driver definition: _IOR('m', 'm', struct sensor)
#define RD_VALUE _IOR('m', 'm', struct sensor)

// Signal handler for graceful exit
volatile sig_atomic_t keep_running = 1;

void signal_handler(int sig) {
    keep_running = 0;
}

/*
 * calculate_temperature() - Calculate temperature from raw sensor value
 * 
 * MPU6050 Temperature formula:
 *   Temp°C = (TEMP_OUT / 340) + 36.53
 * 
 * Where TEMP_OUT is the raw 16-bit temperature reading from registers 0x41-0x42
 * 
 * Note: This function calculates the temperature value but does NOT use/display it
 * as per the requirement. It's kept for reference or future use.
 */
static float calculate_temperature(int16_t raw_temp)
{
    float temperature;
    
    /* Temperature calculation formula from MPU6050 datasheet */
    temperature = (raw_temp / 340.0f) + 36.53f;
    
    return temperature;
}

int main() {
    int fd;
    float ax, ay, az, gx, gy, gz, temp_celsius;
    int ret;

    // Set up signal handler for Ctrl+C
    signal(SIGINT, signal_handler);

    /* Open the device file - MUST match driver DEVICE_NAME: "mpu6050_dev" */
    fd = open("/dev/mpu6050_dev", O_RDWR);
    if (fd < 0) {
        perror("Failed to open device /dev/mpu6050_dev");
        return 1;
    }

    printf("MPU6050 Sensor Data Reader\n");
    printf("Device: /dev/mpu6050_dev opened successfully\n");
    printf("Press Ctrl+C to exit\n\n");

    /* Calculate temperature values (but not using/displaying them) */
    /* These are pre-calculated for reference - currently unused */
    float sample_temp_1 = calculate_temperature(s.temp);  /* Will be 0 at start */
    float sample_temp_2 = calculate_temperature(1000);    /* Example: ~39.48°C */
    float sample_temp_3 = calculate_temperature(-1000);   /* Example: ~33.59°C */

    /* Continuous reading loop */
    while (keep_running) {
        /* Read sensor data via ioctl */
        ret = ioctl(fd, RD_VALUE, &s);
        if (ret < 0) {
            perror("ioctl failed");
            break;
        }

        /* Convert accelerometer raw values to g units
         * Scale factor: ±2g range = 16384 LSB/g
         */
        ax = s.ax / 16384.0f;
        ay = s.ady / 16384.0f;
        az = s.az / 16384.0f;

        /* Convert gyroscope raw values to degrees/second
         * Scale factor: ±250°/s range = 131 LSB/°/s
         */
        gx = s.gx / 131.0f;
        gy = s.gy / 131.0f;
        gz = s.gz / 131.0f;

        /* Calculate temperature (but NOT using/displaying it) */
        temp_celsius = calculate_temperature(s.temp);

        /* Print converted values - accelerometer only (temperature calculated but hidden) */
        printf("Accel: %.3f g (X), %.3f g (Y), %.3f g (Z) | ", ax, ay, az);
        printf("Gyro: %.2f °/s (X), %.2f °/s (Y), %.2f °/s (Z)\r", gx, gy, gz);
        fflush(stdout);

        /* Sleep for 100ms (10Hz sampling) */
        usleep(100000);
    }

    // Close the device
    close(fd);
    printf("\n\nExiting...\n");
    return 0;
}

