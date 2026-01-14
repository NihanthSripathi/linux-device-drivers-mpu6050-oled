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

// Structure to hold sensor data
struct sensor {
    int32_t adx;  // Accelerometer X-axis raw value
    int32_t ady;  // Accelerometer Y-axis raw value
    int32_t adz;  // Accelerometer Z-axis raw value
    int32_t gdx;  // Gyroscope X-axis raw value
    int32_t gdy;  // Gyroscope Y-axis raw value
    int32_t gdz;  // Gyroscope Z-axis raw value
    int32_t td;   // Temperature raw value
    uint8_t self_test_x;  // Self-test X register
    uint8_t self_test_y;  // Self-test Y register
    uint8_t self_test_z;  // Self-test Z register
    uint8_t self_test_a;  // Self-test A register
};

struct sensor s;

// IOCTL command definition
#define RD_VALUE _IOWR('m', 'm', int32_t*)

// Signal handler for graceful exit
volatile sig_atomic_t keep_running = 1;

void signal_handler(int sig) {
    keep_running = 0;
}

int main() {
    int fd;
    float ax, ay, az, gx, gy, gz, t;

    // Set up signal handler for Ctrl+C
    signal(SIGINT, signal_handler);

    // Open the device file
    fd = open("/dev/mk_device", O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    // Calculate XA_TEST (5 bits): bits 4-2 from SELF_TEST_X, bits 1-0 from SELF_TEST_A
    int xa_test = ((s.self_test_x & 0x1C) >> 2) | (s.self_test_a & 0x03);
    // Calculate YA_TEST (5 bits): bits 4-2 from SELF_TEST_Y, bits 3-2 from SELF_TEST_A
    int ya_test = ((s.self_test_y & 0x1C) >> 2) | ((s.self_test_a & 0x0C) >> 2);
    // Calculate ZA_TEST (5 bits): bits 4-2 from SELF_TEST_Z, bits 5-4 from SELF_TEST_A
    int za_test = ((s.self_test_z & 0x1C) >> 2) | ((s.self_test_a & 0x30) >> 4);

    // Calculate Factory Trim (FT) for each axis in LSB units
    float ft_xa = (xa_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (xa_test - 1) / 30.0f) : 0.0f;
    float ft_ya = (ya_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (ya_test - 1) / 30.0f) : 0.0f;
    float ft_za = (za_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (za_test - 1) / 30.0f) : 0.0f;

    printf("MPU6050 Sensor Data Reader with Calibration\n");
    printf("Factory Trim Offsets: X=%.3f, Y=%.3f, Z=%.3f\n", ft_xa, ft_ya, ft_za);
    printf("Press Ctrl+C to exit\n\n");

    // Continuous reading loop
    while (keep_running) {
        // Read sensor data via ioctl
        if (ioctl(fd, RD_VALUE, (struct sensor*)&s) < 0) {
            perror("ioctl failed");
            break;
        }

        // Print raw values
        // printf("Raw Sensor Data:\n");
        // printf("  Accel X: %d, Y: %d, Z: %d\n", s.adx, s.ady, s.adz);
        // printf("  Gyro  X: %d, Y: %d, Z: %d\n", s.gdx, s.gdy, s.gdz);
        // printf("  Temp   : %d\n", s.td);

        // Convert to meaningful units
        // Accelerometer: calibrated / 16384.0 = g (gravity units) - calibration applied in driver
        ax = s.adx / 16384.0;
        ay = s.ady / 16384.0;
        az = s.adz / 16384.0;

        // Gyroscope: raw / 131.0 = degrees/second
        gx = s.gdx / 131.0;
        gy = s.gdy / 131.0;
        gz = s.gdz / 131.0;

        // Temperature: (raw / 340.0) + 36.53 = Celsius
        t = (s.td / 340.0) + 36.53;

        // Print converted values
        printf("Calibrated Values:\n");
        printf("  Accel X: %.3f g, Y: %.3f g, Z: %.3f g\n", ax, ay, az);
        // printf("  Gyro  X: %.3f °/s, Y: %.3f °/s, Z: %.3f °/s\n", gx, gy, gz);
        // printf("  Temp   : %.3f °C\n", t);

        // printf("------------------------\n");

        // Sleep for 1 second
        sleep(1);
    }

    // Close the device
    close(fd);
    printf("\nExiting...\n");
    return 0;
}
