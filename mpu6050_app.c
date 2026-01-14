#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <math.h>

/* MPU6050 IOCTL Commands */
#define MPU_IOC_MAGIC 'm'
#define MPU_GET_DATA _IOR(MPU_IOC_MAGIC, 1, struct sensor_data)

struct sensor_data {
    int16_t adx, ady, adz;
    int16_t gdx, gdy, gdz;
    int16_t td;
    uint8_t self_test_x;
    uint8_t self_test_y;
    uint8_t self_test_z;
    uint8_t self_test_a;
};

int main()
{
    int fd;
    struct sensor_data s;

    /* Open MPU device */
    fd = open("/dev/mpu6050", O_RDWR);
    if (fd < 0) {
        perror("Failed to open MPU device");
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

    printf("Standalone MPU6050 Data Reader with Calibration\n");
    printf("Factory Trim Offsets: X=%.3f, Y=%.3f, Z=%.3f\n", ft_xa, ft_ya, ft_za);
    printf("Press Ctrl+C to exit\n\n");

    while (1) {
        /* Read sensor data */
        if (ioctl(fd, MPU_GET_DATA, &s) < 0) {
            perror("Failed to get MPU data");
            sleep(1);
            continue;
        }

        /* Print raw data */
        printf("Raw Sensor Data:\n");
        printf("  Accel X: %d, Y: %d, Z: %d\n", s.adx, s.ady, s.adz);
        printf("  Gyro  X: %d, Y: %d, Z: %d\n", s.gdx, s.gdy, s.gdz);
        printf("  Temp   : %d\n", s.td);

        /* Convert to meaningful units */
        // Accelerometer: raw / 16384.0 = g (gravity units)
        float ax = s.adx / 16384.0;
        float ay = s.ady / 16384.0;
        float az = s.adz / 16384.0;

        // Gyroscope: raw / 131.0 = degrees/second
        float gx = s.gdx / 131.0;
        float gy = s.gdy / 131.0;
        float gz = s.gdz / 131.0;

        // Temperature: (raw / 340.0) + 36.53 = Celsius
        float t = (s.td / 340.0) + 36.53;

        /* Print calibrated values */
        printf("Calibrated Values:\n");
        printf("  Accel X: %.3f g, Y: %.3f g, Z: %.3f g\n", ax, ay, az);
        printf("  Gyro  X: %.3f °/s, Y: %.3f °/s, Z: %.3f °/s\n", gx, gy, gz);
        printf("  Temp   : %.3f °C\n", t);
        printf("------------------------\n");

        sleep(1);
    }

    close(fd);
    return 0;
}
