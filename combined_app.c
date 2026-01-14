#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* OLED IOCTL Commands */
#define OLED_IOC_MAGIC 'o'
#define OLED_INIT _IO(OLED_IOC_MAGIC, 1)
#define OLED_CLEAR _IO(OLED_IOC_MAGIC, 2)
#define OLED_SET_CURSOR _IOW(OLED_IOC_MAGIC, 3, struct oled_cursor)
#define OLED_PRINT _IOW(OLED_IOC_MAGIC, 4, char *)

struct oled_cursor {
    uint8_t page;
    uint8_t col;
};

/* MPU6050 IOCTL Commands (from ioctlapp.c) */
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
    int oled_fd, mpu_fd;
    struct sensor_data s;
    char line[32];

    /* Open OLED device */
    oled_fd = open("/dev/oleddevice", O_RDWR);
    if (oled_fd < 0) {
        perror("Failed to open OLED device");
        return 1;
    }

    /* Open MPU device */
    mpu_fd = open("/dev/mpu6050", O_RDWR);
    if (mpu_fd < 0) {
        perror("Failed to open MPU device");
        close(oled_fd);
        return 1;
    }

    /* Initialize OLED */
    if (ioctl(oled_fd, OLED_INIT) < 0) {
        perror("Failed to initialize OLED");
        close(oled_fd);
        close(mpu_fd);
        return 1;
    }

    /* Clear OLED */
    ioctl(oled_fd, OLED_CLEAR);

    /* Calculate XA_TEST (5 bits): bits 4-2 from SELF_TEST_X, bits 1-0 from SELF_TEST_A */
    int xa_test = ((s.self_test_x & 0x1C) >> 2) | (s.self_test_a & 0x03);
    /* Calculate YA_TEST (5 bits): bits 4-2 from SELF_TEST_Y, bits 3-2 from SELF_TEST_A */
    int ya_test = ((s.self_test_y & 0x1C) >> 2) | ((s.self_test_a & 0x0C) >> 2);
    /* Calculate ZA_TEST (5 bits): bits 4-2 from SELF_TEST_Z, bits 5-4 from SELF_TEST_A */
    int za_test = ((s.self_test_z & 0x1C) >> 2) | ((s.self_test_a & 0x30) >> 4);

    /* Calculate Factory Trim (FT) for each axis in LSB units */
    float ft_xa = (xa_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (xa_test - 1) / 30.0f) : 0.0f;
    float ft_ya = (ya_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (ya_test - 1) / 30.0f) : 0.0f;
    float ft_za = (za_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (za_test - 1) / 30.0f) : 0.0f;

    /* Set cursor and print initial message */
    struct oled_cursor cursor = {0, 0};
    ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
    strcpy(line, "MPU6050 DATA");
    ioctl(oled_fd, OLED_PRINT, line);

    while (1) {
        /* Read sensor data from MPU driver */
        if (ioctl(mpu_fd, MPU_GET_DATA, &s) < 0) {
            perror("Failed to get MPU data");
            continue;
        }

        /* Clear OLED */
        ioctl(oled_fd, OLED_CLEAR);

        /* Display accelerometer data */
        cursor.page = 0; cursor.col = 0;
        ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
        sprintf(line, "AX:%d", s.adx);
        ioctl(oled_fd, OLED_PRINT, line);

        cursor.page = 2; cursor.col = 0;
        ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
        sprintf(line, "AY:%d", s.ady);
        ioctl(oled_fd, OLED_PRINT, line);

        cursor.page = 4; cursor.col = 0;
        ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
        sprintf(line, "AZ:%d", s.adz);
        ioctl(oled_fd, OLED_PRINT, line);

        /* Display gyroscope data */
        cursor.page = 6; cursor.col = 0;
        ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
        sprintf(line, "GX:%d", s.gdx);
        ioctl(oled_fd, OLED_PRINT, line);

        cursor.page = 0; cursor.col = 64;
        ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
        sprintf(line, "GY:%d", s.gdy);
        ioctl(oled_fd, OLED_PRINT, line);

        cursor.page = 2; cursor.col = 64;
        ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
        sprintf(line, "GZ:%d", s.gdz);
        ioctl(oled_fd, OLED_PRINT, line);

        /* Display temperature */
        cursor.page = 4; cursor.col = 64;
        ioctl(oled_fd, OLED_SET_CURSOR, &cursor);
        sprintf(line, "T:%d", s.td);
        ioctl(oled_fd, OLED_PRINT, line);

        sleep(1);
    }

    close(oled_fd);
    close(mpu_fd);
    return 0;
}
