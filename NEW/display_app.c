#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>

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

int main()
{
    int fd;
    struct oled_cursor cursor;
    char *message = "Standalone OLED Display";

    /* Open OLED device */
    fd = open("/dev/oleddevice", O_RDWR);
    if (fd < 0) {
        perror("Failed to open OLED device");
        return 1;
    }

    /* Initialize OLED */
    if (ioctl(fd, OLED_INIT) < 0) {
        perror("Failed to initialize OLED");
        close(fd);
        return 1;
    }

    /* Clear OLED */
    ioctl(fd, OLED_CLEAR);

    /* Set cursor and print message */
    cursor.page = 0;
    cursor.col = 0;
    ioctl(fd, OLED_SET_CURSOR, &cursor);
    ioctl(fd, OLED_PRINT, message);

    /* Print another message */
    cursor.page = 2;
    cursor.col = 0;
    ioctl(fd, OLED_SET_CURSOR, &cursor);
    message = "Hello World!";
    ioctl(fd, OLED_PRINT, message);

    printf("OLED display updated with standalone messages.\n");

    close(fd);
    return 0;
}
