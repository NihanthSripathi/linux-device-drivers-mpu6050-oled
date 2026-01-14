#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* I2C Addresses */
#define OLED_ADDR 0x3C
#define MPU_ADDR  0x68

/* Minimalist 5x7 Font (ASCII 32–90) */
const unsigned char font[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5f,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00},{0x14,0x7f,0x14,0x7f,0x14},
    {0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1c,0x22,0x41,0x00},{0x00,0x41,0x22,0x1c,0x00},
    {0x14,0x08,0x3e,0x08,0x14},{0x08,0x08,0x3e,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3e,0x51,0x49,0x45,0x3e},{0x00,0x42,0x7f,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},
    {0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1e},
    {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3e},{0x7e,0x11,0x11,0x11,0x7e},
    {0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
    {0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},
    {0x7f,0x09,0x09,0x09,0x01},{0x3e,0x41,0x49,0x49,0x7a},
    {0x7f,0x08,0x08,0x08,0x7f},{0x00,0x41,0x7f,0x41,0x00},
    {0x20,0x40,0x41,0x3f,0x01},{0x7f,0x08,0x14,0x22,0x41},
    {0x7f,0x40,0x40,0x40,0x40},{0x7f,0x02,0x0c,0x02,0x7f},
    {0x7f,0x04,0x08,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
    {0x7f,0x09,0x09,0x09,0x06},{0x3e,0x41,0x51,0x21,0x5e},
    {0x7f,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7f,0x01,0x01},{0x3f,0x40,0x40,0x40,0x3f},
    {0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43}
};

/* =============== OLED FUNCTIONS =============== */
void send_cmd(int fd, uint8_t cmd)
 {
     uint8_t buf[2]={0x00,cmd};
      write(fd,buf,2);
}
void send_data(int fd, uint8_t data) 
{
     uint8_t buf[2]={0x40,data};
    write(fd,buf,2); 
}

void oled_init(int fd)
{
    send_cmd(fd,0xAE); send_cmd(fd,0xD5); send_cmd(fd,0x80);
    send_cmd(fd,0xA8); send_cmd(fd,0x3F);
    send_cmd(fd,0xD3); send_cmd(fd,0x00);
    send_cmd(fd,0x40);
    send_cmd(fd,0x8D); send_cmd(fd,0x14);
    send_cmd(fd,0x20); send_cmd(fd,0x00);
    send_cmd(fd,0xA1); send_cmd(fd,0xC8);
    send_cmd(fd,0xDA); send_cmd(fd,0x12);
    send_cmd(fd,0x81); send_cmd(fd,0xCF);
    send_cmd(fd,0xD9); send_cmd(fd,0xF1);
    send_cmd(fd,0xDB); send_cmd(fd,0x40);
    send_cmd(fd,0xA4); send_cmd(fd,0xA6);
    send_cmd(fd,0xAF);
}

void oled_clear(int fd)
{
    for(int page=0; page<8; page++)
    {
        send_cmd(fd,0xB0+page);
         send_cmd(fd,0x00); 
         send_cmd(fd,0x10);
        for(int i=0;i<128;i++) 
            send_data(fd,0x00);
    }
}

void oled_set_cursor(int fd,uint8_t page,uint8_t col)
{
    send_cmd(fd,0xB0+page);
    send_cmd(fd,(col>>4)|0x10);
    send_cmd(fd,col&0x0F);
}

void oled_print(int fd,const char* str)
{
    while(*str)
    {
        char c=*str;
        if(c>='a' && c<='z') c-=32;
        if(c>=32 && c<=90)
        {
            for(int i=0;i<5;i++) send_data(fd,font[c-32][i]);
            send_data(fd,0x00);
        }
        else
        {
            for(int i=0;i<6;i++) send_data(fd,0x00);
        }
        str++;
    }
}

/* =============== MPU6050 FUNCTIONS =============== */
void mpu_write(int fd,uint8_t reg,uint8_t val){ 
    uint8_t buf[2]={reg,val}; 
    write(fd,buf,2); 
}
void mpu_read(int fd,uint8_t reg,uint8_t* buf,int len){
     write(fd,&reg,1);
     read(fd,buf,len);
    }
int16_t combine(uint8_t h,uint8_t l){ return (int16_t)((h<<8)|l); } // Combine high and low bytes into 16-bit signed integer (MSB first)

/* =============== MAIN =============== */
int main()
{
    int fd = open("/dev/i2c-1",O_RDWR);
    if(fd<0)
    {
         perror("I2C open"); 
         return 1; 
    }

    /* Initialize OLED */
    if(ioctl(fd,I2C_SLAVE,OLED_ADDR)<0)
    {
         perror("OLED not found"); 
         return 1; 
    }
    oled_init(fd); 
    oled_clear(fd);
    oled_set_cursor(fd,0,0);
    oled_print(fd,"SHAIK MASTAN");
    oled_set_cursor(fd,2,0);
    oled_print(fd,"IT SOLUTIONS");
    printf("OLED DISPLAY OK\n");

    /* Initialize MPU6050 */
    if(ioctl(fd,I2C_SLAVE,MPU_ADDR)<0)
    {
         perror("MPU6050 not found"); 
         return 1; 
    }
    mpu_write(fd,0x6B,0x00); // PWR_MGMT_1: Clear sleep bit (bit 6) to wake up MPU6050 from sleep mode

    // Read self-test registers for accelerometer factory trim calibration
    uint8_t self_test_x, self_test_y, self_test_z, self_test_a;
    mpu_read(fd,0x0D,&self_test_x,1); // SELF_TEST_X: Read X-axis self-test bits
    mpu_read(fd,0x0E,&self_test_y,1); // SELF_TEST_Y: Read Y-axis self-test bits
    mpu_read(fd,0x0F,&self_test_z,1); // SELF_TEST_Z: Read Z-axis self-test bits
    mpu_read(fd,0x10,&self_test_a,1); // SELF_TEST_A: Read accel self-test bits

    // Calculate XA_TEST (5 bits): bits 4-2 from SELF_TEST_X, bits 1-0 from SELF_TEST_A
    int xa_test = ((self_test_x & 0x1C) >> 2) | (self_test_a & 0x03);
    // Calculate YA_TEST (5 bits): bits 4-2 from SELF_TEST_Y, bits 3-2 from SELF_TEST_A
    int ya_test = ((self_test_y & 0x1C) >> 2) | ((self_test_a & 0x0C) >> 2);
    // Calculate ZA_TEST (5 bits): bits 4-2 from SELF_TEST_Z, bits 5-4 from SELF_TEST_A
    int za_test = ((self_test_z & 0x1C) >> 2) | ((self_test_a & 0x30) >> 4);

    // Calculate Factory Trim (FT) for each axis in LSB units
    float ft_xa = (xa_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (xa_test - 1) / 30.0f) : 0.0f;
    float ft_ya = (ya_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (ya_test - 1) / 30.0f) : 0.0f;
    float ft_za = (za_test != 0) ? 4096.0f * 0.34f * powf(0.92f / 0.34f, (za_test - 1) / 30.0f) : 0.0f;

    uint8_t data[14];
    char line[20];

    while(1)
    {
        ioctl(fd,I2C_SLAVE,MPU_ADDR);
        mpu_read(fd,0x3B,data,14); // Read 14 bytes starting from ACCEL_XOUT_H (0x3B) to GYRO_ZOUT_L (0x48)

        int ax=combine(data[0],data[1]); // ACCEL_XOUT: Raw accelerometer X-axis value (16-bit signed)
        int ay=combine(data[2],data[3]); // ACCEL_YOUT: Raw accelerometer Y-axis value (16-bit signed)
        int az=combine(data[4],data[5]); // ACCEL_ZOUT: Raw accelerometer Z-axis value (16-bit signed)

        // Apply factory trim calibration offsets
        float ax_cal = ax - ft_xa; // Subtract X-axis factory trim
        float ay_cal = ay - ft_ya; // Subtract Y-axis factory trim
        float az_cal = az - ft_za; // Subtract Z-axis factory trim

        // Convert calibrated LSB to g units: sensitivity for ±2g range is 16384 LSB/g
        float ax_g = ax_cal / 16384.0f; // Convert calibrated LSB to g-force for X-axis
        float ay_g = ay_cal / 16384.0f; // Convert calibrated LSB to g-force for Y-axis
        float az_g = az_cal / 16384.0f; // Convert calibrated LSB to g-force for Z-axis

        printf("AX=%.3f AY=%.3f AZ=%.3f\n",ax_g,ay_g,az_g);

        /* Print to OLED */
        ioctl(fd,I2C_SLAVE,OLED_ADDR);
        oled_clear(fd);

        oled_set_cursor(fd,0,0);
        sprintf(line,"AX:%d",ax_g); oled_print(fd,line);

        oled_set_cursor(fd,2,0);
        sprintf(line,"AY:%d",ay_g); oled_print(fd,line);

        oled_set_cursor(fd,4,0);
        sprintf(line,"AZ:%d",az_g); oled_print(fd,line);

        sleep(1);
    }

    close(fd);
    return 0;
}
