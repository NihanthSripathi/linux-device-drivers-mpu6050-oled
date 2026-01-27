/*
 * MPU6050 I2C Character Device Driver
 * 
 * This driver creates a character device interface to communicate with
 * the MPU6050 6-axis motion sensor (3-axis accelerometer + 3-axis gyroscope)
 * over the I2C bus.
 * 
 * This version reads I2C data on-demand only when the application calls ioctl,
 * rather than continuously polling in a kernel thread. No mutex is needed since
 * there is no concurrent access (no kernel thread, only ioctl calls are sequential).
 * 
 * Register Map Reference:
 * -----------------------
 * 0x00-0x0C: Self-test & Bias registers
 * 0x0D-0x17: Accelerometer & Gyroscope configuration
 * 0x1A-0x1F: Configuration & FIFO
 * 0x20-0x23: I2C control
 * 0x24-0x36: Data output registers
 * 0x38: FIFO enable
 * 0x3B-0x48: Sensor data output (accel, temp, gyro)
 * 0x60-0x64: Signal path reset
 * 0x68-0x6A: Motion detection
 * 0x6B-0x6F: Power management
 * 0x70-0x77: FIFO
 * 0x75: WHO_AM_I (device ID)
 */

#include <linux/module.h>      // Mandatory for all kernel modules (MODULE_LICENSE, module_init)
#include <linux/init.h>        // Provides __init and __exit macros
#include <linux/fs.h>          // Required for file_operations structure
#include <linux/cdev.h>        // Needed to create and register character devices
#include <linux/device.h>      // Used for class_create() and device_create()
#include <linux/i2c.h>         // Core Linux I2C framework APIs
#include <linux/uaccess.h>     // Required for copy_to_user()
#include <linux/delay.h>       // Provides msleep() delay function

#include "mpu6050_registers.h" // Contains MPU6050 register address definitions


/* ==================== DEVICE DEFINITIONS ==================== */

#define DEVICE_NAME     "mpu6050_dev"      // Name of character device (/dev/mpu6050_dev)
#define CLASS_NAME      "mpu6050_class"    // Device class name for udev
#define I2C_SLAVE_NAME  "mpu6050"          // I2C device name (used for driver matching)
#define MPU6050_ADDR    0x68               // I2C slave address of MPU6050

#define RD_VALUE _IOR('m', 'm', struct sensor) // IOCTL command to read sensor data


/* ==================== MPU6050 CONFIGURATION ==================== */

/* Accelerometer Full-Scale Range Settings (Register 0x1C) */
#define ACCEL_FS_2G     0x00   // ±2g range (16384 LSB/g)
#define ACCEL_FS_4G     0x08   // ±4g range (8192 LSB/g)
#define ACCEL_FS_8G     0x10   // ±8g range (4096 LSB/g)
#define ACCEL_FS_16G    0x18   // ±16g range (2048 LSB/g)

/* Gyroscope Full-Scale Range Settings (Register 0x1B) */
#define GYRO_FS_250     0x00   // ±250°/s range (131 LSB/°/s)
#define GYRO_FS_500     0x08   // ±500°/s range (65.5 LSB/°/s)
#define GYRO_FS_1000    0x10   // ±1000°/s range (32.8 LSB/°/s)
#define GYRO_FS_2000    0x18   // ±2000°/s range (16.4 LSB/°/s)

/* Digital Low Pass Filter (DLPF) Settings (Register 0x1A) */
#define DLPF_260HZ      0x00   // Bandwidth 260Hz, delay 0ms
#define DLPF_184HZ      0x01   // Bandwidth 184Hz, delay 2.0ms
#define DLPF_94HZ       0x02   // Bandwidth 94Hz, delay 2.3ms
#define DLPF_44HZ       0x03   // Bandwidth 44Hz, delay 4.35ms
#define DLPF_21HZ       0x04   // Bandwidth 21Hz, delay 6.7ms
#define DLPF_10HZ       0x05   // Bandwidth 10Hz, delay 11.8ms
#define DLPF_5HZ        0x06   // Bandwidth 5Hz, delay 18.6ms

/* Sample Rate Divider (Register 0x19) - Sample Rate = 1kHz / (1 + SMPLRT_DIV) */
#define SMPLRT_DIV      9      // Sample rate = 100Hz (1000 / (1+9))


/* ==================== DATA STRUCTURES ==================== */

/* Structure to store sensor values */
struct sensor {
    s16 ax, ay, az;    // Accelerometer X, Y, Z data (raw 16-bit)
    s16 gx, gy, gz;    // Gyroscope X, Y, Z data (raw 16-bit)
    s16 temp;          // Temperature data (raw 16-bit)
};

/* Structure for configuration settings (optional user-space control) */
struct mpu_config {
    u8 accel_range;    // Accelerometer range (0=±2g, 1=±4g, 2=±8g, 3=±16g)
    u8 gyro_range;     // Gyroscope range (0=±250, 1=±500, 2=±1000, 3=±2000 °/s)
    u8 dlpf;           // Digital Low Pass Filter setting (0-6)
    u8 sample_rate;    // Sample rate divisor (0-255)
};

static struct sensor sensor_data; // Global structure holding latest sensor readings
static struct mpu_config mpu_settings; // Configuration settings


/* ==================== GLOBAL VARIABLES ==================== */

/* Character device variables */
static dev_t dev;                 // Stores major and minor numbers
static struct cdev mpu_cdev;      // Character device structure
static struct class *mpu_class;   // Device class pointer


/* I2C related variables */
static struct i2c_client *mpu_client;   // Pointer to I2C client (represents MPU6050)


/* ==================== CONFIGURATION FUNCTIONS ==================== */

/*
 * mpu6050_configure_sensor() - Configure MPU6050 sensor settings
 * 
 * This function sets up the accelerometer range, gyroscope range,
 * DLPF filter, and sample rate for the MPU6050 sensor.
 * 
 * Configuration Registers Used:
 * ----------------------------
 * ACCEL_CONFIG (0x1C) - Sets accelerometer full-scale range
 * GYRO_CONFIG (0x1B)  - Sets gyroscope full-scale range
 * CONFIG (0x1A)       - Sets DLPF bandwidth
 * SMPLRT_DIV (0x19)   - Sets sample rate divider
 */
static void mpu6050_configure_sensor(void)
{
    /* Configure Accelerometer Full-Scale Range (Register 0x1C)
     * Bits [4:3] control the range:
     * 00 = ±2g, 01 = ±4g, 10 = ±8g, 11 = ±16g
     * Default: ±2g (ACCEL_FS_2G = 0x00)
     */
    i2c_smbus_write_byte_data(mpu_client, ACCEL_CONFIG, ACCEL_FS_2G);
    
    /* Configure Gyroscope Full-Scale Range (Register 0x1B)
     * Bits [4:3] control the range:
     * 00 = ±250°/s, 01 = ±500°/s, 10 = ±1000°/s, 11 = ±2000°/s
     * Default: ±250°/s (GYRO_FS_250 = 0x00)
     */
    i2c_smbus_write_byte_data(mpu_client, GYRO_CONFIG, GYRO_FS_250);
    
    /* Configure DLPF (Digital Low Pass Filter) (Register 0x1A)
     * Bits [2:0] control the filter bandwidth and delay:
     * DLPF_260HZ (0x00) - 260Hz bandwidth, 0ms delay
     * DLPF_184HZ (0x01) - 184Hz bandwidth, 2.0ms delay
     * DLPF_94HZ  (0x02) - 94Hz bandwidth, 2.3ms delay
     * DLPF_44HZ  (0x03) - 44Hz bandwidth, 4.35ms delay
     * DLPF_21HZ  (0x04) - 21Hz bandwidth, 6.7ms delay
     * DLPF_10HZ  (0x05) - 10Hz bandwidth, 11.8ms delay
     * DLPF_5HZ   (0x06) - 5Hz bandwidth, 18.6ms delay
     * Default: DLPF_260HZ (0x00)
     */
    i2c_smbus_write_byte_data(mpu_client, CONFIG, DLPF_260HZ);
    
    /* Configure Sample Rate Divider (Register 0x19)
     * Sample Rate = 1kHz / (1 + SMPLRT_DIV)
     * With SMPLRT_DIV = 9: Sample rate = 1000 / 10 = 100Hz
     * Value range: 0-255
     */
    i2c_smbus_write_byte_data(mpu_client, SMPLRT_DIV, SMPLRT_DIV);
    
    /* Store current settings for reference */
    mpu_settings.accel_range = ACCEL_FS_2G;
    mpu_settings.gyro_range = GYRO_FS_250;
    mpu_settings.dlpf = DLPF_260HZ;
    mpu_settings.sample_rate = SMPLRT_DIV;
    
    pr_info("MPU6050 configured: Accel=±2g, Gyro=±250°/s, DLPF=260Hz, Rate=100Hz\n");
}


/* ==================== DATA READ FUNCTIONS ==================== */

/*
 * mpu6050_read_data() - Read all sensor data from MPU6050
 * 
 * This function reads accelerometer (X, Y, Z), temperature, and gyroscope
 * data from the MPU6050 sensor registers.
 * 
 * Data Register Map:
 * ------------------
 * Accelerometer Output (16-bit signed, High byte first):
 *   AX: 0x3B (High), 0x3C (Low)
 *   AY: 0x3D (High), 0x3E (Low)
 *   AZ: 0x3F (High), 0x40 (Low)
 *   
 * Temperature Output (16-bit signed):
 *   TEMP: 0x41 (High), 0x42 (Low)
 *   Formula: Temp°C = (TEMP_OUT / 340) + 36.53
 *   
 * Gyroscope Output (16-bit signed):
 *   GX: 0x43 (High), 0x44 (Low)
 *   GY: 0x45 (High), 0x46 (Low)
 *   GZ: 0x47 (High), 0x48 (Low)
 * 
 * Note: No mutex needed since this function is called only from ioctl
 * which executes sequentially (no concurrent access).
 */
static void mpu6050_read_data(void)
{
    /* Read Accelerometer X-axis (Registers 0x3B, 0x3C) */
    sensor_data.ax = (i2c_smbus_read_byte_data(mpu_client, ACCEL_XOUT_H) << 8) |  // 0x3B
                      i2c_smbus_read_byte_data(mpu_client, ACCEL_XOUT_L);         // 0x3C

    /* Read Accelerometer Y-axis (Registers 0x3D, 0x3E) */
    sensor_data.ay = (i2c_smbus_read_byte_data(mpu_client, ACCEL_YOUT_H) << 8) |  // 0x3D
                      i2c_smbus_read_byte_data(mpu_client, ACCEL_YOUT_L);         // 0x3E

    /* Read Accelerometer Z-axis (Registers 0x3F, 0x40) */
    sensor_data.az = (i2c_smbus_read_byte_data(mpu_client, ACCEL_ZOUT_H) << 8) |  // 0x3F
                      i2c_smbus_read_byte_data(mpu_client, ACCEL_ZOUT_L);         // 0x40

    /* Read Temperature (Registers 0x41, 0x42) */
    sensor_data.temp = (i2c_smbus_read_byte_data(mpu_client, TEMP_OUT_H) << 8) |  // 0x41
                        i2c_smbus_read_byte_data(mpu_client, TEMP_OUT_L);         // 0x42

    /* Read Gyroscope X-axis (Registers 0x43, 0x44) */
    sensor_data.gx = (i2c_smbus_read_byte_data(mpu_client, GYRO_XOUT_H) << 8) |   // 0x43
                      i2c_smbus_read_byte_data(mpu_client, GYRO_XOUT_L);          // 0x44

    /* Read Gyroscope Y-axis (Registers 0x45, 0x46) */
    sensor_data.gy = (i2c_smbus_read_byte_data(mpu_client, GYRO_YOUT_H) << 8) |   // 0x45
                      i2c_smbus_read_byte_data(mpu_client, GYRO_YOUT_L);          // 0x46

    /* Read Gyroscope Z-axis (Registers 0x47, 0x48) */
    sensor_data.gz = (i2c_smbus_read_byte_data(mpu_client, GYRO_ZOUT_H) << 8) |   // 0x47
                      i2c_smbus_read_byte_data(mpu_client, GYRO_ZOUT_L);          // 0x48
}


/* ==================== IOCTL HANDLER ==================== */

/*
 * mpu_ioctl() - IOCTL handler for user-space communication
 * 
 * This function handles IOCTL commands from user-space applications.
 * When RD_VALUE command is received, it reads fresh sensor data from the
 * MPU6050 I2C device and returns it to the user application.
 * 
 * Supported commands:
 *   RD_VALUE (0x...): Read sensor data structure (reads I2C on-demand)
 * 
 * IOCTL Number: _IOR('m', 'm', struct sensor)
 * 'm' = magic number (identifies driver type)
 * 'm' = command number within magic group
 */
static long mpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    
    /* Check for valid IOCTL command */
    if (cmd == RD_VALUE) {
        /* Read fresh sensor data from I2C device on-demand */
        mpu6050_read_data();
        
        /* Copy sensor data to user space */
        if (copy_to_user((struct sensor *)arg,
                         &sensor_data,
                         sizeof(sensor_data))) {
            pr_err("Failed to copy data to user space\n");
            return -EFAULT;  // Bad address error
        }
        pr_debug("Sensor data sent to user space\n");
    }
    else {
        pr_err("Unknown IOCTL command: 0x%X\n", cmd);
        return -ENOTTY;  // Inappropriate ioctl
    }
    
    return ret;
}


/* ==================== FILE OPERATIONS ==================== */

/* File operations structure for character device */
static struct file_operations fops = {
    .owner          = THIS_MODULE,    // Prevent module unload while in use
    .unlocked_ioctl = mpu_ioctl,      // IOCTL callback
};


/* ==================== I2C DRIVER CALLBACKS ==================== */

/*
 * mpu6050_probe() - Called when MPU6050 device is detected
 * 
 * This function is called by the I2C core when a matching device is found.
 * It initializes the sensor (no kernel thread is started).
 * 
 * Key Steps:
 * 1. Verify device identity by reading WHO_AM_I register (0x75)
 * 2. Wake up the sensor by writing to PWR_MGMT_1 register (0x6B)
 * 3. Configure sensor settings (accel/gyro range, DLPF, sample rate)
 * 
 * Note: No kernel thread is started. Data is read on-demand when app calls ioctl.
 * 
 * Return: 0 on success, negative error code on failure
 */
static int mpu6050_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int whoami;

    pr_info("MPU6050 probe called\n");
    
    /* Save I2C client pointer for later use */
    mpu_client = client;

    /* Step 1: Read WHO_AM_I register (0x75) to verify chip identity
     * MPU6050 returns 0x68 (104 decimal) from this register
     * This confirms we have the correct device
     */
    whoami = i2c_smbus_read_byte_data(client, WHO_AM_I); // 0x75 - Device ID register
    if (whoami != 0x68) {
        pr_err("MPU6050 not found! WHO_AM_I=0x%02X (expected 0x68)\n", whoami);
        return -ENODEV;  // No such device
    }
    pr_info("MPU6050 detected, WHO_AM_I=0x%02X\n", whoami);

    /* Step 2: Wake up the sensor
     * PWR_MGMT_1 register (0x6B) controls power management
     * Bit 0 (SLEEP): 0 = normal mode, 1 = sleep mode
     * Writing 0x00 clears the sleep bit and activates the sensor
     */
    i2c_smbus_write_byte_data(client, PWR_MGMT_1, 0x00); // 0x6B - Clear sleep bit
    msleep(100);  // Wait for sensor stabilization

    /* Step 3: Configure sensor settings */
    mpu6050_configure_sensor();

    pr_info("MPU6050 probe successful (on-demand mode, no thread, no mutex)\n");
    return 0;
}


/*
 * mpu6050_remove() - Called when device or module is removed
 * 
 * This function cleans up resources when the device is removed
 * or the module is unloaded.
 * 
 * Note: No kernel thread to stop, no mutex to destroy, just put sensor into sleep mode.
 */
static void mpu6050_remove(struct i2c_client *client)
{
    pr_info("MPU6050 remove called\n");

    /* Put sensor into sleep mode to conserve power */
    i2c_smbus_write_byte_data(client, PWR_MGMT_1, 0x40); // 0x6B - Set sleep bit

    pr_info("MPU6050 removed\n");
}


/* ==================== DEVICE ID TABLE ==================== */

/* I2C device ID table for driver matching
 * This table tells the kernel which I2C devices this driver supports.
 * The name must match the device name in device tree or ACPI tables.
 */
static const struct i2c_device_id mpu_id[] = {
    { I2C_SLAVE_NAME, 0 },  // Matches device name "mpu6050"
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu_id);


/* ==================== I2C DRIVER STRUCTURE ==================== */

/* I2C driver structure
 * This structure registers the probe/remove callbacks with the I2C core.
 */
static struct i2c_driver mpu6050_driver = {
    .driver = {
        .name = I2C_SLAVE_NAME,  // Driver name used for matching
    },
    .probe    = mpu6050_probe,    // Called when device is found
    .remove   = mpu6050_remove,   // Called when device is removed
    .id_table = mpu_id,           // Device ID table
};


/* ==================== MODULE INIT/EXIT ==================== */

/*
 * mpu_init() - Module initialization function
 * 
 * This function is called when the module is loaded.
 * It initializes the character device and registers the I2C driver.
 * 
 * Initialization Sequence (no mutex needed):
 * 1. Allocate major/minor numbers for character device (alloc_chrdev_region)
 * 2. Initialize cdev structure with file operations
 * 3. Add cdev to kernel (cdev_add)
 * 4. Create device class (class_create)
 * 5. Create /dev node (device_create)
 * 6. Register I2C driver (i2c_add_driver)
 */
static int __init mpu_init(void)
{
    pr_info("MPU6050 driver init\n");

    /* Step 1: Allocate character device region
     * alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME)
     * - dev: Output parameter for major/minor numbers
     * - 0: Request first available minor number
     * - 1: Number of minor numbers to allocate
     * - DEVICE_NAME: Name for /proc/devices
     */
    alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME); // 0x00 - Allocate major/minor numbers

    /* Step 2: Initialize character device structure */
    cdev_init(&mpu_cdev, &fops);  // Initialize cdev with file operations

    /* Step 3: Add character device to kernel */
    cdev_add(&mpu_cdev, dev, 1);  // Add cdev, using 1 device number

    /* Step 4: Create device class (for udev) */
    mpu_class = class_create(CLASS_NAME);  // Creates /sys/class/mpu6050_class

    /* Step 5: Create device node in /dev */
    device_create(mpu_class, NULL, dev, NULL, DEVICE_NAME);  // Creates /dev/mpu6050_dev

    /* Step 6: Register I2C driver */
    i2c_add_driver(&mpu6050_driver);

    pr_info("MPU6050 driver loaded successfully (on-demand mode, no mutex)\n");
    return 0;
}


/*
 * mpu_exit() - Module cleanup function
 * 
 * This function is called when the module is unloaded.
 * It reverses all initialization steps in reverse order.
 * No mutex destroy needed.
 */
static void __exit mpu_exit(void)
{
    pr_info("MPU6050 driver exit\n");

    /* Step 1: Unregister I2C driver */
    i2c_del_driver(&mpu6050_driver);

    /* Step 2: Remove device node */
    device_destroy(mpu_class, dev);

    /* Step 3: Destroy class */
    class_destroy(mpu_class);

    /* Step 4: Remove character device */
    cdev_del(&mpu_cdev);

    /* Step 5: Unregister character device region */
    unregister_chrdev_region(dev, 1);

    pr_info("MPU6050 driver unloaded\n");
}


/* ==================== MODULE METADATA ==================== */

module_init(mpu_init);               // Register module init function
module_exit(mpu_exit);               // Register module exit function

MODULE_LICENSE("GPL");               // License declaration
MODULE_AUTHOR("Nihanth");            // Author information
MODULE_DESCRIPTION("MPU6050 I2C Character Device Driver - On-demand data reading (no kernel thread, no mutex)");
MODULE_VERSION("2.0");

