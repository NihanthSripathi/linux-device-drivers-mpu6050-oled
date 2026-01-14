#include<linux/module.h>
#include<linux/init.h>
#include<linux/kdev_t.h>
#include<linux/cdev.h>
#include<linux/device.h>
#include<linux/slab.h>
#include<linux/i2c.h>
#include<linux/delay.h>
#include<linux/kernel.h>
#include<linux/ioctl.h>
#include<linux/uaccess.h>
#include<linux/err.h>
#include<linux/kthread.h>
#include<linux/mutex.h>

struct sensor
{
	int32_t adx;
	int32_t ady;
	int32_t adz;
	int32_t gdx;
	int32_t gdy;
	int32_t gdz;
	int32_t td;
	uint8_t self_test_x;
	uint8_t self_test_y;
	uint8_t self_test_z;
	uint8_t self_test_a;
};
struct sensor s;

#define I2C_BUS_1_AVAILABLE ( 1 )          //i2c bus available in our rpi
#define SLAVE_DEVICE_NAME ( "mpu6050" )  //device and driver name
#define SLAVE_ADDR ( 0X68 )  		 //gyro slave address
#define RD_VALUE _IOWR('m','m',int32_t*)

//registers
#define CONFIG ( 0x1A )
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define TEMP_OUT_H 0x41

int32_t data=0;
dev_t dev=0;
static struct class *etx_class;
static struct cdev etx_cdev;

static struct i2c_adapter *i2c_driver_adapter = NULL;  //i2c adapter structure
static struct i2c_client  *i2c_driver_client_gyro =NULL;    //i2c client(gyro) structure
static struct task_struct *sensor_thread;
static struct mutex sensor_mutex;

//int32_t adx,ady,adz,gdx,gdy,gdz,td;
static int data_read(void)   //read sensor data from i2c client
{
	int ret;
	mutex_lock(&sensor_mutex);
	ret = i2c_smbus_read_byte_data(i2c_driver_client_gyro, ACCEL_XOUT_H);
	if (ret < 0) {
		pr_err("Failed to read ACCEL_XOUT_H: %d\n", ret);
		mutex_unlock(&sensor_mutex);
		return ret;
	}
	s.adx = (ret << 8) | i2c_smbus_read_byte_data(i2c_driver_client_gyro, ACCEL_XOUT_H + 1);
	s.ady = (i2c_smbus_read_byte_data(i2c_driver_client_gyro, ACCEL_YOUT_H) << 8) | i2c_smbus_read_byte_data(i2c_driver_client_gyro, ACCEL_YOUT_H + 1);
	s.adz = (i2c_smbus_read_byte_data(i2c_driver_client_gyro, ACCEL_ZOUT_H) << 8) | i2c_smbus_read_byte_data(i2c_driver_client_gyro, ACCEL_ZOUT_H + 1);
	s.gdx = (i2c_smbus_read_byte_data(i2c_driver_client_gyro, GYRO_XOUT_H) << 8) | i2c_smbus_read_byte_data(i2c_driver_client_gyro, GYRO_XOUT_H + 1);
	s.gdy = (i2c_smbus_read_byte_data(i2c_driver_client_gyro, GYRO_YOUT_H) << 8) | i2c_smbus_read_byte_data(i2c_driver_client_gyro, GYRO_YOUT_H + 1);
	s.gdz = (i2c_smbus_read_byte_data(i2c_driver_client_gyro, GYRO_ZOUT_H) << 8) | i2c_smbus_read_byte_data(i2c_driver_client_gyro, GYRO_ZOUT_H + 1);
	s.td = (i2c_smbus_read_byte_data(i2c_driver_client_gyro, TEMP_OUT_H) << 8) | i2c_smbus_read_byte_data(i2c_driver_client_gyro, TEMP_OUT_H + 1);
	mutex_unlock(&sensor_mutex);
	return 0;
}

static int sensor_thread_fn(void *data)
{
	while (!kthread_should_stop()) {
		data_read();
		msleep(100);  // read every 100ms
	}
	return 0;
}

static long etx_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{
	pr_info("1\n");
	mutex_lock(&sensor_mutex);
	if(copy_to_user((struct sensor*) arg,&s,sizeof(struct sensor)))
	{
		pr_err("failed data writing to application\n");
		mutex_unlock(&sensor_mutex);
		return -EFAULT;
	}
	mutex_unlock(&sensor_mutex);
	return 0;
}

static struct file_operations fops =
{
	.owner=THIS_MODULE,
	.unlocked_ioctl=etx_ioctl,
};

static int gyro_probe(struct i2c_client *client)    //probe function
{
	int who_am_i;
	pr_info("probe function invoked\n");

	// Check WHO_AM_I register
	who_am_i = i2c_smbus_read_byte_data(i2c_driver_client_gyro, 0x75);
	if (who_am_i < 0) {
		pr_err("Failed to read WHO_AM_I: %d\n", who_am_i);
		return who_am_i;
	}
	if (who_am_i != 0x68) {
		pr_err("WHO_AM_I mismatch: expected 0x68, got 0x%x\n", who_am_i);
		return -ENODEV;
	}
	pr_info("MPU6050 detected, WHO_AM_I: 0x%x\n", who_am_i);

	// Reset device
	i2c_smbus_write_byte_data(i2c_driver_client_gyro, 0x6B, 0x80);  // DEVICE_RESET
	msleep(100);
	i2c_smbus_write_byte_data(i2c_driver_client_gyro, 0x68, 0x07);  // SIGNAL_PATH_RESET
	msleep(100);

	// Configure sensor
	i2c_smbus_write_byte_data(i2c_driver_client_gyro, CONFIG, 0x00);//configuration(external frame synchronization(FSYNC),digital low pass filter(DLPF))
	i2c_smbus_write_byte_data(i2c_driver_client_gyro, 0x1B, 0x00);//gyroscope full scale range setting (±250°/s)
	i2c_smbus_write_byte_data(i2c_driver_client_gyro, 0x1C, 0x00);//accelerometer full scale range setting (±2g)
	i2c_smbus_write_byte_data(i2c_driver_client_gyro, 0x6B, 0x00);//power management register: wake up, internal oscillator
	msleep(100);  // Allow sensor to stabilize

	// Read self-test registers for accelerometer factory trim calibration
	s.self_test_x = i2c_smbus_read_byte_data(i2c_driver_client_gyro, 0x0D);
	s.self_test_y = i2c_smbus_read_byte_data(i2c_driver_client_gyro, 0x0E);
	s.self_test_z = i2c_smbus_read_byte_data(i2c_driver_client_gyro, 0x0F);
	s.self_test_a = i2c_smbus_read_byte_data(i2c_driver_client_gyro, 0x10);

	mutex_init(&sensor_mutex);
	sensor_thread = kthread_run(sensor_thread_fn, NULL, "sensor_thread");
	if (IS_ERR(sensor_thread)) {
		pr_err("Failed to create sensor thread\n");
		return PTR_ERR(sensor_thread);
	}

	return 0;
}

static const struct i2c_device_id gyro_id[] = {      //device structure that has slave(gyro) device id
	{ SLAVE_DEVICE_NAME,0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c,gyro_id);            //i2c device table id's to userspace

static void gyro_remove(struct i2c_client *client)
{
	pr_info("remove called\n");
}
static struct i2c_driver gyro_driver = {      //client(gyro) i2c driver structure that has to be added to rpi(master/linux)
	.driver = {
		.name =SLAVE_DEVICE_NAME,
		.owner =THIS_MODULE,
	},
	.probe =gyro_probe,
	.id_table =gyro_id,
	.remove =gyro_remove,
};

static struct i2c_board_info gyro_i2c_board_info = {    //i2c board information structure
	I2C_BOARD_INFO(SLAVE_DEVICE_NAME,SLAVE_ADDR)
};

static int __init i2c_driver_init(void)      //module init function
{
	int ret=-1,x;
	x=alloc_chrdev_region(&dev,0,1,"mk_dev");   //allocating major and minor number
	if(x<0)
	{
		pr_err("can't allocate major&minor\n");
		return -1;
	}
	cdev_init(&etx_cdev,&fops);     //creating cdev structure
	x=cdev_add(&etx_cdev,dev,1);
	if(x<0)
	{
		pr_err("failed adding character device to kernel\n");
		goto d_class;
	}
	if(IS_ERR(etx_class=class_create("mk_class")))
	{
		pr_err("failed to create class\n");
		goto d_class;
	}
	if(IS_ERR(device_create(etx_class,NULL,dev,NULL,"mk_device")))
	{
		pr_err("failed to create device\n");
		goto d_device;
	}
	pr_info("3\n");
	i2c_driver_adapter=i2c_get_adapter(I2C_BUS_1_AVAILABLE);
	if(i2c_driver_adapter!=NULL)
	{
		i2c_driver_client_gyro=i2c_new_client_device(i2c_driver_adapter,&gyro_i2c_board_info);
		if(i2c_driver_client_gyro!=NULL)
		{
			i2c_add_driver(&gyro_driver);
			ret=0;
		}
		i2c_put_adapter(i2c_driver_adapter);
	}

	pr_info("Driver Added\n");
	gyro_probe(i2c_driver_client_gyro);
	pr_info("2\n");
	return ret;

d_device:
	class_destroy(etx_class);
	pr_info("4\n");
d_class:
	unregister_chrdev_region(dev,1);
	pr_info("5\n");
	return -1;

}

static void __exit i2c_driver_exit(void)
{
	if (sensor_thread) {
		kthread_stop(sensor_thread);
	}
	device_destroy(etx_class,dev);
	class_destroy(etx_class);
	cdev_del(&etx_cdev);
	unregister_chrdev_region(dev,1);
	i2c_unregister_device(i2c_driver_client_gyro);
	i2c_del_driver(&gyro_driver);
	pr_info("Driver Removed\n");
}

module_init(i2c_driver_init);
module_exit(i2c_driver_exit);
MODULE_LICENSE("GPL");
