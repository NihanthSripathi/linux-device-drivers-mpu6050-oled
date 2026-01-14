#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>

/* OLED I2C Address */
#define OLED_ADDR 0x3C
#define I2C_BUS_1_AVAILABLE ( 1 )          //i2c bus available in our rpi

/* IOCTL Commands */
#define OLED_IOC_MAGIC 'o'
#define OLED_INIT _IO(OLED_IOC_MAGIC, 1)
#define OLED_CLEAR _IO(OLED_IOC_MAGIC, 2)
#define OLED_SET_CURSOR _IOW(OLED_IOC_MAGIC, 3, struct oled_cursor)
#define OLED_PRINT _IOW(OLED_IOC_MAGIC, 4, char *)

struct oled_cursor {
    uint8_t page;
    uint8_t col;
};

/* Minimalist 5x7 Font (ASCII 32–90) */
static const unsigned char font[][5] = {
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

static struct i2c_client *oled_client;
static int major;
static struct class *oled_class;
static struct device *oled_device;
static struct i2c_adapter *i2c_driver_adapter = NULL;  //i2c adapter structure
static struct i2c_client  *i2c_driver_client_oled = NULL;    //i2c client(oled) structure

static void send_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_send(oled_client, buf, 2);
}

static void send_data(uint8_t data)
{
    uint8_t buf[2] = {0x40, data};
    i2c_master_send(oled_client, buf, 2);
}

static void oled_init(void)
{
    send_cmd(0xAE); send_cmd(0xD5); send_cmd(0x80);
    send_cmd(0xA8); send_cmd(0x3F);
    send_cmd(0xD3); send_cmd(0x00);
    send_cmd(0x40);
    send_cmd(0x8D); send_cmd(0x14);
    send_cmd(0x20); send_cmd(0x00);
    send_cmd(0xA1); send_cmd(0xC8);
    send_cmd(0xDA); send_cmd(0x12);
    send_cmd(0x81); send_cmd(0xCF);
    send_cmd(0xD9); send_cmd(0xF1);
    send_cmd(0xDB); send_cmd(0x40);
    send_cmd(0xA4); send_cmd(0xA6);
    send_cmd(0xAF);
}

static void oled_clear(void)
{
    int page, i;
    for (page = 0; page < 8; page++) {
        send_cmd(0xB0 + page);
        send_cmd(0x00);
        send_cmd(0x10);
        for (i = 0; i < 128; i++)
            send_data(0x00);
    }
}

static void oled_set_cursor(uint8_t page, uint8_t col)
{
    send_cmd(0xB0 + page);
    send_cmd((col >> 4) | 0x10);
    send_cmd(col & 0x0F);
}

static void oled_print(const char *str)
{
    char c;
    int i;
    while ((c = *str++)) {
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c >= 32 && c <= 90) {
            for (i = 0; i < 5; i++) send_data(font[c - 32][i]);
            send_data(0x00);
        } else {
            for (i = 0; i < 6; i++) send_data(0x00);
        }
    }
}

static long oled_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct oled_cursor cursor;
    char *str;

    switch (cmd) {
    case OLED_INIT:
        oled_init();
        break;
    case OLED_CLEAR:
        oled_clear();
        break;
    case OLED_SET_CURSOR:
        if (copy_from_user(&cursor, (void __user *)arg, sizeof(cursor)))
            return -EFAULT;
        oled_set_cursor(cursor.page, cursor.col);
        break;
    case OLED_PRINT:
        str = kmalloc(256, GFP_KERNEL);
        if (!str) return -ENOMEM;
        if (copy_from_user(str, (void __user *)arg, 256)) {
            kfree(str);
            return -EFAULT;
        }
        oled_print(str);
        kfree(str);
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int oled_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int oled_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations oled_fops = {
    .owner = THIS_MODULE,
    .open = oled_open,
    .release = oled_release,
    .unlocked_ioctl = oled_ioctl,
};

static int oled_probe(struct i2c_client *client)
{
    pr_info("probe function invoked\n");
    oled_client = client;
    major = register_chrdev(0, "oled", &oled_fops);
    if (major < 0) return major;

    oled_class = class_create("oled_class");
    if (IS_ERR(oled_class)) {
        unregister_chrdev(major, "oled");
        return PTR_ERR(oled_class);
    }

    oled_device = device_create(oled_class, NULL, MKDEV(major, 0), NULL, "oleddevice");
    if (IS_ERR(oled_device)) {
        class_destroy(oled_class);
        unregister_chrdev(major, "oled");
        return PTR_ERR(oled_device);
    }

    pr_info("OLED driver probed\n");
    return 0;
}

static void oled_remove(struct i2c_client *client)
{
    device_destroy(oled_class, MKDEV(major, 0));
    class_destroy(oled_class);
    unregister_chrdev(major, "oled");
    pr_info("OLED driver removed\n");
}

static const struct i2c_device_id oled_id[] = {
    { "oled", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, oled_id);

static struct i2c_driver oled_driver = {
    .driver = {
        .name = "oled",
        .owner = THIS_MODULE,
    },
    .probe = oled_probe,
    .remove = oled_remove,
    .id_table = oled_id,
};

static struct i2c_board_info oled_i2c_board_info = {    //i2c board information structure
	I2C_BOARD_INFO("oled",OLED_ADDR)
};

static int __init oled_driver_init(void)      //module init function
{
	int ret=-1;
	pr_info("3\n");
	i2c_driver_adapter=i2c_get_adapter(I2C_BUS_1_AVAILABLE);
	if(i2c_driver_adapter!=NULL)
	{
		i2c_driver_client_oled=i2c_new_client_device(i2c_driver_adapter,&oled_i2c_board_info);
		if(i2c_driver_client_oled!=NULL)
		{
			i2c_add_driver(&oled_driver);
			ret=0;
		}
		i2c_put_adapter(i2c_driver_adapter);
	}

	pr_info("OLED Driver Added\n");
	return ret;
}

static void __exit oled_driver_exit(void)
{
	i2c_unregister_device(i2c_driver_client_oled);
	i2c_del_driver(&oled_driver);
	pr_info("OLED Driver Removed\n");
}

module_init(oled_driver_init);
module_exit(oled_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AI Assistant");
MODULE_DESCRIPTION("OLED Display Driver");
