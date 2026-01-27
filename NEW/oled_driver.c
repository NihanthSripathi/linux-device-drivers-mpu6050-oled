/*
 * OLED SSD1306 I2C Display Driver
 *
 * This driver provides a character device interface to control
 * SSD1306-based OLED displays over the I2C bus.
 *
 * Display Specifications:
 * - Resolution: 128x64 pixels
 * - Interface: I2C (default address 0x3C)
 * - Controller: SSD1306
 * - Pages: 8 pages (each page = 8 rows = 64 rows total)
 * - Columns: 128 columns
 *
 * I2C Protocol:
 * - Control byte: 0x00 (command), 0x40 (data)
 * - Maximum transfer: 32 bytes per transaction (Linux I2C limitation)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/slab.h>

/* ==================== DEVICE DEFINITIONS ==================== */

#define OLED_ADDR      0x3C    // SSD1306 I2C slave address
#define I2C_BUS        1       // I2C bus number (Raspberry Pi default)
#define DEVICE_NAME    "oled"  // Character device name (/dev/oled)
#define CLASS_NAME     "oled_class"

/* IOCTL Command Definitions
 * Format: _IO/MR/MW(type, nr)
 * - OLED_IOC_MAGIC: 'o' - identifies this driver
 * - nr: command number (1-4)
 */
#define OLED_IOC_MAGIC 'o'
#define OLED_INIT      _IO(OLED_IOC_MAGIC, 1)   // Initialize display
#define OLED_CLEAR     _IO(OLED_IOC_MAGIC, 2)   // Clear screen
#define OLED_SET_CURSOR _IOW(OLED_IOC_MAGIC, 3, struct oled_pos)
#define OLED_PRINT     _IOW(OLED_IOC_MAGIC, 4, char *)

/* Cursor position structure for OLED_PRINT_SET_CURSOR ioctl
 * Used to specify where text should start on the display
 */
struct oled_pos {
    uint8_t page;   // Page number (0-7, each page = 8 rows)
    uint8_t col;    // Column number (0-127)
};


/* ==================== 5x7 FONT DEFINITION ==================== */

/*
 * 5x7 Pixel Font for ASCII Characters (32-90)
 *
 * HOW THE FONT WORKS:
 * -------------------
 * Each character is represented as a 5x8 bitmap (5 columns x 8 rows).
 * Only the upper 7 rows are used for character data; row 7 is spacing.
 *
 * BIT INTERPRETATION:
 * - Each byte represents ONE column of 8 vertical pixels
 * - Bit 7 (MSB) = top pixel of the column
 * - Bit 0 (LSB) = bottom pixel of the column
 * - Bit value 1 = pixel ON (visible)
 * - Bit value 0 = pixel OFF (empty)
 *
 * EXAMPLE: Letter 'Z' = {0x61, 0x51, 0x49, 0x45, 0x43}
 *
 * Column 0    Column 1    Column 2    Column 3    Column 4
 * 0x61        0x51        0x49        0x45        0x43
 * Binary:     Binary:     Binary:     Binary:     Binary:
 * 01100001    01010001    01001001    01000101    01000011
 *  .            .           .           .           .
 *  .            .           .           .           .
 * 1            1           1           1           1   <- row 3
 * 1            1           .           1           1   <- row 2
 * 1            .           1           .           .   <- row 1
 * .            .           .           .           .   <- row 0 (spacing)
 *
 * Visual 'Z':
 *    ****
 *      *
 *    *
 *   ****
 *
 * EXAMPLE: Letter 'A' = {0x7E, 0x11, 0x11, 0x11, 0x7E}
 *
 * 0x7E = 01111110  (column 0: all top 6 bits ON)
 * 0x11 = 00010001  (column 1: top OFF, then ON, OFF, then ON at bottom)
 * 0x11 = 00010001  (column 2: same as column 1)
 * 0x11 = 00010001  (column 3: same as column 1)
 * 0x7E = 01111110  (column 4: all top 6 bits ON)
 *
 * Visual 'A':
 *   ******
 *   *    *
 *   *    *
 *   ******
 *   *    *
 *   *    *
 *
 * DISPLAY PROCESS:
 * 1. For character 'A', index = 'A' - 32 = 0
 * 2. Send bytes: 0x7E, 0x11, 0x11, 0x11, 0x7E
 * 3. Send 0x00 for 1-pixel spacing after character
 *
 * ARRAY INDEXING:
 * - font[0] = space (ASCII 32)
 * - font[1] = ! (ASCII 33)
 * - font[58] = Z (ASCII 90)
 */
static const unsigned char font[][5] = {
    /* Space */ {0x00,0x00,0x00,0x00,0x00},
    /* ! */     {0x00,0x00,0x5f,0x00,0x00},
    /* " */     {0x00,0x07,0x00,0x07,0x00},
    /* # */     {0x14,0x7f,0x14,0x7f,0x14},
    /* $ */     {0x24,0x2a,0x7f,0x2a,0x12},
    /* % */     {0x23,0x13,0x08,0x64,0x62},
    /* & */     {0x36,0x49,0x55,0x22,0x50},
    /* ' */     {0x00,0x05,0x03,0x00,0x00},
    /* ( */     {0x00,0x1c,0x22,0x41,0x00},
    /* ) */     {0x00,0x41,0x22,0x1c,0x00},
    /* * */     {0x14,0x08,0x3e,0x08,0x14},
    /* + */     {0x08,0x08,0x3e,0x08,0x08},
    /* , */     {0x00,0x50,0x30,0x00,0x00},
    /* - */     {0x08,0x08,0x08,0x08,0x08},
    /* . */     {0x00,0x60,0x60,0x00,0x00},
    /* / */     {0x20,0x10,0x08,0x04,0x02},
    /* 0 */     {0x3e,0x51,0x49,0x45,0x3e},
    /* 1 */     {0x00,0x42,0x7f,0x40,0x00},
    /* 2 */     {0x42,0x61,0x51,0x49,0x46},
    /* 3 */     {0x21,0x41,0x45,0x4b,0x31},
    /* 4 */     {0x18,0x14,0x12,0x7f,0x10},
    /* 5 */     {0x27,0x45,0x45,0x45,0x39},
    /* 6 */     {0x3c,0x4a,0x49,0x49,0x30},
    /* 7 */     {0x01,0x71,0x09,0x05,0x03},
    /* 8 */     {0x36,0x49,0x49,0x49,0x36},
    /* 9 */     {0x06,0x49,0x49,0x29,0x1e},
    /* : */     {0x00,0x36,0x36,0x00,0x00},
    /* ; */     {0x00,0x56,0x36,0x00,0x00},
    /* < */     {0x08,0x14,0x22,0x41,0x00},
    /* = */     {0x14,0x14,0x14,0x14,0x14},
    /* > */     {0x00,0x41,0x22,0x14,0x08},
    /* ? */     {0x02,0x01,0x51,0x09,0x06},
    /* @ */     {0x32,0x49,0x79,0x41,0x3e},
    /* A */     {0x7e,0x11,0x11,0x11,0x7e},
    /* B */     {0x7f,0x49,0x49,0x49,0x36},
    /* C */     {0x3e,0x41,0x41,0x41,0x22},
    /* D */     {0x7f,0x41,0x41,0x22,0x1c},
    /* E */     {0x7f,0x49,0x49,0x49,0x41},
    /* F */     {0x7f,0x09,0x09,0x09,0x01},
    /* G */     {0x3e,0x41,0x49,0x49,0x7a},
    /* H */     {0x7f,0x08,0x08,0x08,0x7f},
    /* I */     {0x00,0x41,0x7f,0x41,0x00},
    /* J */     {0x20,0x40,0x41,0x3f,0x01},
    /* K */     {0x7f,0x08,0x14,0x22,0x41},
    /* L */     {0x7f,0x40,0x40,0x40,0x40},
    /* M */     {0x7f,0x02,0x0c,0x02,0x7f},
    /* N */     {0x7f,0x04,0x08,0x10,0x7f},
    /* O */     {0x3e,0x41,0x41,0x41,0x3e},
    /* P */     {0x7f,0x09,0x09,0x09,0x06},
    /* Q */     {0x3e,0x41,0x51,0x21,0x5e},
    /* R */     {0x7f,0x09,0x19,0x29,0x46},
    /* S */     {0x46,0x49,0x49,0x49,0x31},
    /* T */     {0x01,0x01,0x7f,0x01,0x01},
    /* U */     {0x3f,0x40,0x40,0x40,0x3f},
    /* V */     {0x1f,0x20,0x40,0x20,0x1f},
    /* W */     {0x3f,0x40,0x38,0x40,0x3f},
    /* X */     {0x63,0x14,0x08,0x14,0x63},
    /* Y */     {0x07,0x08,0x70,0x08,0x07},
    /* Z */     {0x61,0x51,0x49,0x45,0x43}
};


/* ==================== GLOBAL VARIABLES ==================== */

static struct i2c_client *oled_client;   // I2C device client - represents our OLED on the bus
static int major;                        // Major number - identifies our driver to kernel
static struct class *oled_class;         // Device class - for sysfs and udev
static struct device *oled_device;       // Device node - the /dev/oleddevice entry


/* ==================== I2C COMMUNICATION ==================== */

/*
 * I2C PROTOCOL FOR SSD1306 OLED:
 * -------------------------------
 * The SSD1306 display expects I2C data in a specific format:
 *
 * [START] [SLAVE_ADDR+W] [ACK] [CONTROL_BYTE] [DATA_BYTE] [ACK] [STOP]
 *
 * SLAVE_ADDR: 0x3C (7-bit address, Linux shifts left to 0x78 for 8-bit)
 * CONTROL_BYTE: 0x00 = command, 0x40 = data
 *
 * EXAMPLE: Send command 0xAE (display off)
 * Buffer: [0x00, 0xAE]
 *          |     |
 *          |     +-- Command byte (0xAE = turn off display)
 *          +-- Control byte (0x00 = this is a command)
 *
 * Physical I2C transaction on the wire:
 * [S]  78  00  [A] AE [A] [P]
 *  |   |   |   |   |       |
 *  |   |   |   |   |       +-- STOP (P)
 *  |   |   |   |   +-- ACK (A) from OLED
 *  |   |   |   +-- Data byte (0xAE = display off command)
 *  |   |   +-- Control byte (0x00 = this is a command)
 *  |   +-- Slave address (0x78 = 0x3C << 1 with write bit)
 *  +-- START condition (S)
 *
 * WHY 2-BYTE BUFFER?
 * - I2C is a byte-oriented protocol
 * - First byte tells OLED: "what I'm sending is a COMMAND"
 * - Second byte is: "the actual command value"
 */

/*
 * send_cmd() - Send command byte to OLED display
 * @cmd: Command byte to send (e.g., 0xAE for display off)
 *
 * Sends a 2-byte sequence: [0x00 | cmd]
 * - 0x00: Control byte (Co=0, D/C#=0) = "this is a command"
 */
static void send_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_send(oled_client, buf, 2);
}

/*
 * send_data() - Send pixel data byte to OLED display
 * @data: Pixel data byte (8 vertical pixels in one column)
 *
 * Sends a 2-byte sequence: [0x40 | data]
 * - 0x40: Control byte (Co=0, D/C#=1) = "this is pixel data"
 *
 * NOTE: The OLED display has 128 columns x 64 rows.
 * Each byte fills 8 vertical pixels in one column.
 */
static void send_data(uint8_t data)
{
    uint8_t buf[2] = {0x40, data};
    i2c_master_send(oled_client, buf, 2);
}


/* ==================== OLED DISPLAY FUNCTIONS ==================== */

/*
 * oled_init() - Initialize OLED display with SSD1306 defaults
 *
 * Initialization sequence based on SSD1306 datasheet:
 * 1. Display off (AEh)
 * 2. Set display clock divide ratio (D5h, 80h = 100Hz)
 * 3. Set multiplex ratio (A8h, 3Fh = 64 rows)
 * 4. Set display offset (D3h, 00h = no offset)
 * 5. Set start line (40h = row 0)
 * 6. Enable charge pump (8Dh, 14h)
 * 7. Set memory addressing mode (20h, 00h = horizontal)
 * 8. Set segment remap (A1h = column 127 = seg0)
 * 9. Set COM output scan direction (C8h = bottom to top)
 * 10. Set COM pins (DAh, 12h)
 * 11. Set contrast (81h, CFh)
 * 12. Set pre-charge period (D9h, F1h)
 * 13. Set VCOMH level (DBh, 40h)
 * 14. Display all on resume (A4h)
 * 15. Display on (AFh)
 */
static void oled_init(void)
{
    send_cmd(0xAE);  // Display off
    send_cmd(0xD5); send_cmd(0x80);  // Clock divider: fosc/1, Ffosc=100Hz
    send_cmd(0xA8); send_cmd(0x3F);  // Multiplex: 64 rows
    send_cmd(0xD3); send_cmd(0x00);  // Display offset: 0
    send_cmd(0x40);  // Start line: row 0
    send_cmd(0x8D); send_cmd(0x14);  // Charge pump: enable
    send_cmd(0x20); send_cmd(0x00);  // Mode: horizontal addressing
    send_cmd(0xA1);  // Seg remap: column 127 -> seg0
    send_cmd(0xC8);  // Scan dir: bottom to top
    send_cmd(0xDA); send_cmd(0x12);  // COM pins: alt config
    send_cmd(0x81); send_cmd(0xCF);  // Contrast: 207/255
    send_cmd(0xD9); send_cmd(0xF1);  // Pre-charge: 15 clocks
    send_cmd(0xDB); send_cmd(0x40);  // VCOMH: 0.77*Vcc
    send_cmd(0xA4);  // Display all on: follow RAM
    send_cmd(0xA6);  // Display mode: normal
    send_cmd(0xAF);  // Display on
}

/*
 * oled_clear() - Clear entire display (set all pixels to off)
 *
 * OLED is organized in pages:
 * - 8 pages (0-7), each page = 8 rows x 128 columns
 * - Total: 64 rows x 128 columns = 512 bytes
 */
static void oled_clear(void)
{
    int page, col;
    for (page = 0; page < 8; page++) {
        /* Set page address
         * 0xB0 + page = page start address command
         * e.g., page 0 = 0xB0, page 1 = 0xB1, etc. */
        send_cmd(0xB0 + page);

        /* Set column address to 0
         * Column addressing uses two commands:
         * - Lower 4 bits: 0x00-0x0F
         * - Upper 4 bits: 0x10-0x1F
         * Example: column 0 = 0x00 + 0x10 = 0x10 */
        send_cmd(0x00);   // Lower column address
        send_cmd(0x10);   // Upper column address

        /* Clear 128 columns by sending 0x00 (all pixels off) */
        for (col = 0; col < 128; col++)
            send_data(0x00);
    }
}

/*
 * oled_set_cursor() - Set cursor position for text/graphics
 * @page: Page number (0-7, each page = 8 rows)
 * @col: Column number (0-127)
 *
 * Commands used:
 * - 0xB0+page: Set page start address
 * - 0x00-0x0F: Set lower column address
 * - 0x10-0x1F: Set higher column address
 *
 * EXAMPLE: Set cursor to page 2, column 50
 * - send_cmd(0xB2) = page 2
 * - send_cmd(0x32) = column 50 = (50 >> 4) | 0x10 = 0x03 | 0x10 = 0x13? No!
 * Let's recalculate:
 * - col = 50
 * - col >> 4 = 50 / 16 = 3 = 0x03
 * - (col >> 4) | 0x10 = 0x03 | 0x10 = 0x13
 * - col & 0x0F = 50 % 16 = 2 = 0x02
 * So: send_cmd(0x13), send_cmd(0x02)
 */
static void oled_set_cursor(uint8_t page, uint8_t col)
{
    send_cmd(0xB0 + page);           // Page address (0xB0 to 0xB7)
    send_cmd((col >> 4) | 0x10);     // High nibble of column + 0x10
    send_cmd(col & 0x0F);            // Low nibble of column
}

/*
 * oled_print() - Print string on OLED display
 * @str: Null-terminated string to display
 *
 * Features:
 * - Converts lowercase to uppercase
 * - Ignores unsupported characters
 * - Adds 1-pixel spacing between characters
 */
static void oled_print(const char *str)
{
    char c;
    int i;

    /* Loop through each character until null terminator */
    while ((c = *str++)) {
        /* Convert lowercase to uppercase
         * ASCII 'a' = 97, 'A' = 65, difference = 32 */
        if (c >= 'a' && c <= 'z')
            c -= 32;

        /* Check if character is in font table (32-90)
         * Our font array only has characters from space (32) to Z (90) */
        if (c >= 32 && c <= 90) {
            /* Send 5 columns of pixel data for this character
             * Index = character ASCII - 32
             * e.g., 'A' = 65 - 32 = 33, but wait... 65-32=33, our array has 'A' at index 29
             * Let me recount:
             * font[0] = space = 32
             * font[1] = ! = 33
             * ...
             * font[29] = 'A' = 32 + 29 = 65 = 'A'
             * So index = c - 32 is correct! */
            for (i = 0; i < 5; i++)
                send_data(font[c - 32][i]);

            /* 1-pixel column spacing between characters
             * Send 0x00 (all pixels off) for spacing */
            send_data(0x00);
        }
    }
}


/* ==================== IOCTL HANDLER ==================== */

/*
 * oled_ioctl() - Handle user-space IOCTL commands
 * @file: File pointer (unused)
 * @cmd: IOCTL command
 * @arg: Argument pointer
 *
 * Supported commands:
 * - OLED_INIT: Initialize display
 * - OLED_CLEAR: Clear screen
 * - OLED_SET_CURSOR: Set cursor position
 * - OLED_PRINT: Print string
 */
static long oled_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct oled_pos pos;
    char *str;

    switch (cmd) {
    case OLED_INIT:
        /* Initialize the OLED display */
        oled_init();
        break;

    case OLED_CLEAR:
        /* Clear the entire display */
        oled_clear();
        break;

    case OLED_SET_CURSOR:
        /* Copy cursor position from user space
         * copy_from_user(dest, src, count) returns bytes not copied (0 = success) */
        if (copy_from_user(&pos, (void __user *)arg, sizeof(pos)))
            return -EFAULT;  /* Bad address error */
        oled_set_cursor(pos.page, pos.col);
        break;

    case OLED_PRINT:
        /* Allocate kernel memory for string (max 255 chars + null) */
        str = kmalloc(256, GFP_KERNEL);
        if (!str)
            return -ENOMEM;  /* Out of memory */

        /* Copy string from user space to kernel space */
        if (copy_from_user(str, (void __user *)arg, 256)) {
            kfree(str);
            return -EFAULT;  /* Bad address error */
        }

        /* Print string to OLED and free memory */
        oled_print(str);
        kfree(str);
        break;

    default:
        return -EINVAL;  /* Invalid IOCTL command */
    }

    return 0;  /* Success */
}


/* ==================== FILE OPERATIONS ==================== */

/* Called when user opens /dev/oleddevice */
static int oled_open(struct inode *inode, struct file *file)
{
    return 0;  /* Allow open - no restrictions */
}

/* Called when user closes /dev/oleddevice */
static int oled_release(struct inode *inode, struct file *file)
{
    return 0;  /* Allow close - no cleanup needed */
}

/* File operations structure - links system calls to our functions
 * This tells the kernel:
 * - .open = oled_open
 * - .release = oled_release
 * - .unlocked_ioctl = oled_ioctl
 */
static struct file_operations oled_fops = {
    .owner          = THIS_MODULE,
    .open           = oled_open,
    .release        = oled_release,
    .unlocked_ioctl = oled_ioctl,
};


/* ==================== I2C DRIVER FUNCTIONS ==================== */

/*
 * oled_probe() - Called when I2C device is matched and found
 * @client: I2C client structure (represents OLED on the I2C bus)
 *
 * This function is called automatically by the kernel when:
 * 1. Our I2C device is detected on the bus
 * 2. The device tree or ACPI matches our driver
 *
 * Steps:
 * 1. Save the client pointer (so we can use it later to send I2C commands)
 * 2. Register a character device (creates /dev/oleddevice)
 * 3. Create a device class (for udev to create device node)
 * 4. Create the actual device node
 */
static int oled_probe(struct i2c_client *client)
{
    pr_info("OLED probe started\n");
    oled_client = client;

    /* Step 1: Register character device with kernel
     * register_chrdev(major, name, fops)
     * - major = 0 means auto-allocate (kernel picks one)
     * - Returns the major number (or negative error)
     * - This makes our driver known to the kernel
     * - Name appears in /proc/devices */
    major = register_chrdev(0, DEVICE_NAME, &oled_fops);

    /* Step 2: Create device class
     * class_create(name)
     * - Creates entry in /sys/class/CLASS_NAME
     * - udev reads this to create /dev entries
     * - Example: creates /sys/class/oled_class */
    oled_class = class_create(CLASS_NAME);

    /* Step 3: Create device node /dev/oleddevice
     * device_create(class, parent, devt, drvdata, fmt, ...)
     * - class = oled_class created above
     * - parent = NULL (no parent device)
     * - devt = MKDEV(major, 0) = device number (major from register_chrdev, minor=0)
     * - drvdata = NULL (no driver data)
     * - fmt = "oleddevice" (name of device file)
     * - Result: creates /dev/oleddevice
     * - udev creates symlink based on class name */
    oled_device = device_create(oled_class, NULL, MKDEV(major, 0), NULL, "oleddevice");

    pr_info("OLED driver probed\n");
    return 0;
}

/*
 * oled_remove() - Called when device or driver is removed
 * @client: I2C client structure
 *
 * Cleanup function - reverses what probe() did
 * Order matters: reverse of creation order
 */
static void oled_remove(struct i2c_client *client)
{
    /* Remove /dev/oleddevice */
    device_destroy(oled_class, MKDEV(major, 0));

    /* Destroy /sys/class/oled_class */
    class_destroy(oled_class);

    /* Unregister character device
     * This removes /proc/devices entry and frees major number */
    unregister_chrdev(major, DEVICE_NAME);

    pr_info("OLED driver removed\n");
}


/* ==================== DEVICE ID TABLE ==================== */

/* This table tells the kernel which I2C devices this driver supports
 * When an I2C device with matching name is found, probe() is called
 * Format: { name, id }
 * - name = "oled" (must match device tree or board file)
 * - id = 0 (not used for I2C, always 0) */
static const struct i2c_device_id oled_id[] = {
    { "oled", 0 },
    { }
};

/* Export table to kernel (required for I2C device matching) */
MODULE_DEVICE_TABLE(i2c, oled_id);


/* ==================== I2C DRIVER STRUCTURE ==================== */

/* I2C driver structure - registers our driver with the I2C core
 * This structure is passed to i2c_add_driver() in init function */
static struct i2c_driver oled_driver = {
    .driver = {
        .name = "oled",           /* Driver name (appears in /sys/bus/i2c/drivers) */
        .owner = THIS_MODULE,     /* Prevents module from being unloaded while in use */
    },
    .probe  = oled_probe,         /* Called when device is found */
    .remove = oled_remove,        /* Called when device is removed */
    .id_table = oled_id,          /* Device IDs this driver supports */
};


/* ==================== I2C BOARD INFO ==================== */

/* Board-level device information
 * This tells the kernel about our I2C device before it's detected
 * - Used when device tree is not available
 * - I2C_BOARD_INFO(name, addr) creates the structure
 * - name = "oled" (must match id_table above)
 * - addr = 0x3C (I2C address of OLED) */
static struct i2c_board_info oled_i2c_info = {
    I2C_BOARD_INFO("oled", OLED_ADDR)
};


/* ==================== MODULE INIT/EXIT ==================== */

/*
 * oled_driver_init() - Module initialization function
 *
 * This is called when module is loaded with insmod or modprobe
 *
 * Steps:
 * 1. Get I2C adapter for the bus we want
 * 2. Register our I2C device on that bus
 * 3. Register our I2C driver
 */
static int __init oled_driver_init(void)
{
    struct i2c_adapter *adapter;

    pr_info("OLED driver init\n");

    /* Step 1: Get I2C adapter for specified bus
     * i2c_get_adapter(bus_number)
     * - Returns pointer to i2c_adapter or NULL if bus doesn't exist
     * - bus_number = 1 (I2C bus 1 on Raspberry Pi)
     * - This gives us access to the I2C bus hardware */
    adapter = i2c_get_adapter(I2C_BUS);

    /* Step 2: Register I2C device on the bus
     * i2c_new_client_device(adapter, info)
     * - adapter = the bus we got above
     * - info = device information (name and address)
     * - Creates a struct i2c_client representing our OLED
     * - Kernel will try to match this with a driver
     * - If matched, probe() is called */
    i2c_new_client_device(adapter, &oled_i2c_info);

    /* Release the adapter reference (we're done with it) */
    i2c_put_adapter(adapter);

    /* Step 3: Register our I2C driver
     * i2c_add_driver(driver)
     * - Adds driver to I2C core
     * - Kernel will match against all registered devices
     * - If device name matches, probe() is called */
    i2c_add_driver(&oled_driver);

    pr_info("OLED driver loaded\n");
    return 0;
}

/*
 * oled_driver_exit() - Module cleanup function
 *
 * This is called when module is unloaded with rmmod
 *
 * Steps:
 * 1. Unregister I2C device (if registered)
 * 2. Unregister I2C driver
 */
static void __exit oled_driver_exit(void)
{
    /* Unregister the I2C device
     * This removes the device and calls remove() if driver matched */
    i2c_unregister_device(i2c_driver_client_oled);

    /* Unregister the I2C driver
     * This removes driver from I2C core */
    i2c_del_driver(&oled_driver);

    pr_info("OLED driver unloaded\n");
}

/* Register module init/exit functions
 * module_init() = called when module loads
 * module_exit() = called when module unloads */
module_init(oled_driver_init);
module_exit(oled_driver_exit);

/* ==================== MODULE METADATA ==================== */

MODULE_LICENSE("GPL");                              /* License - required for kernel modules */
MODULE_AUTHOR("You");                               /* Author name */
MODULE_DESCRIPTION("SSD1306 OLED I2C Display Driver"); /* Short description */
MODULE_VERSION("1.0");                              /* Module version */

