# Raspberry Pi MPU6050 OLED Display Project

A complete embedded Linux project demonstrating **Linux device driver development** with **I2C communication** for the **MPU6050 motion sensor** and **OLED display integration** on **Raspberry Pi 3B**.

## 📋 Table of Contents
- [Project Overview](#project-overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Project Structure](#project-structure)
- [Driver Architecture](#driver-architecture)
- [Installation & Build](#installation--build)
- [Running the Application](#running-the-application)
- [Features](#features)
- [Technical Details](#technical-details)
- [API Reference](#api-reference)
- [Contributing](#contributing)
- [License](#license)

---

## 🎯 Project Overview

This project implements:
1. **MPU6050 Kernel Driver** - Linux character device driver for 6-axis motion sensor
2. **OLED Display Driver** - Linux character device driver for 128x64 OLED display
3. **Combined Application** - User-space application that reads sensor data and displays it on OLED

### Key Technologies
| Technology | Description |
|------------|-------------|
| **Linux Kernel Module** | Character device drivers with ioctl interface |
| **I2C Protocol** | Inter-Integrated Circuit communication |
| **Raspberry Pi 3B** | Target hardware platform |
| **MPU6050** | 3-axis accelerometer + 3-axis gyroscope |
| **OLED Display** | 128x64 pixel I2C display |

---

## 🔧 Hardware Requirements

- **Raspberry Pi 3B** (or compatible)
- **MPU6050 Module** (3-axis gyroscope + accelerometer)
- **OLED Display Module** (128x64, I2C, SSD1306 compatible)
- **Jumper wires** for connections
- **Breadboard** (optional)

### Pin Connections

```
Raspberry Pi 3B          MPU6050 OLED
───────────────          ──────── ─────
GPIO 2 (SDA)   --------> SDA  <---> SDA
GPIO 3 (SCL)   --------> SCL  <---> SCL
3.3V           --------> VCC  <---> VCC
GND            --------> GND  <---> GND
```

> **Note:** MPU6050 typically uses address `0x68` and OLED uses `0x3C`

---

## 💻 Software Requirements

- **Linux Kernel Headers** (for cross-compilation)
- **Raspberry Pi OS** (or compatible distro)
- **GCC Compiler**
- **Make**

### Cross-Compilation Setup
```bash
# Install cross-compilation tools (on host machine)
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

# Install kernel headers on Raspberry Pi
sudo apt-get install raspberrypi-kernel-headers
```
---

## 🏗️ Driver Architecture

### MPU6050 Driver (`mpu6050_driver.c`)

```
┌─────────────────────────────────────────┐
│         MPU6050 Kernel Driver           │
├─────────────────────────────────────────┤
│  Character Device (/dev/mpu6050)        │
│                                         │
│  Functions:                             │
│  ├── gyro_probe()      - Device init    │
│  ├── etx_ioctl()       - Data retrieval │
│  ├── data_read()       - I2C read       │
│  └── sensor_thread_fn()- Continuous read│
└─────────────────────────────────────────┘
```

**Key Features:**
- Creates character device `/dev/mpu6050`
- I2C communication with MPU6050 (address 0x68)
- Reads: Accelerometer (X, Y, Z), Gyroscope (X, Y, Z), Temperature
- Continuous sensor reading via kernel thread (100ms interval)
- Supports self-test calibration

### OLED Driver (`oled_driver.c`)

```
┌─────────────────────────────────────────┐
│          OLED Kernel Driver             │
├─────────────────────────────────────────┤
│  Character Device (/dev/oleddevice)     │
│                                         │
│  IOCTL Commands:                        │
│  ├── OLED_INIT          - Initialize    │
│  ├── OLED_CLEAR         - Clear display │
│  ├── OLED_SET_CURSOR    - Position      │
│  └── OLED_PRINT         - Display text  │
└─────────────────────────────────────────┘
```

**Key Features:**
- Creates character device `/dev/oleddevice`
- I2C communication with OLED (address 0x3C)
- 128x64 pixel display support
- Built-in 5x7 ASCII font (32-90 characters)
- Page-based display architecture

---

## 🛠️ Installation & Build

### Step 1: Clone and Navigate
```bash
cd /path/to/ALL_codes
```

### Step 2: Build the Drivers and Application

#### For Native Compilation (on Raspberry Pi)
```bash
make
```

#### For Cross-Compilation
```bash
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
```

### Step 3: Load the Drivers

```bash
# Load MPU6050 driver
sudo insmod mpu6050_driver.ko

# Load OLED driver
sudo insmod oled_driver.ko

# Verify device creation
ls -l /dev/mpu6050 /dev/oleddevice
```

### Step 4: Check Driver Messages
```bash
dmesg | tail
```

Expected output should show:
```
Driver Added
probe function invoked
OLED driver probed
MPU6050 detected, WHO_AM_I: 0x68
```

---

## ▶️ Running the Application

```bash
# Compile the application (if not already done)
gcc -o combined_app combined_app.c

# Run the application
sudo ./combined_app
```

### Expected Output on OLED
```
┌────────────────────┐
│ MPU6050 DATA       │
│ AX:1234   GX:567   │
│ AY:2345   GY:678   │
│ AZ:3456   GZ:789   │
│        T:25        │
└────────────────────┘
```

> The display updates every second showing real-time sensor data

---

## ✨ Features

### MPU6050 Driver Features
| Feature | Description |
|---------|-------------|
| ✅ Accelerometer | 3-axis acceleration (±2g default) |
| ✅ Gyroscope | 3-axis angular velocity (±250°/s default) |
| ✅ Temperature | On-chip temperature sensor |
| ✅ Self-Test | Factory calibration support |
| ✅ WHO_AM_I | Device identification (0x68) |
| ✅ Continuous Reading | Kernel thread @ 100ms interval |

### OLED Driver Features
| Feature | Description |
|---------|-------------|
| ✅ Text Display | ASCII characters (5x7 font) |
| ✅ Page Control | 8 pages × 128 columns |
| ✅ Cursor Control | Position text anywhere |
| ✅ Display Control | Init, clear, update |
| ✅ Built-in Font | 59 ASCII characters |

### Application Features
| Feature | Description |
|---------|-------------|
| ✅ Real-time Display | Updates every second |
| ✅ Split Layout | Left: Accel, Right: Gyro/Temp |
| ✅ Error Handling | Graceful failure handling |
| ✅ Resource Management | Proper file descriptor cleanup |

---

## 🔍 Technical Details

### MPU6050 Register Map
```
Address  │ Register         │ Description
─────────┼──────────────────┼────────────────────────
0x3B-3C  │ ACCEL_XOUT_H/L   │ Accelerometer X axis
0x3D-3E  │ ACCEL_YOUT_H/L   │ Accelerometer Y axis
0x3F-40  │ ACCEL_ZOUT_H/L   │ Accelerometer Z axis
0x41-42  │ TEMP_OUT_H/L     │ Temperature
0x43-44  │ GYRO_XOUT_H/L    │ Gyroscope X axis
0x45-46  │ GYRO_YOUT_H/L    │ Gyroscope Y axis
0x47-48  │ GYRO_ZOUT_H/L    │ Gyroscope Z axis
0x68     │ WHO_AM_I         │ Device ID (should be 0x68)
0x75     │ (same as above)  │ Alternative WHO_AM_I
```

### IOCTL Interface

#### MPU6050 IOCTL
```c
#define MPU_IOC_MAGIC 'm'
#define MPU_GET_DATA _IOR(MPU_IOC_MAGIC, 1, struct sensor_data)

struct sensor_data {
    int16_t adx, ady, adz;      // Accelerometer
    int16_t gdx, gdy, gdz;      // Gyroscope
    int16_t td;                 // Temperature
    uint8_t self_test_x, y, z, a; // Self-test values
};
```

#### OLED IOCTL
```c
#define OLED_IOC_MAGIC 'o'
#define OLED_INIT            _IO(OLED_IOC_MAGIC, 1)
#define OLED_CLEAR           _IO(OLED_IOC_MAGIC, 2)
#define OLED_SET_CURSOR      _IOW(OLED_IOC_MAGIC, 3, struct oled_cursor)
#define OLED_PRINT           _IOW(OLED_IOC_MAGIC, 4, char *)

struct oled_cursor {
    uint8_t page;   // 0-7
    uint8_t col;    // 0-127
};
```

---

## 📚 API Reference

### User-Space Application Functions

#### Opening Devices
```c
int oled_fd = open("/dev/oleddevice", O_RDWR);
int mpu_fd = open("/dev/mpu6050", O_RDWR);
```

#### Reading Sensor Data
```c
struct sensor_data s;
ioctl(mpu_fd, MPU_GET_DATA, &s);
// s.adx, s.ady, s.adz - accelerometer values
// s.gdx, s.gdy, s.gdz - gyroscope values
// s.td - temperature
```

#### Displaying Text
```c
// Set cursor position (page 0-7, column 0-127)
struct oled_cursor cursor = {0, 0};
ioctl(oled_fd, OLED_SET_CURSOR, &cursor);

// Print text
ioctl(oled_fd, OLED_PRINT, "Hello");
```

#### Control Operations
```c
ioctl(oled_fd, OLED_INIT);    // Initialize display
ioctl(oled_fd, OLED_CLEAR);   // Clear display
```

---

## 🧹 Unloading Drivers

```bash
# Remove application (if running)
# Press Ctrl+C

# Unload drivers
sudo rmmod oled_driver
sudo rmmod mpu6050_driver

# Check kernel messages
dmesg | tail
```

---

## 🚀 Future Improvements

- [ ] Add interrupt-driven data collection
- [ ] Support multiple MPU6050 devices
- [ ] Implement device tree overlay
- [ ] Add error recovery mechanisms
- [ ] Support for more display features (graphics, images)
- [ ] Add sysfs interface for debugging
- [ ] Implement power management
- [ ] Support for other display sizes

---

## 📝 Notes

1. **I2C Bus**: This project uses I2C bus 1 (`/dev/i2c-1`)
2. **Permissions**: You may need `sudo` to access `/dev` entries
3. **Kernel Version**: Tested with Raspberry Pi kernel 5.x
4. **Device Addresses**: Verify actual I2C addresses with `i2cdetect -y 1`

---

## 📄 License

This project is open source and available under the GPL License.

---

## 👥 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## 📞 Support

For questions or issues, please open an issue in the repository.

---

**Built with ❤️ for the embedded systems community**

