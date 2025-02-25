


# UARTLite XDMA Linux Driver

This repository contains a Linux kernel driver for AXI UART Lite over PCIe XDMA. The driver provides a TTY interface for seamless serial communication between the host and FPGA-based designs.

## Features
- Supports AXI UART Lite via PCIe XDMA
- Provides a TTY interface (`/dev/ttyUL0`)
- Supports RX polling for receiving data
- Compatible with Minicom and other serial tools

## Build and Install

### 1. Build the Kernel Module
Ensure you have the necessary kernel headers installed, then run:

```sh
make
```

### 2. Load the Module
Once built, load the kernel module using:

```sh
sudo insmod uartlite_xdma.ko
```

To check if the module is loaded:

```sh
lsmod | grep uartlite_xdma
```

### 3. Verify Device Registration
After loading the module, check if the device is available:

```sh
dmesg | grep UARTlite
ls /dev/ttyUL*
```

If the device exists, the driver is properly loaded.

## 🖥 Testing with Minicom
Minicom is a terminal program that allows communication with the UART device.

### 1. Install Minicom (if not installed)
```sh
sudo apt-get install minicom
```
### 2. Run Minicom
```sh
sudo minicom -D /dev/ttyUL0 
```
-D /dev/ttyUL0 specifies the device
### 3. Sending and Receiving Data
Type messages in the terminal, and they will be sent over UART.
Any received data will be displayed in the terminal.
### 4. Exit Minicom
Press CTRL + A, then X, and confirm to exit.

## ❌ Uninstalling the Module
To remove the module:
```sh
sudo rmmod uartlite_xdma
```

To clean up compiled files:
```sh
make clean
```

# 📜 License
This project is licensed under the GPL v2 License.

# 👤 Author
Konstantin

This README provides clear instructions for building, testing, and using the driver with Minicom. Let me know if you need any modifications! 🚀







