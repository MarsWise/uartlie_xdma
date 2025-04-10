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

---

```sh
make
```

---

### 2. Load the Module

Once built, load the kernel module using:

---

```sh
sudo insmod uartlite_xdma.ko
```

---

To check if the module is loaded:

---

```sh
lsmod | grep uartlite_xdma
```

---

### 3. Verify Device Registration

After loading the module, check if the device is available:

---

```sh
dmesg | grep UARTlite
ls /dev/ttyUL*
```

---

If the device exists, the driver is properly loaded.

## 🖥 Testing with Minicom

Minicom is a terminal program that allows communication with the UART device.

### 1. Install Minicom (if not installed)

---

```sh
sudo apt-get install minicom
```

---

### 2. Run Minicom

---

```sh
sudo minicom -D /dev/ttyUL0 
```

---

-D /dev/ttyUL0 specifies the device

### 3. Sending and Receiving Data

Type messages in the terminal, and they will be sent over UART.
Any received data will be displayed in the terminal.

### 4. Exit Minicom

Press CTRL + A, then X, and confirm to exit.



## 🐍 Alternative: Access UARTLite via XDMA and Python

In addition to the kernel driver, UARTLite can be accessed directly through PCIe XDMA using a lightweight **Python API** without the need for a custom kernel module.

This is useful for:

- Prototyping

- Debugging

- Bypassing the kernel for user-space experiments

### Python Script: `uaxdma.py`

**Features:**

- Direct memory access to UARTLite registers over XDMA

- Supports byte-by-byte send and receive

- Supports **asyncio** for low CPU usage during data reception

- Works with `/dev/xdma0_user` device

**Key Operations:**

- **Reset FIFO** at startup

- **Polling** UARTLite status register for RX/TX readiness

- **Sending data** to TX FIFO

- **Receiving data** from RX FIFO

- **Async methods** for efficient non-blocking reads



### Example Usage:

Simple receive loop (synchronous):

```python
if __name__ == "__main__":
    uart = XdmaUartLite(device_index=0)

    print("Waiting for UARTLite data...")
    while True:
        data = uart.recv_data(128)  # Read 128 bytes
        if data:
            print(data.decode(errors="ignore").strip())
```

Or **async** version for optimal CPU usage:

```python
async def main():
    uart = XdmaUartLite()
    while True:
        data = await uart.recv_data_async(128)
        print(data.decode(errors="ignore"))

asyncio.run(main())
```

## ❌ Uninstalling the Module

To remove the module:
---

```sh
sudo rmmod uartlite_xdma
```

---

To clean up compiled files:
---

```sh
make clean
```

---

# 👤 Author

Konstantin

This README provides clear instructions for building, testing, and using the driver with Minicom. Let me know if you need any modifications! 🚀
