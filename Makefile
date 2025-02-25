# Build the kernel module
obj-m += uartlite_xdma.o

# Kernel build directory (default: current running kernel)
KDIR ?= /lib/modules/$(shell uname -r)/build

# Build the module
all:
	make CC=/usr/bin/gcc-13 -C $(KDIR) M=$(PWD) modules

# Clean up compiled files
clean:
	make -C $(KDIR) M=$(PWD) clean

# Install the module into the system
install:
	make -C $(KDIR) M=$(PWD) modules_install

# Enable debugging symbols (-g flag for debugging)
EXTRA_CFLAGS += -g

# Define targets that are not actual files
.PHONY: all clean install

