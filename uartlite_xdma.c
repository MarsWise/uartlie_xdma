/*
 * UARTlite TTY Driver over XDMA
 *
 * Author: Konstantin
 * Date: 25.02.2025
 *
 * This driver enables communication with AXI UART Lite over PCIe XDMA.
 * It implements a TTY interface (ttyULx) for user-space interaction and supports
 * RX polling using a work queue mechanism.
 *
 * License: GPL v2
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h> // tty_insert_flip_string и tty_flip_buffer_push
#include <linux/io.h>
#include <linux/workqueue.h> // work_struct

// External information
#define DRIVER_NAME "uartlite_xdma" // Driver name
#define VENDOR_ID 0x10EE  // Xilinx Vendor ID
#define DEVICE_ID 0x7011  // Device ID for 7-Series FPGA Hard PCIe block
#define UARTLITE_BASE_OFFSET 0x40000 // AXI base address

// AXI UART Lite Register Offsets
#define UARTLITE_RX_FIFO  0x00  // Receive FIFO
#define UARTLITE_TX_FIFO  0x04  // Transmit FIFO
#define UARTLITE_STATUS   0x08  // Status register
#define UARTLITE_CONTROL  0x0C  // Control register

// Status Register Flags
#define STATUS_RXVALID    BIT(0) // Data available in RX FIFO
#define STATUS_TXFULL     BIT(3) // TX FIFO is full



struct uartlite_priv {
    void __iomem *base;
    struct tty_port port;
    struct work_struct rx_work; // Polling
    bool running;
};

static struct tty_driver *uartlite_tty_driver;

/* UART Lite Functions */
static int uartlite_tx_ready(struct uartlite_priv *priv)
{
    return !(ioread32(priv->base + UARTLITE_STATUS) & STATUS_TXFULL);
}

static void uartlite_write_byte(struct uartlite_priv *priv, u8 val)
{
    iowrite32(val, priv->base + UARTLITE_TX_FIFO);
}

static int uartlite_rx_ready(struct uartlite_priv *priv)
{
    return ioread32(priv->base + UARTLITE_STATUS) & STATUS_RXVALID;
}

static u8 uartlite_read_byte(struct uartlite_priv *priv)
{
    return ioread32(priv->base + UARTLITE_RX_FIFO);
}

/* Work function for polling RX FIFO */
static void uartlite_rx_work(struct work_struct *work)
{
    struct uartlite_priv *priv = container_of(work, struct uartlite_priv, rx_work);
    struct tty_struct *tty = tty_port_tty_get(&priv->port);
    unsigned char buf[16];
    int i, count;

    if (!tty)
        return;

    while (priv->running && uartlite_rx_ready(priv)) {
        count = 0;
        for (i = 0; i < sizeof(buf) && uartlite_rx_ready(priv); i++) {
            buf[i] = uartlite_read_byte(priv);
            count++;
        }
        if (count) {
            tty_insert_flip_string(&priv->port, buf, count);
            tty_flip_buffer_push(&priv->port);
        }
    }

    if (priv->running)
        schedule_work(&priv->rx_work);

    tty_kref_put(tty);
}

/* TTY Operations */
static int uartlite_tty_open(struct tty_struct *tty, struct file *filp)
{
    struct uartlite_priv *priv = container_of(tty->port, struct uartlite_priv, port);
    priv->running = true;
    schedule_work(&priv->rx_work);
    return tty_port_open(tty->port, tty, filp);
}

static void uartlite_tty_close(struct tty_struct *tty, struct file *filp)
{
    struct uartlite_priv *priv = container_of(tty->port, struct uartlite_priv, port);
    priv->running = false;
    cancel_work_sync(&priv->rx_work);
    tty_port_close(tty->port, tty, filp);
}

static ssize_t uartlite_tty_write(struct tty_struct *tty, const u8 *buf, size_t count)
{
    struct uartlite_priv *priv = tty->driver_data;
    size_t i;

    for (i = 0; i < count; i++) {
        while (!uartlite_tx_ready(priv))
            cpu_relax();
        uartlite_write_byte(priv, buf[i]);
    }
    return i;
}

static unsigned int uartlite_tty_write_room(struct tty_struct *tty)
{
    struct uartlite_priv *priv = tty->driver_data;
    return uartlite_tx_ready(priv) ? 16 : 0;
}

static unsigned int uartlite_tty_chars_in_buffer(struct tty_struct *tty)
{
    return 0;
}

static const struct tty_operations uartlite_tty_ops = {
    .open = uartlite_tty_open,
    .close = uartlite_tty_close,
    .write = uartlite_tty_write,
    .write_room = uartlite_tty_write_room,
    .chars_in_buffer = uartlite_tty_chars_in_buffer,
};

/* TTY Port Initialization */
static int uartlite_port_activate(struct tty_port *port, struct tty_struct *tty)
{
    struct uartlite_priv *priv = container_of(port, struct uartlite_priv, port);
    tty->driver_data = priv;
    return 0;
}

static void uartlite_port_shutdown(struct tty_port *port)
{
    struct uartlite_priv *priv = container_of(port, struct uartlite_priv, port);
    priv->running = false;
    cancel_work_sync(&priv->rx_work);
}

static const struct tty_port_operations uartlite_port_ops = {
    .activate = uartlite_port_activate,
    .shutdown = uartlite_port_shutdown,
};

/* PCI Probe Function */
static int uartlite_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    struct uartlite_priv *priv;
    int ret;

    priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    ret = pcim_enable_device(pdev);
    if (ret)
        return ret;

    priv->base = pcim_iomap(pdev, 0, 0);
    if (!priv->base)
        return -ENOMEM;

    priv->base += UARTLITE_BASE_OFFSET;

    tty_port_init(&priv->port);
    priv->port.ops = &uartlite_port_ops;
    INIT_WORK(&priv->rx_work, uartlite_rx_work);
    priv->running = false;

    tty_port_register_device(&priv->port, uartlite_tty_driver, 0, &pdev->dev);

    pci_set_drvdata(pdev, priv);
    dev_info(&pdev->dev, "UARTlite over XDMA registered as TTY");
    return 0;
}

static void uartlite_remove(struct pci_dev *pdev)
{
    struct uartlite_priv *priv = pci_get_drvdata(pdev);
    tty_unregister_device(uartlite_tty_driver, 0);
    tty_port_destroy(&priv->port);
}
 
static const struct pci_device_id uartlite_pci_tbl[] = {
    { PCI_DEVICE(VENDOR_ID, DEVICE_ID) },
    { 0, }
};

static struct pci_driver uartlite_pci_driver = {


    .name = DRIVER_NAME,
    .id_table = uartlite_pci_tbl,
    .probe = uartlite_probe,
    .remove = uartlite_remove,
};

/* Module Initialization */
static int __init uartlite_init(void)
{
    int ret;

    uartlite_tty_driver = tty_alloc_driver(1, TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
    if (IS_ERR(uartlite_tty_driver))
        return PTR_ERR(uartlite_tty_driver);
        
    uartlite_tty_driver->driver_name = DRIVER_NAME;
    uartlite_tty_driver->name = "ttyUL";
    uartlite_tty_driver->major = 0;
    uartlite_tty_driver->minor_start = 0;
    uartlite_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
    uartlite_tty_driver->subtype = SERIAL_TYPE_NORMAL;
    uartlite_tty_driver->init_termios = tty_std_termios;
    uartlite_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;

    tty_set_operations(uartlite_tty_driver, &uartlite_tty_ops);

    ret = tty_register_driver(uartlite_tty_driver);
    if (ret) {
        tty_driver_kref_put(uartlite_tty_driver);
        return ret;
    }

    ret = pci_register_driver(&uartlite_pci_driver);
    if (ret) {
        tty_unregister_driver(uartlite_tty_driver);
        tty_driver_kref_put(uartlite_tty_driver);
    }

    return ret;
}

static void __exit uartlite_exit(void)
{
    pci_unregister_driver(&uartlite_pci_driver);
    tty_unregister_driver(uartlite_tty_driver);
    tty_driver_kref_put(uartlite_tty_driver);
}

module_init(uartlite_init);
module_exit(uartlite_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Konstantin");
MODULE_DESCRIPTION("UARTlite TTY driver over XDMA with RX support");