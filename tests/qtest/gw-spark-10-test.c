/*
 * QTest coverage for the GW-Spark-10 PCI device.
 *
 * Copyright (c) 2024-2025
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/misc/gw-spark-10.h"
#include "libqtest.h"
#include "libqos/qgraph.h"
#include "libqos/pci.h"

typedef struct QGwSpark10 {
    QOSGraphObject obj;
    QPCIDevice pdev;
} QGwSpark10;

static void *gw_spark_10_get_driver(void *object, const char *interface)
{
    QGwSpark10 *device = object;

    if (!g_strcmp0(interface, "pci-device")) {
        return &device->pdev;
    }

    fprintf(stderr, "%s not present in gw-spark-10\n", interface);
    g_assert_not_reached();
}

static void *gw_spark_10_create(void *pci_bus, QGuestAllocator *alloc,
                                void *addr)
{
    QGwSpark10 *device = g_new0(QGwSpark10, 1);
    QPCIBus *bus = pci_bus;

    qpci_device_init(&device->pdev, bus, addr);
    device->obj.get_driver = gw_spark_10_get_driver;
    return &device->obj;
}

static void gw_spark_10_test_registers(void *object, void *data,
                                       QGuestAllocator *alloc)
{
    QGwSpark10 *device = object;
    QPCIDevice *pdev = &device->pdev;
    QPCIBar bar0;
    uint64_t vram_size;
    uint32_t value;

    qpci_device_enable(pdev);
    bar0 = qpci_iomap(pdev, 0, NULL);

    g_assert_cmphex(qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_DEV_ID), ==,
                    GW_SPARK_10_DEV_ID_VALUE);
    g_assert_cmphex(qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_DEV_VERSION),
                    ==, GW_SPARK_10_DEV_VERSION_VALUE);

    vram_size = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_VRAM_SIZE_LO);
    vram_size |= (uint64_t)qpci_io_readl(
        pdev, bar0, GW_SPARK_10_REG_VRAM_SIZE_HI) << 32;
    g_assert_cmpuint(vram_size, ==, GW_SPARK_10_DEFAULT_VRAM_SIZE);

    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_X, 7);
    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_X);
    g_assert_cmpuint(value, ==, 7);

    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GLOBAL_CTRL,
                   GW_SPARK_10_CTRL_ENABLE);
    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_GLOBAL_CTRL);
    g_assert_cmpuint(value, ==, GW_SPARK_10_CTRL_ENABLE);

    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GLOBAL_CTRL,
                   GW_SPARK_10_CTRL_RESET);
    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_GLOBAL_STATUS);
    g_assert_cmpuint(value, ==, GW_SPARK_10_STATUS_READY);
    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_X);
    g_assert_cmpuint(value, ==, 0);

    qpci_iounmap(pdev, bar0);
}

static void gw_spark_10_test_vram_and_dma(void *object, void *data,
                                          QGuestAllocator *alloc)
{
    QGwSpark10 *device = object;
    QPCIDevice *pdev = &device->pdev;
    QPCIBar bar0;
    QPCIBar bar2;
    uint32_t guest_value = 0x51a7c0de;
    uint32_t read_value;
    uint64_t guest_addr;

    qpci_device_enable(pdev);
    bar0 = qpci_iomap(pdev, 0, NULL);
    bar2 = qpci_iomap(pdev, 2, NULL);

    qpci_io_writel(pdev, bar2, 0x100, guest_value);
    g_assert_cmphex(qpci_io_readl(pdev, bar2, 0x100), ==, guest_value);

    guest_addr = guest_alloc(alloc, sizeof(guest_value));
    qtest_memwrite(pdev->bus->qts, guest_addr, &guest_value,
                   sizeof(guest_value));
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DMA_SRC_LO, guest_addr);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DMA_SRC_HI,
                   guest_addr >> 32);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DMA_DST_LO, 0x200);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DMA_DST_HI, 0);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DMA_SIZE,
                   sizeof(guest_value));
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DMA_CTRL,
                   GW_SPARK_10_DMA_START | GW_SPARK_10_DMA_DIR_TO_VRAM);

    read_value = qpci_io_readl(pdev, bar2, 0x200);
    g_assert_cmphex(read_value, ==, guest_value);
    g_assert_cmpuint(qpci_io_readl(pdev, bar0,
                                   GW_SPARK_10_REG_DMA_STATUS) &
                     GW_SPARK_10_DMA_COMPLETE, !=, 0);

    qpci_iounmap(pdev, bar0);
    qpci_iounmap(pdev, bar2);
}

static void gw_spark_10_test_kernel_execution(void *object, void *data,
                                              QGuestAllocator *alloc)
{
    static const uint32_t kernel[] = {
        0xF1402373,
        0x01F37313,
        0x00231393,
        0x00001E37,
        0x007E0E33,
        0x006E2023,
        0x00100073,
    };
    QGwSpark10 *device = object;
    QPCIDevice *pdev = &device->pdev;
    QPCIBar bar0;
    QPCIBar bar2;
    uint32_t thread_count = 8;
    uint32_t value;

    qpci_device_enable(pdev);
    bar0 = qpci_iomap(pdev, 0, NULL);
    bar2 = qpci_iomap(pdev, 2, NULL);

    for (size_t i = 0; i < ARRAY_SIZE(kernel); i++) {
        qpci_io_writel(pdev, bar2, i * sizeof(kernel[0]), kernel[i]);
    }
    for (uint32_t i = 0; i < thread_count; i++) {
        qpci_io_writel(pdev, bar2, 0x1000 + i * sizeof(uint32_t),
                       UINT32_MAX);
    }

    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GLOBAL_CTRL,
                   GW_SPARK_10_CTRL_ENABLE);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_IRQ_ENABLE,
                   GW_SPARK_10_IRQ_KERNEL_DONE);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_KERNEL_ADDR_LO, 0);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_KERNEL_ADDR_HI, 0);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_X, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_Y, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_Z, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_BLOCK_DIM_X, thread_count);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_BLOCK_DIM_Y, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_BLOCK_DIM_Z, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DISPATCH, 1);

    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_GLOBAL_STATUS);
    g_assert_cmpuint(value & GW_SPARK_10_STATUS_READY, !=, 0);
    g_assert_cmpuint(value & GW_SPARK_10_STATUS_ERROR, ==, 0);
    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_IRQ_STATUS);
    g_assert_cmpuint(value & GW_SPARK_10_IRQ_KERNEL_DONE, !=, 0);

    for (uint32_t i = 0; i < thread_count; i++) {
        value = qpci_io_readl(pdev, bar2, 0x1000 + i * sizeof(uint32_t));
        g_assert_cmpuint(value, ==, i);
    }

    qpci_iounmap(pdev, bar0);
    qpci_iounmap(pdev, bar2);
}

static void gw_spark_10_test_invalid_kernel(void *object, void *data,
                                            QGuestAllocator *alloc)
{
    QGwSpark10 *device = object;
    QPCIDevice *pdev = &device->pdev;
    QPCIBar bar0;
    QPCIBar bar2;
    uint32_t value;

    qpci_device_enable(pdev);
    bar0 = qpci_iomap(pdev, 0, NULL);
    bar2 = qpci_iomap(pdev, 2, NULL);

    qpci_io_writel(pdev, bar2, 0, UINT32_MAX);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GLOBAL_CTRL,
                   GW_SPARK_10_CTRL_ENABLE);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_KERNEL_ADDR_LO, 0);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_KERNEL_ADDR_HI, 0);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_X, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_Y, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_GRID_DIM_Z, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_BLOCK_DIM_X, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_BLOCK_DIM_Y, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_BLOCK_DIM_Z, 1);
    qpci_io_writel(pdev, bar0, GW_SPARK_10_REG_DISPATCH, 1);

    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_GLOBAL_STATUS);
    g_assert_cmpuint(value & GW_SPARK_10_STATUS_READY, !=, 0);
    g_assert_cmpuint(value & GW_SPARK_10_STATUS_ERROR, !=, 0);
    value = qpci_io_readl(pdev, bar0, GW_SPARK_10_REG_ERROR_STATUS);
    g_assert_cmpuint(value & GW_SPARK_10_ERR_KERNEL_FAULT, !=, 0);

    qpci_iounmap(pdev, bar0);
    qpci_iounmap(pdev, bar2);
}

static void gw_spark_10_register_nodes(void)
{
    QOSGraphEdgeOptions options = {
        .extra_device_opts = "addr=04.0",
    };

    add_qpci_address(&options, &(QPCIAddress) {
        .devfn = QPCI_DEVFN(4, 0),
        .vendor_id = GW_SPARK_10_VENDOR_ID,
        .device_id = GW_SPARK_10_DEVICE_ID,
    });

    qos_node_create_driver("gw-spark-10", gw_spark_10_create);
    qos_node_consumes("gw-spark-10", "pci-bus", &options);
    qos_node_produces("gw-spark-10", "pci-device");

    qos_add_test("registers", "gw-spark-10",
                 gw_spark_10_test_registers, NULL);
    qos_add_test("vram-and-dma", "gw-spark-10",
                 gw_spark_10_test_vram_and_dma, NULL);
    qos_add_test("kernel-execution", "gw-spark-10",
                 gw_spark_10_test_kernel_execution, NULL);
    qos_add_test("invalid-kernel", "gw-spark-10",
                 gw_spark_10_test_invalid_kernel, NULL);
}

libqos_init(gw_spark_10_register_nodes);
