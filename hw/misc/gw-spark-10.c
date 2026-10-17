/*
 * QEMU GW-Spark-10 Educational GPGPU Device
 *
 * Copyright (c) 2024-2025
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/core/qdev-properties.h"
#include "migration/blocker.h"

#include "hw/misc/gw-spark-10.h"
#include "gw-spark-10-core.h"

static uint64_t gw_spark_10_load_le(const uint8_t *ptr, unsigned size)
{
    uint64_t val = 0;

    for (unsigned i = 0; i < size; i++) {
        val |= (uint64_t)ptr[i] << (i * 8);
    }

    return val;
}

static void gw_spark_10_store_le(uint8_t *ptr, uint64_t val, unsigned size)
{
    for (unsigned i = 0; i < size; i++) {
        ptr[i] = extract64(val, i * 8, 8);
    }
}

static bool gw_spark_10_vram_range_valid(GwSpark10State *s, hwaddr addr,
                                         unsigned size)
{
    return size <= 8 && addr <= s->vram_size && size <= s->vram_size - addr;
}

static void gw_spark_10_set_error(GwSpark10State *s, uint32_t err)
{
    s->error_status |= err;
    s->global_status &= ~GW_SPARK_10_STATUS_BUSY;
    s->global_status |= GW_SPARK_10_STATUS_READY | GW_SPARK_10_STATUS_ERROR;
    s->irq_status |= GW_SPARK_10_IRQ_ERROR;
}

static void gw_spark_10_finish_dma(GwSpark10State *s)
{
    s->dma.status &= ~GW_SPARK_10_DMA_BUSY;
    s->dma.status |= GW_SPARK_10_DMA_COMPLETE;
    s->dma.ctrl &= ~GW_SPARK_10_DMA_START;
    s->irq_status |= GW_SPARK_10_IRQ_DMA_DONE;
}

static void gw_spark_10_start_dma(GwSpark10State *s)
{
    MemTxResult ret = MEMTX_OK;

    s->dma.status = GW_SPARK_10_DMA_BUSY;

    if (s->dma.size == 0) {
        gw_spark_10_finish_dma(s);
        return;
    }

    if (s->dma.ctrl & GW_SPARK_10_DMA_DIR_FROM_VRAM) {
        if (s->dma.src_addr > s->vram_size ||
            s->dma.size > s->vram_size - s->dma.src_addr) {
            s->dma.status = GW_SPARK_10_DMA_ERROR;
            gw_spark_10_set_error(s, GW_SPARK_10_ERR_DMA_FAULT);
            return;
        }

        ret = pci_dma_write(&s->parent_obj, s->dma.dst_addr,
                            s->vram_ptr + s->dma.src_addr, s->dma.size);
    } else {
        if (s->dma.dst_addr > s->vram_size ||
            s->dma.size > s->vram_size - s->dma.dst_addr) {
            s->dma.status = GW_SPARK_10_DMA_ERROR;
            gw_spark_10_set_error(s, GW_SPARK_10_ERR_DMA_FAULT);
            return;
        }

        ret = pci_dma_read(&s->parent_obj, s->dma.src_addr,
                           s->vram_ptr + s->dma.dst_addr, s->dma.size);
    }

    if (ret != MEMTX_OK) {
        s->dma.status = GW_SPARK_10_DMA_ERROR;
        gw_spark_10_set_error(s, GW_SPARK_10_ERR_DMA_FAULT);
        return;
    }

    gw_spark_10_finish_dma(s);
}

static void gw_spark_10_finish_kernel(GwSpark10State *s, int ret)
{
    s->global_status &= ~GW_SPARK_10_STATUS_BUSY;
    s->global_status |= GW_SPARK_10_STATUS_READY;

    if (ret) {
        gw_spark_10_set_error(s, GW_SPARK_10_ERR_KERNEL_FAULT);
        return;
    }

    s->irq_status |= GW_SPARK_10_IRQ_KERNEL_DONE;
}

static void gw_spark_10_start_kernel(GwSpark10State *s)
{
    int ret;

    if (!(s->global_ctrl & GW_SPARK_10_CTRL_ENABLE)) {
        gw_spark_10_set_error(s, GW_SPARK_10_ERR_INVALID_CMD);
        return;
    }

    s->global_status &= ~(GW_SPARK_10_STATUS_READY |
                          GW_SPARK_10_STATUS_ERROR);
    s->global_status |= GW_SPARK_10_STATUS_BUSY;

    ret = gw_spark_10_core_execute_kernel(s);
    gw_spark_10_finish_kernel(s, ret);
}

static void gw_spark_10_reset_state(GwSpark10State *s, bool clear_vram)
{
    s->global_ctrl = 0;
    s->global_status = GW_SPARK_10_STATUS_READY;
    s->error_status = 0;
    s->irq_enable = 0;
    s->irq_status = 0;
    memset(&s->kernel, 0, sizeof(s->kernel));
    memset(&s->dma, 0, sizeof(s->dma));
    memset(&s->simt, 0, sizeof(s->simt));

    if (clear_vram && s->vram_ptr) {
        memset(s->vram_ptr, 0, s->vram_size);
    }
}

static uint64_t gw_spark_10_ctrl_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    GwSpark10State *s = opaque;

    if (size != 4) {
        return 0;
    }

    switch (addr) {
    case GW_SPARK_10_REG_DEV_ID:
        return GW_SPARK_10_DEV_ID_VALUE;
    case GW_SPARK_10_REG_DEV_VERSION:
        return GW_SPARK_10_DEV_VERSION_VALUE;
    case GW_SPARK_10_REG_DEV_CAPS:
        return (s->num_cus & 0xff) |
               ((s->warps_per_cu & 0xff) << 8) |
               ((s->warp_size & 0xff) << 16);
    case GW_SPARK_10_REG_VRAM_SIZE_LO:
        return extract64(s->vram_size, 0, 32);
    case GW_SPARK_10_REG_VRAM_SIZE_HI:
        return extract64(s->vram_size, 32, 32);
    case GW_SPARK_10_REG_GLOBAL_CTRL:
        return s->global_ctrl;
    case GW_SPARK_10_REG_GLOBAL_STATUS:
        return s->global_status;
    case GW_SPARK_10_REG_ERROR_STATUS:
        return s->error_status;
    case GW_SPARK_10_REG_IRQ_ENABLE:
        return s->irq_enable;
    case GW_SPARK_10_REG_IRQ_STATUS:
        return s->irq_status;
    case GW_SPARK_10_REG_KERNEL_ADDR_LO:
        return extract64(s->kernel.kernel_addr, 0, 32);
    case GW_SPARK_10_REG_KERNEL_ADDR_HI:
        return extract64(s->kernel.kernel_addr, 32, 32);
    case GW_SPARK_10_REG_KERNEL_ARGS_LO:
        return extract64(s->kernel.kernel_args, 0, 32);
    case GW_SPARK_10_REG_KERNEL_ARGS_HI:
        return extract64(s->kernel.kernel_args, 32, 32);
    case GW_SPARK_10_REG_GRID_DIM_X:
        return s->kernel.grid_dim[0];
    case GW_SPARK_10_REG_GRID_DIM_Y:
        return s->kernel.grid_dim[1];
    case GW_SPARK_10_REG_GRID_DIM_Z:
        return s->kernel.grid_dim[2];
    case GW_SPARK_10_REG_BLOCK_DIM_X:
        return s->kernel.block_dim[0];
    case GW_SPARK_10_REG_BLOCK_DIM_Y:
        return s->kernel.block_dim[1];
    case GW_SPARK_10_REG_BLOCK_DIM_Z:
        return s->kernel.block_dim[2];
    case GW_SPARK_10_REG_SHARED_MEM_SIZE:
        return s->kernel.shared_mem_size;
    case GW_SPARK_10_REG_DMA_SRC_LO:
        return extract64(s->dma.src_addr, 0, 32);
    case GW_SPARK_10_REG_DMA_SRC_HI:
        return extract64(s->dma.src_addr, 32, 32);
    case GW_SPARK_10_REG_DMA_DST_LO:
        return extract64(s->dma.dst_addr, 0, 32);
    case GW_SPARK_10_REG_DMA_DST_HI:
        return extract64(s->dma.dst_addr, 32, 32);
    case GW_SPARK_10_REG_DMA_SIZE:
        return s->dma.size;
    case GW_SPARK_10_REG_DMA_CTRL:
        return s->dma.ctrl;
    case GW_SPARK_10_REG_DMA_STATUS:
        return s->dma.status;
    case GW_SPARK_10_REG_THREAD_ID_X:
        return s->simt.thread_id[0];
    case GW_SPARK_10_REG_THREAD_ID_Y:
        return s->simt.thread_id[1];
    case GW_SPARK_10_REG_THREAD_ID_Z:
        return s->simt.thread_id[2];
    case GW_SPARK_10_REG_BLOCK_ID_X:
        return s->simt.block_id[0];
    case GW_SPARK_10_REG_BLOCK_ID_Y:
        return s->simt.block_id[1];
    case GW_SPARK_10_REG_BLOCK_ID_Z:
        return s->simt.block_id[2];
    case GW_SPARK_10_REG_WARP_ID:
        return s->simt.warp_id;
    case GW_SPARK_10_REG_LANE_ID:
        return s->simt.lane_id;
    case GW_SPARK_10_REG_THREAD_MASK:
        return s->simt.thread_mask;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "gw-spark-10: invalid control read at 0x%"
                      HWADDR_PRIx "\n",
                      addr);
        break;
    }

    return 0;
}

static void gw_spark_10_ctrl_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    GwSpark10State *s = opaque;
    uint64_t reg64;

    if (size != 4) {
        return;
    }

    switch (addr) {
    case GW_SPARK_10_REG_GLOBAL_CTRL:
        if (val & GW_SPARK_10_CTRL_RESET) {
            gw_spark_10_reset_state(s, false);
        } else {
            s->global_ctrl = val & GW_SPARK_10_CTRL_ENABLE;
        }
        break;
    case GW_SPARK_10_REG_ERROR_STATUS:
        s->error_status &= ~val;
        if (!s->error_status) {
            s->global_status &= ~GW_SPARK_10_STATUS_ERROR;
        }
        break;
    case GW_SPARK_10_REG_IRQ_ENABLE:
        s->irq_enable = val & (GW_SPARK_10_IRQ_KERNEL_DONE |
                               GW_SPARK_10_IRQ_DMA_DONE |
                               GW_SPARK_10_IRQ_ERROR);
        break;
    case GW_SPARK_10_REG_IRQ_ACK:
        s->irq_status &= ~val;
        break;
    case GW_SPARK_10_REG_KERNEL_ADDR_LO:
        s->kernel.kernel_addr = (s->kernel.kernel_addr & ~0xffffffffULL) |
                                (val & 0xffffffffULL);
        break;
    case GW_SPARK_10_REG_KERNEL_ADDR_HI:
        s->kernel.kernel_addr = (s->kernel.kernel_addr & 0xffffffffULL) |
                                ((val & 0xffffffffULL) << 32);
        break;
    case GW_SPARK_10_REG_KERNEL_ARGS_LO:
        s->kernel.kernel_args = (s->kernel.kernel_args & ~0xffffffffULL) |
                                (val & 0xffffffffULL);
        break;
    case GW_SPARK_10_REG_KERNEL_ARGS_HI:
        s->kernel.kernel_args = (s->kernel.kernel_args & 0xffffffffULL) |
                                ((val & 0xffffffffULL) << 32);
        break;
    case GW_SPARK_10_REG_GRID_DIM_X:
        s->kernel.grid_dim[0] = val;
        break;
    case GW_SPARK_10_REG_GRID_DIM_Y:
        s->kernel.grid_dim[1] = val;
        break;
    case GW_SPARK_10_REG_GRID_DIM_Z:
        s->kernel.grid_dim[2] = val;
        break;
    case GW_SPARK_10_REG_BLOCK_DIM_X:
        s->kernel.block_dim[0] = val;
        break;
    case GW_SPARK_10_REG_BLOCK_DIM_Y:
        s->kernel.block_dim[1] = val;
        break;
    case GW_SPARK_10_REG_BLOCK_DIM_Z:
        s->kernel.block_dim[2] = val;
        break;
    case GW_SPARK_10_REG_SHARED_MEM_SIZE:
        s->kernel.shared_mem_size = val;
        break;
    case GW_SPARK_10_REG_DISPATCH:
        gw_spark_10_start_kernel(s);
        break;
    case GW_SPARK_10_REG_DMA_SRC_LO:
        reg64 = s->dma.src_addr & ~0xffffffffULL;
        s->dma.src_addr = reg64 | (val & 0xffffffffULL);
        break;
    case GW_SPARK_10_REG_DMA_SRC_HI:
        reg64 = s->dma.src_addr & 0xffffffffULL;
        s->dma.src_addr = reg64 | ((val & 0xffffffffULL) << 32);
        break;
    case GW_SPARK_10_REG_DMA_DST_LO:
        reg64 = s->dma.dst_addr & ~0xffffffffULL;
        s->dma.dst_addr = reg64 | (val & 0xffffffffULL);
        break;
    case GW_SPARK_10_REG_DMA_DST_HI:
        reg64 = s->dma.dst_addr & 0xffffffffULL;
        s->dma.dst_addr = reg64 | ((val & 0xffffffffULL) << 32);
        break;
    case GW_SPARK_10_REG_DMA_SIZE:
        s->dma.size = val;
        break;
    case GW_SPARK_10_REG_DMA_CTRL:
        s->dma.ctrl = val;
        if (val & GW_SPARK_10_DMA_START) {
            gw_spark_10_start_dma(s);
        }
        break;
    case GW_SPARK_10_REG_THREAD_ID_X:
        s->simt.thread_id[0] = val;
        break;
    case GW_SPARK_10_REG_THREAD_ID_Y:
        s->simt.thread_id[1] = val;
        break;
    case GW_SPARK_10_REG_THREAD_ID_Z:
        s->simt.thread_id[2] = val;
        break;
    case GW_SPARK_10_REG_BLOCK_ID_X:
        s->simt.block_id[0] = val;
        break;
    case GW_SPARK_10_REG_BLOCK_ID_Y:
        s->simt.block_id[1] = val;
        break;
    case GW_SPARK_10_REG_BLOCK_ID_Z:
        s->simt.block_id[2] = val;
        break;
    case GW_SPARK_10_REG_WARP_ID:
        s->simt.warp_id = val;
        break;
    case GW_SPARK_10_REG_LANE_ID:
        s->simt.lane_id = val;
        break;
    case GW_SPARK_10_REG_BARRIER:
        s->simt.barrier_count++;
        s->simt.barrier_active = true;
        break;
    case GW_SPARK_10_REG_THREAD_MASK:
        s->simt.thread_mask = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "gw-spark-10: invalid control write at 0x%"
                      HWADDR_PRIx "\n",
                      addr);
        gw_spark_10_set_error(s, GW_SPARK_10_ERR_INVALID_CMD);
        break;
    }
}

static const MemoryRegionOps gw_spark_10_ctrl_ops = {
    .read = gw_spark_10_ctrl_read,
    .write = gw_spark_10_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t gw_spark_10_vram_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    GwSpark10State *s = opaque;

    if (!gw_spark_10_vram_range_valid(s, addr, size)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "gw-spark-10: invalid VRAM read at 0x%" HWADDR_PRIx
                      ", size %u\n", addr, size);
        gw_spark_10_set_error(s, GW_SPARK_10_ERR_VRAM_FAULT);
        return 0;
    }

    return gw_spark_10_load_le(s->vram_ptr + addr, size);
}

static void gw_spark_10_vram_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    GwSpark10State *s = opaque;

    if (!gw_spark_10_vram_range_valid(s, addr, size)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "gw-spark-10: invalid VRAM write at 0x%" HWADDR_PRIx
                      ", size %u\n", addr, size);
        gw_spark_10_set_error(s, GW_SPARK_10_ERR_VRAM_FAULT);
        return;
    }

    gw_spark_10_store_le(s->vram_ptr + addr, val, size);
}

static const MemoryRegionOps gw_spark_10_vram_ops = {
    .read = gw_spark_10_vram_read,
    .write = gw_spark_10_vram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static uint64_t gw_spark_10_doorbell_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    (void)opaque;
    (void)addr;
    (void)size;
    return 0;
}

static void gw_spark_10_doorbell_write(void *opaque, hwaddr addr,
                                       uint64_t val, unsigned size)
{
    (void)opaque;
    (void)addr;
    (void)val;
    (void)size;
}

static const MemoryRegionOps gw_spark_10_doorbell_ops = {
    .read = gw_spark_10_doorbell_read,
    .write = gw_spark_10_doorbell_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void gw_spark_10_realize(PCIDevice *pdev, Error **errp)
{
    GwSpark10State *s = GW_SPARK_10(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    s->vram_ptr = g_malloc0(s->vram_size);
    if (!s->vram_ptr) {
        error_setg(errp, "GW-Spark-10: failed to allocate VRAM");
        return;
    }

    /* BAR 0: control registers */
    memory_region_init_io(&s->ctrl_mmio, OBJECT(s), &gw_spark_10_ctrl_ops, s,
                          "gw-spark-10-ctrl", GW_SPARK_10_CTRL_BAR_SIZE);
    pci_register_bar(pdev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->ctrl_mmio);

    /* BAR 2: VRAM */
    memory_region_init_io(&s->vram, OBJECT(s), &gw_spark_10_vram_ops, s,
                          "gw-spark-10-vram", s->vram_size);
    pci_register_bar(pdev, 2,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64 |
                     PCI_BASE_ADDRESS_MEM_PREFETCH,
                     &s->vram);

    /* BAR 4: doorbell registers */
    memory_region_init_io(&s->doorbell_mmio, OBJECT(s),
                          &gw_spark_10_doorbell_ops, s,
                          "gw-spark-10-doorbell",
                          GW_SPARK_10_DOORBELL_BAR_SIZE);
    pci_register_bar(pdev, 4,
                     PCI_BASE_ADDRESS_SPACE_MEMORY,
                     &s->doorbell_mmio);

    if (msix_init(pdev, GW_SPARK_10_MSIX_VECTORS,
                  &s->ctrl_mmio, 0, 0xFE000,
                  &s->ctrl_mmio, 0, 0xFF000,
                  0, errp)) {
        g_free(s->vram_ptr);
        return;
    }

    msi_init(pdev, 0, 1, true, false, errp);

    s->global_status = GW_SPARK_10_STATUS_READY;

    error_setg(&s->migration_blocker,
               "GW-Spark-10 does not support migration because VRAM and "
               "execution state are not serialized");
    if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
        return;
    }
}

static void gw_spark_10_exit(PCIDevice *pdev)
{
    GwSpark10State *s = GW_SPARK_10(pdev);

    migrate_del_blocker(&s->migration_blocker);
    g_free(s->vram_ptr);
    msix_uninit(pdev, &s->ctrl_mmio, &s->ctrl_mmio);
    msi_uninit(pdev);
}

static void gw_spark_10_reset(DeviceState *dev)
{
    GwSpark10State *s = GW_SPARK_10(dev);

    gw_spark_10_reset_state(s, true);
}

static const Property gw_spark_10_properties[] = {
    DEFINE_PROP_UINT32("num_cus", GwSpark10State, num_cus,
                       GW_SPARK_10_DEFAULT_NUM_CUS),
    DEFINE_PROP_UINT32("warps_per_cu", GwSpark10State, warps_per_cu,
                       GW_SPARK_10_DEFAULT_WARPS_PER_CU),
    DEFINE_PROP_UINT32("warp_size", GwSpark10State, warp_size,
                       GW_SPARK_10_DEFAULT_WARP_SIZE),
    DEFINE_PROP_UINT64("vram_size", GwSpark10State, vram_size,
                       GW_SPARK_10_DEFAULT_VRAM_SIZE),
};

static void gw_spark_10_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = gw_spark_10_realize;
    pc->exit = gw_spark_10_exit;
    pc->vendor_id = GW_SPARK_10_VENDOR_ID;
    pc->device_id = GW_SPARK_10_DEVICE_ID;
    pc->revision = GW_SPARK_10_REVISION;
    pc->class_id = GW_SPARK_10_CLASS_CODE;

    device_class_set_legacy_reset(dc, gw_spark_10_reset);
    dc->desc = "GW-Spark-10 educational GPGPU device";
    device_class_set_props(dc, gw_spark_10_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo gw_spark_10_type_info = {
    .name          = TYPE_GW_SPARK_10,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(GwSpark10State),
    .class_init    = gw_spark_10_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void gw_spark_10_register_types(void)
{
    type_register_static(&gw_spark_10_type_info);
}

type_init(gw_spark_10_register_types)
