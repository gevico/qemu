/*
 * GW-Spark-10 educational GPGPU device.
 *
 * Copyright (c) 2024-2025
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_GW_SPARK_10_H
#define HW_MISC_GW_SPARK_10_H

#include "hw/pci/pci_device.h"
#include "hw/pci/pci_ids.h"
#include "qom/object.h"

#define TYPE_GW_SPARK_10 "gw-spark-10"
OBJECT_DECLARE_SIMPLE_TYPE(GwSpark10State, GW_SPARK_10)

#define GW_SPARK_10_VENDOR_ID         0x1234
#define GW_SPARK_10_DEVICE_ID         0x1337
#define GW_SPARK_10_REVISION          0x01
#define GW_SPARK_10_CLASS_CODE        PCI_CLASS_DISPLAY_3D

#define GW_SPARK_10_CTRL_BAR_SIZE     (1 * 1024 * 1024)
#define GW_SPARK_10_VRAM_BAR_SIZE     (64 * 1024 * 1024)
#define GW_SPARK_10_DOORBELL_BAR_SIZE (64 * 1024)

#define GW_SPARK_10_DEFAULT_NUM_CUS       4
#define GW_SPARK_10_DEFAULT_WARPS_PER_CU  4
#define GW_SPARK_10_DEFAULT_WARP_SIZE     32
#define GW_SPARK_10_DEFAULT_VRAM_SIZE     (64 * 1024 * 1024)

#define GW_SPARK_10_MAX_DISPATCH_THREADS       65536
#define GW_SPARK_10_MAX_THREADS_PER_BLOCK      8192
#define GW_SPARK_10_MAX_DISPATCH_INSTRUCTIONS  (1U << 20)
#define GW_SPARK_10_MAX_INSTRUCTIONS_PER_LANE  10000

#define GW_SPARK_10_REG_DEV_ID            0x0000
#define GW_SPARK_10_REG_DEV_VERSION       0x0004
#define GW_SPARK_10_REG_DEV_CAPS          0x0008
#define GW_SPARK_10_REG_VRAM_SIZE_LO      0x000C
#define GW_SPARK_10_REG_VRAM_SIZE_HI      0x0010

#define GW_SPARK_10_REG_GLOBAL_CTRL       0x0100
#define GW_SPARK_10_REG_GLOBAL_STATUS     0x0104
#define GW_SPARK_10_REG_ERROR_STATUS      0x0108

#define GW_SPARK_10_REG_IRQ_ENABLE        0x0200
#define GW_SPARK_10_REG_IRQ_STATUS        0x0204
#define GW_SPARK_10_REG_IRQ_ACK           0x0208

#define GW_SPARK_10_REG_KERNEL_ADDR_LO    0x0300
#define GW_SPARK_10_REG_KERNEL_ADDR_HI    0x0304
#define GW_SPARK_10_REG_KERNEL_ARGS_LO    0x0308
#define GW_SPARK_10_REG_KERNEL_ARGS_HI    0x030C
#define GW_SPARK_10_REG_GRID_DIM_X        0x0310
#define GW_SPARK_10_REG_GRID_DIM_Y        0x0314
#define GW_SPARK_10_REG_GRID_DIM_Z        0x0318
#define GW_SPARK_10_REG_BLOCK_DIM_X       0x031C
#define GW_SPARK_10_REG_BLOCK_DIM_Y       0x0320
#define GW_SPARK_10_REG_BLOCK_DIM_Z       0x0324
#define GW_SPARK_10_REG_SHARED_MEM_SIZE   0x0328
#define GW_SPARK_10_REG_DISPATCH          0x0330

#define GW_SPARK_10_REG_DMA_SRC_LO        0x0400
#define GW_SPARK_10_REG_DMA_SRC_HI        0x0404
#define GW_SPARK_10_REG_DMA_DST_LO        0x0408
#define GW_SPARK_10_REG_DMA_DST_HI        0x040C
#define GW_SPARK_10_REG_DMA_SIZE          0x0410
#define GW_SPARK_10_REG_DMA_CTRL          0x0414
#define GW_SPARK_10_REG_DMA_STATUS        0x0418

#define GW_SPARK_10_REG_THREAD_ID_X       0x1000
#define GW_SPARK_10_REG_THREAD_ID_Y       0x1004
#define GW_SPARK_10_REG_THREAD_ID_Z       0x1008
#define GW_SPARK_10_REG_BLOCK_ID_X        0x1010
#define GW_SPARK_10_REG_BLOCK_ID_Y        0x1014
#define GW_SPARK_10_REG_BLOCK_ID_Z        0x1018
#define GW_SPARK_10_REG_WARP_ID           0x1020
#define GW_SPARK_10_REG_LANE_ID           0x1024

#define GW_SPARK_10_REG_BARRIER           0x2000
#define GW_SPARK_10_REG_THREAD_MASK       0x2004

#define GW_SPARK_10_CTRL_ENABLE           (1 << 0)
#define GW_SPARK_10_CTRL_RESET            (1 << 1)

#define GW_SPARK_10_STATUS_READY          (1 << 0)
#define GW_SPARK_10_STATUS_BUSY           (1 << 1)
#define GW_SPARK_10_STATUS_ERROR          (1 << 2)

#define GW_SPARK_10_ERR_INVALID_CMD       (1 << 0)
#define GW_SPARK_10_ERR_VRAM_FAULT        (1 << 1)
#define GW_SPARK_10_ERR_KERNEL_FAULT      (1 << 2)
#define GW_SPARK_10_ERR_DMA_FAULT         (1 << 3)

#define GW_SPARK_10_IRQ_KERNEL_DONE       (1 << 0)
#define GW_SPARK_10_IRQ_DMA_DONE          (1 << 1)
#define GW_SPARK_10_IRQ_ERROR             (1 << 2)

#define GW_SPARK_10_DMA_START             (1 << 0)
#define GW_SPARK_10_DMA_DIR_TO_VRAM       (0 << 1)
#define GW_SPARK_10_DMA_DIR_FROM_VRAM     (1 << 1)
#define GW_SPARK_10_DMA_IRQ_ENABLE        (1 << 2)

#define GW_SPARK_10_DMA_BUSY              (1 << 0)
#define GW_SPARK_10_DMA_COMPLETE          (1 << 1)
#define GW_SPARK_10_DMA_ERROR             (1 << 2)

#define GW_SPARK_10_DEV_ID_VALUE          0x47505055
#define GW_SPARK_10_DEV_VERSION_VALUE     0x00010000

#define GW_SPARK_10_MSIX_VECTORS          4
#define GW_SPARK_10_MSIX_VEC_KERNEL       0
#define GW_SPARK_10_MSIX_VEC_DMA          1
#define GW_SPARK_10_MSIX_VEC_ERROR        2

typedef struct GwSpark10KernelParams {
    uint64_t kernel_addr;
    uint64_t kernel_args;
    uint32_t grid_dim[3];
    uint32_t block_dim[3];
    uint32_t shared_mem_size;
} GwSpark10KernelParams;

typedef struct GwSpark10DmaState {
    uint64_t src_addr;
    uint64_t dst_addr;
    uint32_t size;
    uint32_t ctrl;
    uint32_t status;
} GwSpark10DmaState;

typedef struct GwSpark10SimtContext {
    uint32_t thread_id[3];
    uint32_t block_id[3];
    uint32_t warp_id;
    uint32_t lane_id;
    uint32_t barrier_count;
    uint32_t barrier_target;
    bool barrier_active;
    uint32_t thread_mask;
} GwSpark10SimtContext;

struct GwSpark10State {
    PCIDevice parent_obj;

    MemoryRegion ctrl_mmio;
    MemoryRegion vram;
    MemoryRegion doorbell_mmio;

    uint32_t num_cus;
    uint32_t warps_per_cu;
    uint32_t warp_size;
    uint64_t vram_size;
    uint8_t *vram_ptr;

    uint32_t global_ctrl;
    uint32_t global_status;
    uint32_t error_status;
    uint32_t irq_enable;
    uint32_t irq_status;

    GwSpark10KernelParams kernel;
    GwSpark10DmaState dma;
    GwSpark10SimtContext simt;
};

#endif /* HW_MISC_GW_SPARK_10_H */
