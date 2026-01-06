/*
 * RISC-V Debug Module glue helpers
 *
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_RISCV_DEBUG_H
#define HW_RISCV_DEBUG_H

#include "hw/sysbus.h"
#include "hw/riscv/riscv_hart.h"

#define TYPE_RISCV_DEBUG_MODULE "riscv.debug-module"

typedef struct RISCVDebugModuleState RISCVDebugModuleState;

OBJECT_DECLARE_SIMPLE_TYPE(RISCVDebugModuleState, RISCV_DEBUG_MODULE)

void riscv_debug_module_add_hart_array(RISCVDebugModuleState *dm,
                                       RISCVHartArrayState *array);

#endif /* HW_RISCV_DEBUG_H */
