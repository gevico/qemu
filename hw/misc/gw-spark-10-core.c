/*
 * QEMU GW-Spark-10 RISC-V SIMT Core Implementation
 *
 * Copyright (c) 2024-2025
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/gw-spark-10.h"
#include "gw-spark-10-core.h"

#define GW_SPARK_10_CORE_WARP_SIZE 32
#define GW_SPARK_10_CORE_NUM_REGS 32
#define GW_SPARK_10_CORE_NUM_FREGS 32
#define GW_SPARK_10_CSR_FCSR 0x003
#define GW_SPARK_10_CSR_MHARTID 0xF14

#define GW_SPARK_10_MHARTID_ENCODE(block, warp, lane) \
    (((block) << 13) | ((warp) << 5) | ((lane) & 0x1F))
#define GW_SPARK_10_MHARTID_THREAD(id) ((id) & 0x1F)

#define GW_SPARK_10_CORE_CTRL_BASE        0x80000000
#define GW_SPARK_10_CORE_CTRL_THREAD_ID_X (GW_SPARK_10_CORE_CTRL_BASE + 0x00)
#define GW_SPARK_10_CORE_CTRL_THREAD_ID_Y (GW_SPARK_10_CORE_CTRL_BASE + 0x04)
#define GW_SPARK_10_CORE_CTRL_THREAD_ID_Z (GW_SPARK_10_CORE_CTRL_BASE + 0x08)
#define GW_SPARK_10_CORE_CTRL_BLOCK_ID_X  (GW_SPARK_10_CORE_CTRL_BASE + 0x10)
#define GW_SPARK_10_CORE_CTRL_BLOCK_ID_Y  (GW_SPARK_10_CORE_CTRL_BASE + 0x14)
#define GW_SPARK_10_CORE_CTRL_BLOCK_ID_Z  (GW_SPARK_10_CORE_CTRL_BASE + 0x18)
#define GW_SPARK_10_CORE_CTRL_BLOCK_DIM_X (GW_SPARK_10_CORE_CTRL_BASE + 0x20)
#define GW_SPARK_10_CORE_CTRL_BLOCK_DIM_Y (GW_SPARK_10_CORE_CTRL_BASE + 0x24)
#define GW_SPARK_10_CORE_CTRL_BLOCK_DIM_Z (GW_SPARK_10_CORE_CTRL_BASE + 0x28)
#define GW_SPARK_10_CORE_CTRL_GRID_DIM_X  (GW_SPARK_10_CORE_CTRL_BASE + 0x30)
#define GW_SPARK_10_CORE_CTRL_GRID_DIM_Y  (GW_SPARK_10_CORE_CTRL_BASE + 0x34)
#define GW_SPARK_10_CORE_CTRL_GRID_DIM_Z  (GW_SPARK_10_CORE_CTRL_BASE + 0x38)

typedef struct GwSpark10Lane {
    uint32_t gpr[GW_SPARK_10_CORE_NUM_REGS];
    uint32_t fpr[GW_SPARK_10_CORE_NUM_FREGS];
    uint32_t pc;
    uint32_t mhartid;
    uint32_t fcsr;
    bool active;
} GwSpark10Lane;

typedef struct GwSpark10Warp {
    GwSpark10Lane lanes[GW_SPARK_10_CORE_WARP_SIZE];
    uint32_t active_mask;
    uint32_t thread_id_base;
    uint32_t warp_id;
    uint32_t block_id[3];
} GwSpark10Warp;

typedef union GwSpark10Float32 {
    uint32_t u;
    float f;
} GwSpark10Float32;

typedef struct GwSpark10MiniFloatFormat {
    unsigned exp_bits;
    unsigned mant_bits;
    int bias;
    bool exp_max_special;
    bool e4m3fn_nan;
} GwSpark10MiniFloatFormat;

static int32_t gw_spark_10_core_sign_extend(uint32_t val, unsigned bits)
{
    return (int32_t)(val << (32 - bits)) >> (32 - bits);
}

static uint32_t gw_spark_10_core_get_bits(uint32_t insn, unsigned start,
                                           unsigned len)
{
    return extract32(insn, start, len);
}

static uint32_t gw_spark_10_core_float32_to_bits(float f)
{
    GwSpark10Float32 v = { .f = f };

    return v.u;
}

static float gw_spark_10_core_bits_to_float32(uint32_t u)
{
    GwSpark10Float32 v = { .u = u };

    return v.f;
}

static float gw_spark_10_core_absf(float v)
{
    return v < 0.0f ? -v : v;
}

static float gw_spark_10_core_pow2i(int exp)
{
    float v = 1.0f;

    while (exp > 0) {
        v *= 2.0f;
        exp--;
    }
    while (exp < 0) {
        v *= 0.5f;
        exp++;
    }

    return v;
}

static bool gw_spark_10_core_vram_access_ok(GwSpark10State *s,
                                             uint32_t addr, unsigned size)
{
    return addr <= s->vram_size && size <= s->vram_size - addr;
}

static bool gw_spark_10_core_load_u32(GwSpark10State *s, uint32_t addr,
                                      uint32_t *val)
{
    uint8_t *ptr;

    if (!gw_spark_10_core_vram_access_ok(s, addr, sizeof(uint32_t))) {
        return false;
    }

    ptr = s->vram_ptr + addr;
    *val = (uint32_t)ptr[0] |
           ((uint32_t)ptr[1] << 8) |
           ((uint32_t)ptr[2] << 16) |
           ((uint32_t)ptr[3] << 24);
    return true;
}

static bool gw_spark_10_core_store_u32(GwSpark10State *s, uint32_t addr,
                                      uint32_t val)
{
    uint8_t *ptr;

    if (!gw_spark_10_core_vram_access_ok(s, addr, sizeof(uint32_t))) {
        return false;
    }

    ptr = s->vram_ptr + addr;
    ptr[0] = val;
    ptr[1] = val >> 8;
    ptr[2] = val >> 16;
    ptr[3] = val >> 24;
    return true;
}

static uint32_t gw_spark_10_core_read_control(GwSpark10State *s,
                                               uint32_t addr)
{
    switch (addr) {
    case GW_SPARK_10_CORE_CTRL_THREAD_ID_X:
        return s->simt.thread_id[0];
    case GW_SPARK_10_CORE_CTRL_THREAD_ID_Y:
        return s->simt.thread_id[1];
    case GW_SPARK_10_CORE_CTRL_THREAD_ID_Z:
        return s->simt.thread_id[2];
    case GW_SPARK_10_CORE_CTRL_BLOCK_ID_X:
        return s->simt.block_id[0];
    case GW_SPARK_10_CORE_CTRL_BLOCK_ID_Y:
        return s->simt.block_id[1];
    case GW_SPARK_10_CORE_CTRL_BLOCK_ID_Z:
        return s->simt.block_id[2];
    case GW_SPARK_10_CORE_CTRL_BLOCK_DIM_X:
        return s->kernel.block_dim[0];
    case GW_SPARK_10_CORE_CTRL_BLOCK_DIM_Y:
        return s->kernel.block_dim[1];
    case GW_SPARK_10_CORE_CTRL_BLOCK_DIM_Z:
        return s->kernel.block_dim[2];
    case GW_SPARK_10_CORE_CTRL_GRID_DIM_X:
        return s->kernel.grid_dim[0];
    case GW_SPARK_10_CORE_CTRL_GRID_DIM_Y:
        return s->kernel.grid_dim[1];
    case GW_SPARK_10_CORE_CTRL_GRID_DIM_Z:
        return s->kernel.grid_dim[2];
    default:
        return 0;
    }
}

static void gw_spark_10_core_set_simt_context(GwSpark10State *s,
                                              const GwSpark10Warp *warp,
                                              const GwSpark10Lane *lane)
{
    uint32_t lane_id = GW_SPARK_10_MHARTID_THREAD(lane->mhartid);
    uint32_t thread_linear = warp->thread_id_base + lane_id;
    uint32_t block_x = s->kernel.block_dim[0] ? s->kernel.block_dim[0] : 1;
    uint32_t block_y = s->kernel.block_dim[1] ? s->kernel.block_dim[1] : 1;
    uint32_t xy = block_x * block_y;

    s->simt.thread_id[0] = thread_linear % block_x;
    s->simt.thread_id[1] = (thread_linear / block_x) % block_y;
    s->simt.thread_id[2] = xy ? thread_linear / xy : 0;
    s->simt.block_id[0] = warp->block_id[0];
    s->simt.block_id[1] = warp->block_id[1];
    s->simt.block_id[2] = warp->block_id[2];
    s->simt.warp_id = warp->warp_id;
    s->simt.lane_id = lane_id;
    s->simt.thread_mask = warp->active_mask;
}

static uint32_t gw_spark_10_core_float32_to_int32(uint32_t bits)
{
    uint32_t abs_bits = bits & 0x7fffffff;
    float f;

    if (abs_bits >= 0x7f800000) {
        return bits >> 31 ? (uint32_t)INT32_MIN : INT32_MAX;
    }

    f = gw_spark_10_core_bits_to_float32(bits);
    if (f >= 2147483000.0f) {
        return INT32_MAX;
    }
    if (f <= -2147483000.0f) {
        return (uint32_t)INT32_MIN;
    }

    return (uint32_t)(int32_t)f;
}

static uint16_t gw_spark_10_core_float32_to_bf16(uint32_t bits)
{
    uint32_t lsb = (bits >> 16) & 1;
    uint32_t rounded = bits + 0x7fff + lsb;

    return rounded >> 16;
}

static uint32_t gw_spark_10_core_bf16_to_float32(uint16_t bf16)
{
    return (uint32_t)bf16 << 16;
}

static bool gw_spark_10_core_minifloat_value(
    uint32_t mag, const GwSpark10MiniFloatFormat *fmt, float *value)
{
    uint32_t exp_mask = (1u << fmt->exp_bits) - 1;
    uint32_t mant_mask = (1u << fmt->mant_bits) - 1;
    uint32_t exp = (mag >> fmt->mant_bits) & exp_mask;
    uint32_t mant = mag & mant_mask;
    float scale = (float)(1u << fmt->mant_bits);

    if (fmt->e4m3fn_nan && exp == exp_mask && mant == mant_mask) {
        return false;
    }

    if (fmt->exp_max_special && exp == exp_mask) {
        return false;
    }

    if (exp == 0) {
        *value = mant == 0 ? 0.0f :
                 ((float)mant / scale) *
                 gw_spark_10_core_pow2i(1 - fmt->bias);
        return true;
    }

    *value = (1.0f + (float)mant / scale) *
             gw_spark_10_core_pow2i((int)exp - fmt->bias);
    return true;
}

static uint32_t gw_spark_10_core_float32_to_minifloat(
    uint32_t bits, const GwSpark10MiniFloatFormat *fmt)
{
    uint32_t sign_shift = fmt->exp_bits + fmt->mant_bits;
    uint32_t mag_limit = 1u << sign_shift;
    uint32_t sign = bits >> 31;
    uint32_t abs_bits = bits & 0x7fffffff;
    uint32_t best = 0;
    uint32_t max_code = 0;
    float best_diff = 3.4e38f;
    float max_val = 0.0f;
    float input;

    if (abs_bits == 0) {
        return sign << sign_shift;
    }

    for (uint32_t code = 0; code < mag_limit; code++) {
        float value;

        if (!gw_spark_10_core_minifloat_value(code, fmt, &value)) {
            continue;
        }
        if (value >= max_val) {
            max_val = value;
            max_code = code;
        }
    }

    if (abs_bits >= 0x7f800000) {
        return (sign << sign_shift) | max_code;
    }

    input = gw_spark_10_core_bits_to_float32(abs_bits);
    if (input >= max_val) {
        return (sign << sign_shift) | max_code;
    }

    for (uint32_t code = 0; code < mag_limit; code++) {
        float value;
        float diff;

        if (!gw_spark_10_core_minifloat_value(code, fmt, &value)) {
            continue;
        }

        diff = gw_spark_10_core_absf(input - value);
        if (diff < best_diff ||
            (diff == best_diff && (code & 1) == 0 && (best & 1) != 0)) {
            best_diff = diff;
            best = code;
        }
    }

    return (sign << sign_shift) | best;
}

static uint32_t gw_spark_10_core_minifloat_to_float32(
    uint32_t code, const GwSpark10MiniFloatFormat *fmt)
{
    uint32_t sign_shift = fmt->exp_bits + fmt->mant_bits;
    uint32_t sign = code >> sign_shift;
    uint32_t mag = code & ((1u << sign_shift) - 1);
    float value;

    if (!gw_spark_10_core_minifloat_value(mag, fmt, &value)) {
        value = 0.0f;
    }
    if (sign) {
        value = -value;
    }

    return gw_spark_10_core_float32_to_bits(value);
}

static int gw_spark_10_core_execute_float(GwSpark10Lane *lane, uint32_t insn)
{
    static const GwSpark10MiniFloatFormat bf8_e4m3 = {
        .exp_bits = 4,
        .mant_bits = 3,
        .bias = 7,
        .exp_max_special = false,
        .e4m3fn_nan = true,
    };
    static const GwSpark10MiniFloatFormat bf8_e5m2 = {
        .exp_bits = 5,
        .mant_bits = 2,
        .bias = 15,
        .exp_max_special = true,
        .e4m3fn_nan = false,
    };
    static const GwSpark10MiniFloatFormat bf4_e2m1 = {
        .exp_bits = 2,
        .mant_bits = 1,
        .bias = 1,
        .exp_max_special = false,
        .e4m3fn_nan = false,
    };
    uint32_t rd = gw_spark_10_core_get_bits(insn, 7, 5);
    uint32_t rm = gw_spark_10_core_get_bits(insn, 12, 3);
    uint32_t rs1 = gw_spark_10_core_get_bits(insn, 15, 5);
    uint32_t rs2 = gw_spark_10_core_get_bits(insn, 20, 5);
    uint32_t funct7 = gw_spark_10_core_get_bits(insn, 25, 7);
    float lhs;
    float rhs;

    switch (funct7) {
    case 0x00:
        lhs = gw_spark_10_core_bits_to_float32(lane->fpr[rs1]);
        rhs = gw_spark_10_core_bits_to_float32(lane->fpr[rs2]);
        lane->fpr[rd] = gw_spark_10_core_float32_to_bits(lhs + rhs);
        return 0;
    case 0x08:
        lhs = gw_spark_10_core_bits_to_float32(lane->fpr[rs1]);
        rhs = gw_spark_10_core_bits_to_float32(lane->fpr[rs2]);
        lane->fpr[rd] = gw_spark_10_core_float32_to_bits(lhs * rhs);
        return 0;
    case 0x22:
        if (rs2 == 1) {
            lane->fpr[rd] = gw_spark_10_core_float32_to_bf16(lane->fpr[rs1]);
            return 0;
        }
        if (rs2 == 0) {
            lane->fpr[rd] = gw_spark_10_core_bf16_to_float32(lane->fpr[rs1]);
            return 0;
        }
        return 1;
    case 0x24:
        if (rs2 == 1) {
            lane->fpr[rd] = gw_spark_10_core_float32_to_minifloat(
                lane->fpr[rs1], &bf8_e4m3);
            return 0;
        }
        if (rs2 == 0) {
            lane->fpr[rd] = gw_spark_10_core_minifloat_to_float32(
                lane->fpr[rs1] & 0xff, &bf8_e4m3);
            return 0;
        }
        if (rs2 == 3) {
            lane->fpr[rd] = gw_spark_10_core_float32_to_minifloat(
                lane->fpr[rs1], &bf8_e5m2);
            return 0;
        }
        if (rs2 == 2) {
            lane->fpr[rd] = gw_spark_10_core_minifloat_to_float32(
                lane->fpr[rs1] & 0xff, &bf8_e5m2);
            return 0;
        }
        return 1;
    case 0x26:
        if (rs2 == 1) {
            lane->fpr[rd] = gw_spark_10_core_float32_to_minifloat(
                lane->fpr[rs1], &bf4_e2m1);
            return 0;
        }
        if (rs2 == 0) {
            lane->fpr[rd] = gw_spark_10_core_minifloat_to_float32(
                lane->fpr[rs1] & 0xf, &bf4_e2m1);
            return 0;
        }
        return 1;
    case 0x60:
        (void)rm;
        lane->gpr[rd] = gw_spark_10_core_float32_to_int32(lane->fpr[rs1]);
        return 0;
    case 0x68:
        lane->fpr[rd] = gw_spark_10_core_float32_to_bits(
            (float)(int32_t)lane->gpr[rs1]);
        return 0;
    case 0x70:
        lane->gpr[rd] = lane->fpr[rs1];
        return 0;
    case 0x78:
        lane->fpr[rd] = lane->gpr[rs1];
        return 0;
    default:
        return 1;
    }
}

static int gw_spark_10_core_execute_instruction(GwSpark10State *s,
                                                GwSpark10Warp *warp,
                                                GwSpark10Lane *lane)
{
    uint32_t insn;
    uint32_t opcode;
    uint32_t rd;
    uint32_t rs1;
    uint32_t rs2;
    uint32_t funct3;
    uint32_t funct7;
    uint32_t addr;
    int32_t imm;

    gw_spark_10_core_set_simt_context(s, warp, lane);

    if (!gw_spark_10_core_load_u32(s, lane->pc, &insn)) {
        return 1;
    }
    lane->pc += 4;

    opcode = gw_spark_10_core_get_bits(insn, 0, 7);
    rd = gw_spark_10_core_get_bits(insn, 7, 5);
    funct3 = gw_spark_10_core_get_bits(insn, 12, 3);
    rs1 = gw_spark_10_core_get_bits(insn, 15, 5);
    rs2 = gw_spark_10_core_get_bits(insn, 20, 5);
    funct7 = gw_spark_10_core_get_bits(insn, 25, 7);

    switch (opcode) {
    case 0x03:
        imm = gw_spark_10_core_sign_extend(
            gw_spark_10_core_get_bits(insn, 20, 12), 12);
        addr = lane->gpr[rs1] + (uint32_t)imm;
        if (funct3 != 0x2) {
            return 1;
        }
        if (addr >= GW_SPARK_10_CORE_CTRL_BASE) {
            lane->gpr[rd] = gw_spark_10_core_read_control(s, addr);
        } else if (!gw_spark_10_core_load_u32(s, addr, &lane->gpr[rd])) {
            return 1;
        }
        break;
    case 0x13:
        imm = gw_spark_10_core_sign_extend(
            gw_spark_10_core_get_bits(insn, 20, 12), 12);
        switch (funct3) {
        case 0x0:
            lane->gpr[rd] = lane->gpr[rs1] + (uint32_t)imm;
            break;
        case 0x1:
            if (funct7 != 0) {
                return 1;
            }
            lane->gpr[rd] = lane->gpr[rs1] << rs2;
            break;
        case 0x7:
            lane->gpr[rd] = lane->gpr[rs1] & (uint32_t)imm;
            break;
        default:
            return 1;
        }
        break;
    case 0x23:
        imm = (gw_spark_10_core_get_bits(insn, 25, 7) << 5) |
              gw_spark_10_core_get_bits(insn, 7, 5);
        imm = gw_spark_10_core_sign_extend(imm, 12);
        addr = lane->gpr[rs1] + (uint32_t)imm;
        if (funct3 != 0x2 || addr >= GW_SPARK_10_CORE_CTRL_BASE ||
            !gw_spark_10_core_store_u32(s, addr, lane->gpr[rs2])) {
            return 1;
        }
        break;
    case 0x33:
        if (funct3 == 0 && funct7 == 0) {
            lane->gpr[rd] = lane->gpr[rs1] + lane->gpr[rs2];
        } else {
            return 1;
        }
        break;
    case 0x37:
        lane->gpr[rd] = insn & 0xfffff000;
        break;
    case 0x53:
        if (gw_spark_10_core_execute_float(lane, insn)) {
            return 1;
        }
        break;
    case 0x73:
        imm = gw_spark_10_core_get_bits(insn, 20, 12);
        if (funct3 == 0 && imm == 1) {
            lane->active = false;
            break;
        }
        if (funct3 == 2) {
            switch (imm) {
            case GW_SPARK_10_CSR_MHARTID:
                lane->gpr[rd] = lane->mhartid;
                break;
            case GW_SPARK_10_CSR_FCSR:
                lane->gpr[rd] = lane->fcsr;
                break;
            default:
                lane->gpr[rd] = 0;
                break;
            }
            break;
        }
        return 1;
    default:
        return 1;
    }

    lane->gpr[0] = 0;
    return 0;
}

static void gw_spark_10_core_init_warp(
    GwSpark10Warp *warp, uint32_t pc, uint32_t thread_id_base,
    const uint32_t block_id[3], uint32_t num_threads, uint32_t warp_id,
    uint32_t block_id_linear)
{
    memset(warp, 0, sizeof(*warp));

    warp->thread_id_base = thread_id_base;
    warp->warp_id = warp_id;
    warp->block_id[0] = block_id[0];
    warp->block_id[1] = block_id[1];
    warp->block_id[2] = block_id[2];

    for (uint32_t lane = 0; lane < GW_SPARK_10_CORE_WARP_SIZE; lane++) {
        GwSpark10Lane *state = &warp->lanes[lane];

        state->pc = pc;
        state->mhartid = GW_SPARK_10_MHARTID_ENCODE(block_id_linear,
                                                    warp_id, lane);
        state->active = lane < num_threads;

        if (state->active) {
            warp->active_mask |= 1u << lane;
        }
    }
}

static int gw_spark_10_core_execute_warp(GwSpark10State *s,
                                         GwSpark10Warp *warp,
                                         uint32_t max_cycles,
                                         uint32_t *instruction_budget)
{
    for (uint32_t lane = 0; lane < GW_SPARK_10_CORE_WARP_SIZE; lane++) {
        GwSpark10Lane *state = &warp->lanes[lane];
        uint32_t cycles = 0;

        while (state->active) {
            if (cycles >= max_cycles || !*instruction_budget) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "gw-spark-10: warp %u lane %u execution failed\n",
                              warp->warp_id, lane);
                return 1;
            }
            cycles++;
            if (gw_spark_10_core_execute_instruction(s, warp, state)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "gw-spark-10: warp %u lane %u execution failed\n",
                              warp->warp_id, lane);
                return 1;
            }
            (*instruction_budget)--;
        }
    }

    return 0;
}

static bool gw_spark_10_core_count_dimensions(const uint32_t dimensions[3],
                                              uint64_t limit,
                                              uint64_t *count)
{
    *count = 1;
    for (size_t i = 0; i < 3; i++) {
        if (!dimensions[i] ||
            *count > limit / dimensions[i]) {
            return false;
        }
        *count *= dimensions[i];
    }

    return true;
}

int gw_spark_10_core_execute_kernel(GwSpark10State *s)
{
    uint32_t grid_x = s->kernel.grid_dim[0];
    uint32_t grid_y = s->kernel.grid_dim[1];
    uint32_t grid_z = s->kernel.grid_dim[2];
    uint64_t grid_blocks;
    uint64_t threads_per_block;
    uint64_t total_threads;
    uint32_t block_linear = 0;
    uint32_t instruction_budget = GW_SPARK_10_MAX_DISPATCH_INSTRUCTIONS;

    if (s->kernel.kernel_addr > s->vram_size ||
        !gw_spark_10_core_count_dimensions(
            s->kernel.grid_dim, GW_SPARK_10_MAX_DISPATCH_THREADS,
            &grid_blocks) ||
        !gw_spark_10_core_count_dimensions(s->kernel.block_dim,
                                            GW_SPARK_10_MAX_THREADS_PER_BLOCK,
                                            &threads_per_block) ||
        grid_blocks > GW_SPARK_10_MAX_DISPATCH_THREADS / threads_per_block) {
        return 1;
    }

    total_threads = grid_blocks * threads_per_block;
    if (total_threads > GW_SPARK_10_MAX_DISPATCH_THREADS) {
        return 1;
    }

    for (uint32_t bz = 0; bz < grid_z; bz++) {
        for (uint32_t by = 0; by < grid_y; by++) {
            for (uint32_t bx = 0; bx < grid_x; bx++, block_linear++) {
                uint32_t block_id[3] = { bx, by, bz };

                for (uint32_t base = 0; base < threads_per_block;
                     base += GW_SPARK_10_CORE_WARP_SIZE) {
                    GwSpark10Warp warp;
                    uint32_t remaining = threads_per_block - base;
                    uint32_t active_threads = MIN(
                        remaining, GW_SPARK_10_CORE_WARP_SIZE);

                    gw_spark_10_core_init_warp(
                        &warp, s->kernel.kernel_addr, base, block_id,
                        active_threads, base / GW_SPARK_10_CORE_WARP_SIZE,
                        block_linear);

                    if (gw_spark_10_core_execute_warp(
                            s, &warp, GW_SPARK_10_MAX_INSTRUCTIONS_PER_LANE,
                            &instruction_budget)) {
                        return 1;
                    }
                }
            }
        }
    }

    return 0;
}
