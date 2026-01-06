/*
 * RISC-V Debug Module implementation
 *
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/bitops.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/riscv/riscv_debug.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/riscv-debug-rom.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "system/memory.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "chardev/char-fe.h"
#include "target/riscv/cpu.h"
#include "trace.h"

#define DM_DATA0               0x04
#define DM_DMCONTROL           0x10
#define DM_DMSTATUS            0x11
#define DM_HARTINFO            0x12
#define DM_ABSTRACTCS          0x16
#define DM_COMMAND             0x17
#define DM_ABSTRACTAUTO        0x18
#define DM_PROGBUF0            0x20
#define DM_AUTHDATA            0x30
#define DM_DMCS2               0x32
#define DM_SBCS                0x38
#define DM_SBADDRESS0          0x39
#define DM_SBADDRESS1          0x3a
#define DM_SBADDRESS2          0x3b
#define DM_SBADDRESS3          0x37
#define DM_SBDATA0             0x3c
#define DM_SBDATA1             0x3d
#define DM_SBDATA2             0x3e
#define DM_SBDATA3             0x3f

#define DM_DMCONTROL_DMACTIVE          BIT(0)
#define DM_DMCONTROL_NDMRESET          BIT(1)
#define DM_DMCONTROL_CLRRESETHALTREQ   BIT(2)
#define DM_DMCONTROL_SETRESETHALTREQ   BIT(3)
#define DM_DMCONTROL_CLRKEEPALIVE      BIT(4)
#define DM_DMCONTROL_SETKEEPALIVE      BIT(5)
#define DM_DMCONTROL_HARTSELHI_SHIFT   6
#define DM_DMCONTROL_HARTSELHI_MASK    (0x3ffu << DM_DMCONTROL_HARTSELHI_SHIFT)
#define DM_DMCONTROL_HARTSELLO_SHIFT   16
#define DM_DMCONTROL_HARTSELLO_MASK    (0x3ffu << DM_DMCONTROL_HARTSELLO_SHIFT)
#define DM_DMCONTROL_HASEL             BIT(18)
#define DM_DMCONTROL_ACKHAVERESET      BIT(28)
#define DM_DMCONTROL_HARTRESET         BIT(29)
#define DM_DMCONTROL_RESUMEREQ         BIT(30)
#define DM_DMCONTROL_HALTREQ           BIT(31)

#define DM_DMSTATUS_VERSION_MASK       0xf
#define DM_DMSTATUS_CONFSTRPTRVALID    BIT(4)
#define DM_DMSTATUS_HASRESETHALTREQ    BIT(5)
#define DM_DMSTATUS_AUTHBUSY           BIT(6)
#define DM_DMSTATUS_AUTHENTICATED      BIT(7)
#define DM_DMSTATUS_ANYHALTED          BIT(8)
#define DM_DMSTATUS_ALLHALTED          BIT(9)
#define DM_DMSTATUS_ANYRUNNING         BIT(10)
#define DM_DMSTATUS_ALLRUNNING         BIT(11)
#define DM_DMSTATUS_ANYUNAVAIL         BIT(12)
#define DM_DMSTATUS_ALLUNAVAIL         BIT(13)
#define DM_DMSTATUS_ANYNONEXISTENT     BIT(14)
#define DM_DMSTATUS_ALLNONEXISTENT     BIT(15)
#define DM_DMSTATUS_ANYRESUMEACK       BIT(16)
#define DM_DMSTATUS_ALLRESUMEACK       BIT(17)
#define DM_DMSTATUS_ANYHAVERESET       BIT(18)
#define DM_DMSTATUS_ALLHAVERESET       BIT(19)
#define DM_DMSTATUS_IMPEBREAK          BIT(22)

#define DM_HARTINFO_DATAADDR_SHIFT     0
#define DM_HARTINFO_DATASIZE_SHIFT     12
#define DM_HARTINFO_DATASIZE_MASK      0xf
#define DM_HARTINFO_DATAACCESS         BIT(16)
#define DM_HARTINFO_NSCRATCH_SHIFT     20
#define DM_HARTINFO_NSCRATCH_MASK      0xf

#define DM_ABSTRACTCS_DATACOUNT_MASK   0xf
#define DM_ABSTRACTCS_CMDERR_SHIFT     8
#define DM_ABSTRACTCS_CMDERR_MASK      0x7
#define DM_ABSTRACTCS_BUSY             BIT(12)
#define DM_ABSTRACTCS_PROGBUFSIZE_SHIFT 24
#define DM_ABSTRACTCS_PROGBUFSIZE_MASK 0x1f

#define DM_COMMAND_CMDTYPE_SHIFT       24
#define DM_COMMAND_CMDTYPE_MASK        0xff

#define AC_ACCESS_REGISTER_AARSIZE_SHIFT   20
#define AC_ACCESS_REGISTER_AARSIZE_MASK    0x7
#define AC_ACCESS_REGISTER_AARPOSTINCREMENT BIT(19)
#define AC_ACCESS_REGISTER_POSTEXEC        BIT(18)
#define AC_ACCESS_REGISTER_TRANSFER        BIT(17)
#define AC_ACCESS_REGISTER_WRITE           BIT(16)
#define AC_ACCESS_REGISTER_REGNO_MASK      0xffff

#define DM_ABSTRACTAUTO_AUTOEXECDATA_MASK  0xffff

#define DM_SBCS_SBACCESS_SHIFT         17
#define DM_SBCS_SBACCESS_MASK          0x7
#define DM_SBCS_SBREADONADDR           BIT(20)
#define DM_SBCS_SBAUTOINCREMENT        BIT(16)
#define DM_SBCS_SBREADONDATA           BIT(15)
#define DM_SBCS_SBERROR_SHIFT          12
#define DM_SBCS_SBERROR_MASK           0x7
#define DM_SBCS_SBBUSY                 BIT(21)
#define DM_SBCS_SBBUSYERROR            BIT(22)
#define DM_SBCS_SBVERSION_SHIFT        29
#define DM_SBCS_SBVERSION_MASK         0x7
#define DM_SBCS_SBASIZE_SHIFT          5
#define DM_SBCS_SBASIZE_MASK           0x7f
#define DM_SBCS_SBACCESS32             BIT(2)
#define DM_SBCS_SBACCESS64             BIT(3)

#define DMI_ADDR_BITS                  7
#define DM_MAX_HARTS                  4096
#define DM_DATA_COUNT                   4
#define DM_PROGBUF_COUNT                2
#define DM_ABSTRACT_COUNT              24
#define DM_IMPEBREAK                    0
#define DM_PROGBUF_IMPEBREAK_WORDS     (DM_IMPEBREAK ? 1 : 0)
#define DM_PROGBUF_WORDS               (DM_PROGBUF_COUNT + DM_PROGBUF_IMPEBREAK_WORDS)
#define DM_PROGBUF_BYTES               (DM_PROGBUF_WORDS * 4)
#define DM_DEBUG_PROGBUF_START         (DEBUG_ROM_DATA_START - DM_PROGBUF_BYTES)
#define DM_DEBUG_ABSTRACT_START        (DM_DEBUG_PROGBUF_START - DM_ABSTRACT_COUNT * 4)
#define DM_DATA_BASE                   DEBUG_ROM_DATA_START
#define DM_DATA_BYTES                  (DM_DATA_COUNT * 4)
#define DM_ROM_JUMP_OFFSET             (DM_DEBUG_ABSTRACT_START - DEBUG_ROM_WHERETO)
#define DM_MAX_REMOTE_BUF          (64 * KiB)
#define AC_ACCESS_MEMORY_AAMSIZE_SHIFT      20
#define AC_ACCESS_MEMORY_AAMSIZE_MASK       0x7
#define AC_ACCESS_MEMORY_AAMPOSTINCREMENT   BIT(19)
#define AC_ACCESS_MEMORY_AAMVIRTUAL         BIT(23)
#define AC_ACCESS_MEMORY_WRITE              BIT(16)

#ifndef CSR_DSCRATCH0
#define CSR_DSCRATCH0 0x7b2
#endif
#ifndef CSR_DSCRATCH1
#define CSR_DSCRATCH1 0x7b3
#endif

enum {
    RISCV_REG_ZERO = 0,
    RISCV_REG_RA = 1,
    RISCV_REG_SP = 2,
    RISCV_REG_GP = 3,
    RISCV_REG_TP = 4,
    RISCV_REG_T0 = 5,
    RISCV_REG_T1 = 6,
    RISCV_REG_T2 = 7,
    RISCV_REG_S0 = 8,
    RISCV_REG_S1 = 9,
};

static inline uint32_t riscv_insn_itype(uint32_t opcode, uint32_t rd,
                                        uint32_t funct3, uint32_t rs1,
                                        int32_t imm)
{
    uint32_t simm = (uint32_t)imm & 0xfff;
    return (simm << 20) | (rs1 << 15) | (funct3 << 12) |
           (rd << 7) | opcode;
}

static inline uint32_t riscv_insn_stype(uint32_t opcode, uint32_t funct3,
                                        uint32_t rs1, uint32_t rs2,
                                        int32_t imm)
{
    uint32_t simm = (uint32_t)imm & 0xfff;
    uint32_t imm11_5 = (simm >> 5) & 0x7f;
    uint32_t imm4_0 = simm & 0x1f;
    return (imm11_5 << 25) | (rs2 << 20) | (rs1 << 15) |
           (funct3 << 12) | (imm4_0 << 7) | opcode;
}

static inline uint32_t riscv_insn_utype(uint32_t opcode, uint32_t rd,
                                        int32_t imm)
{
    uint32_t uimm = (uint32_t)imm & 0xfffff000u;
    return uimm | (rd << 7) | opcode;
}

static inline uint32_t riscv_insn_rtype(uint32_t opcode, uint32_t rd,
                                        uint32_t funct3, uint32_t rs1,
                                        uint32_t rs2, uint32_t funct7)
{
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) |
           (funct3 << 12) | (rd << 7) | opcode;
}

static inline uint32_t riscv_insn_jal(uint32_t rd, int32_t offset)
{
    uint32_t simm = (uint32_t)offset;
    uint32_t imm20 = (simm >> 20) & 0x1;
    uint32_t imm10_1 = (simm >> 1) & 0x3ff;
    uint32_t imm11 = (simm >> 11) & 0x1;
    uint32_t imm19_12 = (simm >> 12) & 0xff;
    uint32_t value = 0;

    value |= imm20 << 31;
    value |= imm19_12 << 12;
    value |= imm11 << 20;
    value |= imm10_1 << 21;
    value |= (rd << 7) | 0x6f;
    return value;
}

static inline uint32_t riscv_insn_addi(uint32_t rd, uint32_t rs1, int32_t imm)
{
    return riscv_insn_itype(0x13, rd, 0x0, rs1, imm);
}

static inline uint32_t riscv_insn_lw(uint32_t rd, uint32_t rs1, int32_t imm)
{
    return riscv_insn_itype(0x03, rd, 0x2, rs1, imm);
}

static inline uint32_t riscv_insn_sw(uint32_t rs2, uint32_t rs1, int32_t imm)
{
    return riscv_insn_stype(0x23, 0x2, rs1, rs2, imm);
}

static inline uint32_t riscv_insn_ld(uint32_t rd, uint32_t rs1, int32_t imm)
{
    return riscv_insn_itype(0x03, rd, 0x3, rs1, imm);
}

static inline uint32_t riscv_insn_sd(uint32_t rs2, uint32_t rs1, int32_t imm)
{
    return riscv_insn_stype(0x23, 0x3, rs1, rs2, imm);
}

static inline uint32_t riscv_insn_lui(uint32_t rd, int32_t imm)
{
    return riscv_insn_utype(0x37, rd, imm);
}

static inline uint32_t riscv_insn_csrr(uint32_t rd, uint32_t csr)
{
    return riscv_insn_itype(0x73, rd, 0x2, RISCV_REG_ZERO, csr);
}

static inline uint32_t riscv_insn_csrw(uint32_t csr, uint32_t rs)
{
    return riscv_insn_itype(0x73, RISCV_REG_ZERO, 0x1, rs, csr);
}

static inline uint32_t riscv_insn_ebreak(void)
{
    return 0x00100073;
}

typedef enum DMICommandError {
    CMDERR_NONE = 0,
    CMDERR_BUSY = 1,
    CMDERR_NOTSUP = 2,
    CMDERR_EXCEPTION = 3,
    CMDERR_HALTRESUME = 4,
    CMDERR_OTHER = 7,
} DMICommandError;

typedef enum DMIOp {
    DMI_OP_NOP = 0,
    DMI_OP_READ = 1,
    DMI_OP_WRITE = 2,
} DMIOp;

typedef enum DMIOpStatus {
    DMI_STATUS_SUCCESS = 0,
    DMI_STATUS_RESERVED = 1,
    DMI_STATUS_FAILED = 2,
    DMI_STATUS_BUSY = 3,
} DMIOpStatus;

typedef enum DMCommandType {
    DM_CMD_ACCESS_REGISTER = 0,
    DM_CMD_ACCESS_MEMORY = 2,
} DMCommandType;

typedef struct RISCVDMHartState {
    RISCVCPU *cpu;
    CPUState *cs;
    RISCVDebugModuleState *dm;
    uint32_t hartid;
    bool halted;
    bool resumeack;
    bool havereset;
    bool resethaltreq;
    bool halt_pending;
    bool resume_pending;
    uint8_t halt_cause;
} RISCVDMHartState;

typedef struct RISCVDMSystemBus {
    uint32_t sbcs;
    uint32_t sbaddress[4];
    uint32_t sbdata[4];
    bool read_in_progress;
    bool write_in_progress;
} RISCVDMSystemBus;

typedef enum JtagState {
    JTAG_TEST_LOGIC_RESET,
    JTAG_RUN_TEST_IDLE,
    JTAG_SELECT_DR_SCAN,
    JTAG_CAPTURE_DR,
    JTAG_SHIFT_DR,
    JTAG_EXIT1_DR,
    JTAG_PAUSE_DR,
    JTAG_EXIT2_DR,
    JTAG_UPDATE_DR,
    JTAG_SELECT_IR_SCAN,
    JTAG_CAPTURE_IR,
    JTAG_SHIFT_IR,
    JTAG_EXIT1_IR,
    JTAG_PAUSE_IR,
    JTAG_EXIT2_IR,
    JTAG_UPDATE_IR,
} JtagState;

#define DTM_IR_IDCODE        0x01
#define DTM_IR_DTMCONTROL    0x10
#define DTM_IR_DBUS          0x11
#define DTM_IR_BYPASS        0x1f

#define DTMCONTROL_VERSION_MASK   0xf
#define DTMCONTROL_ABITS_SHIFT    4
#define DTMCONTROL_ABITS_MASK     (0x3f << DTMCONTROL_ABITS_SHIFT)
#define DTMCONTROL_DMISTAT_SHIFT  10
#define DTMCONTROL_DMISTAT_MASK   (0x3 << DTMCONTROL_DMISTAT_SHIFT)
#define DTMCONTROL_IDLE_SHIFT     12
#define DTMCONTROL_IDLE_MASK      (0x7 << DTMCONTROL_IDLE_SHIFT)
#define DTMCONTROL_DMIRESET       BIT(16)
#define DTMCONTROL_DMIHARDRESET   BIT(17)

#define DMI_OP_MASK         0x3
#define DMI_DATA_SHIFT      2
#define DMI_ADDRESS_SHIFT   34

typedef struct RISCVDTMState {
    RISCVDebugModuleState *dm;
    uint32_t dtmcontrol;
    uint64_t dmi;
    uint64_t dr;
    uint32_t ir;
    unsigned dr_length;
    unsigned ir_length;
    unsigned rti_remaining;
    bool busy_stuck;
    bool tck;
    bool tms;
    bool tdi;
    bool tdo;
    uint8_t bypass;
    JtagState state;
} RISCVDTMState;

struct RISCVDebugModuleState {
    SysBusDevice parent_obj;

    CharFrontend chr;

    RISCVDMHartState *harts;
    size_t hart_count;

    uint32_t dmcontrol;
    uint32_t abstractcs;
    uint32_t abstractauto;
    uint32_t command;
    uint32_t data[DM_DATA_COUNT];
    uint32_t progbuf[DM_PROGBUF_COUNT];
    uint32_t abstract[DM_ABSTRACT_COUNT];
    uint32_t command_hartid;
    bool command_active;

    RISCVDMSystemBus sba;
    RISCVDTMState dtm;
    bool reset_notifier_registered;
    MemoryRegion debug_mem;
    bool debug_mem_mapped;
    uint32_t rom_whereto;
    uint8_t rom_flags[DM_MAX_HARTS];
};

static void riscv_dm_execute_command(RISCVDebugModuleState *s, uint32_t command);
static void riscv_dm_schedule_halt(RISCVDebugModuleState *s,
                                   RISCVDMHartState *hart, uint8_t cause);
static void riscv_dm_schedule_resume(RISCVDebugModuleState *s,
                                     RISCVDMHartState *hart);
static void riscv_dm_reset_module(RISCVDebugModuleState *s);
static void riscv_dm_note_hart_reset(RISCVDMHartState *hart);
static void riscv_dm_reset_hart(RISCVDebugModuleState *s,
                                RISCVDMHartState *hart);
static void riscv_dm_machine_reset(void *opaque);
static void riscv_dm_debug_mem_map(RISCVDebugModuleState *s);
static void riscv_dm_set_cmderr(RISCVDebugModuleState *s,
                                DMICommandError err);
static RISCVDMHartState *riscv_dm_hart_by_id(RISCVDebugModuleState *s,
                                             uint32_t hartid);
static void riscv_dm_set_flag(RISCVDebugModuleState *s, uint32_t hartid,
                              unsigned flag);
static void riscv_dm_clear_flag(RISCVDebugModuleState *s, uint32_t hartid,
                                unsigned flag);

static uint64_t riscv_dm_debug_mem_read(void *opaque, hwaddr addr,
                                        unsigned size);
static void riscv_dm_debug_mem_write(void *opaque, hwaddr addr,
                                     uint64_t value, unsigned size);
static inline unsigned riscv_dm_arg_words(unsigned xlen)
{
    return xlen == 64 ? 2 : 1;
}

static inline uint64_t riscv_dm_read_arg(RISCVDebugModuleState *s,
                                         unsigned arg, unsigned xlen)
{
    unsigned words = riscv_dm_arg_words(xlen);
    unsigned index = arg * words;
    uint64_t value = s->data[index];

    if (words > 1 && index + 1 < DM_DATA_COUNT) {
        value |= (uint64_t)s->data[index + 1] << 32;
    }
    return value;
}

static inline void riscv_dm_write_arg(RISCVDebugModuleState *s,
                                      unsigned arg, unsigned xlen,
                                      uint64_t value)
{
    unsigned words = riscv_dm_arg_words(xlen);
    unsigned index = arg * words;

    s->data[index] = value & 0xffffffffu;
    if (words > 1 && index + 1 < DM_DATA_COUNT) {
        s->data[index + 1] = (value >> 32) & 0xffffffffu;
    }
}

static inline void riscv_dm_clear_abstract(RISCVDebugModuleState *s)
{
    memset(s->abstract, 0, sizeof(s->abstract));
}

static inline bool riscv_dm_append_insn(RISCVDebugModuleState *s,
                                        unsigned *index, uint32_t insn)
{
    if (*index >= DM_ABSTRACT_COUNT) {
        riscv_dm_set_cmderr(s, CMDERR_OTHER);
        return false;
    }
    s->abstract[*index] = insn;
    (*index)++;
    return true;
}

static const MemoryRegionOps riscv_dm_debug_mem_ops = {
    .read = riscv_dm_debug_mem_read,
    .write = riscv_dm_debug_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void riscv_dm_debug_mem_map(RISCVDebugModuleState *s)
{
    if (s->debug_mem_mapped) {
        return;
    }
    memory_region_init_io(&s->debug_mem, OBJECT(s),
                          &riscv_dm_debug_mem_ops, s,
                          "riscv.dm-debug", DEBUG_ROM_SIZE);
    memory_region_add_subregion(get_system_memory(), DEBUG_START,
                                &s->debug_mem);
    s->debug_mem_mapped = true;
}

static void riscv_dm_debug_rom_halted(RISCVDebugModuleState *s,
                                      uint32_t hartid)
{
    RISCVDMHartState *hart = riscv_dm_hart_by_id(s, hartid);

    if (!hart) {
        return;
    }

    hart->halted = true;
    hart->resumeack = false;
    hart->halt_pending = false;
    qemu_log_mask(LOG_GUEST_ERROR,
                  "dm halted hart=%u go=%u active=%d\n",
                  hartid,
                  !!(s->rom_flags[hartid] & BIT(DEBUG_ROM_FLAG_GO)),
                  s->command_active);
    if (s->command_active && s->command_hartid == hartid &&
        !(s->rom_flags[hartid] & BIT(DEBUG_ROM_FLAG_GO))) {
        s->abstractcs &= ~DM_ABSTRACTCS_BUSY;
        s->command_active = false;
    }
}

static void riscv_dm_debug_rom_resuming(RISCVDebugModuleState *s,
                                        uint32_t hartid)
{
    RISCVDMHartState *hart = riscv_dm_hart_by_id(s, hartid);

    if (!hart) {
        return;
    }

    hart->halted = false;
    hart->resumeack = true;
    hart->resume_pending = false;
    riscv_dm_clear_flag(s, hartid, DEBUG_ROM_FLAG_RESUME);
}

static uint64_t riscv_dm_debug_mem_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    RISCVDebugModuleState *s = opaque;
    uint64_t value = 0;

    if (addr >= DEBUG_ROM_ENTRY &&
        addr + size <= DEBUG_ROM_ENTRY + riscv_debug_rom_raw_len) {
        memcpy(&value, &riscv_debug_rom_raw[addr - DEBUG_ROM_ENTRY], size);
        return extract64(value, 0, size * 8);
    }

    if (addr >= DEBUG_ROM_WHERETO &&
        addr + size <= DEBUG_ROM_WHERETO + sizeof(s->rom_whereto)) {
        memcpy(&value, (uint8_t *)&s->rom_whereto + (addr - DEBUG_ROM_WHERETO),
               size);
        return extract64(value, 0, size * 8);
    }

    if (addr >= DEBUG_ROM_FLAGS &&
        addr + size <= DEBUG_ROM_FLAGS + DM_MAX_HARTS) {
        hwaddr offset = addr - DEBUG_ROM_FLAGS;
        if (offset < DM_MAX_HARTS) {
            return s->rom_flags[offset];
        }
    }

    if (addr >= DM_DEBUG_ABSTRACT_START &&
        addr + size <= DM_DEBUG_ABSTRACT_START + DM_ABSTRACT_COUNT * 4) {
        memcpy(&value,
               (uint8_t *)s->abstract + (addr - DM_DEBUG_ABSTRACT_START), size);
        return extract64(value, 0, size * 8);
    }

    if (DM_PROGBUF_BYTES &&
        addr >= DM_DEBUG_PROGBUF_START &&
        addr + size <= DM_DEBUG_PROGBUF_START + DM_PROGBUF_BYTES) {
        memcpy(&value,
               (uint8_t *)s->progbuf + (addr - DM_DEBUG_PROGBUF_START), size);
        return extract64(value, 0, size * 8);
    }

    if (addr >= DM_DATA_BASE &&
        addr + size <= DM_DATA_BASE + DM_DATA_BYTES) {
        memcpy(&value, (uint8_t *)s->data + (addr - DM_DATA_BASE), size);
        return extract64(value, 0, size * 8);
    }

    return 0;
}

static void riscv_dm_debug_mem_write(void *opaque, hwaddr addr,
                                     uint64_t value, unsigned size)
{
    RISCVDebugModuleState *s = opaque;
    uint32_t val32 = value;
    uint64_t val64 = value;

    if (size != 4) {
        if (addr >= DM_DATA_BASE &&
            addr + size <= DM_DATA_BASE + DM_DATA_BYTES) {
            memcpy((uint8_t *)s->data + (addr - DM_DATA_BASE), &val64, size);
        }
        return;
    }

    switch (addr) {
    case DEBUG_ROM_HALTED:
        riscv_dm_debug_rom_halted(s, val32);
        break;
    case DEBUG_ROM_GOING:
        riscv_dm_clear_flag(s, val32, DEBUG_ROM_FLAG_GO);
        qemu_log_mask(LOG_GUEST_ERROR, "dm going hart=%u\n", val32);
        break;
    case DEBUG_ROM_RESUMING:
        riscv_dm_debug_rom_resuming(s, val32);
        break;
    case DEBUG_ROM_EXCEPTION:
        riscv_dm_set_cmderr(s, CMDERR_EXCEPTION);
        break;
    default:
        if (addr >= DM_DATA_BASE &&
            addr + size <= DM_DATA_BASE + DM_DATA_BYTES) {
            memcpy((uint8_t *)s->data + (addr - DM_DATA_BASE), &val32, size);
        }
        break;
    }
}

static inline uint32_t dmcontrol_hartsel(uint32_t value)
{
    uint32_t hi = (value & DM_DMCONTROL_HARTSELHI_MASK) >>
                  DM_DMCONTROL_HARTSELHI_SHIFT;
    uint32_t lo = (value & DM_DMCONTROL_HARTSELLO_MASK) >>
                  DM_DMCONTROL_HARTSELLO_SHIFT;
    return (hi << (DM_DMCONTROL_HARTSELLO_SHIFT - DM_DMCONTROL_HARTSELHI_SHIFT))
           | lo;
}

static inline bool dmcs_is_busy(DMICommandError err, bool busy)
{
    return busy || err == CMDERR_BUSY;
}

static inline unsigned riscv_dm_selected_hartid(RISCVDebugModuleState *s)
{
    return dmcontrol_hartsel(s->dmcontrol);
}

static inline size_t riscv_dm_hart_index(RISCVDebugModuleState *s,
                                         RISCVDMHartState *hart)
{
    return hart ? (hart - s->harts) : DM_MAX_HARTS;
}

static void riscv_dm_kick_hart(RISCVDMHartState *hart)
{
    if (!hart) {
        return;
    }
    qemu_cpu_kick(hart->cs);
}

static void riscv_dm_set_flag(RISCVDebugModuleState *s, uint32_t hartid,
                              unsigned flag)
{
    RISCVDMHartState *hart;

    if (hartid >= DM_MAX_HARTS) {
        return;
    }
    s->rom_flags[hartid] |= BIT(flag);
    hart = riscv_dm_hart_by_id(s, hartid);
    riscv_dm_kick_hart(hart);
}

static void riscv_dm_clear_flag(RISCVDebugModuleState *s, uint32_t hartid,
                                unsigned flag)
{
    if (hartid >= DM_MAX_HARTS) {
        return;
    }
    s->rom_flags[hartid] &= ~BIT(flag);
}

static RISCVDMHartState *riscv_dm_hart_by_id(RISCVDebugModuleState *s,
                                             uint32_t hartid)
{
    size_t i;

    for (i = 0; i < s->hart_count; i++) {
        if (s->harts[i].hartid == hartid) {
            return &s->harts[i];
        }
    }
    return NULL;
}

static RISCVDMHartState *riscv_dm_selected_hart(RISCVDebugModuleState *s)
{
    uint32_t hartsel = dmcontrol_hartsel(s->dmcontrol);

    return riscv_dm_hart_by_id(s, hartsel);
}

static void riscv_dm_clear_errors(RISCVDebugModuleState *s)
{
    s->abstractcs &= ~DM_ABSTRACTCS_CMDERR_MASK << DM_ABSTRACTCS_CMDERR_SHIFT;
}

static void riscv_dm_set_cmderr(RISCVDebugModuleState *s, DMICommandError err)
{
    if (((s->abstractcs >> DM_ABSTRACTCS_CMDERR_SHIFT) &
         DM_ABSTRACTCS_CMDERR_MASK) != CMDERR_NONE) {
        return;
    }
    s->abstractcs &= ~(DM_ABSTRACTCS_CMDERR_MASK << DM_ABSTRACTCS_CMDERR_SHIFT);
    s->abstractcs |= ((uint32_t)err & DM_ABSTRACTCS_CMDERR_MASK)
                     << DM_ABSTRACTCS_CMDERR_SHIFT;
}

static void riscv_dm_enter_debug_mode(CPUState *cs, run_on_cpu_data data)
{
    RISCVDMHartState *hart = data.host_ptr;
    RISCVCPU *cpu = hart->cpu;
    CPURISCVState *env = &cpu->env;

    if (!hart->halt_pending) {
        return;
    }

    hart->halt_pending = false;
    if (hart->halted) {
        return;
    }

    riscv_cpu_enter_debug_mode(env, env->pc, hart->halt_cause);
    cs->halted = true;
    hart->halted = true;
    hart->resumeack = false;
    env->pc = DEBUG_ROM_ENTRY;
    fprintf(stderr, "dm entered debug hart=%u\n", hart->hartid);
}

static void riscv_dm_schedule_halt(RISCVDebugModuleState *s,
                                   RISCVDMHartState *hart, uint8_t cause)
{
    if (!hart || hart->halted) {
        return;
    }
    hart->halt_pending = true;
    fprintf(stderr, "dm schedule halt hart=%u\n", hart->hartid);
    hart->halt_cause = cause;
    cpu_exit(hart->cs);
    async_run_on_cpu(hart->cs, riscv_dm_enter_debug_mode,
                     RUN_ON_CPU_HOST_PTR(hart));
}

static void riscv_dm_schedule_resume(RISCVDebugModuleState *s,
                                     RISCVDMHartState *hart)
{
    uint32_t hartid;

    if (!hart || !hart->halted) {
        return;
    }
    hart->resume_pending = true;
    hartid = hart->hartid;
    riscv_dm_set_flag(s, hartid, DEBUG_ROM_FLAG_RESUME);
}

static void riscv_dm_start_abstract_command(RISCVDebugModuleState *s,
                                            RISCVDMHartState *hart)
{
    uint32_t hartid = hart->hartid;

    s->abstractcs |= DM_ABSTRACTCS_BUSY;
    s->command_hartid = hartid;
    s->command_active = true;
    s->rom_whereto = riscv_insn_jal(RISCV_REG_ZERO, DM_ROM_JUMP_OFFSET);
    riscv_dm_set_flag(s, hartid, DEBUG_ROM_FLAG_GO);
}

static uint32_t riscv_dm_dmstatus(RISCVDebugModuleState *s)
{
    RISCVDMHartState *hart = riscv_dm_selected_hart(s);
    bool selected = hart != NULL;
    bool anyhalted = selected && hart->halted;
    bool allhalted = selected && hart->halted;
    bool anyrunning = selected && !hart->halted;
    bool allrunning = selected && !hart->halted;
    bool anyresumeack = selected && hart->resumeack;
    bool allresumeack = selected && hart->resumeack;
    bool anyhavereset = selected && hart->havereset;
    bool allhavereset = selected && hart->havereset;
    bool nonexistent = !selected;

    uint32_t value = DM_DMSTATUS_AUTHENTICATED | DM_DMSTATUS_IMPEBREAK |
                     DM_DMSTATUS_HASRESETHALTREQ;
    value |= (3 & DM_DMSTATUS_VERSION_MASK);
    if (anyhalted) {
        value |= DM_DMSTATUS_ANYHALTED;
    }
    if (allhalted) {
        value |= DM_DMSTATUS_ALLHALTED;
    }
    if (anyrunning) {
        value |= DM_DMSTATUS_ANYRUNNING;
    }
    if (allrunning) {
        value |= DM_DMSTATUS_ALLRUNNING;
    }
    if (anyresumeack) {
        value |= DM_DMSTATUS_ANYRESUMEACK;
    }
    if (allresumeack) {
        value |= DM_DMSTATUS_ALLRESUMEACK;
    }
    if (anyhavereset) {
        value |= DM_DMSTATUS_ANYHAVERESET;
    }
    if (allhavereset) {
        value |= DM_DMSTATUS_ALLHAVERESET;
    }
    if (nonexistent) {
        value |= DM_DMSTATUS_ANYNONEXISTENT | DM_DMSTATUS_ALLNONEXISTENT;
    }
    return value;
}

static void riscv_dm_note_hart_reset(RISCVDMHartState *hart)
{
    if (!hart) {
        return;
    }

    hart->havereset = true;
    hart->halted = false;
    hart->resumeack = false;
    hart->halt_pending = false;
    hart->resume_pending = false;
    hart->halt_cause = DCSR_CAUSE_OTHER;

    if (hart->resethaltreq && hart->dm) {
        riscv_dm_schedule_halt(hart->dm, hart, DCSR_CAUSE_RESET);
    }
}

static void riscv_dm_reset_module(RISCVDebugModuleState *s)
{
    size_t i;

    s->dmcontrol = 0;
    s->abstractcs = (DM_PROGBUF_COUNT & DM_ABSTRACTCS_PROGBUFSIZE_MASK)
                    << DM_ABSTRACTCS_PROGBUFSIZE_SHIFT;
    s->abstractcs |= (DM_DATA_COUNT & DM_ABSTRACTCS_DATACOUNT_MASK);
    s->abstractauto = 0;
    s->command = 0;
    for (i = 0; i < DM_DATA_COUNT; i++) {
        s->data[i] = 0;
    }
    for (i = 0; i < DM_PROGBUF_COUNT; i++) {
        s->progbuf[i] = 0;
    }
    for (i = 0; i < DM_ABSTRACT_COUNT; i++) {
        s->abstract[i] = 0;
    }
    s->command_active = false;
    s->command_hartid = 0;
    for (i = 0; i < s->hart_count; i++) {
        s->harts[i].resethaltreq = false;
        s->harts[i].halt_pending = false;
        s->harts[i].resume_pending = false;
        s->harts[i].resumeack = false;
    }
    memset(&s->sba, 0, sizeof(s->sba));
    s->sba.sbcs = (1 << DM_SBCS_SBVERSION_SHIFT) |
                  (64 << DM_SBCS_SBASIZE_SHIFT) |
                  DM_SBCS_SBACCESS32 | DM_SBCS_SBACCESS64;
    memset(s->rom_flags, 0, sizeof(s->rom_flags));
    s->rom_whereto = riscv_insn_jal(RISCV_REG_ZERO, DM_ROM_JUMP_OFFSET);
}

static void riscv_dm_machine_reset(void *opaque)
{
    RISCVDebugModuleState *s = opaque;
    size_t i;

    for (i = 0; i < s->hart_count; i++) {
        riscv_dm_note_hart_reset(&s->harts[i]);
    }
    memset(s->rom_flags, 0, sizeof(s->rom_flags));
    s->rom_whereto = riscv_insn_jal(RISCV_REG_ZERO, DM_ROM_JUMP_OFFSET);
    s->command_active = false;
}

static void riscv_dm_reset_hart(RISCVDebugModuleState *s,
                                RISCVDMHartState *hart)
{
    if (!hart) {
        return;
    }

    cpu_reset(hart->cs);
    riscv_dm_note_hart_reset(hart);
}

static void riscv_dm_try_autoexec(RISCVDebugModuleState *s, unsigned bit)
{
    if (!(s->abstractauto & BIT(bit))) {
        return;
    }
    if (s->abstractcs & DM_ABSTRACTCS_BUSY) {
        riscv_dm_set_cmderr(s, CMDERR_BUSY);
        return;
    }
    riscv_dm_execute_command(s, s->command);
}

static uint64_t riscv_dm_sba_address(RISCVDebugModuleState *s)
{
    uint64_t addr = s->sba.sbaddress[0];

    addr |= (uint64_t)s->sba.sbaddress[1] << 32;
    return addr;
}

static void riscv_dm_sba_set_address(RISCVDebugModuleState *s, uint64_t addr)
{
    s->sba.sbaddress[0] = addr & 0xffffffffu;
    s->sba.sbaddress[1] = (addr >> 32) & 0xffffffffu;
}

static unsigned riscv_dm_sba_access_bits(RISCVDebugModuleState *s)
{
    unsigned sbaccess = (s->sba.sbcs >> DM_SBCS_SBACCESS_SHIFT) &
                        DM_SBCS_SBACCESS_MASK;

    switch (sbaccess) {
    case 0:
        return 8;
    case 1:
        return 16;
    case 2:
        return 32;
    case 3:
        return 64;
    default:
        return 0;
    }
}

static void riscv_dm_sba_autoincrement(RISCVDebugModuleState *s,
                                       unsigned bits)
{
    uint64_t addr;

    if (!(s->sba.sbcs & DM_SBCS_SBAUTOINCREMENT)) {
        return;
    }

    addr = riscv_dm_sba_address(s);
    addr += bits / 8;
    riscv_dm_sba_set_address(s, addr);
}

static void riscv_dm_sba_set_error(RISCVDebugModuleState *s, unsigned code)
{
    s->sba.sbcs &= ~(DM_SBCS_SBERROR_MASK << DM_SBCS_SBERROR_SHIFT);
    s->sba.sbcs |= (code & DM_SBCS_SBERROR_MASK) << DM_SBCS_SBERROR_SHIFT;
}

static bool riscv_dm_sba_perform_read(RISCVDebugModuleState *s)
{
    uint8_t buf[8] = { 0 };
    uint64_t addr = riscv_dm_sba_address(s);
    unsigned bits = riscv_dm_sba_access_bits(s);
    MemTxResult res;

    if (bits == 0 || bits > 64) {
        riscv_dm_sba_set_error(s, 4);
        return false;
    }

    res = address_space_read(&address_space_memory, addr,
                             MEMTXATTRS_UNSPECIFIED, buf, bits / 8);
    if (res != MEMTX_OK) {
        riscv_dm_sba_set_error(s, 2);
        return false;
    }

    s->sba.sbdata[0] = ldl_le_p(buf);
    if (bits > 32) {
        s->sba.sbdata[1] = ldl_le_p(buf + 4);
    }

    riscv_dm_sba_autoincrement(s, bits);
    return true;
}

static bool riscv_dm_sba_perform_write(RISCVDebugModuleState *s)
{
    uint8_t buf[8] = { 0 };
    uint64_t addr = riscv_dm_sba_address(s);
    unsigned bits = riscv_dm_sba_access_bits(s);
    MemTxResult res;

    if (bits == 0 || bits > 64) {
        riscv_dm_sba_set_error(s, 4);
        return false;
    }

    stl_le_p(buf, s->sba.sbdata[0]);
    if (bits > 32) {
        stl_le_p(buf + 4, s->sba.sbdata[1]);
    }

    res = address_space_write(&address_space_memory, addr,
                              MEMTXATTRS_UNSPECIFIED, buf, bits / 8);
    if (res != MEMTX_OK) {
        riscv_dm_sba_set_error(s, 2);
        return false;
    }

    riscv_dm_sba_autoincrement(s, bits);
    return true;
}

static void riscv_dm_access_register(RISCVDebugModuleState *s, uint32_t command)
{
    RISCVDMHartState *hart = riscv_dm_selected_hart(s);
    unsigned aarsize = (command >> AC_ACCESS_REGISTER_AARSIZE_SHIFT) &
                       AC_ACCESS_REGISTER_AARSIZE_MASK;
    unsigned regno = command & AC_ACCESS_REGISTER_REGNO_MASK;
    bool transfer = command & AC_ACCESS_REGISTER_TRANSFER;
    bool write = command & AC_ACCESS_REGISTER_WRITE;
    bool postexec = command & AC_ACCESS_REGISTER_POSTEXEC;
    bool postinc = command & AC_ACCESS_REGISTER_AARPOSTINCREMENT;
    unsigned xlen_bits;
    unsigned idx = 0;

    if (!hart) {
        riscv_dm_set_cmderr(s, CMDERR_OTHER);
        return;
    }

    if (!hart->halted) {
        riscv_dm_set_cmderr(s, CMDERR_HALTRESUME);
        return;
    }
    if (postexec || postinc || !transfer) {
        riscv_dm_set_cmderr(s, CMDERR_NOTSUP);
        return;
    }

    switch (riscv_cpu_mxl(&hart->cpu->env)) {
    case MXL_RV64:
        xlen_bits = 64;
        break;
    case MXL_RV32:
    default:
        xlen_bits = 32;
        break;
    }

    if ((xlen_bits == 32 && aarsize != 2) ||
        (xlen_bits == 64 && aarsize != 3)) {
        riscv_dm_set_cmderr(s, CMDERR_NOTSUP);
        return;
    }

    riscv_dm_clear_abstract(s);
    if (!riscv_dm_append_insn(s, &idx,
                              riscv_insn_csrw(CSR_DSCRATCH0, RISCV_REG_S0))) {
        return;
    }

    if (regno < 0x1000) {
        if (write) {
            if (!riscv_dm_append_insn(s, &idx,
                                      xlen_bits == 64 ?
                                      riscv_insn_ld(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE) :
                                      riscv_insn_lw(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE))) {
                return;
            }
            if (!riscv_dm_append_insn(s, &idx,
                                      riscv_insn_csrw(regno, RISCV_REG_S0))) {
                return;
            }
        } else {
            if (!riscv_dm_append_insn(s, &idx,
                                      riscv_insn_csrr(RISCV_REG_S0, regno))) {
                return;
            }
            if (!riscv_dm_append_insn(s, &idx,
                                      xlen_bits == 64 ?
                                      riscv_insn_sd(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE) :
                                      riscv_insn_sw(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE))) {
                return;
            }
        }
    } else if (regno >= 0x1000 && regno < 0x1020) {
        unsigned reg = regno - 0x1000;

        if (write) {
            if (!riscv_dm_append_insn(s, &idx,
                                      xlen_bits == 64 ?
                                      riscv_insn_ld(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE) :
                                      riscv_insn_lw(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE))) {
                return;
            }
            if (!riscv_dm_append_insn(s, &idx,
                                      riscv_insn_addi(reg, RISCV_REG_S0, 0))) {
                return;
            }
        } else {
            if (!riscv_dm_append_insn(s, &idx,
                                      riscv_insn_addi(RISCV_REG_S0, reg, 0))) {
                return;
            }
            if (!riscv_dm_append_insn(s, &idx,
                                      xlen_bits == 64 ?
                                      riscv_insn_sd(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE) :
                                      riscv_insn_sw(RISCV_REG_S0,
                                                    RISCV_REG_ZERO,
                                                    DM_DATA_BASE))) {
                return;
            }
        }
    } else {
        riscv_dm_set_cmderr(s, CMDERR_NOTSUP);
        return;
    }

    if (!riscv_dm_append_insn(s, &idx,
                              riscv_insn_csrr(RISCV_REG_S0, CSR_DSCRATCH0))) {
        return;
    }
    if (!riscv_dm_append_insn(s, &idx, riscv_insn_ebreak())) {
        return;
    }
    riscv_dm_start_abstract_command(s, hart);
}

static void riscv_dm_access_memory(RISCVDebugModuleState *s, uint32_t command)
{
    RISCVDMHartState *hart = riscv_dm_selected_hart(s);
    unsigned aamsize = (command >> AC_ACCESS_MEMORY_AAMSIZE_SHIFT) &
                       AC_ACCESS_MEMORY_AAMSIZE_MASK;
    bool postincrement = command & AC_ACCESS_MEMORY_AAMPOSTINCREMENT;
    bool aamvirtual = command & AC_ACCESS_MEMORY_AAMVIRTUAL;
    bool write = command & AC_ACCESS_MEMORY_WRITE;
    unsigned xlen_bits;
    unsigned size_bytes;
    uint64_t addr;
    uint8_t buf[8] = { 0 };
    MemTxResult res;
    uint64_t value;

    if (!hart) {
        riscv_dm_set_cmderr(s, CMDERR_OTHER);
        return;
    }
    if (!hart->halted) {
        riscv_dm_set_cmderr(s, CMDERR_HALTRESUME);
        return;
    }
    if (aamvirtual || aamsize > 3) {
        riscv_dm_set_cmderr(s, CMDERR_NOTSUP);
        return;
    }

    switch (riscv_cpu_mxl(&hart->cpu->env)) {
    case MXL_RV64:
        xlen_bits = 64;
        break;
    case MXL_RV32:
    default:
        xlen_bits = 32;
        break;
    }

    size_bytes = 1u << aamsize;
    addr = riscv_dm_read_arg(s, 1, xlen_bits);
    if (write) {
        unsigned i;

        value = riscv_dm_read_arg(s, 0, xlen_bits);
        for (i = 0; i < size_bytes; i++) {
            buf[i] = (value >> (8 * i)) & 0xff;
        }
        res = address_space_write(&address_space_memory, addr,
                                  MEMTXATTRS_UNSPECIFIED, buf, size_bytes);
    } else {
        unsigned i;

        res = address_space_read(&address_space_memory, addr,
                                 MEMTXATTRS_UNSPECIFIED, buf, size_bytes);
        value = 0;
        for (i = 0; i < size_bytes; i++) {
            value |= (uint64_t)buf[i] << (8 * i);
        }
        riscv_dm_write_arg(s, 0, xlen_bits, value);
    }

    if (res != MEMTX_OK) {
        riscv_dm_set_cmderr(s, CMDERR_OTHER);
        return;
    }

    if (postincrement) {
        addr += size_bytes;
        riscv_dm_write_arg(s, 1, xlen_bits, addr);
    }
}

static void riscv_dm_execute_command(RISCVDebugModuleState *s, uint32_t command)
{
    uint32_t cmdtype = (command >> DM_COMMAND_CMDTYPE_SHIFT) &
                       DM_COMMAND_CMDTYPE_MASK;

    if (!(s->dmcontrol & DM_DMCONTROL_DMACTIVE)) {
        return;
    }
    if (s->abstractcs & DM_ABSTRACTCS_BUSY) {
        riscv_dm_set_cmderr(s, CMDERR_BUSY);
        return;
    }

    switch (cmdtype) {
    case DM_CMD_ACCESS_REGISTER:
        riscv_dm_access_register(s, command);
        break;
    case DM_CMD_ACCESS_MEMORY:
        riscv_dm_access_memory(s, command);
        break;
    default:
        riscv_dm_set_cmderr(s, CMDERR_NOTSUP);
        break;
    }
}

static bool riscv_dm_dmi_read(RISCVDebugModuleState *s, unsigned address,
                              uint32_t *value)
{
    if (address >= DM_DATA0 && address < DM_DATA0 + DM_DATA_COUNT) {
        unsigned index = address - DM_DATA0;

        if (s->abstractcs & DM_ABSTRACTCS_BUSY) {
            riscv_dm_set_cmderr(s, CMDERR_BUSY);
            return false;
        }
        *value = s->data[index];
        riscv_dm_try_autoexec(s, index);
        return true;
    }

    if (address >= DM_PROGBUF0 &&
        address < DM_PROGBUF0 + DM_PROGBUF_COUNT) {
        unsigned index = address - DM_PROGBUF0;

        if (s->abstractcs & DM_ABSTRACTCS_BUSY) {
            riscv_dm_set_cmderr(s, CMDERR_BUSY);
            return false;
        }
        *value = s->progbuf[index];
        return true;
    }

    switch (address) {
    case DM_DMCONTROL:
        *value = s->dmcontrol;
        return true;
    case DM_DMSTATUS:
        *value = riscv_dm_dmstatus(s);
        return true;
    case DM_HARTINFO: {
        uint32_t info = (2 & DM_HARTINFO_NSCRATCH_MASK)
                        << DM_HARTINFO_NSCRATCH_SHIFT;
        info |= DM_HARTINFO_DATAACCESS;
        info |= (DM_DATA_COUNT & DM_HARTINFO_DATASIZE_MASK)
                << DM_HARTINFO_DATASIZE_SHIFT;
        *value = info;
        return true;
    }
    case DM_ABSTRACTCS:
        *value = s->abstractcs;
        return true;
    case DM_COMMAND:
        *value = s->command;
        return true;
    case DM_ABSTRACTAUTO:
        *value = s->abstractauto;
        return true;
    case DM_SBCS:
        *value = s->sba.sbcs;
        return true;
    case DM_SBADDRESS0:
        *value = s->sba.sbaddress[0];
        return true;
    case DM_SBADDRESS1:
        *value = s->sba.sbaddress[1];
        return true;
    case DM_SBDATA0:
        if (s->sba.sbcs & DM_SBCS_SBREADONDATA) {
            riscv_dm_sba_perform_read(s);
        }
        *value = s->sba.sbdata[0];
        return true;
    case DM_SBDATA1:
        *value = s->sba.sbdata[1];
        return true;
    default:
        break;
    }

    return false;
}

static void riscv_dm_write_dmcontrol(RISCVDebugModuleState *s, uint32_t value)
{
    bool dmactive = value & DM_DMCONTROL_DMACTIVE;
    uint32_t hartsel = dmcontrol_hartsel(value);
    bool prev_ndmreset = s->dmcontrol & DM_DMCONTROL_NDMRESET;
    RISCVDMHartState *hart = NULL;

    if (!dmactive) {
        riscv_dm_reset_module(s);
        s->dmcontrol = 0;
        return;
    }

    s->dmcontrol &= ~(DM_DMCONTROL_HARTSELHI_MASK | DM_DMCONTROL_HARTSELLO_MASK |
                      DM_DMCONTROL_HARTRESET | DM_DMCONTROL_NDMRESET |
                      DM_DMCONTROL_HALTREQ | DM_DMCONTROL_RESUMEREQ |
                      DM_DMCONTROL_HASEL);
    s->dmcontrol |= value & (DM_DMCONTROL_HARTSELHI_MASK |
                             DM_DMCONTROL_HARTSELLO_MASK |
                             DM_DMCONTROL_HARTRESET |
                             DM_DMCONTROL_NDMRESET |
                             DM_DMCONTROL_HALTREQ |
                             DM_DMCONTROL_RESUMEREQ);
    s->dmcontrol |= DM_DMCONTROL_DMACTIVE;

    hart = riscv_dm_hart_by_id(s, hartsel);

    if ((value & DM_DMCONTROL_ACKHAVERESET) && hart) {
        hart->havereset = false;
    }
    if ((value & DM_DMCONTROL_SETRESETHALTREQ) && hart) {
        hart->resethaltreq = true;
    }
    if ((value & DM_DMCONTROL_CLRRESETHALTREQ) && hart) {
        hart->resethaltreq = false;
    }
    if (value & DM_DMCONTROL_HALTREQ && hart) {
        riscv_dm_schedule_halt(s, hart, DCSR_CAUSE_HALTREQ);
    }
    if (value & DM_DMCONTROL_RESUMEREQ && hart) {
        hart->resumeack = false;
        riscv_dm_schedule_resume(s, hart);
    }
    if (value & DM_DMCONTROL_HARTRESET && hart) {
        riscv_dm_reset_hart(s, hart);
    }
    if ((value & DM_DMCONTROL_NDMRESET) && !prev_ndmreset) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static bool riscv_dm_dmi_write(RISCVDebugModuleState *s, unsigned address,
                               uint32_t value)
{
    if (address >= DM_DATA0 && address < DM_DATA0 + DM_DATA_COUNT) {
        unsigned index = address - DM_DATA0;

        if (s->abstractcs & DM_ABSTRACTCS_BUSY) {
            riscv_dm_set_cmderr(s, CMDERR_BUSY);
            return false;
        }
        s->data[index] = value;
        riscv_dm_try_autoexec(s, index);
        return true;
    }

    if (address >= DM_PROGBUF0 &&
        address < DM_PROGBUF0 + DM_PROGBUF_COUNT) {
        unsigned index = address - DM_PROGBUF0;

        if (s->abstractcs & DM_ABSTRACTCS_BUSY) {
            riscv_dm_set_cmderr(s, CMDERR_BUSY);
            return false;
        }
        s->progbuf[index] = value;
        return true;
    }

    switch (address) {
    case DM_DMCONTROL:
        riscv_dm_write_dmcontrol(s, value);
        return true;
    case DM_ABSTRACTCS:
        if (value & (DM_ABSTRACTCS_CMDERR_MASK << DM_ABSTRACTCS_CMDERR_SHIFT)) {
            riscv_dm_clear_errors(s);
        }
        return true;
    case DM_COMMAND:
        if (!(s->dmcontrol & DM_DMCONTROL_DMACTIVE)) {
            return false;
        }
        if (s->abstractcs & DM_ABSTRACTCS_BUSY) {
            riscv_dm_set_cmderr(s, CMDERR_BUSY);
            return false;
        }
        s->command = value;
        riscv_dm_execute_command(s, value);
        return true;
    case DM_ABSTRACTAUTO:
        s->abstractauto = value;
        return true;
    case DM_SBCS:
        if (value & DM_SBCS_SBBUSYERROR) {
            s->sba.sbcs &= ~DM_SBCS_SBBUSYERROR;
        }
        if (value & (DM_SBCS_SBERROR_MASK << DM_SBCS_SBERROR_SHIFT)) {
            riscv_dm_sba_set_error(s, 0);
        }
        s->sba.sbcs &= ~(DM_SBCS_SBREADONADDR | DM_SBCS_SBAUTOINCREMENT |
                         DM_SBCS_SBREADONDATA | DM_SBCS_SBACCESS_MASK <<
                         DM_SBCS_SBACCESS_SHIFT);
        s->sba.sbcs |= value & (DM_SBCS_SBREADONADDR | DM_SBCS_SBAUTOINCREMENT |
                                DM_SBCS_SBREADONDATA |
                                DM_SBCS_SBACCESS_MASK << DM_SBCS_SBACCESS_SHIFT);
        return true;
    case DM_SBADDRESS0:
        s->sba.sbaddress[0] = value;
        if (s->sba.sbcs & DM_SBCS_SBREADONADDR) {
            riscv_dm_sba_perform_read(s);
        }
        return true;
    case DM_SBADDRESS1:
        s->sba.sbaddress[1] = value;
        if (s->sba.sbcs & DM_SBCS_SBREADONADDR) {
            riscv_dm_sba_perform_read(s);
        }
        return true;
    case DM_SBDATA0:
        s->sba.sbdata[0] = value;
        return riscv_dm_sba_perform_write(s);
    case DM_SBDATA1:
        s->sba.sbdata[1] = value;
        return true;
    default:
        break;
    }

    return false;
}

static void riscv_dtm_reset(RISCVDTMState *dtm)
{
    dtm->state = JTAG_TEST_LOGIC_RESET;
    dtm->busy_stuck = false;
    dtm->rti_remaining = 0;
    dtm->dtmcontrol = (1 & DTMCONTROL_VERSION_MASK) |
        ((DMI_ADDR_BITS & 0x3f) << DTMCONTROL_ABITS_SHIFT);
    dtm->dmi = DMI_STATUS_SUCCESS;
    dtm->ir = DTM_IR_IDCODE;
    dtm->dr = 0;
    dtm->ir_length = 5;
    dtm->dr_length = 0;
    dtm->tck = false;
    dtm->tms = false;
    dtm->tdi = false;
    dtm->tdo = false;
    dtm->bypass = 0;
}

static void riscv_dtm_run_idle(RISCVDTMState *dtm)
{
    if (dtm->rti_remaining > 0) {
        dtm->rti_remaining--;
    }
}

static void riscv_dtm_capture_dr(RISCVDebugModuleState *s)
{
    RISCVDTMState *dtm = &s->dtm;

    switch (dtm->ir) {
    case DTM_IR_IDCODE:
        dtm->dr = 0xdeadbeef;
        dtm->dr_length = 32;
        break;
    case DTM_IR_DTMCONTROL:
        dtm->dr = dtm->dtmcontrol;
        dtm->dr_length = 32;
        break;
    case DTM_IR_DBUS:
        if (dtm->rti_remaining > 0 || dtm->busy_stuck) {
            dtm->dr = DMI_STATUS_BUSY;
            dtm->busy_stuck = true;
        } else {
            dtm->dr = dtm->dmi;
        }
        dtm->dr_length = 34 + DMI_ADDR_BITS;
        break;
    case DTM_IR_BYPASS:
        dtm->dr = dtm->bypass;
        dtm->dr_length = 1;
        break;
    default:
        dtm->dr = 0;
        dtm->dr_length = 1;
        break;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "dtm capture: ir=0x%x value=0x%" PRIx64 " len=%u\n",
                  dtm->ir, dtm->dr, dtm->dr_length);
}

static void riscv_dtm_update_dr(RISCVDebugModuleState *s)
{
    RISCVDTMState *dtm = &s->dtm;

    if (dtm->ir == DTM_IR_DTMCONTROL) {
        if (dtm->dr & DTMCONTROL_DMIRESET) {
            dtm->busy_stuck = false;
        }
        if (dtm->dr & DTMCONTROL_DMIHARDRESET) {
            riscv_dtm_reset(dtm);
        }
        dtm->dtmcontrol &= ~(DTMCONTROL_IDLE_MASK);
        dtm->dtmcontrol |= dtm->dr & DTMCONTROL_IDLE_MASK;
        return;
    }

    if (dtm->ir == DTM_IR_BYPASS) {
        dtm->bypass = dtm->dr & 1;
        return;
    }

    if (dtm->ir != DTM_IR_DBUS || dtm->busy_stuck) {
        return;
    }

    unsigned op = dtm->dr & DMI_OP_MASK;
    uint32_t data = (dtm->dr >> DMI_DATA_SHIFT) & 0xffffffffu;
    unsigned address = dtm->dr >> DMI_ADDRESS_SHIFT;
    bool success = true;

    dtm->dmi = dtm->dr;

    switch (op) {
    case DMI_OP_NOP:
        break;
    case DMI_OP_READ: {
        uint32_t tmp = 0;
        success = riscv_dm_dmi_read(s, address, &tmp);
        if (success) {
            dtm->dmi &= ~((uint64_t)0xffffffffu << DMI_DATA_SHIFT);
            dtm->dmi |= (uint64_t)tmp << DMI_DATA_SHIFT;
        }
        break;
    }
    case DMI_OP_WRITE:
        success = riscv_dm_dmi_write(s, address, data);
        break;
    default:
        success = false;
        break;
    }

    if (success) {
        dtm->dmi &= ~DMI_OP_MASK;
        dtm->dmi |= DMI_STATUS_SUCCESS;
    } else {
        dtm->dmi &= ~DMI_OP_MASK;
        dtm->dmi |= DMI_STATUS_FAILED;
    }

    dtm->rti_remaining = (op == DMI_OP_NOP) ? 0 : 1;
}

static void riscv_dtm_set_pins(RISCVDebugModuleState *s,
                               bool tck, bool tms, bool tdi)
{
    RISCVDTMState *dtm = &s->dtm;
    static const JtagState next_state[16][2] = {
        [JTAG_TEST_LOGIC_RESET] = { JTAG_RUN_TEST_IDLE, JTAG_TEST_LOGIC_RESET },
        [JTAG_RUN_TEST_IDLE] = { JTAG_RUN_TEST_IDLE, JTAG_SELECT_DR_SCAN },
        [JTAG_SELECT_DR_SCAN] = { JTAG_CAPTURE_DR, JTAG_SELECT_IR_SCAN },
        [JTAG_CAPTURE_DR] = { JTAG_SHIFT_DR, JTAG_EXIT1_DR },
        [JTAG_SHIFT_DR] = { JTAG_SHIFT_DR, JTAG_EXIT1_DR },
        [JTAG_EXIT1_DR] = { JTAG_PAUSE_DR, JTAG_UPDATE_DR },
        [JTAG_PAUSE_DR] = { JTAG_PAUSE_DR, JTAG_EXIT2_DR },
        [JTAG_EXIT2_DR] = { JTAG_SHIFT_DR, JTAG_UPDATE_DR },
        [JTAG_UPDATE_DR] = { JTAG_RUN_TEST_IDLE, JTAG_SELECT_DR_SCAN },
        [JTAG_SELECT_IR_SCAN] = { JTAG_CAPTURE_IR, JTAG_TEST_LOGIC_RESET },
        [JTAG_CAPTURE_IR] = { JTAG_SHIFT_IR, JTAG_EXIT1_IR },
        [JTAG_SHIFT_IR] = { JTAG_SHIFT_IR, JTAG_EXIT1_IR },
        [JTAG_EXIT1_IR] = { JTAG_PAUSE_IR, JTAG_UPDATE_IR },
        [JTAG_PAUSE_IR] = { JTAG_PAUSE_IR, JTAG_EXIT2_IR },
        [JTAG_EXIT2_IR] = { JTAG_SHIFT_IR, JTAG_UPDATE_IR },
        [JTAG_UPDATE_IR] = { JTAG_RUN_TEST_IDLE, JTAG_SELECT_DR_SCAN },
    };

    if (!dtm->tck && tck) {
        switch (dtm->state) {
        case JTAG_SHIFT_DR:
            dtm->dr >>= 1;
            dtm->dr |= (uint64_t)dtm->tdi << (dtm->dr_length - 1);
            break;
        case JTAG_SHIFT_IR:
            dtm->ir >>= 1;
            dtm->ir |= dtm->tdi << (dtm->ir_length - 1);
            break;
        default:
            break;
        }
        dtm->state = next_state[dtm->state][dtm->tms];
    } else {
        switch (dtm->state) {
        case JTAG_RUN_TEST_IDLE:
            riscv_dtm_run_idle(dtm);
            break;
        case JTAG_TEST_LOGIC_RESET:
            dtm->ir = DTM_IR_IDCODE;
            break;
        case JTAG_CAPTURE_DR:
            riscv_dtm_capture_dr(s);
            break;
        case JTAG_SHIFT_DR:
            dtm->tdo = dtm->dr & 1;
            break;
        case JTAG_SHIFT_IR:
            dtm->tdo = dtm->ir & 1;
            break;
        case JTAG_UPDATE_DR:
            riscv_dtm_update_dr(s);
            break;
        default:
            break;
        }
    }

    dtm->tck = tck;
    dtm->tms = tms;
    dtm->tdi = tdi;
}

static bool riscv_dtm_get_tdo(RISCVDebugModuleState *s)
{
    return s->dtm.tdo;
}

static int riscv_dm_chr_can_read(void *opaque)
{
    return DM_MAX_REMOTE_BUF;
}

static void riscv_dm_remote_command(RISCVDebugModuleState *s, uint8_t ch)
{
    if (ch >= '0' && ch <= '7') {
        unsigned val = ch - '0';
        bool tck = val & 0x4;
        bool tms = val & 0x2;
        bool tdi = val & 0x1;

        riscv_dtm_set_pins(s, tck, tms, tdi);
        return;
    }

    switch (ch) {
    case 'R': {
        char out = riscv_dtm_get_tdo(s) ? '1' : '0';
        qemu_chr_fe_write_all(&s->chr, (uint8_t *)&out, 1);
        break;
    }
    case 'r':
        riscv_dtm_reset(&s->dtm);
        break;
    case 'Q':
        qemu_chr_fe_disconnect(&s->chr);
        break;
    case 'B':
    case 'b':
        break;
    default:
        break;
    }
}

static void riscv_dm_chr_read(void *opaque, const uint8_t *buf, int size)
{
    RISCVDebugModuleState *s = opaque;
    int i;

    for (i = 0; i < size; i++) {
        riscv_dm_remote_command(s, buf[i]);
    }
}

static void riscv_dm_chr_event(void *opaque, QEMUChrEvent event)
{
}

static void riscv_debug_module_realize(DeviceState *dev, Error **errp)
{
    RISCVDebugModuleState *s = RISCV_DEBUG_MODULE(dev);

    riscv_dm_reset_module(s);
    riscv_dtm_reset(&s->dtm);
    s->dtm.dm = s;
    riscv_dm_debug_mem_map(s);
    if (!s->reset_notifier_registered) {
        qemu_register_reset(riscv_dm_machine_reset, s);
        s->reset_notifier_registered = true;
    }

    if (qemu_chr_fe_backend_connected(&s->chr)) {
        qemu_chr_fe_set_handlers(&s->chr, riscv_dm_chr_can_read,
                                 riscv_dm_chr_read, riscv_dm_chr_event,
                                 NULL, s, NULL, true);
    }
}

static void riscv_debug_module_unrealize(DeviceState *dev)
{
    RISCVDebugModuleState *s = RISCV_DEBUG_MODULE(dev);

    if (s->reset_notifier_registered) {
        qemu_unregister_reset(riscv_dm_machine_reset, s);
        s->reset_notifier_registered = false;
    }
    if (s->debug_mem_mapped) {
        memory_region_del_subregion(get_system_memory(), &s->debug_mem);
        s->debug_mem_mapped = false;
    }
    qemu_chr_fe_deinit(&s->chr, true);
    g_free(s->harts);
    s->harts = NULL;
    s->hart_count = 0;
}

static const Property riscv_debug_module_props[] = {
    DEFINE_PROP_CHR("chardev", RISCVDebugModuleState, chr),
};

static void riscv_debug_module_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, riscv_debug_module_props);
    dc->realize = riscv_debug_module_realize;
    dc->unrealize = riscv_debug_module_unrealize;
}

static const TypeInfo riscv_debug_module_typeinfo = {
    .name          = TYPE_RISCV_DEBUG_MODULE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RISCVDebugModuleState),
    .class_init    = riscv_debug_module_class_init,
};

static void riscv_debug_module_register_types(void)
{
    type_register_static(&riscv_debug_module_typeinfo);
}

type_init(riscv_debug_module_register_types)

void riscv_debug_module_add_hart_array(RISCVDebugModuleState *s,
                                       RISCVHartArrayState *array)
{
    size_t old = s->hart_count;
    size_t i;

    if (!array) {
        return;
    }

    s->hart_count += array->num_harts;
    s->harts = g_renew(RISCVDMHartState, s->harts, s->hart_count);

    for (i = 0; i < array->num_harts; i++) {
        RISCVDMHartState *hart = &s->harts[old + i];

        hart->cpu = &array->harts[i];
        hart->cs = CPU(hart->cpu);
        hart->dm = s;
        hart->hartid = hart->cpu->env.mhartid;
        hart->halted = false;
        hart->resumeack = false;
        hart->havereset = true;
        hart->resethaltreq = false;
        hart->halt_pending = false;
        hart->resume_pending = false;
        hart->halt_cause = DCSR_CAUSE_OTHER;
        fprintf(stderr, "dm add hart idx=%zu id=%u\n", old + i, hart->hartid);
    }
}
