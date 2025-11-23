#include <glib.h>
#include <elf.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

typedef struct {
    GByteArray *mstatus;
    GByteArray *mie;
    GByteArray *mip;
    GByteArray *mtvec;
    GByteArray *mcause;
    GByteArray *mepc;
} Register;

typedef struct CPU {
    bool in_ctxswitch;
    uint64_t ctxswitch_counter;
    uint64_t insn_counter;
    uint64_t last_ns;
    int8_t call_depth;
    uint64_t prev_insn_vaddr;

    Register* interrupt_reg;
} CPU;

typedef struct TraceFunctionInfo {
    uint64_t start_addr;
    uint64_t end_addr;
    uint64_t exec_counter;
    const char* func_name;
} TraceFunctionInfo;

static GArray *cpus;
static GRWLock expand_array_lock;

static bool is64 = NULL;
static uint64_t mstatus = 0x300;
static uint64_t mie = 0x304;
static uint64_t mip = 0x344;
static uint64_t mtvec = 0x305;
static uint64_t mcause = 0x342;
static uint64_t mepc = 0x341;

static char *switch_function_name;
static TraceFunctionInfo *switch_func;

static GHashTable *function_entries;

static char *elf_path;

static CPU *get_cpu(int vcpu_index)
{
    CPU *c;
    g_rw_lock_reader_lock(&expand_array_lock);
    c = &g_array_index(cpus, CPU, vcpu_index);
    g_rw_lock_reader_unlock(&expand_array_lock);

    return c;
}

static void load_elf_symbols(void)
{
    if (!elf_path) {
        return ;
    }

    int fd = open(elf_path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to load elf file: %s\n", elf_path);
        return ;
    }

    struct stat st;
    if (fstat(fd, &st) < 0) {
        close(fd);
        return ;
    }

    void *map = mmap(NULL, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);

    unsigned char *e_ident = (unsigned char *)map;
    if (e_ident[EI_MAG0] != ELFMAG0 || e_ident[EI_MAG1] != ELFMAG1 ||
        e_ident[EI_MAG2] != ELFMAG2 || e_ident[EI_MAG3] != ELFMAG3) {
        munmap(map, st.st_size);
        close(fd);
        return ;
    }

    is64 = (e_ident[EI_CLASS] == ELFCLASS64);

    if (is64) {
        Elf64_Ehdr *elf64_hdr = (Elf64_Ehdr*)map;
        Elf64_Shdr *section_hdr = (Elf64_Shdr*)(map + elf64_hdr->e_shoff);
        Elf64_Shdr *symtab = NULL;

        for (int i = 0; i < elf64_hdr->e_shnum; i++) {
            if (section_hdr[i].sh_type == SHT_SYMTAB) {
                symtab = &section_hdr[i];
                break;
            }
        }

        if (symtab) {
            Elf64_Shdr *sym_strtab_hdr = &section_hdr[symtab->sh_link];
            Elf64_Sym *sym = (Elf64_Sym*)(map + symtab->sh_offset);
            char *sym_strtab = (char*)(map + sym_strtab_hdr->sh_offset);
            int sym_count = symtab->sh_size / symtab->sh_entsize;
            for (int j = 0; j < sym_count; j++) {
                unsigned char sym_type = ELF32_ST_TYPE(sym[j].st_info);
                if (sym_type != STT_FUNC) {
                    continue;  
                }
                unsigned char sym_bind = ELF32_ST_BIND(sym[j].st_info);
                if (sym_bind == STB_LOCAL) {
                    continue; 
                }
                if (sym[j].st_size == 0) {
                    continue; 
                }
                const char* sym_name = sym_strtab + sym[j].st_name;
                if (sym_name[0] == '\0') {
                    continue;
                }
                
                TraceFunctionInfo *func_info = (TraceFunctionInfo*)malloc(sizeof(TraceFunctionInfo));
                func_info->func_name = g_strdup(sym_name);
                func_info->start_addr = sym[j].st_value;
                func_info->end_addr = sym[j].st_value + sym[j].st_size;
                func_info->exec_counter = 0;
                if (g_str_has_prefix(sym_name, switch_function_name)) {
                    switch_func = func_info;
                }
                g_hash_table_insert(function_entries, GUINT_TO_POINTER(sym[j].st_value), func_info);
            }
        }

    } else {
        Elf32_Ehdr *elf32_hdr = (Elf32_Ehdr*)map;
        Elf32_Shdr *section_hdr = (Elf32_Shdr*)(map + elf32_hdr->e_shoff);
        Elf32_Shdr *symtab = NULL;

        for (int i = 0; i < elf32_hdr->e_shnum; i++) {
            if (section_hdr[i].sh_type == SHT_SYMTAB) {
                symtab = &section_hdr[i];
                break;
            }
        }

        if (symtab) {
            Elf32_Shdr *sym_strtab_hdr = &section_hdr[symtab->sh_link];
            Elf32_Sym *sym = (Elf32_Sym*)(map + symtab->sh_offset);
            char *sym_strtab = (char*)(map + sym_strtab_hdr->sh_offset);
            int sym_count = symtab->sh_size / symtab->sh_entsize;
            for (int j = 0; j < sym_count; j++) {
                unsigned char sym_type = ELF32_ST_TYPE(sym[j].st_info);
                if (sym_type != STT_FUNC) {
                    continue;  // 跳过非函数符号
                }
                unsigned char sym_bind = ELF32_ST_BIND(sym[j].st_info);
                if (sym_bind == STB_LOCAL) {
                    continue;  // 跳过局部符号，只保留全局符号
                }
                if (sym[j].st_size == 0) {
                    continue;  // 跳过大小为0的符号
                }
                const char* sym_name = sym_strtab + sym[j].st_name;
                if (sym_name[0] == '\0') {
                    continue;
                }
                TraceFunctionInfo *func_info = (TraceFunctionInfo*)malloc(sizeof(TraceFunctionInfo));
                func_info->func_name = g_strdup(sym_name);
                func_info->start_addr = sym[j].st_value;
                func_info->end_addr = sym[j].st_value + sym[j].st_size;
                func_info->exec_counter = 0;
                if (g_str_has_prefix(sym_name, switch_function_name)) {
                    switch_func = func_info;
                }
                g_hash_table_insert(function_entries, GUINT_TO_POINTER(sym[j].st_value), func_info);
            }
        }
    }
    munmap(map, st.st_size);
    close(fd);
}

static Register* register_init(int vcpu_index) 
{
    Register* reg = g_new0(Register, 1);
    reg->mcause = g_byte_array_new();
    reg->mepc = g_byte_array_new();
    reg->mie = g_byte_array_new();
    reg->mip = g_byte_array_new();
    reg->mstatus = g_byte_array_new();
    reg->mtvec = g_byte_array_new();
    return reg;
}

static void vcpu_init(qemu_plugin_id_t id, unsigned int vcpu_index)
{
    CPU *c;

    g_rw_lock_writer_lock(&expand_array_lock);
    if (vcpu_index >= cpus->len) {
        g_array_set_size(cpus, vcpu_index + 1);
    }
    g_rw_lock_writer_unlock(&expand_array_lock);

    c = get_cpu(vcpu_index);
    c->insn_counter = 0;
    c->last_ns = 0;
    c->prev_insn_vaddr = 0;
    c->ctxswitch_counter = 0;
    c->in_ctxswitch = false;

    c->interrupt_reg = register_init(vcpu_index);
}

static void insn_check_regs(CPU *cpu)
{
    Register *reg = cpu->interrupt_reg;
    int sz;
    qemu_plugin_read_memory_vaddr(mie, reg->mie, sizeof(uint32_t));
    qemu_plugin_read_memory_vaddr(mcause, reg->mcause, sizeof(uint32_t));

    if ((reg->mie & (1 << 3)) ) {
        
    }

    qemu_plugin_read_memory_vaddr(mstatus, reg->mstatus, sizeof(uint32_t));
    qemu_plugin_read_memory_vaddr(mip, reg->mip, sizeof(uint32_t));
    qemu_plugin_read_memory_vaddr(mepc, reg->mepc, sizeof(uint32_t));
    qemu_plugin_read_memory_vaddr(mtvec, reg->mtvec, sizeof(uint32_t));

}

static void insn_exec(unsigned int vcpu_index, void *userdata)
{
    CPU *cpu = get_cpu(vcpu_index);
    uint64_t insn_vaddr = ((uint64_t)(uintptr_t)userdata);

    if (cpu->insn_counter > 0) {
        if (cpu->interrupt_reg) {
            insn_check_regs(cpu);
        }
    }

    if (switch_func) {
        bool is_entry = (insn_vaddr == switch_func->start_addr);

        bool in_range = (insn_vaddr >= switch_func->start_addr &&
                     insn_vaddr < switch_func->end_addr);

        bool prev_in_range = (cpu->prev_insn_vaddr != 0 &&
                          cpu->prev_insn_vaddr >= switch_func->start_addr &&
                          cpu->prev_insn_vaddr < switch_func->end_addr);

        if (is_entry && !cpu->in_ctxswitch) {
            cpu->in_ctxswitch = true;
            cpu->call_depth = 1;
            cpu->last_ns = cpu->insn_counter;
        }

        if (cpu->in_ctxswitch && cpu->prev_insn_vaddr != 0) {

            if (cpu->call_depth == 1 && !in_range && prev_in_range) {
                cpu->ctxswitch_counter++;
                uint64_t insn_counter = cpu->insn_counter - cpu->last_ns;
                char *msg = g_strdup_printf(
                    "Context switch #%" PRIu64 "completed on CPU %u: %" PRIu64 " instruction cost | Fcutinon Name: %s \n",
                    cpu->ctxswitch_counter, vcpu_index, insn_counter, switch_func->func_name
                );
                qemu_plugin_outs(msg);
                g_free(msg);

                cpu->in_ctxswitch = false;
            }

            if (!in_range && prev_in_range) {
                cpu->call_depth++;
            } else if (in_range && !prev_in_range & !is_entry) {
                if (cpu->call_depth > 0) {
                    cpu->call_depth--;
                }
            } 
        }
        cpu->prev_insn_vaddr = insn_vaddr;
    }
    cpu->insn_counter += 1;
    if (cpu->insn_counter > 1000000) {
        exit(0);
    }
}

static void hot_func_exec(unsigned int vcpu_index, void *userdata) 
{
    TraceFunctionInfo *entry_t = (TraceFunctionInfo*) userdata;
    entry_t->exec_counter++;
}

static void vcpu_tb_trans(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    struct qemu_plugin_insn *insn;

    size_t n_insns = qemu_plugin_tb_n_insns(tb);
    for (size_t i = 0; i < n_insns; i++) {
        insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t insn_vaddr = qemu_plugin_insn_vaddr(insn);

        if (function_entries) {
            TraceFunctionInfo *entry_t = g_hash_table_lookup(function_entries, GUINT_TO_POINTER(insn_vaddr));
            if (entry_t) {
                qemu_plugin_register_vcpu_insn_exec_cb(
                    insn, hot_func_exec,
                    QEMU_PLUGIN_CB_NO_REGS,
                    entry_t);
            }
        }

        if (switch_func) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, insn_exec,
                QEMU_PLUGIN_CB_R_REGS,
                (void *)(uintptr_t)(insn_vaddr));
        }
    }
}

static gint compare_entries_by_count(gconstpointer a, gconstpointer b) {
    TraceFunctionInfo *prev = (TraceFunctionInfo*) a;
    TraceFunctionInfo *curr = (TraceFunctionInfo*) b;
    
    return (prev->exec_counter < curr->exec_counter) ? 1 : 
           (prev->exec_counter > curr->exec_counter) ? -1 : 0;
}

static void vcpu_exit(qemu_plugin_id_t id, void *p)
{
    GList *list = g_hash_table_get_values(function_entries);
    GList *sort = g_list_sort(list, compare_entries_by_count);
    char *msg = g_strdup_printf(
        "|              Function Name              |    Start Address    |    End Address    |    Counter    |\n");
    qemu_plugin_outs(msg);
    for (int i = 0; sort && i < 50; i++, sort = sort->next) {
        TraceFunctionInfo* entry = (TraceFunctionInfo*)sort->data;
        msg = g_strdup_printf(
            "%-42s,0x%-19lx,0x%-17lx,%lu \n",
            entry->func_name, entry->start_addr, entry->end_addr, entry->exec_counter
        );
        qemu_plugin_outs(msg);
    }
    g_free(msg);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info, int argc,
                                           char **argv)
{
    cpus = g_array_sized_new(true, true, sizeof(CPU),
                             info->system_emulation ? info->system.max_vcpus : 1);

    function_entries = g_hash_table_new_full(g_direct_hash, g_direct_equal,
                                             NULL, g_free);

    for (int i = 0; i < argc; i++) {
        char *opt = argv[i];
        g_auto(GStrv) tokens = g_strsplit(opt, "=", 2);
        if (g_strcmp0(tokens[0], "swtich") == 0) {
             switch_function_name = g_strdup(tokens[1]);
        } else if (g_strcmp0(tokens[0], "elf") == 0) {
            elf_path = g_strdup(tokens[1]);
        } else {
            fprintf(stderr, "option parsing failed: %s\n", opt);
            return -1;
        }
    }
    load_elf_symbols();

    qemu_plugin_register_vcpu_init_cb(id, vcpu_init);
    qemu_plugin_register_vcpu_tb_trans_cb(id, vcpu_tb_trans);
    qemu_plugin_register_atexit_cb(id, vcpu_exit, NULL);

    return 0;
}
