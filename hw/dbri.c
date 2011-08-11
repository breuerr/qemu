/*
 * QEMU DBRI audio interface
 *
 * Copyright (c) 2011-2012 Bob Breuer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw.h"
#include "sysbus.h"
#include "sun4m.h"
#include "trace.h"

/*
 *  DBRI (Dual Basic Rate ISDN)
 *  interface with the audio codec (CS4215) in several SPARCstation models
 *    SS20 (internal codec), SS10 (external codec), LX, ...
 *
 *  Documentation at: http://www.freesoft.org/Linux/DBRI/
 *  Linux 2.6 driver: sound/sparc/dbri.[ch]
 *  NetBSD driver: src/sys/dev/sbus/dbri*
 *
 *  Unimplemented:
 *    audio codec
 *    isdn
 *    iommu errors (sbus faults)
 *    monitor pipes (for 8-bit stereo?)
 */


typedef struct DBRIPipeState {
    uint32_t setup, ptr, data;
    uint32_t in_desc, out_desc;
    uint32_t in_next, out_next;
} DBRIPipeState;

typedef struct DBRIState {
    SysBusDevice busdev;
    MemoryRegion container;
    MemoryRegion mem_rom;
    MemoryRegion mem_rom_alias;
    MemoryRegion mem_io;

    qemu_irq irq;
    void *iommu;

    uint32_t pio_default;
    int32_t codec_offset;

    uint32_t reg[2];
    uint32_t pio;
    uint32_t cmdq_ptr;
    uint32_t intq_ptr;
    uint32_t intq_idx;
    uint32_t chi_global_mode;
    uint32_t chi_data_mode;

    DBRIPipeState pipe[32];
} DBRIState;


#define DBRI_ROM_SIZE   0x40
#define DBRI_REG_SIZE   0x40
#define DBRI_REG_OFFSET 0x10000

/* reg0 = status/control */
#define DBRI_COMMAND_VALID      (1 << 15)
#define DBRI_BURST4             (1 << 14)
#define DBRI_CHI_ACTIVATE       (1 << 4)
#define DBRI_FORCE_TIMEOUT      (1 << 3)
#define DBRI_SOFT_RESET         (1 << 0)
#define DBRI_REG0_INIT          (DBRI_BURST4 | DBRI_FORCE_TIMEOUT)

/* reg1 = interrupt status */
#define DBRI_INT_STATUS         (1 << 0)

/* reg2 = PIO (Parallel I/O)
 * 4 bits of I/O: high nibble = output enable, low nibble = value
 *  PIO3: codec D/C, 1=data, 0=control  default = high
 *  PIO2: 1=external speakerbox
 *  PIO1: 0=codec reset                 default = low
 *  PIO0: 1=internal codec
 *
 *  0x09 = SS-20 internal, codec offset 8
 *  0x0c = speakerbox?, codec offset 0
 */
#define DBRI_PIO_EN             0xf0
#define DBRI_PIO3               0x08
#define DBRI_PIO2               0x04
#define DBRI_PIO1               0x02
#define DBRI_PIO0               0x01
#define DBRI_PIO_DEFAULT_INTERNAL   (DBRI_PIO3 | DBRI_PIO0)
#define DBRI_PIO_DEFAULT_EXTERNAL   (DBRI_PIO3 | DBRI_PIO2)
#define DBRI_CODEC_RESET(s)         (((s)->pio & 0x22) != 0x22)
#define DBRI_CODEC_DATA_MODE(s)     (((s)->pio & 0x88) != 0x80)

enum dbri_int_code {    /* interrupt codes */
    INTR_BRDY = 1,      /* Receive buffer ready */
    INTR_MINT = 2,      /* Marker interrupt in TD/RD */
    INTR_EOL  = 5,      /* End of list */
    INTR_CMDI = 6,      /* Command has been read */
    INTR_XCMP = 8,      /* Transmit complete */
    INTR_FXDT = 10,     /* Fixed data change */
    INTR_CHIL = 11,     /* CHI lost frame */
};

/* TD control (descriptor offset 0) */
#define DBRI_TD_EOF             (1 << 31)  /* end of frame */
#define DBRI_TD_FINT            (1 << 15)  /* only valid with EOF */
#define DBRI_TD_MINT            (1 << 14)
#define DBRI_TD_EOF_INT         (DBRI_TD_EOF | DBRI_TD_FINT)
/* TD status (descriptor offset 12) */
#define DBRI_TD_ABORT           (1 << 2)
#define DBRI_TD_TBC             (1 << 0)   /* tx buffer complete */
/* RD status (descriptor offset 0) */
#define DBRI_RD_EOF             (1 << 31)
#define DBRI_RD_COM             (1 << 30)  /* completed buffer */
#define DBRI_RD_ABORT           (1 << 5)
/* RD control (descriptor offset 12) */
#define DBRI_RD_FINT            (1 << 15)
#define DBRI_RD_MINT            (1 << 14)

/* ctrl is from the TX/RX descriptors */
#define DBRI_TXBUF_LEN(s)       (((s)->play.ctrl >> 16) & 0x1fff)
#define DBRI_RXBUF_LEN(s)       ((s)->rec.ctrl & 0x1fff)

/* pipe setup from SDP command */
#define SDP_CHANGE              (2 << 18)  /* report any changes */
#define SDP_EOL                 (1 << 17)
#define SDP_MODE_MASK           (7 << 13)
#define SDP_MODE_FIXED          (6 << 13)
#define SDP_MODE_MEM            (0 << 13)
#define SDP_DIR_OUT             (1 << 12)  /* direction */
#define SDP_MSB                 (1 << 11)  /* bit order within byte */
#define SDP_PTR_VALID           (1 << 10)
#define SDP_ABORT               (1 << 8)
#define SDP_CLEAR               (1 << 7)

#define SDP_MODE(setup)             ((setup) & SDP_MODE_MASK)
#define SDP_REPORT_CHANGE(setup)    ((setup) & SDP_CHANGE)

/* time slot defined by DTS command */
#define DTS_VIN                 (1 << 17)  /* valid in */
#define DTS_VOUT                (1 << 16)  /* valid out */
#define DTS_INSERT              (1 << 15)  /* 1=insert, 0=delete */
#define DTS_PIPE_IN_PREV(dts)   (((dts) >> 10) & 0x1f)
#define DTS_PIPE_OUT_PREV(dts)  (((dts) >> 5) & 0x1f)
/* timeslot fields for in and out slots */
#define SLOT_LEN(ts)            (((ts) >> 24) & 0xff)
#define SLOT_START(ts)          (((ts) >> 14) & 0x3ff)
#define SLOT_MODE(ts)           (((ts) >> 10) & 0x7)
#define SLOT_MON(ts)            (((ts) >> 5) & 0x1f)
#define SLOT_NEXT(ts)           ((ts) & 0x1f)

/* CHI data mode */
#define DBRI_CHI_XEN            (1 << 1)  /* transmit enable */
#define DBRI_CHI_REN            (1 << 0)  /* receive enable */
/* CHI global mode from CHI command */
#define DBRI_CHI_CLOCK(s)       (((s)->chi_global_mode >> 16) & 0xff)
#define DBRI_CHI_IS_MASTER(s)   (DBRI_CHI_CLOCK(s) >= 3)


static uint32_t dbri_dma_read32(DBRIState *s, uint32_t addr)
{
    uint32_t val;

    sparc_iommu_memory_read(s->iommu, addr, (uint8_t *)&val, 4);
    val = be32_to_cpu(val);
    trace_dbri_dma_read32(addr, val);
    return val;
}

static void dbri_dma_write32(DBRIState *s, uint32_t addr, uint32_t val)
{
    trace_dbri_dma_write32(addr, val);
    val = cpu_to_be32(val);
    sparc_iommu_memory_write(s->iommu, addr, (uint8_t *)&val, 4);
}

static const char *intr_code[16] = {
    "code 0", "BRDY", "MINT", "IBEG", "IEND", "EOL", "CMDI", "code 7",
    "XCMP", "SBRI", "FXDT", "CHIL/COLL", "DBYT", "RBYT", "LINT", "UNDR"
};

static void dbri_post_interrupt(DBRIState *s, unsigned int chan,
                                enum dbri_int_code code, uint32_t field)
{
    uint32_t addr, val;

    addr = s->intq_ptr;
    if (!addr) {
        /* disabled */
        return;
    }

    if (s->intq_idx == 64) {
        /* read next interrupt block from 1st word */
        addr = dbri_dma_read32(s, addr);
        if (!addr) {
            /* out of space? */
            return;
        }
        s->intq_ptr = addr;
        s->intq_idx = 1;
    }

    field &= 0xfffff;
    val = 0x80000000 | (chan << 24) | (code << 20) | field;

    trace_dbri_interrupt(val, intr_code[code], chan, field);

    addr += 4 * s->intq_idx;
    s->intq_idx++;

    dbri_dma_write32(s, addr, val);
    s->reg[1] |= DBRI_INT_STATUS;
    qemu_irq_raise(s->irq);
}

static void dbri_cmd_pause(DBRIState *s, uint32_t *cmd)
{
}

static void dbri_cmd_jump(DBRIState *s, uint32_t *cmd)
{
    s->cmdq_ptr = cmd[1];
}

/* initialize interrupt queue */
static void dbri_cmd_iiq(DBRIState *s, uint32_t *cmd)
{
    s->intq_ptr = cmd[1];
    s->intq_idx = 1;
}

/* setup data pipe, set data pointer */
static void dbri_cmd_sdp(DBRIState *s, uint32_t *cmd)
{
    unsigned int i = cmd[0] & 0x1f;

    s->pipe[i].setup = cmd[0];
    if (cmd[0] & SDP_PTR_VALID) {
        trace_dbri_setup_data_pointer(i, cmd[1]);
        s->pipe[i].ptr = cmd[1];
    }
}

/* continue data pipe */
static void dbri_cmd_cdp(DBRIState *s, uint32_t *cmd)
{
}

/* define time slot */
static void dbri_cmd_dts(DBRIState *s, uint32_t *cmd)
{
    unsigned int prev, next;
    unsigned int i = cmd[0] & 0x1f;

    if (cmd[0] & DTS_VIN) {
        prev = DTS_PIPE_IN_PREV(cmd[0]);
        if (cmd[0] & DTS_INSERT) {
            next = SLOT_NEXT(cmd[1]);

            s->pipe[i].in_next = next;
            s->pipe[prev].in_next = i;
            s->pipe[i].in_desc = cmd[1];

            trace_dbri_insert_time_slot_in(i, prev, next,
                SLOT_START(cmd[1]), SLOT_LEN(cmd[1]));
        } else {
            /* delete */
            next = s->pipe[i].in_next;
            s->pipe[prev].in_next = next;

            trace_dbri_delete_time_slot_in(i, prev);
        }
    }
    if (cmd[0] & DTS_VOUT) {
        prev = DTS_PIPE_OUT_PREV(cmd[0]);
        if (cmd[0] & DTS_INSERT) {
            next = SLOT_NEXT(cmd[2]);

            s->pipe[i].out_next = next;
            s->pipe[prev].out_next = i;
            s->pipe[i].out_desc = cmd[2];

            trace_dbri_insert_time_slot_out(i, prev, next,
                SLOT_START(cmd[2]), SLOT_LEN(cmd[2]));
        } else {
            /* delete */
            next = s->pipe[i].out_next;
            s->pipe[prev].out_next = next;

            trace_dbri_delete_time_slot_out(i, prev);
        }
    }
}

/* set short pipe data */
static void dbri_cmd_ssp(DBRIState *s, uint32_t *cmd)
{
    unsigned int i = cmd[0] & 0x1f;

    /* short pipe only */
    if (i > 16) {
        s->pipe[i].data = cmd[1];
    }
}

/* set CHI global mode */
static void dbri_cmd_chi(DBRIState *s, uint32_t *cmd)
{
    uint32_t status;

    s->chi_global_mode = cmd[0];

    if (s->chi_global_mode & 0x8000) {
        /* report status */
        status = s->chi_data_mode & (DBRI_CHI_XEN | DBRI_CHI_REN);
        dbri_post_interrupt(s, 36, INTR_CHIL, status);
    }
}

/* set CHI data mode */
static void dbri_cmd_cdm(DBRIState *s, uint32_t *cmd)
{
    s->chi_data_mode = cmd[0];
}

static const struct {
    const char *name;
    int len;
    void(*action)(DBRIState *, uint32_t *);
} command_list[16] = {
    { "WAIT",  0, NULL              },
    { "PAUSE", 4, dbri_cmd_pause    },
    { "JUMP",  4, dbri_cmd_jump     },
    { "IIQ",   8, dbri_cmd_iiq      },  /* Initialize Interrupt Queue */
    { "REX",   4, NULL              },  /* Report command EXecution   */
    { "SDP",   8, dbri_cmd_sdp      },  /* Setup Data Pipe            */
    { "CDP",   4, dbri_cmd_cdp      },  /* Continue Data Pipe         */
    { "DTS",  12, dbri_cmd_dts      },  /* Define Time Slot           */
    { "SSP",   8, dbri_cmd_ssp      },  /* Set Short Pipe             */
    { "CHI",   4, dbri_cmd_chi      },  /* Set CHI Global Mode        */
    { "NT",    4, NULL              },
    { "TE",    4, NULL              },
    { "CDEC",  4, NULL              },  /* Codec Setup                */
    { "TEST", 12, NULL              },
    { "CDM",   4, dbri_cmd_cdm      },  /* Set CHI Data Mode          */
    { "Reserved", -1, NULL          }
};

static void dbri_run_commands(DBRIState *s)
{
    uint32_t cmd[3];
    uint32_t val;
    unsigned int i;
    bool cmd_valid;

    for (cmd_valid = true; cmd_valid; ) {
        cmd[0] = dbri_dma_read32(s, s->cmdq_ptr);
        i = cmd[0] >> 28;

        trace_dbri_command(cmd[0], command_list[i].name);

        /* interrupt on command? */
        if (cmd[0] & (1<<27)) {
            val = (i << 16) | (cmd[0] & 0xffff);
            dbri_post_interrupt(s, 38, INTR_CMDI, val);
        }

        switch (command_list[i].len) {
        case 12:
            cmd[2] = dbri_dma_read32(s, s->cmdq_ptr+8);
            /* fall through */
        case 8:
            cmd[1] = dbri_dma_read32(s, s->cmdq_ptr+4);
            /* fall through */
        case 4:
            s->cmdq_ptr += command_list[i].len;
            if (command_list[i].action) {
                command_list[i].action(s, cmd);
            }
            break;
        case 0:
        default:
            s->reg[0] &= ~DBRI_COMMAND_VALID;
            cmd_valid = false;
            break;
        }
    }
}

static void dbri_reset(DeviceState *dev)
{
    DBRIState *s = container_of(dev, DBRIState, busdev.qdev);
    unsigned int i;

    qemu_irq_lower(s->irq);

    /* set defaults */
    s->reg[0] = DBRI_REG0_INIT;
    s->reg[1] = 0;
    if (s->codec_offset) {
        s->pio_default = DBRI_PIO_DEFAULT_INTERNAL;
    } else {
        s->pio_default = DBRI_PIO_DEFAULT_EXTERNAL;
    }
    s->pio = s->pio_default;
    /* reset pointers */
    s->cmdq_ptr = 0;
    s->intq_ptr = 0;

    s->chi_global_mode = 0;
    s->chi_data_mode = 0;

    /* reset linked list */
    s->pipe[16].in_next = 16;
    s->pipe[16].out_next = 16;

    /* clear all pipes */
    for (i = 0; i < 32; i++) {
        s->pipe[i].setup = 0;
        s->pipe[i].ptr = 0;
        s->pipe[i].data = 0;
    }
}

/* registers */
static uint64_t dbri_reg_read(void *opaque, target_phys_addr_t addr,
                              unsigned size)
{
    DBRIState *s = opaque;
    uint32_t val;

    switch (addr) {
    case 0x00: /* Status and Control */
        val = s->reg[0];
        break;
    case 0x04: /* Mode and Interrupt */
        val = s->reg[1];
        if (val) {
            /* clear interrupt status */
            s->reg[1] = 0;
            qemu_irq_lower(s->irq);
        }
        break;
    case 0x08: /* I/O */
        val = s->pio;
        break;
    case 0x20: /* Command Queue Pointer */
        val = s->cmdq_ptr;
        break;
    case 0x24: /* Interrupt Queue Pointer */
        val = s->intq_ptr;
        break;
    default:
        val = 0;
        break;
    }

    trace_dbri_readl(addr, val);

    return val;
}

static void dbri_reg_write(void *opaque, target_phys_addr_t addr,
                           uint64_t val64, unsigned size)
{
    DBRIState *s = opaque;
    uint32_t val = val64;

    trace_dbri_writel(addr, val);

    switch (addr) {
    case 0x00: /* Status and Control */
        s->reg[0] = val;
        if (val & DBRI_SOFT_RESET) {
            dbri_reset(&s->busdev.qdev);
        } else {
            if (val & (DBRI_COMMAND_VALID | DBRI_CHI_ACTIVATE)) {
                dbri_run_commands(s);
            }
        }
        break;
    case 0x08: /* I/O */
        s->pio = (val & DBRI_PIO_EN) ? val : s->pio_default;
        break;
    case 0x20: /* Command Queue Pointer */
        s->cmdq_ptr = val;
        s->reg[0] |= DBRI_COMMAND_VALID;
        dbri_run_commands(s);
        break;
    default:
        break;
    }
}

static int vmstate_dbri_post_load(void *opaque, int version_id)
{
    DBRIState *s = opaque;

    if (s->intq_ptr && s->reg[1]) {
        qemu_irq_raise(s->irq);
    }

    return 0;
}


/* the FCode rom:
 *   start1
 *   " SUNW,DBRIe" device-name
 *   my-address h# 1000 + my-space h# 100 reg
 *   5 sbus-intr>cpu encode-int 0 encode-int encode+ " intr" property
 *   end0
 * creates these properties for SS-20:
 *   intr   00000039  00000000
 *   reg    0000000e  00010000  00000100
 *   name   SUNW,DBRIe
 */
static const uint8_t dbri_rom[DBRI_ROM_SIZE] = {
    0xf1, 0x08, 0x06, 0x83, 0x00, 0x00, 0x00, 0x3e,
    0x12, 0x0a, 0x53, 0x55, 0x4e, 0x57, 0x2c, 0x44,
    0x42, 0x52, 0x49, 0x65, 0x02, 0x01, 0x01, 0x02,
    0x10, 0x00, 0x01, 0x00, 0x00, 0x1e, 0x01, 0x03,
    0x10, 0x00, 0x00, 0x01, 0x00, 0x01, 0x16, 0x10,
    0x00, 0x00, 0x00, 0x05, 0x01, 0x31, 0x01, 0x11,
    0xa5, 0x01, 0x11, 0x01, 0x12, 0x12, 0x04, 0x69,
    0x6e, 0x74, 0x72, 0x01, 0x10, 0x00
};

static const struct MemoryRegionOps dbri_ops = {
    .read = dbri_reg_read,
    .write = dbri_reg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static int dbri_init1(SysBusDevice *dev)
{
    DBRIState *s = FROM_SYSBUS(DBRIState, dev);
    uint32_t size = DBRI_REG_OFFSET + DBRI_REG_SIZE;
    void *rom_ptr;

    memory_region_init_ram(&s->mem_rom, "dbri.rom", DBRI_ROM_SIZE);
    memory_region_set_readonly(&s->mem_rom, true);
    rom_ptr = memory_region_get_ram_ptr(&s->mem_rom);
    memcpy(rom_ptr, dbri_rom, DBRI_ROM_SIZE);

    /* the SS-20 bootrom looks for the rom at 0x1000 instead of 0 */
    memory_region_init_alias(&s->mem_rom_alias, "dbri.rom-alias",
                             &s->mem_rom, 0, DBRI_ROM_SIZE);
    memory_region_set_readonly(&s->mem_rom_alias, true);
    memory_region_init_io(&s->mem_io, &dbri_ops, s, "dbri.regs",
                          DBRI_REG_SIZE);

    memory_region_init(&s->container, "dbri", size);
    memory_region_add_subregion(&s->container, 0, &s->mem_rom);
    memory_region_add_subregion(&s->container, 0x1000, &s->mem_rom_alias);
    memory_region_add_subregion(&s->container, DBRI_REG_OFFSET, &s->mem_io);
    sysbus_init_mmio(dev, &s->container);
    sysbus_init_irq(dev, &s->irq);

    return 0;
}

static const VMStateDescription vmstate_dbri_pipe = {
    .name = "dbri_pipe",
    .version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(setup,    DBRIPipeState),
        VMSTATE_UINT32(ptr,      DBRIPipeState),
        VMSTATE_UINT32(data,     DBRIPipeState),
        VMSTATE_UINT32(in_desc,  DBRIPipeState),
        VMSTATE_UINT32(out_desc, DBRIPipeState),
        VMSTATE_UINT32(in_next,  DBRIPipeState),
        VMSTATE_UINT32(out_next, DBRIPipeState),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_dbri = {
    .name = "dbri",
    .version_id = 1,
    .post_load = vmstate_dbri_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(reg[0],   DBRIState),
        VMSTATE_UINT32(reg[1],   DBRIState),
        VMSTATE_UINT32(pio,      DBRIState),
        VMSTATE_UINT32(cmdq_ptr, DBRIState),
        VMSTATE_UINT32(intq_ptr, DBRIState),
        VMSTATE_UINT32(intq_idx, DBRIState),
        VMSTATE_UINT32(chi_global_mode, DBRIState),
        VMSTATE_UINT32(chi_data_mode, DBRIState),

        VMSTATE_STRUCT_ARRAY(pipe, DBRIState, 32, 1,
                             vmstate_dbri_pipe, DBRIPipeState),
        VMSTATE_END_OF_LIST()
    }
};

static Property dbri_properties[] = {
    DEFINE_PROP_PTR("iommu_opaque",   DBRIState, iommu),
    DEFINE_PROP_INT32("codec_offset", DBRIState, codec_offset, 8),
    DEFINE_PROP_END_OF_LIST(),
};

static void dbri_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = dbri_init1;
    dc->desc  = "Sun DBRI audio interface";
    dc->reset = dbri_reset;
    dc->vmsd = &vmstate_dbri;
    dc->props = dbri_properties;
}

static TypeInfo dbri_info = {
    .name  = "SUNW,DBRIe",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(DBRIState),
    .class_init = dbri_class_init,
};

static void dbri_register_types(void)
{
    type_register_static(&dbri_info);
}

type_init(dbri_register_types)
