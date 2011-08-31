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
 *    audio
 *    isdn
 *    iommu errors (sbus faults)
 *    monitor pipes (for 8-bit stereo?)
 */


typedef struct CS4215State {
    int data_mode;
    int freq;

    uint8_t status;
    uint8_t data_format;
    uint8_t port_control;
    uint8_t settings[4];
} CS4215State;

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

    bool pipe_update;

    struct {
        uint32_t pipe, ctrl;
        bool stopped;
    } play, rec;

    CS4215State codec;
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


/* audio codec */
/* control slot 1 = status */
#define CS4215_CLB              (1 << 2)  /* control latch bit */
#define CS4215_STATUS_RWMASK    0x1f
#define CS4215_STATUS_FIXED     0x20      /* upper 3 bits fixed at 001 */
#define CS4215_CLB_CLEAR(st)    (((st) & 0x1b) | 0x20)

/* control slot 2 = data format */
#define CS4215_DFR_MASK         (7 << 3)  /* data frequency rate */
#define CS4215_ST               (1 << 2)  /* stereo */
#define CS4215_DF_MASK          (3 << 0)  /* data format */
#define CS4215_DF_U8            (3 << 0)
#define CS4215_DF_ALAW          (2 << 0)
#define CS4215_DF_ULAW          (1 << 0)
#define CS4215_DF_S16           (0 << 0)
#define CS4215_DFR_EXTRACT(d)   (((d) & CS4215_DFR_MASK) >> 3)

/* control slot 3 = port control */
#define CS4215_MCK_MASK         (7 << 4)  /* clock source select */
#define CS4215_MCK_XTAL2        (2 << 4)
#define CS4215_MCK_XTAL1        (1 << 4)
#define CS4215_BSEL_MASK        (3 << 2)
#define CS4215_BSEL_256         (2 << 2)
#define CS4215_XCLK             (1 << 1)  /* transmit clock master mode */
#define CS4215_XEN              (1 << 0)

#define CODEC_IS_MASTER(c)  \
            ((c)->data_mode && ((c)->port_control & CS4215_XCLK))

/* 2 clocks, 8 clock dividers */
#define AUD_CLK1  24576000
#define AUD_CLK2  16934400

static const int cs4215_clk_div[8] = {
    3072, 1536, 896, 768, 448, 384, 512, 2560
};


/* reverse bits in a byte */
static uint8_t byte_rev(uint8_t b)
{
    b = ((b & 0xf0) >> 4) | ((b & 0x0f) << 4);
    b = ((b & 0xcc) >> 2) | ((b & 0x33) << 2);
    b = ((b & 0xaa) >> 1) | ((b & 0x55) << 1);

    return b;
}

static void cs4215_reset(CS4215State *s)
{
    s->status = CS4215_STATUS_FIXED | CS4215_CLB;
    s->data_format = CS4215_DF_ULAW;
    s->port_control = CS4215_BSEL_256 | CS4215_XEN;
    s->settings[0] = 0x3f;
    s->settings[1] = 0xbf;
    s->settings[2] = 0xc0;
    s->settings[3] = 0xf0;
}

static void cs4215_setmode(CS4215State *s, int mode)
{
    int div;

    s->data_mode = mode;

    if (mode) {
        /* calculate frequency */
        div = cs4215_clk_div[CS4215_DFR_EXTRACT(s->data_format)];

        switch (s->port_control & CS4215_MCK_MASK) {
        case CS4215_MCK_XTAL1:
            s->freq = AUD_CLK1 / div;
            break;
        case CS4215_MCK_XTAL2:
            s->freq = AUD_CLK2 / div;
            break;
        default:
            s->freq = 0;
            break;
        }
    } else {
        s->freq = 0;
        s->status = CS4215_CLB_CLEAR(s->status);
    }
}

static uint8_t cs4215_read(CS4215State *s, int slot)
{
    uint8_t val;

    /* 1-based slot number */
    if (s->data_mode) {
        switch (slot) {
        case 5:
            val = s->settings[0];
            break;
        case 6:
            val = s->settings[1] & 0x7f;
            break;
        case 7:
            val = s->settings[2] & 0xdf;
            break;
        case 8:
            val = s->settings[3];
            break;
        default:
            val = 0;
            break;
        }
        trace_dbri_codec_read_data(slot, val);
    } else {
        switch (slot) {
        case 1:
            val = s->status;
            break;
        case 2:
            val = s->data_format;
            break;
        case 3:
            val = s->port_control;
            break;
        case 4:
            val = 0; /* test */
            break;
        case 5:
            val = 0xc0; /* codec pio */
            break;
        case 7:
            val = 0x02; /* Rev E */
            break;
        default:
            val = 0;
            break;
        }
        trace_dbri_codec_read_control(slot, val);
    }
    return val;
}

static void cs4215_write(CS4215State *s, int slot, uint8_t val)
{
    if (s->data_mode) {
        trace_dbri_codec_write_data(slot, val);
        if (slot >= 5 && slot <= 8) {
            s->settings[slot - 5] = val;
        }
    } else {
        trace_dbri_codec_write_control(slot, val);
        switch (slot) {
        case 1:
            s->status = (val & CS4215_STATUS_RWMASK) | CS4215_STATUS_FIXED;
            break;
        case 2:
            s->data_format = val;
            break;
        case 3:
            s->port_control = val;
            break;
        }
    }
}

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

static void dbri_next_txbuffer(DBRIState *s, unsigned int pipe)
{
    uint32_t td_ptr = s->pipe[pipe].ptr;  /* transmit descriptor */

    s->play.ctrl = dbri_dma_read32(s, td_ptr); /* control/length */
    if (s->play.ctrl & DBRI_TD_MINT) {
        /* marker int */
        dbri_post_interrupt(s, pipe, INTR_MINT, td_ptr);
    }
    /* get data pointer, re-use pipe data field */
    s->pipe[pipe].data = dbri_dma_read32(s, td_ptr+4);
}

static void dbri_next_rxbuffer(DBRIState *s, unsigned int pipe)
{
    uint32_t rd_ptr = s->pipe[pipe].ptr;  /* receive descriptor */

    s->rec.ctrl = dbri_dma_read32(s, rd_ptr+12); /* control/length */
    if (s->rec.ctrl & DBRI_RD_MINT) {
        /* marker int */
        dbri_post_interrupt(s, pipe, INTR_MINT, rd_ptr);
    }
    /* get data pointer, re-use pipe data field */
    s->pipe[pipe].data = dbri_dma_read32(s, rd_ptr+4);
}

static void dbri_start_audio_out(DBRIState *s, unsigned int pipe)
{
    if (pipe == s->play.pipe && !s->play.stopped) {
        return;
    }
    /* prepare data buffer */
    dbri_next_txbuffer(s, pipe);

    s->play.pipe = pipe;
    s->play.stopped = false;
}

static void dbri_start_audio_in(DBRIState *s, unsigned int pipe)
{
    if (pipe == s->rec.pipe && !s->rec.stopped) {
        return;
    }
    /* prepare data buffer */
    dbri_next_rxbuffer(s, pipe);

    s->rec.pipe = pipe;
    s->rec.stopped = false;
}

static void dbri_stop_audio(DBRIState *s, unsigned int pipe, int abort)
{
    if (pipe == s->play.pipe) {
        if (abort && s->pipe[pipe].ptr) {
            dbri_dma_write32(s, s->pipe[pipe].ptr+12, DBRI_TD_ABORT);
        }
        s->play.stopped = true;
        s->play.pipe = 0;
    }
    if (pipe == s->rec.pipe) {
        if (abort && s->pipe[pipe].ptr) {
            dbri_dma_write32(s, s->pipe[pipe].ptr, DBRI_RD_ABORT);
        }
        s->rec.stopped = true;
        s->rec.pipe = 0;
    }
}

static bool dbri_chi_is_active(DBRIState *s)
{
    if ((s->reg[0] & DBRI_CHI_ACTIVATE)
        && (DBRI_CHI_IS_MASTER(s) || CODEC_IS_MASTER(&s->codec))) {
        return true;
    } else {
        return false;
    }
}

static void dbri_run_pipes(DBRIState *s)
{
    unsigned int i;
    int start, len, codec_slot;
    uint32_t val;
    uint32_t been_here;

    if (!dbri_chi_is_active(s)) {
        return;
    }

    /* run through the pipes */
    s->pipe_update = false;

    /* pipe 16 is the anchor where the CHI starts and ends */
    if (s->chi_data_mode & DBRI_CHI_XEN) {
        i = s->pipe[16].out_next;
        been_here = 1 << 16;
        while (i != 16) {
            if (been_here & (1 << i)) {
                trace_dbri_pipe_out_list_error();
                break;
            }
            been_here |= 1 << i;
            start = SLOT_START(s->pipe[i].out_desc);
            len = SLOT_LEN(s->pipe[i].out_desc);

            /* use bits per frame instead of 0 for OUT pipes */
            if (start >= 64) {
                start = 0;
            }
            trace_dbri_pipe_out_list(i, s->pipe[i].out_desc, start, len);

            switch (SDP_MODE(s->pipe[i].setup)) {
            case SDP_MODE_MEM:
                if (s->pipe[i].ptr && start == s->codec_offset) {
                    dbri_start_audio_out(s, i);
                }
                break;

            case SDP_MODE_FIXED:
                /* assume 8-bit alignment */
                codec_slot = 1 + (start - s->codec_offset) / 8;

                /* fixed pipe is LSB first, codec is MSB first */
                val = s->pipe[i].data;
                do {
                    cs4215_write(&s->codec, codec_slot, byte_rev(val & 0xff));
                    codec_slot++;
                    val >>= 8;
                    len -= 8;
                } while (len > 0);
                break;
            }
            i = s->pipe[i].out_next;
        }
    }

    i = s->pipe[16].in_next;
    been_here = 1 << 16;
    while (i != 16) {
        if (been_here & (1 << i)) {
            trace_dbri_pipe_in_list_error();
            break;
        }
        been_here |= 1 << i;
        start = SLOT_START(s->pipe[i].in_desc);
        len = SLOT_LEN(s->pipe[i].in_desc);

        trace_dbri_pipe_in_list(i, s->pipe[i].in_desc, start, len);

        switch (SDP_MODE(s->pipe[i].setup)) {
        case SDP_MODE_MEM:
            if (s->pipe[i].ptr && start == s->codec_offset) {
                dbri_start_audio_in(s, i);
            }
            break;

        case SDP_MODE_FIXED:
            /* assume 8-bit alignment */
            codec_slot = 1 + (start - s->codec_offset) / 8;

            /* fixed pipe is LSB first, codec is MSB first */
            val = byte_rev(cs4215_read(&s->codec, codec_slot));
            if (len > 8) {
                val |= byte_rev(cs4215_read(&s->codec, codec_slot+1)) << 8;
            }
            if (s->pipe[i].data != val) {
                s->pipe[i].data = val;
                if (SDP_REPORT_CHANGE(s->pipe[i].setup)) {
                    dbri_post_interrupt(s, i, INTR_FXDT, val);
                }
            }
            break;
        }
        i = s->pipe[i].in_next;
    }
}

static void dbri_cmd_pause(DBRIState *s, uint32_t *cmd)
{
    dbri_run_pipes(s);
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
    if (cmd[0] & (SDP_PTR_VALID | SDP_CLEAR | SDP_ABORT)) {
        if (i) {
            dbri_stop_audio(s, i, cmd[0] & SDP_ABORT);
        }
    }
    if (cmd[0] & SDP_PTR_VALID) {
        trace_dbri_setup_data_pointer(i, cmd[1]);
        s->pipe[i].ptr = cmd[1];
    }
}

/* continue data pipe */
static void dbri_cmd_cdp(DBRIState *s, uint32_t *cmd)
{
    unsigned int i = cmd[0] & 0x1f;

    if (i == s->play.pipe) {
        s->play.stopped = false;
    }
    if (i == s->rec.pipe) {
        s->rec.stopped = false;
    }
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
            s->pipe_update = true;
            if (command_list[i].action) {
                command_list[i].action(s, cmd);
            }
            break;
        case 0:
        default:
            s->reg[0] &= ~DBRI_COMMAND_VALID;
            cmd_valid = false;
            if (s->pipe_update) {
                dbri_run_pipes(s);
            }
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
    s->pipe_update = false;
    for (i = 0; i < 32; i++) {
        s->pipe[i].setup = 0;
        s->pipe[i].ptr = 0;
        s->pipe[i].data = 0;
    }
    s->play.pipe = 0;
    s->rec.pipe = 0;
    s->play.stopped = true;
    s->rec.stopped = true;

    cs4215_reset(&s->codec);
    cs4215_setmode(&s->codec, DBRI_CODEC_DATA_MODE(s));
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
        if (DBRI_CODEC_RESET(s)) {
            cs4215_reset(&s->codec);
        }
        cs4215_setmode(&s->codec, DBRI_CODEC_DATA_MODE(s));
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

    cs4215_setmode(&s->codec, DBRI_CODEC_DATA_MODE(s));

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
        VMSTATE_BOOL(pipe_update,  DBRIState),

        VMSTATE_UINT32(play.pipe,  DBRIState),
        VMSTATE_UINT32(play.ctrl,  DBRIState),
        VMSTATE_BOOL(play.stopped, DBRIState),
        VMSTATE_UINT32(rec.pipe,   DBRIState),
        VMSTATE_UINT32(rec.ctrl,   DBRIState),
        VMSTATE_BOOL(rec.stopped,  DBRIState),

        VMSTATE_UINT8(codec.status,         DBRIState),
        VMSTATE_UINT8(codec.data_format,    DBRIState),
        VMSTATE_UINT8(codec.port_control,   DBRIState),
        VMSTATE_UINT8_ARRAY(codec.settings, DBRIState, 4),
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
