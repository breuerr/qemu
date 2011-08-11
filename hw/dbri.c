/*
 * QEMU DBRI audio interface
 *
 * Copyright (c) 2011 Bob Breuer
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
#include "audio/audio.h"
#include "sysbus.h"
#include "sun4m.h"

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
 *    volume control
 *    audio record - alaw, ulaw for record?
 *    isdn
 *    iommu errors (sbus faults)
 *    monitor pipes (for 8-bit stereo?)
 */

//#define DEBUG_DBRI
//#define DEBUG_PIPE
//#define DEBUG_DMA
//#define DEBUG_CODEC

#ifdef DEBUG_DBRI
#define DBRI_DPRINTF(fmt, ...)                          \
    printf("DBRI: " fmt , ## __VA_ARGS__)
#else
#define DBRI_DPRINTF(fmt, ...)
#endif

#ifdef DEBUG_PIPE
#define PIPE_DPRINTF(fmt, ...)                          \
    printf("DBRI Pipe: " fmt , ## __VA_ARGS__)
#else
#define PIPE_DPRINTF(fmt, ...)
#endif

#ifdef DEBUG_DMA
#define DMA_DPRINTF(fmt, ...)                           \
    printf("DBRI Dma: " fmt , ## __VA_ARGS__)
#else
#define DMA_DPRINTF(fmt, ...)
#endif

#ifdef DEBUG_CODEC
#define CODEC_DPRINTF(fmt, ...)                         \
    printf("CS4215: " fmt , ## __VA_ARGS__)
#else
#define CODEC_DPRINTF(fmt, ...)
#endif


typedef struct CS4215State {
    int data_mode;
    int freq;
    const int16_t *tbl;

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
    QEMUSoundCard card;
    SWVoiceOut *voice_out;

    qemu_irq irq;
    void *iommu;

    uint32_t pio_default;
    int32_t codec_offset;

    uint32_t reg[2];
    uint32_t pio;
    uint32_t cmdq_ptr; /* REG8 */
    uint32_t intq_ptr; /* REG9 */
    uint32_t intq_idx;
    uint32_t chi_global_mode;
    uint32_t chi_data_mode;

    bool pipe_update;
    bool chi_active;

    struct {
        uint32_t pipe, ctrl;
        int offset;
        bool stopped;
    } play, rec;

    DBRIPipeState pipe[32];

    CS4215State codec;
} DBRIState;


#define DBRI_ROM_SIZE   0x30
#define DBRI_REG_SIZE   0x100
#define DBRI_REG_OFFSET 0x10000

/* bits in reg0 (status/control */
#define DBRI_COMMAND_VALID      (1 << 15)
#define DBRI_CHI_ACTIVATE       (1 << 4)
#define DBRI_SOFT_RESET         (1 << 0)

/* reg1 = interrupt status */
#define DBRI_INT_STATUS         (1 << 0)
/* reg2 = PIO (Parallel I/O)
 * 4 bits of I/O: high nibble=enable, low nibble = value
 *  PIO0: 1=internal codec
 *  PIO1: 0=codec reset                 default = low
 *  PIO2: 1=external speakerbox
 *  PIO3: codec D/C, 1=data, 0=control  default = high
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

/* interrupt codes */
#define INTR_BRDY               1   /* Receive buffer ready */
#define INTR_MINT               2   /* Marker interrupt in TD/RD */
#define INTR_EOL                5   /* End of list */
#define INTR_CMDI               6   /* Command has been read */
#define INTR_XCMP               8   /* Transmit complete */
#define INTR_FXDT               10  /* Fixed data change */
#define INTR_CHIL               11  /* CHI lost frame */

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
#define CS4215_DF_MASK          (3 << 0)  /* data format */
#define CS4215_DF_S16           (0 << 0)
#define CS4215_DF_ULAW          (1 << 0)
#define CS4215_DF_ALAW          (2 << 0)
#define CS4215_DF_U8            (3 << 0)
#define CS4215_ST               (1 << 2)  /* stereo */
#define CS4215_DFR_MASK         (7 << 3)  /* data frequency rate */
#define CS4215_DFR_EXTRACT(d)   (((d) & CS4215_DFR_MASK) >> 3)

/* control slot 3 = port control */
#define CS4215_XEN              (1 << 0)
#define CS4215_XCLK             (1 << 1)  /* transmit clock master mode */
#define CS4215_BSEL_MASK        (3 << 2)
#define CS4215_BSEL_256         (2 << 2)
#define CS4215_MCK_MASK         (7 << 4)  /* clock source select */
#define CS4215_MCK_XTAL1        (1 << 4)
#define CS4215_MCK_XTAL2        (2 << 4)

#define CODEC_IS_MASTER(c)  \
            ((c)->data_mode && ((c)->port_control & CS4215_XCLK))

/* 2 clocks, 8 clock dividers */
#define AUD_CLK1  24576000
#define AUD_CLK2  16934400

static const int cs4215_clk_div[8] = {
    3072, 1536, 896, 768, 448, 384, 512, 2560
};

/* MuLaw/ALaw tables are also in cs4231a.c, move to common code? */
/* Tables courtesy http://hazelware.luggle.com/tutorials/mulawcompression.html */
static const int16_t MuLawDecompressTable[256] =
{
     -32124,-31100,-30076,-29052,-28028,-27004,-25980,-24956,
     -23932,-22908,-21884,-20860,-19836,-18812,-17788,-16764,
     -15996,-15484,-14972,-14460,-13948,-13436,-12924,-12412,
     -11900,-11388,-10876,-10364, -9852, -9340, -8828, -8316,
      -7932, -7676, -7420, -7164, -6908, -6652, -6396, -6140,
      -5884, -5628, -5372, -5116, -4860, -4604, -4348, -4092,
      -3900, -3772, -3644, -3516, -3388, -3260, -3132, -3004,
      -2876, -2748, -2620, -2492, -2364, -2236, -2108, -1980,
      -1884, -1820, -1756, -1692, -1628, -1564, -1500, -1436,
      -1372, -1308, -1244, -1180, -1116, -1052,  -988,  -924,
       -876,  -844,  -812,  -780,  -748,  -716,  -684,  -652,
       -620,  -588,  -556,  -524,  -492,  -460,  -428,  -396,
       -372,  -356,  -340,  -324,  -308,  -292,  -276,  -260,
       -244,  -228,  -212,  -196,  -180,  -164,  -148,  -132,
       -120,  -112,  -104,   -96,   -88,   -80,   -72,   -64,
        -56,   -48,   -40,   -32,   -24,   -16,    -8,     0,
      32124, 31100, 30076, 29052, 28028, 27004, 25980, 24956,
      23932, 22908, 21884, 20860, 19836, 18812, 17788, 16764,
      15996, 15484, 14972, 14460, 13948, 13436, 12924, 12412,
      11900, 11388, 10876, 10364,  9852,  9340,  8828,  8316,
       7932,  7676,  7420,  7164,  6908,  6652,  6396,  6140,
       5884,  5628,  5372,  5116,  4860,  4604,  4348,  4092,
       3900,  3772,  3644,  3516,  3388,  3260,  3132,  3004,
       2876,  2748,  2620,  2492,  2364,  2236,  2108,  1980,
       1884,  1820,  1756,  1692,  1628,  1564,  1500,  1436,
       1372,  1308,  1244,  1180,  1116,  1052,   988,   924,
        876,   844,   812,   780,   748,   716,   684,   652,
        620,   588,   556,   524,   492,   460,   428,   396,
        372,   356,   340,   324,   308,   292,   276,   260,
        244,   228,   212,   196,   180,   164,   148,   132,
        120,   112,   104,    96,    88,    80,    72,    64,
         56,    48,    40,    32,    24,    16,     8,     0
};

static const int16_t ALawDecompressTable[256] =
{
     -5504, -5248, -6016, -5760, -4480, -4224, -4992, -4736,
     -7552, -7296, -8064, -7808, -6528, -6272, -7040, -6784,
     -2752, -2624, -3008, -2880, -2240, -2112, -2496, -2368,
     -3776, -3648, -4032, -3904, -3264, -3136, -3520, -3392,
     -22016,-20992,-24064,-23040,-17920,-16896,-19968,-18944,
     -30208,-29184,-32256,-31232,-26112,-25088,-28160,-27136,
     -11008,-10496,-12032,-11520,-8960, -8448, -9984, -9472,
     -15104,-14592,-16128,-15616,-13056,-12544,-14080,-13568,
     -344,  -328,  -376,  -360,  -280,  -264,  -312,  -296,
     -472,  -456,  -504,  -488,  -408,  -392,  -440,  -424,
     -88,   -72,   -120,  -104,  -24,   -8,    -56,   -40,
     -216,  -200,  -248,  -232,  -152,  -136,  -184,  -168,
     -1376, -1312, -1504, -1440, -1120, -1056, -1248, -1184,
     -1888, -1824, -2016, -1952, -1632, -1568, -1760, -1696,
     -688,  -656,  -752,  -720,  -560,  -528,  -624,  -592,
     -944,  -912,  -1008, -976,  -816,  -784,  -880,  -848,
      5504,  5248,  6016,  5760,  4480,  4224,  4992,  4736,
      7552,  7296,  8064,  7808,  6528,  6272,  7040,  6784,
      2752,  2624,  3008,  2880,  2240,  2112,  2496,  2368,
      3776,  3648,  4032,  3904,  3264,  3136,  3520,  3392,
      22016, 20992, 24064, 23040, 17920, 16896, 19968, 18944,
      30208, 29184, 32256, 31232, 26112, 25088, 28160, 27136,
      11008, 10496, 12032, 11520, 8960,  8448,  9984,  9472,
      15104, 14592, 16128, 15616, 13056, 12544, 14080, 13568,
      344,   328,   376,   360,   280,   264,   312,   296,
      472,   456,   504,   488,   408,   392,   440,   424,
      88,    72,   120,   104,    24,     8,    56,    40,
      216,   200,   248,   232,   152,   136,   184,   168,
      1376,  1312,  1504,  1440,  1120,  1056,  1248,  1184,
      1888,  1824,  2016,  1952,  1632,  1568,  1760,  1696,
      688,   656,   752,   720,   560,   528,   624,   592,
      944,   912,  1008,   976,   816,   784,   880,   848
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

static void cs4215_getformat(CS4215State *s, struct audsettings *as)
{
    as->freq = s->freq;
    as->nchannels = (s->data_format & CS4215_ST) ? 2 : 1;
    as->endianness = AUDIO_HOST_ENDIANNESS;

    CODEC_DPRINTF("audio format: %d Hz, %d chan, fmt %d\n",
        as->freq, as->nchannels, s->data_format & CS4215_DF_MASK);

    s->tbl = NULL; /* conversion table */

    switch (s->data_format & CS4215_DF_MASK) {
    case CS4215_DF_S16: /* 16-bit 2's complement */
        as->fmt = AUD_FMT_S16;
        as->endianness = 1;   /* big-endian */
        break;
    case CS4215_DF_ULAW: /* 8-bit Mu-law */
        s->tbl = MuLawDecompressTable;
        as->fmt = AUD_FMT_S16;
        break;
    case CS4215_DF_ALAW: /* 8-bit A-law */
        s->tbl = ALawDecompressTable;
        as->fmt = AUD_FMT_S16;
        break;
    case CS4215_DF_U8: /* 8-bit unsigned */
        as->fmt = AUD_FMT_U8;
        break;
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
    }
    CODEC_DPRINTF("read 0x%02x from %s slot %d\n",
        val, s->data_mode ? "data" : "control", slot);
    return val;
}

static void cs4215_write(CS4215State *s, int slot, uint8_t val)
{
    CODEC_DPRINTF("write 0x%02x to %s slot %d\n",
        val, s->data_mode ? "data" : "control", slot);

    if (s->data_mode) {
        if (slot >= 5 && slot <= 8) {
            s->settings[slot-5] = val;
        }
    } else {
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

static uint32_t dbri_dma_readl(DBRIState *s, uint32_t addr)
{
    uint32_t val;

    sparc_iommu_memory_read(s->iommu, addr, (uint8_t *)&val, 4);
    val = be32_to_cpu(val);
    DMA_DPRINTF("readl 0x%08x\n", val);
    return val;
}

static void dbri_dma_writel(DBRIState *s, uint32_t addr, uint32_t val)
{
    DMA_DPRINTF("writel 0x%08x\n", val);
    val = cpu_to_be32(val);
    sparc_iommu_memory_write(s->iommu, addr, (uint8_t *)&val, 4);
}

#ifdef DEBUG_DBRI
static const char *intr_code[16] = {
    "code 0", "BRDY", "MINT", "IBEG", "IEND", "EOL", "CMDI", "code 7",
    "XCMP", "SBRI", "FXDT", "CHIL/COLL", "DBYT", "RBYT", "LINT", "UNDR"
};
#endif

static void dbri_post_interrupt(DBRIState *s, int chan, int code, int field)
{
    uint32_t addr, val;

    addr = s->intq_ptr;
    if (!addr) {
        /* disabled */
        return;
    }

    if (s->intq_idx == 64) {
        /* read next interrupt block from 1st word */
        addr = dbri_dma_readl(s, addr);
        if (!addr) {
            /* out of space? */
            return;
        }
        s->intq_ptr = addr;
        s->intq_idx = 1;
    }

    field &= 0xfffff;
    val = 0x80000000 | (chan << 24) | (code << 20) | field;

    DBRI_DPRINTF("interrupt (0x%08x) %s, chan %d, field 0x%05x\n",
        val, intr_code[code], chan, field);

    addr += 4 * s->intq_idx;
    s->intq_idx++;

    dbri_dma_writel(s, addr, val);
    s->reg[1] |= DBRI_INT_STATUS;
    qemu_irq_raise(s->irq);
}

static void dbri_out_cb(void *opaque, int free)
{
    DBRIState *s = opaque;
    int pipe = s->play.pipe;
    int left, used;
    int len, buf_len;
    uint32_t buf_ptr;
    uint8_t tmpbuf[1024];
    int16_t linbuf[1024];

    if (!pipe || s->play.stopped) {
        AUD_set_active_out(s->voice_out, 0);
        return;
    }

    used = 0;
    while (free) {
        buf_len = DBRI_TXBUF_LEN(s);
        left = buf_len - s->play.offset;

        /* end of buffer? */
        if (left <= 0) {
            uint32_t td_ptr = s->pipe[pipe].ptr;

            if (!td_ptr) {
                /* bad descriptor pointer - flag a fault? */
                s->play.stopped = 1;
                break;
            }

            if (s->pipe[pipe].data) {
                /* clear our data pointer to flag buffer done */
                s->pipe[pipe].data = 0;

                /* update status */
                dbri_dma_writel(s, td_ptr+12, 0x01);
                if ((s->play.ctrl & 0x80008000) == 0x80008000) {
                    /* transmit complete */
                    dbri_post_interrupt(s, pipe, INTR_XCMP, 0);
                }
            }
            /* next descriptor */
            td_ptr = dbri_dma_readl(s, td_ptr+8);
            if (!td_ptr) {
                DBRI_DPRINTF("out of tx data after %d bytes\n", used);
                if (used) {
                    /*
                     * maybe the host buffers are too big,
                     * give the guest a chance to catch up
                     */
                    break;
                }
                if (s->pipe[pipe].setup & SDP_EOL) {
                    /* end of list */
                    dbri_post_interrupt(s, pipe, INTR_EOL, td_ptr);
                }
                s->play.stopped = 1;
                break;
            }
            s->pipe[pipe].ptr = td_ptr;
            s->play.ctrl = dbri_dma_readl(s, td_ptr);
            if (s->play.ctrl & 0x4000) {
                /* marker int */
                dbri_post_interrupt(s, pipe, INTR_MINT, td_ptr);
            }
            /* setup next buffer */
            s->pipe[pipe].data = dbri_dma_readl(s, td_ptr+4);
            s->play.offset = 0;
            if (!s->pipe[pipe].data) {
                /* NULL data pointer, what now? */
            }
            left = buf_len = DBRI_TXBUF_LEN(s);

            DBRI_DPRINTF("play buffer @ 0x%08x, len %d\n",
                s->pipe[pipe].data, buf_len);
        }

        if (!s->pipe[pipe].data) {
            /* bad data pointer - flag a fault? */
            s->play.stopped = 1;
            break;
        }
        buf_ptr = s->pipe[pipe].data + s->play.offset;
        len = audio_MIN(left, sizeof(tmpbuf));

        if (s->codec.tbl) {
            /* convert from A/Mu-Law to 16-bit linear */
            const int16_t *tbl;
            int i;

            len = audio_MIN(len, free >> 1);

            sparc_iommu_memory_read(s->iommu, buf_ptr, tmpbuf, len);

            tbl = s->codec.tbl;
            for (i = 0; i < len; i++) {
                linbuf[i] = tbl[tmpbuf[i]];
            }
            len = AUD_write(s->voice_out, linbuf, len << 1);
            len >>= 1;
        } else {
            len = audio_MIN(len, free);

            sparc_iommu_memory_read(s->iommu, buf_ptr, tmpbuf, len);

            len = AUD_write(s->voice_out, tmpbuf, len);
        }

        if (!len) {
            break;
        }

        s->play.offset += len;
        free -= len;
        used += len;
    }
}

static void dbri_update_audio(DBRIState *s)
{
    struct audsettings as;

    cs4215_getformat(&s->codec, &as);
    s->voice_out = AUD_open_out(&s->card, s->voice_out,
                                "dbri_out", s, dbri_out_cb, &as);

    /* zero is not a valid pipe */
    if (s->play.pipe && !s->play.stopped) {
        AUD_set_active_out(s->voice_out, 1);
    } else {
        AUD_set_active_out(s->voice_out, 0);
    }
}

static void dbri_start_audio_out(DBRIState *s, int pipe)
{
    uint32_t td_ptr;  /* transmit descriptor */

    if (pipe == s->play.pipe && !s->play.stopped) {
        return;
    }
    s->play.pipe = pipe;

    /* ptr was validated by caller */
    td_ptr = s->pipe[pipe].ptr;

    s->play.ctrl = dbri_dma_readl(s, td_ptr);   /* control/length */
    if (s->play.ctrl & 0x4000) {
        /* marker int */
        dbri_post_interrupt(s, pipe, INTR_MINT, td_ptr);
    }
    /* get data pointer, re-use pipe data field */
    s->pipe[pipe].data = dbri_dma_readl(s, td_ptr+4);

    /* prepare buffer */
    s->play.offset = 0;
    s->play.stopped = 0;

    DBRI_DPRINTF("play buffer @ 0x%08x, len %d\n",
        s->pipe[pipe].data, DBRI_TXBUF_LEN(s));

    dbri_update_audio(s);
}

static void dbri_start_audio_in(DBRIState *s, int pipe)
{
    uint32_t rd_ptr;  /* receive descriptor */

    if (pipe == s->rec.pipe && !s->rec.stopped) {
        return;
    }
    s->rec.pipe = pipe;

    /* setup format */

    rd_ptr = s->pipe[pipe].ptr;

    s->rec.ctrl = dbri_dma_readl(s, rd_ptr+12); /* control/length */
    if (s->rec.ctrl & 0x4000) {
        /* marker int */
        dbri_post_interrupt(s, pipe, INTR_MINT, rd_ptr);
    }
    /* get data pointer, re-use pipe data field */
    s->pipe[pipe].data = dbri_dma_readl(s, rd_ptr+4);

    /* prepare buffer */
    s->rec.offset = 0;
    s->rec.stopped = 0;

    /* TODO: audio record, this is just a placeholder */
    if (0) {
        /* write buffer */
        uint32_t len = DBRI_RXBUF_LEN(s);

        /* update status - completed and EOF */
        dbri_dma_writel(s, rd_ptr, 0xc0000000 | (len << 16));
        if (s->rec.ctrl & 0x8000) {
            /* buffer ready */
            dbri_post_interrupt(s, pipe, INTR_BRDY, rd_ptr);
        }
        /* next */
        rd_ptr = dbri_dma_readl(s, rd_ptr+8);
        s->pipe[pipe].ptr = rd_ptr;
    }
    if (s->pipe[pipe].setup & SDP_EOL) {
        /* end of list */
        dbri_post_interrupt(s, pipe, INTR_EOL, rd_ptr);
    }
    s->rec.stopped = 1;
    dbri_update_audio(s);
}

static void dbri_stop_audio(DBRIState *s, int pipe, int abort)
{
    if (pipe == s->play.pipe) {
        if (abort && s->pipe[pipe].ptr) {
            /* abort */
            dbri_dma_writel(s, s->pipe[pipe].ptr+12, 0x04);
            AUD_set_active_out(s->voice_out, 0);
        }
        s->play.stopped = 1;
        s->play.pipe = 0;
    }
    if (pipe == s->rec.pipe) {
        if (abort && s->pipe[pipe].ptr) {
            /* abort */
            dbri_dma_writel(s, s->pipe[pipe].ptr, 0x20);
        }
        s->rec.stopped = 1;
        s->rec.pipe = 0;
    }
}

static void dbri_update_chi_status(DBRIState *s)
{
    if ((s->reg[0] & DBRI_CHI_ACTIVATE)
        && (DBRI_CHI_IS_MASTER(s) || CODEC_IS_MASTER(&s->codec))) {
        s->chi_active = 1;
    } else {
        s->chi_active = 0;
    }
}

static void dbri_run_pipes(DBRIState *s)
{
    int i, start, len, codec_slot;
    uint32_t val;
    uint32_t been_here;

    if (!s->chi_active) {
        return; /* CHI not active */
    }

    /* run through the pipes */
    s->pipe_update = 0;

    /* pipe 16 is the anchor where the CHI starts and ends */
    if (s->chi_data_mode & 0x02) {
        i = s->pipe[16].out_next;
        been_here = 1 << 16;
        PIPE_DPRINTF("OUT pipes:\n");
        while (i != 16) {
            if (been_here & (1 << i)) {
                PIPE_DPRINTF("linked list loop before OUT anchor\n");
                break;
            }
            been_here |= 1 << i;
            start = SLOT_START(s->pipe[i].out_desc);
            len = SLOT_LEN(s->pipe[i].out_desc);

            /* use bits per frame instead of 0 for OUT pipes */
            if (start >= 64) {
                start = 0;
            }
            PIPE_DPRINTF(" %d, 0x%08x = start %d, len %d\n",
                i, s->pipe[i].out_desc, start, len);

            switch (SDP_MODE(s->pipe[i].setup)) {
            case SDP_MODE_MEM:
                PIPE_DPRINTF("  TD @ 0x%08x\n", s->pipe[i].ptr);

                if (s->pipe[i].ptr && start == s->codec_offset) {
                    dbri_start_audio_out(s, i);
                }
                break;

            case SDP_MODE_FIXED:
                PIPE_DPRINTF("  Fixed 0x%08x\n", s->pipe[i].data);

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
    PIPE_DPRINTF("IN pipes:\n");
    while (i != 16) {
        if (been_here & (1 << i)) {
            PIPE_DPRINTF("linked list loop before IN anchor\n");
            break;
        }
        been_here |= 1 << i;
        start = SLOT_START(s->pipe[i].in_desc);
        len = SLOT_LEN(s->pipe[i].in_desc);

        PIPE_DPRINTF(" %d, 0x%08x = start %d, len %d\n",
            i, s->pipe[i].in_desc, start, len);

        switch (SDP_MODE(s->pipe[i].setup)) {
        case SDP_MODE_MEM:
            PIPE_DPRINTF("  RD @ 0x%08x\n", s->pipe[i].ptr);

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
            PIPE_DPRINTF("  Fixed 0x%08x\n", s->pipe[i].data);
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
    int i = cmd[0] & 0x1f;

    PIPE_DPRINTF("Setup pipe %d for %s: mode %d, IRM 0x%x, clear %d\n",
        i, (cmd[0] & SDP_DIR_OUT) ? "output" : "input",
        (cmd[0] >> 13) & 7, (cmd[0] >> 16) & 0xf,
        (cmd[0] >> 7) & 1);

    s->pipe[i].setup = cmd[0];
    if (cmd[0] & (SDP_PTR_VALID | SDP_CLEAR | SDP_ABORT)) {
        /* stop any audio on this pipe */
        if (i) {
            dbri_stop_audio(s, i, cmd[0] & SDP_ABORT);
        }
    }
    if (cmd[0] & SDP_PTR_VALID) {
        PIPE_DPRINTF(" pipe pointer = 0x%08x\n", cmd[1]);
        s->pipe[i].ptr = cmd[1];
    }
}

/* continue data pipe */
static void dbri_cmd_cdp(DBRIState *s, uint32_t *cmd)
{
    int i = cmd[0] & 0x1f;

    if (i == s->play.pipe) {
        s->play.stopped = 0;
    }
    if (i == s->rec.pipe) {
        s->rec.stopped = 0;
    }
    dbri_update_audio(s);
}

/* define time slot */
static void dbri_cmd_dts(DBRIState *s, uint32_t *cmd)
{
    int prev, next;
    int i = cmd[0] & 0x1f;

    PIPE_DPRINTF("%s time slots for pipe %d\n",
        (cmd[0] & DTS_INSERT) ? "add/modify" : "delete", i);

    if (cmd[0] & DTS_VIN) {
        prev = DTS_PIPE_IN_PREV(cmd[0]);
        if (cmd[0] & DTS_INSERT) {
            next = SLOT_NEXT(cmd[1]);

            s->pipe[i].in_next = next;
            s->pipe[prev].in_next = i;
            s->pipe[i].in_desc = cmd[1];
        } else {
            /* delete */
            next = s->pipe[i].in_next;
            s->pipe[prev].in_next = next;
        }

        PIPE_DPRINTF("In:  prev=%d, next=%d, mode=%d, len=%d, cycle=%d\n",
            prev, next, SLOT_MODE(cmd[1]),
            SLOT_LEN(cmd[1]), SLOT_START(cmd[1]));
    }
    if (cmd[0] & DTS_VOUT) {
        prev = DTS_PIPE_OUT_PREV(cmd[0]);
        if (cmd[0] & DTS_INSERT) {
            next = SLOT_NEXT(cmd[2]);

            s->pipe[i].out_next = next;
            s->pipe[prev].out_next = i;
            s->pipe[i].out_desc = cmd[2];
        } else {
            /* delete */
            next = s->pipe[i].out_next;
            s->pipe[prev].out_next = next;
        }

        PIPE_DPRINTF("Out: prev=%d, next=%d, mode=%d, len=%d, cycle=%d\n",
            prev, next, SLOT_MODE(cmd[2]),
            SLOT_LEN(cmd[2]), SLOT_START(cmd[2]));
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
    int active;
    uint32_t status;

    active = DBRI_CHI_IS_MASTER(s) || CODEC_IS_MASTER(&s->codec);
    s->chi_global_mode = cmd[0];

    if (s->chi_global_mode & 0x8000) {
        /* report status */
        status = s->chi_data_mode & 0x03;
        if (!active) {
            status |= 0x04;
        }
        dbri_post_interrupt(s, 36, INTR_CHIL, status);
    }
    dbri_update_chi_status(s);
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
    int stopped, i;

    for (stopped = 0; !stopped; ) {
        cmd[0] = dbri_dma_readl(s, s->cmdq_ptr);
        i = cmd[0] >> 28;

        /* interrupt on command? */
        if (cmd[0] & (1<<27)) {
            val = (i << 16) | (cmd[0] & 0xffff);
            dbri_post_interrupt(s, 38, INTR_CMDI, val);
        }

        DBRI_DPRINTF("cmd %s = 0x%08x\n", command_list[i].name, cmd[0]);

        switch (command_list[i].len) {
        case 12:
            cmd[2] = dbri_dma_readl(s, s->cmdq_ptr+8);
        case 8:
            cmd[1] = dbri_dma_readl(s, s->cmdq_ptr+4);
        case 4:
            s->cmdq_ptr += command_list[i].len;
            s->pipe_update = 1;
            if (command_list[i].action) {
                command_list[i].action(s, cmd);
            }
            break;
        case 0:
        default:
            stopped = 1;
            s->reg[0] &= ~DBRI_COMMAND_VALID;
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
    int i;

    AUD_set_active_out(s->voice_out, 0);
    qemu_irq_lower(s->irq);

    /* set defaults */
    s->reg[0] = 0x4008;
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
    s->chi_active = 0;

    /* reset linked list */
    s->pipe[16].in_next = 16;
    s->pipe[16].out_next = 16;

    /* clear all pipes */
    s->pipe_update = 0;
    for (i = 0; i < 32; i++) {
        s->pipe[i].setup = 0;
        s->pipe[i].ptr = 0;
        s->pipe[i].data = 0;
    }
    s->play.pipe = 0;
    s->rec.pipe = 0;
    s->play.stopped = 1;
    s->rec.stopped = 1;

    cs4215_reset(&s->codec);
    cs4215_setmode(&s->codec, DBRI_CODEC_DATA_MODE(s));
}

/* registers */
static uint32_t dbri_reg_readl(void *opaque, target_phys_addr_t addr)
{
    DBRIState *s = opaque;
    int val;

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

    DBRI_DPRINTF("readl 0x%08x from reg " TARGET_FMT_plx "\n", val, addr);

    return val;
}

static void dbri_reg_writel(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    DBRIState *s = opaque;

    DBRI_DPRINTF("writel 0x%08x to reg " TARGET_FMT_plx "\n", val, addr);

    switch (addr) {
    case 0x00: /* Status and Control */
        s->reg[0] = val;
        if (val & DBRI_SOFT_RESET) {
            dbri_reset(&s->busdev.qdev);
        } else {
            dbri_update_chi_status(s);
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
        dbri_update_chi_status(s);
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

static CPUReadMemoryFunc * const dbri_reg_read[3] = {
    NULL,
    NULL,
    dbri_reg_readl,
};

static CPUWriteMemoryFunc * const dbri_reg_write[3] = {
    NULL,
    NULL,
    dbri_reg_writel,
};

static int vmstate_dbri_post_load(void *opaque, int version_id)
{
    DBRIState *s = opaque;

    cs4215_setmode(&s->codec, DBRI_CODEC_DATA_MODE(s));
    dbri_update_chi_status(s);

    if (s->intq_ptr && s->reg[1]) {
        qemu_irq_raise(s->irq);
    }

    /* resume playback */
    dbri_update_audio(s);

    return 0;
}


/* the FCode rom */
static const uint8_t dbri_rom[DBRI_ROM_SIZE] = {
    0xfd, 0x00, 0x09, 0xea, 0x00, 0x00, 0x00, 0x30, 0x12, 0x0a,
    'S',  'U',  'N',  'W',  ',',  'D',  'B',  'R',  'I',  'e',
    0x01, 0x14, 0x12, 0x04, 'n',  'a',  'm',  'e',  0x01, 0x10,
    0x01, 0x02, 0xa5, 0xa6, 0x7d, 0x1e, 0x01, 0x03, 0xa6, 0x80,
    0x01, 0x16, 0xa8, 0x63, 0xa5, 0x01, 0x17, 0x00
};

/* everything is an offset within our sbus slot */
static void dbri_sbus_map(SysBusDevice *dev, target_phys_addr_t base)
{
    DBRIState *s = FROM_SYSBUS(DBRIState, dev);
    int rom, regs;

    rom = qemu_ram_alloc(NULL, "dbri.rom", DBRI_ROM_SIZE);
    /* the rom is at offset 0 */
    cpu_register_physical_memory(base, DBRI_ROM_SIZE, rom|IO_MEM_ROM);
    cpu_physical_memory_write_rom(base, dbri_rom, DBRI_ROM_SIZE);

    /* mirror at 0x1000, where the SS-20 bootrom looks for it */
    cpu_register_physical_memory(base+0x1000, DBRI_ROM_SIZE, rom|IO_MEM_ROM);

    regs = cpu_register_io_memory(dbri_reg_read, dbri_reg_write, s,
        DEVICE_NATIVE_ENDIAN);
    cpu_register_physical_memory(base+DBRI_REG_OFFSET, DBRI_REG_SIZE, regs);
}

static int dbri_init1(SysBusDevice *dev)
{
    DBRIState *s = FROM_SYSBUS(DBRIState, dev);

    sysbus_init_mmio_cb(dev, DBRI_REG_OFFSET+DBRI_REG_SIZE, dbri_sbus_map);
    sysbus_init_irq(dev, &s->irq);

    AUD_register_card("sun_dbri", &s->card);

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

        VMSTATE_UINT32(play.pipe,  DBRIState),
        VMSTATE_UINT32(rec.pipe,   DBRIState),
        VMSTATE_UINT32(play.ctrl,  DBRIState),
        VMSTATE_UINT32(rec.ctrl,   DBRIState),
        VMSTATE_BOOL(play.stopped, DBRIState),
        VMSTATE_BOOL(rec.stopped,  DBRIState),

        VMSTATE_BOOL(pipe_update,  DBRIState),
        VMSTATE_STRUCT_ARRAY(pipe, DBRIState, 32, 1,
                             vmstate_dbri_pipe, DBRIPipeState),

        VMSTATE_UINT8(codec.status,         DBRIState),
        VMSTATE_UINT8(codec.data_format,    DBRIState),
        VMSTATE_UINT8(codec.port_control,   DBRIState),
        VMSTATE_UINT8_ARRAY(codec.settings, DBRIState, 4),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo dbri_info = {
    .init = dbri_init1,
    .qdev.name  = "SUNW,DBRIe",
    .qdev.desc  = "Sun DBRI audio interface",
    .qdev.size  = sizeof(DBRIState),
    .qdev.reset = dbri_reset,
    .qdev.vmsd  = &vmstate_dbri,
    .qdev.props = (Property[]) {
        DEFINE_PROP_PTR("iommu_opaque",   DBRIState, iommu),
        DEFINE_PROP_INT32("codec_offset", DBRIState, codec_offset, 8),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void dbri_register_devices(void)
{
    sysbus_register_withprop(&dbri_info);
}

device_init(dbri_register_devices);
