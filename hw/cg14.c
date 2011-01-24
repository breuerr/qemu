/*
 * QEMU CG14 Frame buffer
 *
 * Copyright (c) 2011 Bob Breuer  <breuerr@mc.net>
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

#include "console.h"
#include "sysbus.h"

//#define DEBUG_CG14
//#define DEBUG_CONFIG

/*
 * Sun CG14 framebuffer (without SX)
 *   CG14 = vsimm framebuffer (video ram and dac)
 *   SX = pixel processor (acceleration) built into chipset
 *
 * Documentation: not publicly documented by Sun
 *   linux driver: drivers/video/cg14.c
 *   NetBSD/OpenBSD: src/sys/arch/sparc/dev/cgfourteen*
 *
 * Takes up one memory slot:
 *   A[28:26] = slot number (4 to 7)
 *   regs: size   0x10000 @ 0x09c000000  (0x80000000 + slot * 64M)
 *   vmem: size upto 16MB @ 0x0fc000000  (0xE0000000 + slot * 64M)
 *
 * SS-20 OBP only supports slots 7 (onboard output) and 4 (AVB output)
 *
 * memory map:
 * reg+0x0000 = control registers
 * reg+0x1000 = cursor registers
 * reg+0x2000 = dac registers (ADV7152)
 * reg+0x3000 = xlut
 * reg+0x4000 = clut1
 * reg+0x5000 = clut2
 * reg+0x6000 = clut3 (if implemented)
 * reg+0xf000 = autoinc
 *
 * mem+0x0000000 = XBGR (01234567)
 * mem+0x1000000 = BGR  (.123.567)  writes to X are blocked, reads are ok
 * mem+0x2000000 = X16  (0246)
 * mem+0x2800000 = C16  (1357)
 * mem+0x3000000 = X32  (04)
 * mem+0x3400000 = B32  (15)
 * mem+0x3800000 = G32  (26)
 * mem+0x3c00000 = R32  (37)
 */

#define CG14_REG_SIZE         0x10000
#define CG14_VMEM_SLOTSIZE    (64 << 20)

#define CG14_MONID_1024x768   0
#define CG14_MONID_1600x1280  1
#define CG14_MONID_1280x1024  2
#define CG14_MONID_1152x900   7

#define CG14_MONID_DEFAULT    CG14_MONID_1024x768


#define CG14_MCR_INTENABLE     0x80
#define CG14_MCR_VIDENABLE     0x40
#define CG14_MCR_PIXMODE_MASK  0x30
#define   CG14_MCR_PIXMODE_8     0x00
#define   CG14_MCR_PIXMODE_16    0x20  /* 8+8 (X16,C16) */
#define   CG14_MCR_PIXMODE_32    0x30  /* XBGR */

/* index for timing registers */
enum {
    HBLANK_START = 0,
    HBLANK_CLEAR,
    HSYNC_START,
    HSYNC_CLEAR,
    CSYNC_CLEAR,
    VBLANK_START,
    VBLANK_CLEAR,
    VSYNC_START,
    VSYNC_CLEAR,
    CG14_TIMING_MAX
};

#ifdef DEBUG_CG14
#define DPRINTF(fmt, ...)                                       \
    printf("CG14: " fmt , ## __VA_ARGS__)
#else
#define DPRINTF(fmt, ...)
#endif

#ifdef DEBUG_CONFIG
#define DPRINTF_CONFIG(fmt, ...)                                \
    printf("CG14: " fmt , ## __VA_ARGS__)
#else
#define DPRINTF_CONFIG(fmt, ...)
#endif

#define CG14_INFO(fmt, ...)                                     \
    do { printf("CG14: " fmt , ## __VA_ARGS__); } while (0)
#define CG14_ERROR(fmt, ...)                                    \
    do { printf("CG14: " fmt , ## __VA_ARGS__); } while (0)


typedef struct CG14State {
    SysBusDevice busdev;
    DisplayState *ds;

    uint8_t *vram;
    uint32_t vram_size;
    uint32_t vram_amask;
    uint16_t width, height;
    int dirty, size_changed;

    struct {
        uint8_t mcr;
        uint8_t ppr;
        uint8_t msr;
    } ctrl;
    uint16_t timing[CG14_TIMING_MAX];
    uint8_t xlut[256];
    uint32_t clut1[256];
    uint32_t clut2[256];
} CG14State;

static void cg14_screen_dump(void *opaque, const char *filename);
static void cg14_invalidate_display(void *opaque);

typedef void cg14_draw_line_func(const CG14State *s, void *dst, const uint8_t *src, int width);

static inline uint32_t bgr_to_rgb(uint32_t bgr)
{
    uint32_t rgb;

    /* swap r & b */
    rgb = (bgr & 0x00FF00)
        | (bgr & 0x0000FF) << 16
        | (bgr & 0xFF0000) >> 16;
    return rgb;
}

// TODO: draw_line templates
static void cg14_draw_line8_rgb32(const CG14State *s, void *dst, const uint8_t *src, int width)
{
    int i;
    const uint32_t *clut;
    uint32_t *p = dst;

    clut = s->clut1;
    for (i = 0; i < width; i++) {
        *p++ = bgr_to_rgb(clut[src[i]]);
    }
}

static void cg14_draw_line32_rgb32(const CG14State *s, void *dst, const uint8_t *src, int width)
{
    int i;
    unsigned int r, g, b;
    uint32_t dval;
    uint32_t *p = dst;

    for (i = 0; i < width; i++) {
        b = src[1];
        g = src[2];
        r = src[3];
        src += 4;

        /* xlut = 0x00 for true-color */
        dval = r << 16 | g << 8 | b;

        *p++ = dval;
    }
}

/* TODO: use VGA_DIRTY_FLAG */
static void cg14_update_display(void *opaque)
{
    CG14State *s = opaque;
    int y, src_linesize;
    uint8_t *pix;
    uint8_t *data;
    int new_width, new_height;
    cg14_draw_line_func *draw_line;

    if (s->size_changed) {
        new_width = 4 * (s->timing[HBLANK_START] - s->timing[HBLANK_CLEAR]);
        new_height = s->timing[VBLANK_START] - s->timing[VBLANK_CLEAR];
        s->size_changed = 0;
        if ((new_width != s->width || new_height != s->height) && new_width > 0 && new_height > 0) {
            s->width = new_width;
            s->height = new_height;
            CG14_INFO("new resolution = %d x %d\n", new_width, new_height);
            qemu_console_resize(s->ds, s->width, s->height);
            s->dirty = 1;
        }
    }

    if (!s->dirty || !s->width || !s->height) {
        return;
    }

    if (ds_get_bits_per_pixel(s->ds) != 32) {
        CG14_ERROR("cg14_update: FIXME: bpp (%d) != 32, linesize %d\n",
            ds_get_bits_per_pixel(s->ds), ds_get_linesize(s->ds));
        return;
    }
    // if (is_surface_bgr(s->ds->surface))

    draw_line = NULL;
    src_linesize = s->width;
    if (s->ctrl.mcr & CG14_MCR_VIDENABLE) {
        switch (s->ctrl.mcr & CG14_MCR_PIXMODE_MASK) {
        case CG14_MCR_PIXMODE_8:
            draw_line = cg14_draw_line8_rgb32;
            break;
        case CG14_MCR_PIXMODE_16:
            src_linesize *= 2;
            break;
        case CG14_MCR_PIXMODE_32:
            src_linesize *= 4;
            draw_line = cg14_draw_line32_rgb32;
            break;
        }
    }
    if (!draw_line) {
        /* blank */
        memset(ds_get_data(s->ds), 0, ds_get_linesize(s->ds) * ds_get_height(s->ds));
        dpy_update(s->ds, 0, 0, s->width, s->height);
        s->dirty = 0;
        return;
    }

    pix = s->vram;
    data = ds_get_data(s->ds);

    for (y = 0; y < s->height; y++) {
        draw_line(s, data, pix, s->width);
        pix += src_linesize;
        data += ds_get_linesize(s->ds);
    }
    dpy_update(s->ds, 0, 0, s->width, s->height);
    s->dirty = 0;
}

static void cg14_invalidate_display(void *opaque)
{
    CG14State *s = opaque;

    s->dirty = 1;
}

static uint32_t cg14_reg_readb(void *opaque, target_phys_addr_t addr)
{
    CG14State *s = opaque;
    uint32_t val;
    uint32_t i;

    switch (addr) {
    case 0x0000:
        val = s->ctrl.mcr;
        break;
    case 0x0001:
        val = s->ctrl.ppr;
        break;
    case 0x0004: /* status ? */
        val = s->ctrl.msr;
        break;
    case 0x0006: /* hw version */
        //val = 0x00; /* old version */
        val = 0x30;
        break;
    case 0x020e: /* VBC version */
        val = 0x14;
        break;
    case 0x3000 ... 0x30ff: /* xlut */
        i = addr & 0xff;
        val = s->xlut[i];
        break;
    default:
        val = 0;
        CG14_INFO("readb from reg %x\n", (int)addr);
        break;
    }

    return val;
}

static void cg14_reg_writeb(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CG14State *s = opaque;
    uint32_t i;

    s->dirty = 1;

    switch (addr) {
    case 0x0000:
        s->ctrl.mcr = val;
        if (val & ~0x71) {
            CG14_ERROR("control register (0x%02x) has unimplemented bits set\n", val);
        } else {
            DPRINTF_CONFIG("write 0x%02x to MCR\n", val);
        }
        break;
    case 0x0001:
        s->ctrl.ppr = val & 0xf0;
        DPRINTF_CONFIG("write 0x%02x to PPR\n", val);
        break;
    case 0x0007: /* clock control (ICS1562AM-001) */
        DPRINTF("write 0x%02x to clock control\n", val);
        break;
    case 0x1100: /* cursor control */
        break;
    case 0x2000 ... 0x23ff: /* dac */
        break;
    case 0x3000 ... 0x30ff: /* xlut */
        i = addr & 0xff;
        if (s->xlut[i] != val) {
            s->xlut[i] = val;
            if (val && val != 0x40)
                CG14_ERROR("writeb xlut[%d] = 0x%02x\n", i, val);
        }
        break;
    default:
        CG14_ERROR("writeb 0x%02x to reg %x\n", val, (int)addr);
        break;
    }
}

static uint32_t cg14_reg_readw(void *opaque, target_phys_addr_t addr)
{
    CG14State *s = opaque;
    uint32_t val;
    int i;

    switch (addr) {
    case 0x0018 ... 0x0028:
        i = (addr - 0x0018) >> 1;
        val = s->timing[i];
        break;
    default:
        val = 0;
        CG14_INFO("readw from reg %x\n", (int)addr);
        break;
    }

    return val;
}

static void cg14_reg_writew(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CG14State *s = opaque;
    int i;

    DPRINTF_CONFIG("writew 0x%04x to reg %x\n", val, (int)addr);

    /* timing registers are 16bit */
    switch (addr) {
    case 0x0018 ... 0x0028:
        i = (addr - 0x0018) >> 1;
        s->timing[i] = val;
        if (i == HBLANK_CLEAR || i == VBLANK_CLEAR) {
            s->size_changed = 1;
        }
        break;
    }
}

static uint32_t cg14_reg_readl(void *opaque, target_phys_addr_t addr)
{
    CG14State *s = opaque;
    uint32_t val;
    uint32_t i;

    i = addr & 0x3fc;
    switch (addr) {
    case 0x4000 ... 0x43ff:
        val = s->clut1[i >> 2];
        break;
    case 0x5000 ... 0x53ff:
        val = s->clut2[i >> 2];
        break;
    default:
        val = 0;
        CG14_ERROR("readl %08x from reg %x\n", val, (int)addr);
        break;
    }

    return val;
}

static void cg14_reg_writel(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CG14State *s = opaque;
    uint32_t i;

    s->dirty = 1;

    i = addr & 0x3fc;
    switch (addr) {
    case 0x1000 ... 0x10ff: /* cursor - not implemented */
        break;
    case 0x3000 ... 0x30ff:
        s->xlut[i+0] = (uint8_t)(val >> 24);
        s->xlut[i+1] = (uint8_t)(val >> 16);
        s->xlut[i+2] = (uint8_t)(val >> 8);
        s->xlut[i+3] = (uint8_t)val;
        break;
    case 0x4000 ... 0x43ff:
        s->clut1[i >> 2] = val;
        break;
    case 0x5000 ... 0x53ff:
        s->clut2[i >> 2] = val;
        break;
    default:
        CG14_ERROR("writel %08x to reg %x\n", val, (int)addr);
        break;
    }
}

static CPUReadMemoryFunc * const cg14_reg_read[3] = {
    cg14_reg_readb,
    cg14_reg_readw,
    cg14_reg_readl,
};

static CPUWriteMemoryFunc * const cg14_reg_write[3] = {
    cg14_reg_writeb,
    cg14_reg_writew,
    cg14_reg_writel,
};

static uint32_t cg14_vram_readb(void *opaque, target_phys_addr_t addr)
{
    CG14State *s = opaque;
    uint32_t offset;
    uint32_t val = 0;

    switch (addr & 0x3000000) {
    case 0x0000000:
    case 0x1000000:
        offset = addr & s->vram_amask;
        val = ldub_p(s->vram+offset);
        break;
    case 0x2000000:
        offset = ((addr << 1) & s->vram_amask) + ((addr >> 23) & 1);
        val = ldub_p(s->vram+offset);
        break;
    case 0x3000000:
        offset = ((addr << 2) & s->vram_amask) + ((addr >> 22) & 3);
        val = ldub_p(s->vram+offset);
        break;
    }

    return val;
}

static void cg14_vram_writeb(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CG14State *s = opaque;
    uint32_t offset;

    s->dirty = 1;

    switch (addr & 0x3000000) {
    case 0x0000000:
        offset = addr & s->vram_amask;
        stb_p(s->vram+offset, val);
        break;
    case 0x1000000:
        offset = addr & s->vram_amask;
        /* block writes to X */
        if (offset & 3) {
            stb_p(s->vram+offset, val);
        }
        break;
    case 0x2000000:
        offset = ((addr << 1) & s->vram_amask) + ((addr >> 23) & 1);
        stb_p(s->vram+offset, val);
        break;
    case 0x3000000:
        offset = ((addr << 2) & s->vram_amask) + ((addr >> 22) & 3);
        stb_p(s->vram+offset, val);
        break;
    }
}

static uint32_t cg14_vram_readw(void *opaque, target_phys_addr_t addr)
{
    uint32_t val;

    val = cg14_vram_readb(opaque, addr) << 8;
    val |= cg14_vram_readb(opaque, addr+1);

    return val;
}

static void cg14_vram_writew(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    cg14_vram_writeb(opaque, addr, val >> 8);
    cg14_vram_writeb(opaque, addr+1, val & 0xff);
}

static uint32_t cg14_vram_readl(void *opaque, target_phys_addr_t addr)
{
    CG14State *s = opaque;
    uint32_t offset;
    uint32_t val = 0;

    switch (addr & 0x3000000) {
    case 0x0000000:
    case 0x1000000:
        offset = addr & s->vram_amask;
        val = ldl_be_p(s->vram+offset);
        break;
    case 0x2000000:
        offset = ((addr << 1) & s->vram_amask) + ((addr >> 23) & 1);
        val =  ldub_p(s->vram+offset+0) << 24;
        val |= ldub_p(s->vram+offset+2) << 16;
        val |= ldub_p(s->vram+offset+4) << 8;
        val |= ldub_p(s->vram+offset+6);
        break;
    case 0x3000000:
        offset = ((addr << 2) & s->vram_amask) + ((addr >> 22) & 3);
        val =  ldub_p(s->vram+offset+0) << 24;
        val |= ldub_p(s->vram+offset+4) << 16;
        val |= ldub_p(s->vram+offset+8) << 8;
        val |= ldub_p(s->vram+offset+12);
        break;
    }

    return val;
}

static void cg14_vram_writel(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CG14State *s = opaque;
    uint32_t offset;

    s->dirty = 1;

    switch (addr & 0x3000000) {
    case 0x0000000:
        offset = addr & s->vram_amask;
        stl_be_p(s->vram+offset, val);
        break;
    case 0x1000000:
        offset = addr & s->vram_amask;
        /* block writes to X */
        stb_p(s->vram+offset+1, val >> 16);
        stb_p(s->vram+offset+2, val >> 8);
        stb_p(s->vram+offset+3, val);
        break;
    case 0x2000000:
        offset = ((addr << 1) & s->vram_amask) + ((addr >> 23) & 1);
        stb_p(s->vram+offset+0, val >> 24);
        stb_p(s->vram+offset+2, val >> 16);
        stb_p(s->vram+offset+4, val >> 8);
        stb_p(s->vram+offset+6, val);
        break;
    case 0x3000000:
        offset = ((addr << 2) & s->vram_amask) + ((addr >> 22) & 3);
        stb_p(s->vram+offset+0,  val >> 24);
        stb_p(s->vram+offset+4,  val >> 16);
        stb_p(s->vram+offset+8,  val >> 8);
        stb_p(s->vram+offset+12, val);
        break;
    }
}

static CPUReadMemoryFunc * const cg14_vram_read[3] = {
    cg14_vram_readb,
    cg14_vram_readw,
    cg14_vram_readl,
};

static CPUWriteMemoryFunc * const cg14_vram_write[3] = {
    cg14_vram_writeb,
    cg14_vram_writew,
    cg14_vram_writel,
};


static void cg14_set_monitor_id(CG14State *s)
{
    uint8_t id;

    /* pick something close, used as a default by Sun's OBP */
    if (s->width >= 1600) {
        id = CG14_MONID_1600x1280;
    } else if (s->width >= 1280) {
        id = CG14_MONID_1280x1024;
    } else if (s->width >= 1152) {
        id = CG14_MONID_1152x900;
    } else if (s->width >= 1024) {
        id = CG14_MONID_1024x768;
    } else {
        id = CG14_MONID_DEFAULT;
    }

    /* monitor code in bits 1..3 */
    s->ctrl.msr = id << 1;
}

static int cg14_init1(SysBusDevice *dev)
{
    CG14State *s = FROM_SYSBUS(CG14State, dev);
    ram_addr_t vram_offset;
    uint8_t *vram;
    int ctrl_memtype, vram_memtype;

    vram_offset = qemu_ram_alloc(NULL, "cg14.vram", s->vram_size);
    vram = qemu_get_ram_ptr(vram_offset);

    s->vram = vram;
    s->vram_amask = s->vram_size - 1;

    ctrl_memtype = cpu_register_io_memory(cg14_reg_read, cg14_reg_write, s,
                                          DEVICE_BIG_ENDIAN);
    sysbus_init_mmio(dev, CG14_REG_SIZE, ctrl_memtype);

    /* TODO: register first vram mapping as ram with dirty tracking */
    vram_memtype = cpu_register_io_memory(cg14_vram_read, cg14_vram_write,
                                          s, DEVICE_BIG_ENDIAN);
    sysbus_init_mmio(dev, CG14_VMEM_SLOTSIZE, vram_memtype);

    s->ds = graphic_console_init(cg14_update_display,
                                 cg14_invalidate_display,
                                 cg14_screen_dump, NULL, s);

    cg14_set_monitor_id(s);

    qemu_console_resize(s->ds, s->width, s->height);
    return 0;
}

/* save to file */
static void cg14_screen_dump(void *opaque, const char *filename)
{
    CG14State *s = opaque;
    FILE *f;
    int y, src_linesize, dst_linesize;
    void *buf;
    uint8_t *pix;
    cg14_draw_line_func *draw_line = NULL;

    switch (s->ctrl.mcr & CG14_MCR_PIXMODE_MASK) {
    case CG14_MCR_PIXMODE_8:
        src_linesize = s->width;
        // draw_line = cg14_draw_line8_bgr24;
        break;
    case CG14_MCR_PIXMODE_16:
        src_linesize = s->width * 2;
        // draw_line = cg14_draw_line16_bgr24;
        break;
    case CG14_MCR_PIXMODE_32:
        src_linesize = s->width * 4;
        // draw_line = cg14_draw_line32_bgr24;
        break;
    default:
        /* blank */
        return;
    }

    f = fopen(filename, "wb");
    if (!f) {
        return;
    }
    fprintf(f, "P6\n%d %d\n%d\n", s->width, s->height, 255);

    dst_linesize = s->width * 3;
    buf = qemu_mallocz(dst_linesize);
    pix = s->vram;

    for (y = 0; y < s->height; y++) {
        if (draw_line) {
            draw_line(s, buf, pix, s->width);
        }
        fwrite(buf, 1, dst_linesize, f);
        pix += src_linesize;
    }

    qemu_free(buf);
    fclose(f);
}

static void cg14_reset(DeviceState *d)
{
    CG14State *s = container_of(d, CG14State, busdev.qdev);

    /* set to 8bpp so last prom output might be visible */
    s->ctrl.mcr = CG14_MCR_VIDENABLE | CG14_MCR_PIXMODE_8;
    s->dirty = 1;
}

static SysBusDeviceInfo cg14_info = {
    .init = cg14_init1,
    .qdev.name = "cg14",
    .qdev.desc = "Sun CG14 Framebuffer",
    .qdev.size = sizeof(CG14State),
    .qdev.reset = cg14_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_HEX32("vram_size", CG14State, vram_size, 0x800000),
        DEFINE_PROP_UINT16("width",    CG14State, width,     0),
        DEFINE_PROP_UINT16("height",   CG14State, height,    0),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void cg14_register_devices(void)
{
    sysbus_register_withprop(&cg14_info);
}

device_init(cg14_register_devices)
