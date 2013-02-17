/*
 * QEMU CG3 Frame buffer
 *
 * Copyright (c) 2012 Bob Breuer
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
#include "loader.h"

//#define DEBUG_CG3

#define CG3_ROM_FILE  "./cg3.bin"
#define CG3_ROM_SIZE  2048
#define CG3_REG_SIZE  0x20
#define CG3_VRAM_SIZE 0x100000

#define CG3_REG_OFFSET  0x400000
#define CG3_VRAM_OFFSET 0x800000
#define CG3_SLOT_SIZE   (CG3_VRAM_OFFSET + CG3_VRAM_SIZE)


#ifdef DEBUG_CG3
#define DPRINTF(fmt, ...)                                       \
    printf("CG3: " fmt , ## __VA_ARGS__)
#else
#define DPRINTF(fmt, ...)
#endif


typedef struct CG3State {
    SysBusDevice busdev;
    qemu_irq irq;
    DisplayState *ds;

    ram_addr_t vram_offset;
    int full_update;
    uint16_t width, height;
    uint8_t regs[16];
    uint8_t r[256], g[256], b[256];
    uint8_t dac_index, dac_state;
} CG3State;

static void cg3_update_display(void *opaque)
{
    CG3State *s = opaque;
    const uint8_t *pix;
    uint32_t *data;
    uint32_t dval;
    int x, y, y_start;
    unsigned int width, height;
    ram_addr_t page, page1, page_min, page_max;
    ram_addr_t offset;

    if (ds_get_bits_per_pixel(s->ds) != 32) {
        return;
    }
    width = s->width;
    height = s->height;

    y_start = -1;
    page_min = -1;
    page_max = 0;
    offset = s->vram_offset;
    pix = qemu_get_ram_ptr(offset);
    data = (uint32_t*)ds_get_data(s->ds);

    for (y = 0; y < height; y++) {
        int update = s->full_update;

        page = offset & TARGET_PAGE_MASK;
        page1 = (offset + width - 1) & TARGET_PAGE_MASK;
        update |= cpu_physical_memory_get_dirty(page, VGA_DIRTY_FLAG);
        if (page != page1) {
            update |= cpu_physical_memory_get_dirty(page+1, VGA_DIRTY_FLAG);
        }
        if (update) {
            if (y_start < 0) {
                y_start = y;
            }
            if (page < page_min) {
                page_min = page;
            }
            page_max = page1;

            for (x = 0; x < width; x++) {
                dval = *pix++;
                dval = (s->r[dval] << 16) | (s->g[dval] << 8) | s->b[dval];
                *data++ = dval;
            }
        } else {
            if (y_start >= 0) {
                dpy_update(s->ds, 0, y_start, s->width, y - y_start);
                y_start = -1;
            }
            pix += width;
            data += width;
        }
        offset += width;
    }
    s->full_update = 0;
    if (y_start >= 0) {
        dpy_update(s->ds, 0, y_start, s->width, y - y_start);
    }
    if (page_max >= page_min) {
        cpu_physical_memory_reset_dirty(page_min, page_max + TARGET_PAGE_SIZE,
                                        VGA_DIRTY_FLAG);
    }
    /* vsync interrupt? */
    if (s->regs[0] & 0x80) {
        s->regs[1] |= 0x80;
        qemu_irq_raise(s->irq);
    }
}

static void cg3_invalidate_display(void *opaque)
{
    CG3State *s = opaque;
    unsigned int i;

    for (i = 0; i < CG3_VRAM_SIZE; i += TARGET_PAGE_SIZE) {
        cpu_physical_memory_set_dirty(s->vram_offset + i);
    }
}

static void cg3_screen_dump(void *opaque, const char *filename)
{
    CG3State *s = opaque;
    FILE *f;
    int x, y;
    uint8_t *pix;
    uint8_t v;

    f = fopen(filename, "wb");
    if (!f) {
        return;
    }
    fprintf(f, "P6\n%d %d\n%d\n", s->width, s->height, 255);

    pix = qemu_get_ram_ptr(s->vram_offset);

    for (y = 0; y < s->height; y++) {
        for (x = 0; x < s->width; x++) {
            v = *pix++;
            fputc(s->r[v], f);
            fputc(s->g[v], f);
            fputc(s->b[v], f);
        }
    }

    fclose(f);
}

static uint32_t cg3_reg_readb(void *opaque, target_phys_addr_t addr)
{
    CG3State *s = opaque;
    int val;

    switch (addr) {
    case 0x10:
        val = s->regs[0];
        break;
    case 0x11:
        val = s->regs[1] | 0x71; /* monitor ID 7, board type = 1 (color) */
        break;
    case 0x12 ... 0x1f:
        val = s->regs[addr - 0x10];
        break;
    default:
        val = 0;
        break;
    }
    DPRINTF("readb %02x from reg %x\n", val, (int)addr);
    return val;
}

static void cg3_reg_writeb(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CG3State *s = opaque;

    DPRINTF("writeb %02x to reg %x\n", val, (int)addr);

    switch (addr) {
    case 0:
        s->dac_index = val;
        s->dac_state = 0;
        break;
    case 4:
        switch (s->dac_state) {
        case 0:
            s->r[s->dac_index] = val;
            s->dac_state++;
            break;
        case 1:
            s->g[s->dac_index] = val;
            s->dac_state++;
            break;
        case 2:
            s->b[s->dac_index] = val;
            s->dac_index = (s->dac_index + 1) & 255; // Index autoincrement
        default:
            s->dac_state = 0;
            break;
        }
        s->full_update = 1;
        break;
    case 0x10:
        s->regs[0] = val;
        break;
    case 0x11:
        if (s->regs[1] & 0x80) {
            /* clear interrupt */
            s->regs[1] &= ~0x80;
            qemu_irq_lower(s->irq);
        }
        break;
    case 0x12 ... 0x1f:
        s->regs[addr - 0x10] = val;
        break;
    default:
        break;
    }
}

static void cg3_reg_writel(void *opaque, target_phys_addr_t addr, uint32_t val)
{
    CG3State *s = opaque;
    uint8_t dac_next;

    DPRINTF("writel %08x to reg %x\n", val, (int)addr);

    switch (addr) {
    case 0:
        s->dac_index = val;
        s->dac_state = 0;
        break;
    case 4:
        /* strange, as it seems to be an 8-bit bus */
        dac_next = (s->dac_index + 1) & 255;
        switch (s->dac_state) {
        case 0:
            s->r[s->dac_index] = val >> 24;
            s->g[s->dac_index] = val >> 16;
            s->b[s->dac_index] = val >> 8;
            s->r[dac_next] = val;
            s->dac_state++;
            break;
        case 1:
            s->g[s->dac_index] = val >> 24;
            s->b[s->dac_index] = val >> 16;
            s->r[dac_next] = val >> 8;
            s->g[dac_next] = val;
            s->dac_state++;
            break;
        case 2:
            s->b[s->dac_index] = val >> 24;
            s->r[dac_next] = val >> 16;
            s->g[dac_next] = val >> 8;
            s->b[dac_next] = val;
            dac_next = (dac_next + 1) & 255;
            /* fall-through */
        default:
            s->dac_state = 0;
            break;
        }
        s->dac_index = dac_next;
        s->full_update = 1;
        break;
    default:
        break;
    }
}

static CPUReadMemoryFunc * const cg3_reg_read[3] = {
    cg3_reg_readb,
    NULL,
    NULL,
};

static CPUWriteMemoryFunc * const cg3_reg_write[3] = {
    cg3_reg_writeb,
    NULL,
    cg3_reg_writel,
};

/* everything is an offset within our sbus slot */
static void cg3_sbus_map(SysBusDevice *dev, target_phys_addr_t base)
{
    CG3State *s = FROM_SYSBUS(CG3State, dev);
    ram_addr_t rom_offset;
    int regs, ret;

    rom_offset = qemu_ram_alloc(NULL, "cg3.rom", CG3_ROM_SIZE);
    cpu_register_physical_memory(base, CG3_ROM_SIZE, rom_offset|IO_MEM_ROM);

    ret = load_image_targphys(CG3_ROM_FILE, base, CG3_ROM_SIZE);
    if (ret <= 0) {
        hw_error("CG3 failure loading rom: '%s'", CG3_ROM_FILE);
    }

    regs = cpu_register_io_memory(cg3_reg_read, cg3_reg_write, s, DEVICE_NATIVE_ENDIAN);
    cpu_register_physical_memory(base+CG3_REG_OFFSET, CG3_REG_SIZE, regs);

    cpu_register_physical_memory(base+CG3_VRAM_OFFSET, CG3_VRAM_SIZE, s->vram_offset);
}

static int cg3_init1(SysBusDevice *dev)
{
    CG3State *s = FROM_SYSBUS(CG3State, dev);

    s->vram_offset = qemu_ram_alloc(NULL, "cg3.vram", CG3_VRAM_SIZE);

    sysbus_init_mmio_cb(dev, CG3_SLOT_SIZE, cg3_sbus_map);
    sysbus_init_irq(dev, &s->irq);

    s->ds = graphic_console_init(cg3_update_display,
                                 cg3_invalidate_display,
                                 cg3_screen_dump, NULL, s);
    qemu_console_resize(s->ds, s->width, s->height);

    return 0;
}

static void cg3_reset(DeviceState *d)
{
    CG3State *s = container_of(d, CG3State, busdev.qdev);

    s->full_update = 1;
    qemu_irq_lower(s->irq);
}

static SysBusDeviceInfo cg3_info = {
    .init = cg3_init1,
    .qdev.name = "SUNW,cg3",
    .qdev.desc = "Sun CG3 Framebuffer",
    .qdev.size = sizeof(CG3State),
    .qdev.reset = cg3_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT16("width",    CG3State, width,  1152),
        DEFINE_PROP_UINT16("height",   CG3State, height, 900),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void cg3_register_devices(void)
{
    sysbus_register_withprop(&cg3_info);
}

device_init(cg3_register_devices)
