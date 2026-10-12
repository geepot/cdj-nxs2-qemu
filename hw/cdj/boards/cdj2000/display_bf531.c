/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "sh7763.h"
#include "cdj_getenv.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "ui/surface.h"
#include "cdj_gui_keys.h"
#include "bfin/bf531.h"
/*
 * CDJ-2000/CDJ-2000NXS display processor: the ADSP-BF531 on the other end
 * of SPORT1, in its own window. The core itself is plain C with no QEMU
 * dependency (hw/cdj/bfin/); this file is the QEMU side of it, a display
 * surface and a timer that steps the chip on its own clock.
 *
 * SPORT1 is MAIN's link (display_link.c): a MAIN packet lands the moment
 * the firmware arms its receive DMA, and each answer it transmits goes
 * straight to MAIN's receive block. CDJ_BF531_SPORT1 hands it one
 * hex-encoded packet the same way, for bring-up without MAIN's sender.
 */

#define CDJ_BF531_W     480
#define CDJ_BF531_H     255

/* One PPI/DMA0 unit's worth of the display processor's own 400 MHz clock
 * (see bf531.c's dma_start_unit): stepping this much a QEMU timer tick
 * keeps its own frame rate without syncing it to MAIN's icount. */
#define CDJ_BF531_QUANTUM (BF531_CCLK_HZ / cdj_bf531_tick_hz)

/* CDJ_BF531_TICK_HZ: ticks per virtual second (default 60). A link that
 * answers frame by frame pays one or two ticks per round trip, so it needs a
 * finer quantum than the stock firmware's lockstep. */
static unsigned cdj_bf531_tick_hz = 60;

typedef struct CdjBf531 {
    bf531 *chip;
    QemuConsole *con;
    QEMUTimer *tick;
    unsigned w, h;
    uint64_t frames;
    GByteArray *snap;           /* the chip's image, only while saving or loading */
    Notifier exit;
} CdjBf531;

static CdjBf531 cdj_bf531;

void cdj2000_display_send(const uint8_t *pkt, size_t len)
{
    bf531_sport1_rx(cdj_bf531.chip, pkt, len);
}

static void cdj_bf531_answer(void *opaque, const uint8_t *pkt, size_t len)
{
    cdj2000_display_link_receive(pkt, len);
}

/* PF1, which MAIN reads as bit 2 of its port latch +0x48: low once the
 * firmware's link is up, and set (FIO_FLAG_S at 0xB7C13E) just before each
 * answer it transmits. MAIN starts the link on a 0 and takes an answer only
 * when the bit is 1; any other answer it logs as the display asking for the
 * packet again (0x042134CC). */
bool cdj2000_display_pf1(void)
{
    return bf531_flags(cdj_bf531.chip) & (1u << 1);
}

static void cdj_bf531_frame(void *opaque, const uint16_t *px, unsigned w, unsigned h)
{
    CdjBf531 *s = opaque;
    DisplaySurface *ds;
    uint32_t *dst;
    unsigned n;

    if (w != s->w || h != s->h) {
        dpy_gfx_replace_surface(s->con, qemu_create_displaysurface(w, h));
        qemu_console_resize(s->con, w, h);
        s->w = w;
        s->h = h;
    }
    ds = qemu_console_surface(s->con);
    dst = (uint32_t *)surface_data(ds);
    for (n = 0; n < (unsigned)w * h; n++) {
        uint16_t p = px[n];

        dst[n] = 0xFF000000 | (((p >> 10) & 0x1F) * 255 / 31) << 16 |
                 (((p >> 5) & 0x1F) * 255 / 31) << 8 | ((p & 0x1F) * 255 / 31);
    }
    s->frames++;
    dpy_gfx_update(s->con, 0, 0, w, h);
}

static void cdj_bf531_invalidate(void *opaque)
{
}

static void cdj_bf531_update(void *opaque)
{
}

static const GraphicHwOps cdj_bf531_ops = {
    .invalidate = cdj_bf531_invalidate,
    .gfx_update = cdj_bf531_update,
};

static void cdj_bf531_tick(void *opaque)
{
    CdjBf531 *s = opaque;
    bfin_stop stop = bf531_run(s->chip, CDJ_BF531_QUANTUM);

    if (stop == BFIN_STOP_UNDEF) {
        error_report("cdj_bf531: unimplemented instruction 0x%" PRIx64
                     " at 0x%08x", bfin_trap_insn(bf531_core(s->chip)),
                     bfin_trap_pc(bf531_core(s->chip)));
        return;
    }
    timer_mod(s->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / cdj_bf531_tick_hz);
}

static void cdj_bf531_report(Notifier *n, void *opaque)
{
    CdjBf531 *s = container_of(n, CdjBf531, exit);

    static const struct { const char *name; uint32_t addr, len; } banks[] = {
        { "l1a.bin", 0xFF800000, 0x8000 }, { "l1b.bin", 0xFF900000, 0x8000 },
        { "l1s.bin", 0xFFB00000, 0x1000 },
    };
    const char *dir = getenv("CDJ_BF531_DUMP");

    info_report("cdj_bf531: %" PRIu64 " PPI frames rendered", s->frames);
    /* CDJ_BF531_DUMP=<dir>: the GUI's memory at exit, for offline decoding. */
    if (dir && s->chip) {
        uint32_t size;
        const uint8_t *sdram = bf531_sdram(s->chip, &size);
        g_autofree char *p = g_build_filename(dir, "sdram.bin", NULL);

        if (!g_file_set_contents(p, (const char *)sdram, size, NULL)) {
            error_report("cdj_bf531: cannot write %s", p);
        }
        for (unsigned i = 0; i < ARRAY_SIZE(banks); i++) {
            g_autofree char *q = g_build_filename(dir, banks[i].name, NULL);
            const uint8_t *m = bf531_mem(s->chip, banks[i].addr, banks[i].len);

            if (!m || !g_file_set_contents(q, (const char *)m, banks[i].len, NULL)) {
                error_report("cdj_bf531: cannot write %s", q);
            }
        }
        info_report("cdj_bf531: memory dumped to %s", dir);
    }
}

/* The chip runs only inside the tick, on the main loop, so between two ticks
 * it is always at a point it can be saved from. Its SDRAM is in the image:
 * tens of MiB, held only while the save or load lasts. */
static int cdj_bf531_pre_save(void *opaque)
{
    CdjBf531 *s = opaque;
    size_t len = bf531_save(s->chip, NULL);

    s->snap = g_byte_array_sized_new(len);
    g_byte_array_set_size(s->snap, len);
    bf531_save(s->chip, s->snap->data);
    return 0;
}

static int cdj_bf531_post_save(void *opaque)
{
    CdjBf531 *s = opaque;

    g_byte_array_free(s->snap, true);
    s->snap = NULL;
    return 0;
}

static int cdj_bf531_post_load(void *opaque, int version_id)
{
    CdjBf531 *s = opaque;
    bool fits = s->snap && s->snap->len == bf531_save(s->chip, NULL);

    if (fits) {
        bf531_load(s->chip, s->snap->data);
    } else {
        error_report("cdj_bf531: the saved display does not fit this machine");
    }
    cdj_bf531_post_save(s);
    return fits ? 0 : -EINVAL;
}

static const VMStateDescription vmstate_cdj_bf531 = {
    .name = "cdj-bf531",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = cdj_bf531_pre_save,
    .post_save = cdj_bf531_post_save,
    .post_load = cdj_bf531_post_load,
    .fields = (const VMStateField[]) {
        CDJ_VMSTATE_BYTES(snap, CdjBf531),
        VMSTATE_TIMER_PTR(tick, CdjBf531),
        VMSTATE_UINT64(frames, CdjBf531),
        VMSTATE_END_OF_LIST()
    }
};

static size_t cdj_bf531_parse_hex(const char *s, uint8_t *out, size_t cap)
{
    size_t n = 0;

    while (s[0] && s[1] && n < cap) {
        unsigned byte;

        if (sscanf(s, "%2x", &byte) != 1) {
            break;
        }
        out[n++] = byte;
        s += 2;
    }
    return n;
}

/*
 * Host keyboard to front panel (cdj_gui_keys.c), the NXS2 window's layout on
 * this model's report bytes. The source keys sit one bit lower than on the
 * NXS2 because of the extra SD key, F. Only the keys with a bit in the
 * decoder's name table are here: no loop, slip, reverse, sync, master, tempo
 * range, master tempo or jog mode, and no REKORDBOX key (LINK is the first
 * source key). Keys pressed on the deck and seen to act are confirmed.
 */
static const CdjGuiKey cdj_bf531_keys[] = {
    { Q_KEY_CODE_SPC,           0x10, 0x01, "PLAY/PAUSE",   false },
    { Q_KEY_CODE_C,             0x10, 0x02, "CUE",          false },
    { Q_KEY_CODE_RET,           0x11, 0x01, "ROTARY PUSH",  true  },
    { Q_KEY_CODE_KP_ENTER,      0x11, 0x01, "ROTARY PUSH",  true  },
    { Q_KEY_CODE_RIGHT,         0x11, 0x01, "ROTARY PUSH",  true  },
    { Q_KEY_CODE_COMMA,         0x12, 0x02, "TRACK -",      false },
    { Q_KEY_CODE_DOT,           0x12, 0x04, "TRACK +",      false },
    { Q_KEY_CODE_BRACKET_LEFT,  0x12, 0x08, "SEARCH -",     false },
    { Q_KEY_CODE_BRACKET_RIGHT, 0x12, 0x10, "SEARCH +",     false },
    { Q_KEY_CODE_L,             0x13, 0x01, "LINK",         false },
    { Q_KEY_CODE_U,             0x13, 0x02, "USB",          true  },
    { Q_KEY_CODE_F,             0x13, 0x04, "SD",           false },
    { Q_KEY_CODE_D,             0x13, 0x08, "DISC",         false },
    { Q_KEY_CODE_B,             0x14, 0x01, "BROWSE",       true  },
    { Q_KEY_CODE_T,             0x14, 0x02, "TAG LIST",     false },
    { Q_KEY_CODE_I,             0x14, 0x04, "INFO",         false },
    { Q_KEY_CODE_M,             0x14, 0x08, "MENU",         false },
    { Q_KEY_CODE_ESC,           0x14, 0x10, "BACK",         false },
    { Q_KEY_CODE_LEFT,          0x14, 0x10, "BACK",         false },
    { Q_KEY_CODE_BACKSPACE,     0x14, 0x10, "BACK",         false },
};

/*
 * The CDJ-2000NXS decoder (the update's key-state writer) keeps the same
 * report bytes as the NXS2: payload bit to state bit, checked against the
 * name table for every key here. Its source keys are LINK, USB, SD and DISC
 * on bits 1-4 of byte 0x13 (bit 0 is the unnamed state bit +94/0x10). Left out
 * because the state bit they set carries no key name: slip, direction, sync,
 * master and the REKORDBOX key.
 */
static const CdjGuiKey cdj_bf531_nxs_keys[] = {
    { Q_KEY_CODE_SPC,           0x10, 0x01, "PLAY/PAUSE",   false },
    { Q_KEY_CODE_C,             0x10, 0x02, "CUE",          false },
    { Q_KEY_CODE_E,             0x10, 0x04, "RELOOP/EXIT",  false },
    { Q_KEY_CODE_W,             0x10, 0x08, "LOOP OUT",     false },
    { Q_KEY_CODE_Q,             0x10, 0x10, "LOOP IN",      false },
    { Q_KEY_CODE_RET,           0x11, 0x01, "ROTARY PUSH",  false },
    { Q_KEY_CODE_KP_ENTER,      0x11, 0x01, "ROTARY PUSH",  false },
    { Q_KEY_CODE_RIGHT,         0x11, 0x01, "ROTARY PUSH",  false },
    { Q_KEY_CODE_COMMA,         0x12, 0x04, "TRACK -",      false },
    { Q_KEY_CODE_DOT,           0x12, 0x08, "TRACK +",      false },
    { Q_KEY_CODE_BRACKET_LEFT,  0x12, 0x10, "SEARCH -",     false },
    { Q_KEY_CODE_BRACKET_RIGHT, 0x12, 0x20, "SEARCH +",     false },
    { Q_KEY_CODE_L,             0x13, 0x02, "LINK",         false },
    { Q_KEY_CODE_U,             0x13, 0x04, "USB",          false },
    { Q_KEY_CODE_F,             0x13, 0x08, "SD",           false },
    { Q_KEY_CODE_D,             0x13, 0x10, "DISC",         false },
    { Q_KEY_CODE_B,             0x14, 0x01, "BROWSE",       false },
    { Q_KEY_CODE_T,             0x14, 0x02, "TAG LIST",     false },
    { Q_KEY_CODE_I,             0x14, 0x04, "INFO",         false },
    { Q_KEY_CODE_M,             0x14, 0x08, "MENU",         false },
    { Q_KEY_CODE_ESC,           0x14, 0x10, "BACK",         false },
    { Q_KEY_CODE_LEFT,          0x14, 0x10, "BACK",         false },
    { Q_KEY_CODE_BACKSPACE,     0x14, 0x10, "BACK",         false },
    { Q_KEY_CODE_J,             0x15, 0x01, "JOG MODE",     false },
    { Q_KEY_CODE_P,             0x15, 0x08, "TEMPO RANGE",  false },
    { Q_KEY_CODE_K,             0x15, 0x10, "MASTER TEMPO", false },
};

const Cdj2000Display cdj2000_display = {
    .sdram_size = 16 * MiB,
    .keys = cdj_bf531_keys,
    .key_count = ARRAY_SIZE(cdj_bf531_keys),
    .lever_fwd = true,
};

const Cdj2000Display cdj2000nxs_display = {
    .sdram_size = 32 * MiB,
    .keys = cdj_bf531_nxs_keys,
    .key_count = ARRAY_SIZE(cdj_bf531_nxs_keys),
    .lever_fwd = true,
};

/* CDJ_BF531_UPD=<path to a Pioneer GUI .UPD section> turns the display
 * processor on; without it this board runs MAIN alone, as before. */
bool cdj2000_display_init(const Cdj2000Display *desc)
{
    CdjBf531 *s = &cdj_bf531;
    bf531_host host = { s, cdj_bf531_frame, cdj_bf531_answer };
    const char *img_path = getenv("CDJ_BF531_UPD");
    const char *sport1 = getenv("CDJ_BF531_SPORT1");
    const char *tick_hz = getenv("CDJ_BF531_TICK_HZ");
    uint8_t *img;
    gsize len;

    if (desc->init) {
        desc->init();
        return false;
    }
    if (!img_path) {
        return false;
    }
    if (tick_hz) {
        unsigned hz = strtoul(tick_hz, NULL, 10);

        if (hz < 60 || hz > 100000) {
            error_report("cdj_bf531: CDJ_BF531_TICK_HZ=%s out of 60..100000", tick_hz);
            exit(1);
        }
        cdj_bf531_tick_hz = hz;
    }
    if (!g_file_get_contents(img_path, (gchar **)&img, &len, NULL)) {
        error_report("cdj_bf531: cannot read '%s'", img_path);
        return false;
    }

    s->chip = bf531_new(desc->sdram_size, &host, stderr);
    if (sport1) {
        uint8_t pkt[128];
        size_t n = cdj_bf531_parse_hex(sport1, pkt, sizeof(pkt));

        bf531_sport1_rx(s->chip, pkt, n);
    }
    if (bf531_load_update(s->chip, img, len)) {
        error_report("cdj_bf531: '%s' is not an LDR boot stream", img_path);
        bf531_free(s->chip);
        s->chip = NULL;
        g_free(img);
        return false;
    }
    g_free(img);

    s->w = CDJ_BF531_W;
    s->h = CDJ_BF531_H;
    s->con = graphic_console_init(NULL, 0, &cdj_bf531_ops, s);
    dpy_gfx_replace_surface(s->con, qemu_create_displaysurface(s->w, s->h));
    qemu_console_resize(s->con, s->w, s->h);
    cdj_gui_keys_init(desc->keys, desc->key_count, "cdj_bf531");
    cdj_gui_pointer_init();

    s->exit.notify = cdj_bf531_report;
    cdj_add_exit_report(&s->exit);

    s->tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, cdj_bf531_tick, s);
    timer_mod(s->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    vmstate_register_any(NULL, &vmstate_cdj_bf531, s);
    return true;
}
