/* SPDX-License-Identifier: GPL-3.0-only */
/* The Snoop37 screen mirror (firmware/src/mirror.c, firmware/src/mirror_proto.h, tools/mirror/mirror_decode.c) on the
 * UI harness of tests/ui_pages_test.c: every lcd_blit / lcd_fill of the real UI goes into the encoder (the harness's
 * LCD_HOOK_* stubs), packets go to the viewer's decoder, its picture is compared with the screen. mirror.c's
 * MIRROR_MS / MIRROR_WAIT are this test's clock and its reader.
 *
 *   part 1 (pass / fail)
 *     reference  tests/ui_pages_test.c's own main (every page, layer, screen, visualiser style, the menu, then its
 *                20000 frames of random use), the stream drained after every lcd call and at every wait of
 *                mirror_blit (a reader as fast as the writer: what is tested here is the content) and every UI pass:
 *                after every pass the decoder's picture == the screen, all 57600 pixels; no record dropped; the
 *                bytes counted here (a copy of the encoder's RLE sizing) == mirror_stat.bytes. Packets of 1023,
 *                196, 64. Also listed: the lcd_blit calls bigger than the ring (they wait for the reader)
 *     loss       the same with 1 packet in 50 lost (windows of 300 passes with losses, 40 without): MD_RESYNC sets
 *                mirror_redraw as usb.c will; at the end of every clean window, once resynced: picture == screen
 *     encoder    random fills / blits (noise, runs of every length, 1..240 wide), packets of 5..1023 bytes; on the
 *                device's timing: a 240x240 blit waits at most the panel's transfer, the repair once the backlog
 *                is <= RING / 4; a reader that stops: one wait of MIRROR_STALL_MS, then none; MIRROR_HOLD
 *     protocol   losses at the worst places: a packet lost while the viewer is already out of step (the redraw's
 *                first packet); the last packet before a still screen (seen by the beat 'E', MIRROR_BEAT_MS)
 *     stall      the visualiser, P=512, the viewer stops reading for 1 s: one wait of MIRROR_STALL_MS at most,
 *                the UI's pace kept; it reads again: the picture right again
 *     fuzz       the decoder fed random and mutated packets (FUZZ_N, default 3 000 000; each in a buffer of its
 *                exact size: build with -fsanitize=address,undefined): no fault, its state stays in bounds
 *   part 2 (measure: a table, the verdict "fits" = no record dropped after the first second) columns: UI passes a
 *     second (each scenario also with the mirror off: the UI's own pace); stream bytes a pass (mean, max: records +
 *     'E', not the 4-byte packet headers); bytes a second and that as a share of the USB's (P - 4) * 1000; raw /
 *     stream (raw: 2 bytes a pixel drawn); records dropped (all, after the first second); full redraws forced by
 *     mirror_frame_end; the most bytes in the ring; the biggest single lcd call; mirror_stat.waits; ms a second
 *     spent inside mirror_blit (the panel's transfer runs meanwhile) and past the transfer (what the UI pays).
 *     The device's timing on a virtual clock: lcd_fill / lcd_blit of N pixels take N * 16 / 12 us (SPI 12 MHz;
 *     mirror_blit runs alongside the blit's transfer, the next lcd call after both), a UI pass no faster than
 *     every 15 ms, one USB packet of P bytes each millisecond; the audio runs for the time that went by.
 *     Scenarios: a every static screen (first draw, 2 s idle), b1 normal use (pages, knobs, keys, the sequencer
 *     running, 10 s), b2 the reference's random use (10 s), c every visualiser style (5 s each, playing),
 *     d the Snoop37 boot intro (snoop_splash.c drawn here as mk_frame does, 5.2 s, then the fade-in passes).
 *     The ring is MIRROR_RING (build with -DMIRROR_RING=16384u etc.). Not modelled: the CPU time of drawing and of
 *     the encoder (the passes are SPI-bound here), so the time inside mirror_blit is shown, not charged: on the
 *     device it is taken from drawing that would otherwise run during the transfer; lcd_dim (the intro's fade-in
 *     is drawn at full brightness). MT_DETAIL=1: every static screen.
 *
 *   cc -O2 -w -Ibuild/gen -Ifirmware/src -Ifirmware/hal -o build/host/mirror_test tests/mirror_test.c -lm
 *   build/host/mirror_test build/host/mirror               part 1
 *   build/host/mirror_test build/host/mirror measure       part 2 (add -DMIRROR_RING=16384u builds)
 * This is the FM-1 (SLOOP) UI harness: the SMK-37 build's own screens (DAW, pad picker, FADER BANK, its menu
 * rows) are not in it; the Snoop37 intro is (scenario d). */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <setjmp.h>
#include "../tools/mirror/mirror_decode.c"
/* mirror.c's clock and its wait for the reader: the virtual clock (part 2: the device's timing; part 1: the UI's
 * fm1_ms, plus 1 ms a wait, the reader draining everything at each) */
static uint64_t vt_us;
static uint32_t mt_ms(void);
static void mt_wait(void);
#define MIRROR_MS() mt_ms()
#define MIRROR_WAIT() mt_wait()
#include "../firmware/src/mirror.c"

static void mt_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint16_t *p);
static void mt_fill(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t c);
static void mt_pass(void);
#define LCD_HOOK_BLIT(x, y, w, h, p) mt_blit(x, y, w, h, p)
#define LCD_HOOK_FILL(x, y, w, h, c) mt_fill(x, y, w, h, c)
#define UI_PASS_HOOK() mt_pass()
#define UI_PAGES_MAIN ui_pages_main
#include "ui_pages_test.c"
#define MK_HOST 1
#include "../firmware/src/snoop_jingle.c"
#include "../firmware/src/snoop_splash.c"

/* ---------------------------------------------------------------- the link */
static mirror_dec_t dec;
static struct {
    int timed;                     /* part 2: the virtual clock; else every lcd call drained at once */
    uint32_t P;                    /* packet size */
    uint32_t loss;                 /* 1 in loss packets lost (0: none) */
    int lossy;                     /* losses on now */
    int check;                     /* compare after every pass */
    int paused;                    /* the reader takes nothing (a viewer that stopped reading) */
    int frozen;                    /* no time passes and no reader runs (interrupts off) */
} mt = {0, 1023u, 0, 0, 1, 0, 0};
static struct {
    uint64_t passes, cmp, bad_passes, bad_px, pkts, lost, resyncs, forced, offered, px;
    uint32_t windows, win_ok, win_bad, win_unsettled, win_tail;
    int lost_tail;                 /* a packet lost since the last one delivered */
} st;
static uint32_t mt_seed = 12345u;
static uint32_t mt_rnd(uint32_t n) { mt_seed = mt_seed * 1664525u + 1013904223u; return (mt_seed >> 8) % (n ? n : 1u); }
static FILE *rep;                  /* the report (stdout before the reference's own output goes to /dev/null) */

/* the encoder's sizes, counted here (as mirror.c mr_rle): what a record costs whether it fits or not */
static uint32_t rle_size(const uint16_t *p, uint32_t n)
{
    uint32_t i = 0, j, s = 0;
    while (i < n) {
        for (j = i + 1u; j < n && j - i < 127u && p[j] == p[i]; j++)
            ;
        if (j - i >= 2u) {
            s += 3u;
            i = j;
            continue;
        }
        for (j = i + 1u; j < n && j - i < 128u; j++)
            if (j + 2u < n && p[j] == p[j + 1u] && p[j] == p[j + 2u])
                break;
        s += 1u + 2u * (j - i);
        i = j;
    }
    return s;
}
static uint32_t blit_size(uint32_t w, uint32_t h, const uint16_t *p)
{
    uint32_t rows = MIRROR_BAND_PX / w, r, s = 0;
    while (h) {
        r = h < rows ? h : rows;
        s += 7u + rle_size(p, w * r);
        p += w * r;
        h -= r;
    }
    return s;
}

static int deliver(const uint8_t *b, uint32_t n)
{
    int r;
    uint8_t *q = malloc(n);        /* (its exact size: ASan sees a read past the end) */
    memcpy(q, b, n);
    r = md_packet(&dec, q, n);
    free(q);
    if (r & MD_RESYNC) {
        st.resyncs++;
        mirror_redraw = 1;         /* (usb.c: the viewer's MIRROR_REQ_RESYNC) */
    }
    return r;
}
/* one USB packet; 0: nothing to send */
/* the stream bytes the viewer never applied: a lost packet, and what it skips after it until it picks the stream up
 * again at a record's start ([start, end) in mirror.c's free-running stream offsets, in order) */
#define NSPAN 65536u
static uint32_t span_a[NSPAN], span_b[NSPAN], nspan;
static int span_open;
static void span_note(uint32_t rd0, const uint8_t *b, uint32_t n, int lost)
{
    uint32_t first = (uint32_t)b[2] | (uint32_t)b[3] << 8;
    if (lost) {
        if (!span_open && nspan < NSPAN) {
            span_a[nspan] = rd0;
            span_open = 1;
        }
        return;
    }
    if (span_open && first != MIRROR_NO_FIRST && first < n - MIRROR_PKT_HDR) {
        span_b[nspan++] = rd0 + first;
        span_open = 0;
    }
}
/* the record [a, b) was not applied by the viewer */
static int span_hit(uint32_t a, uint32_t b)
{
    uint32_t lo = 0, hi = nspan;
    if (span_open && nspan < NSPAN && b > span_a[nspan])
        return 1;
    while (lo < hi) {                              /* (the last span starting before b) */
        uint32_t m = (lo + hi) / 2u;
        if (span_a[m] < b) lo = m + 1u; else hi = m;
    }
    return lo > 0u && span_b[lo - 1u] > a;
}
static int usb_take(void)
{
    uint8_t b[1024];
    uint32_t rd0 = mr_rd, n = mt.paused ? 0u : mirror_packet(b, mt.P);
    if (!n)
        return 0;
    if (n <= MIRROR_PKT_HDR || n > mt.P) {
        fprintf(rep, "mirror: FAIL mirror_packet returned %u bytes (max %u)\n", n, mt.P);
        exit(2);
    }
    st.pkts++;
    if (mt.lossy && mt.loss && mt_rnd(mt.loss) == 0) {
        st.lost++;
        st.lost_tail = 1;
        span_note(rd0, b, n, 1);
        return 1;
    }
    st.lost_tail = 0;
    span_note(rd0, b, n, 0);
    deliver(b, n);
    return 1;
}
static void drain(void) { while (usb_take()) ; }

/* ---------------------------------------------------------------- the virtual clock (part 2) */
static uint64_t usb_next_us, aud_smp;
static uint32_t ms_base;
static struct {
    uint64_t t0, passes, bytes, raw, pass_max, drops0, steady_from, steady_drops0, forced, frames;
    uint32_t backlog, burst, waits0;
    uint64_t wait_us, over_us, wait_max;   /* time inside mirror_blit; past the panel's transfer (what the UI pays) */
    int steady_taken;
} ms;
static uint32_t diff_count(int *bx0, int *by0, int *bx1, int *by1);
static int settle_watch, settle_ok;
static void vt_go(uint64_t us)
{
    vt_us += us;
    while (usb_next_us <= vt_us) {
        usb_take();
        usb_next_us += 1000u;
        if (settle_watch && !settle_ok && mr_wr == mr_rd && dec.in_sync) {   /* (all sent: the picture?) */
            int a, b, c, d;
            settle_ok = !diff_count(&a, &b, &c, &d);
        }
    }
    if (!ms.steady_taken && vt_us >= ms.steady_from) {
        ms.steady_taken = 1;
        ms.steady_drops0 = mirror_stat.drops;
    }
}
static void lcd_time(uint64_t px)
{
    uint32_t b = mr_wr - mr_rd;
    if (b > ms.backlog)
        ms.backlog = b;
    vt_go(px * 16u / 12u);
}
static uint32_t p1_wait_ms;                          /* (part 1: the waits, on top of the UI's fm1_ms) */
static uint32_t mt_ms(void) { return mt.timed ? ms_base + (uint32_t)(vt_us / 1000u) : fm1_ms + p1_wait_ms; }
static jmp_buf frozen_out;
static uint32_t frozen_spins;
static void mt_wait(void)
{
    if (mt.frozen) {                                /* (bounded here: on the device this loop would not end) */
        if (++frozen_spins >= 10000000u)
            longjmp(frozen_out, 1);
        return;
    }
    if (mt.timed) {
        vt_go(1000u);
    } else {
        p1_wait_ms++;
        drain();
    }
}

/* the loss test: who drew each pixel last (pass, screen, call, the stream bytes of its record), and which pixels each
 * forced pass (ui.force) draws, per screen */
#define NSCR 96u
static char scr_names[NSCR][40];
static uint32_t nscr;
static struct { uint32_t n, min, max; uint64_t sum; } scr_st[NSCR];
static uint8_t scr_ever[NSCR][240 * 240];         /* drawn by some forced pass of that screen */
static uint32_t px_a[240 * 240], px_b[240 * 240], px_pass[240 * 240], px_gen[240 * 240];
static uint8_t px_scr[240 * 240], px_kind[240 * 240], px_rect[240 * 240][4];
static uint32_t pass_gen = 1;
#define NFORCED 65536u
static uint32_t forced_pass[NFORCED], nforced;      /* (the passes that were forced, in order) */
static int pass_first = 1, pass_forced;
static uint32_t scr_id(void)
{
    static const char *const LY[] = {"PLAY", "FX", "ERASE", "ROLL", "STEP", "SCALE", "MIX", "SONG"};
    char b[40];
    uint32_t i;
    if (ui.menu) snprintf(b, sizeof b, ui.menu == 2 ? "about" : "menu");
    else if (ui.hold_kind) snprintf(b, sizeof b, "hold screen %u", (unsigned)ui.hold_kind);
    else if (ui.layer != LY_PLAY) snprintf(b, sizeof b, "layer %s", LY[ui.layer % 8u]);
    else if (vis_shown()) snprintf(b, sizeof b, "vis %s", VIS_NAME[vis_style % VIS_N]);
    else if (rec_wait || ft_on) snprintf(b, sizeof b, "REC");
    else if (ui.home) snprintf(b, sizeof b, "HOME");
    else snprintf(b, sizeof b, "%.24s%s", cur_page()->title, on_drum_page() ? " (drums)" : "");
    for (i = 0; i < nscr; i++)
        if (!strcmp(scr_names[i], b))
            return i;
    if (nscr == NSCR)
        return NSCR - 1u;
    snprintf(scr_names[nscr], 40, "%s", b);
    return nscr++;
}
static void pix_note(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t a, uint32_t b, int kind, uint32_t cx, uint32_t cy,
                     uint32_t cw, uint32_t ch)
{
    uint32_t i, j, s;
    if (pass_first) {                              /* (the pass's first call: ui_draw clears ui.force only at its end) */
        pass_first = 0;
        pass_forced = ui.force;
    }
    s = scr_id();
    for (j = y; j < y + h; j++)
        for (i = x; i < x + w; i++) {
            uint32_t k = j * 240u + i;
            px_a[k] = a, px_b[k] = b, px_pass[k] = (uint32_t)st.passes + 1u, px_gen[k] = pass_gen;
            px_scr[k] = (uint8_t)s, px_kind[k] = (uint8_t)kind;
            px_rect[k][0] = (uint8_t)cx, px_rect[k][1] = (uint8_t)cy, px_rect[k][2] = (uint8_t)cw, px_rect[k][3] = (uint8_t)ch;
        }
}
/* after a pass of the loss test: the pixels a forced pass did not draw, per screen */
static void pix_pass_end(void)
{
    if (pass_forced && !pass_first) {
        uint32_t i, n = 0, s = scr_id();
        for (i = 0; i < 240u * 240u; i++)
            if (px_gen[i] == pass_gen)
                scr_ever[s][i] = 1;
            else
                n++;
        if (!scr_st[s].n || n < scr_st[s].min) scr_st[s].min = n;
        if (n > scr_st[s].max) scr_st[s].max = n;
        scr_st[s].sum += n;
        scr_st[s].n++;
        if (nforced < NFORCED)
            forced_pass[nforced++] = (uint32_t)st.passes + 1u;
    }
    pass_gen++;
    pass_first = 1;
    pass_forced = 0;
}
/* the loss test's comparison: every pixel whose last draw the viewer got must match. A pixel whose last draw went
 * out in records the viewer never applied, and that nothing has drawn since, is stale on the viewer (the device
 * has no framebuffer to send it again): counted, not a failure */
static uint64_t stale_px, stale_checks, stale_windows;
static uint32_t stale_shown;
static int compare(const char *why);
static int compare_loss(const char *why)
{
    uint32_t i, real = 0, stale = 0;
    for (i = 0; i < 240u * 240u; i++) {
        if (dec.shadow[i] == swap16(screen[i]))
            continue;
        if (px_pass[i] && span_hit(px_a[i], px_b[i])) {
            stale++;
            if (stale_shown < 4u) {
                uint32_t cur = scr_id(), k, since = 0;
                for (k = 0; k < nforced; k++)
                    since += forced_pass[k] > px_pass[i];
                stale_shown++;
                fprintf(rep, "mirror:   note: %s: pass %llu, screen \"%s\": pixel (%u,%u) stale on the viewer: last drawn at pass %u "
                        "on \"%s\" by %s (%u,%u %ux%u), its record (stream %u..%u) never applied; %u forced redraws since then, none "
                        "drew it; \"%s\": %u forced passes, %s that pixel, undrawn pixels min / mean / max %u / %llu / %u\n", why,
                        (unsigned long long)st.passes, scr_names[cur], i % 240u, i / 240u, px_pass[i], scr_names[px_scr[i]],
                        px_kind[i] == 'B' ? "lcd_blit" : "lcd_fill", px_rect[i][0], px_rect[i][1], px_rect[i][2], px_rect[i][3],
                        px_a[i], px_b[i], since, scr_names[cur], scr_st[cur].n,
                        scr_ever[cur][i] ? "some of them draw" : "none draws", scr_st[cur].min,
                        (unsigned long long)(scr_st[cur].n ? scr_st[cur].sum / scr_st[cur].n : 0u), scr_st[cur].max);
            }
        } else {
            real++;
        }
    }
    stale_px += stale;
    stale_windows += stale != 0u;
    stale_checks++;
    if (real)
        return compare(why);                       /* (fails: its report and pictures) */
    st.cmp++;
    return 1;
}
static void scr_report(void)
{
    uint32_t order[NSCR], i, j, n = 0, t;
    for (i = 0; i < nscr; i++)
        if (scr_st[i].n)
            order[n++] = i;
    for (i = 0; i < n; i++)                        /* (the most pixels a forced redraw leaves alone first) */
        for (j = i + 1u; j < n; j++)
            if (scr_st[order[j]].max > scr_st[order[i]].max) { t = order[i]; order[i] = order[j]; order[j] = t; }
    for (j = 0, i = 0; i < n; i++)
        j += scr_st[order[i]].max == 0u;
    fprintf(rep, "mirror:   note: stale pixels at the clean-window checks: %llu in %llu of %llu checks. Pixels a forced redraw "
            "does not draw (forced passes: min / mean / max of 57600), %u screens, %u of them draw every pixel; the most:",
            (unsigned long long)stale_px, (unsigned long long)stale_windows, (unsigned long long)stale_checks, n, j);
    for (i = 0; i < n && i < 8u; i++)
        fprintf(rep, " \"%s\" (%u: %u / %llu / %u)", scr_names[order[i]], scr_st[order[i]].n, scr_st[order[i]].min,
                (unsigned long long)(scr_st[order[i]].sum / scr_st[order[i]].n), scr_st[order[i]].max);
    fprintf(rep, "\n");
}

/* lcd_blit calls whose records cannot all be in the ring at once (mirror_blit writes every band before the reader
 * can take one): on the device they are dropped whatever the USB does */
static struct { uint32_t n, w, h, size, kinds; uint32_t kw[8], kh[8], ks[8], kn[8]; } big;
static void big_note(uint32_t w, uint32_t h, uint32_t s)
{
    uint32_t i;
    big.n++;
    for (i = 0; i < big.kinds; i++)
        if (big.kw[i] == w && big.kh[i] == h)
            break;
    if (i == big.kinds && big.kinds < 8u) {
        big.kw[i] = w, big.kh[i] = h, big.ks[i] = 0, big.kn[i] = 0;
        big.kinds++;
    }
    if (i < big.kinds) {
        big.kn[i]++;
        if (s > big.ks[i]) big.ks[i] = s;
    }
}
static void mt_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint16_t *p)
{
    uint32_t s;
    if (!w || !h)                  /* (lcd.c lcd_blit returns first) */
        return;
    s = blit_size(w, h, p);
    st.px += (uint64_t)w * h;
    st.offered += s;
    if (s > MIRROR_RING - (mr_wr - mr_rd))
        big_note(w, h, s);
    if (s > ms.burst)
        ms.burst = s;
    if (mt.timed) {                /* the device: the transfer started, mirror_blit (may wait for the reader) at the
                                    * same time; the next lcd call once both are done */
        uint64_t t0 = vt_us, spi = (uint64_t)w * h * 16u / 12u, b = mr_wr - mr_rd;
        if (b > ms.backlog)
            ms.backlog = b;
        mirror_blit(x, y, w, h, p);
        b = mr_wr - mr_rd;
        if (b > ms.backlog)
            ms.backlog = b;
        ms.wait_us += vt_us - t0;
        if (vt_us - t0 > ms.wait_max)
            ms.wait_max = vt_us - t0;
        if (vt_us - t0 > spi)
            ms.over_us += vt_us - t0 - spi;
        if (vt_us < t0 + spi)
            vt_go(t0 + spi - vt_us);
    } else {                       /* part 1: drained after the call, and at each wait */
        if (mt.loss) {                 /* (each band's record: its place in the stream, its pixels) */
            uint32_t rows = MIRROR_BAND_PX / w, r, yy = y, hh = h, off = mr_wr;
            const uint16_t *q = p;
            while (hh) {
                uint32_t sz;
                r = hh < rows ? hh : rows;
                sz = 7u + rle_size(q, w * r);
                pix_note(x, yy, w, r, off, off + sz, 'B', x, y, w, h);
                off += sz;
                q += w * r;
                yy += r;
                hh -= r;
            }
        }
        mirror_blit(x, y, w, h, p);
        drain();
    }
}
static void big_report(void)
{
    uint32_t i;
    if (!big.n)
        return;
    fprintf(rep, "mirror:   note: %u lcd_blit calls bigger than the free ring (%u B) in one go (they wait for the reader):", big.n, (unsigned)MIRROR_RING);
    for (i = 0; i < big.kinds; i++)
        fprintf(rep, " %ux%u (x%u, up to %u B)", big.kw[i], big.kh[i], big.kn[i], big.ks[i]);
    fprintf(rep, "\n");
}
static void mt_fill(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t c)
{
    if (!w || !h)
        return;
    st.px += (uint64_t)w * h;
    st.offered += 9u;
    if (mt.loss && !mt.timed)
        pix_note(x, y, w, h, mr_wr, mr_wr + 9u, 'F', x, y, w, h);
    mirror_fill(x, y, w, h, c);
    if (mt.timed)
        lcd_time((uint64_t)w * h);
    else
        drain();
}

static uint32_t diff_count(int *bx0, int *by0, int *bx1, int *by1)
{
    uint32_t i, n = 0;
    *bx0 = *by0 = 240;
    *bx1 = *by1 = -1;
    for (i = 0; i < 240u * 240u; i++)
        if (dec.shadow[i] != swap16(screen[i])) {
            int x = (int)(i % 240u), y = (int)(i / 240u);
            n++;
            if (x < *bx0) *bx0 = x;
            if (y < *by0) *by0 = y;
            if (x > *bx1) *bx1 = x;
            if (y > *by1) *by1 = y;
        }
    return n;
}
static void dump_shadow(const char *name)
{
    char path[512];
    FILE *f;
    unsigned i;
    snprintf(path, sizeof path, "%s/%s.ppm", outdir, name);
    f = fopen(path, "wb");
    if (!f)
        return;
    fprintf(f, "P6\n240 240\n255\n");
    for (i = 0; i < 240u * 240u; i++) {
        uint16_t p = dec.shadow[i];
        uint8_t rgb[3] = {(uint8_t)((p >> 11) * 255 / 31), (uint8_t)(((p >> 5) & 63) * 255 / 63), (uint8_t)((p & 31) * 255 / 31)};
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}
static int compare(const char *why)
{
    int x0, y0, x1, y1;
    uint32_t n = diff_count(&x0, &y0, &x1, &y1);
    st.cmp++;
    if (!n)
        return 1;
    st.bad_passes++;
    st.bad_px += n;
    if (st.bad_passes <= 5u) {
        char nm[64];
        fprintf(rep, "mirror:   %s: pass %llu: %u pixels differ in [%d..%d] x [%d..%d]\n", why,
                (unsigned long long)st.passes, n, x0, x1, y0, y1);
        snprintf(nm, sizeof nm, "mirror-bad%llu-screen", (unsigned long long)st.bad_passes);
        ppm(nm);
        snprintf(nm, sizeof nm, "mirror-bad%llu-shadow", (unsigned long long)st.bad_passes);
        dump_shadow(nm);
    }
    return 0;
}

/* after a UI pass of the harness (ui_pages_test.c frame()) */
#define WIN_LOSSY 300u
#define WIN_CLEAN 40u
static void mt_pass(void)
{
    if (mt.timed)
        return;                    /* (part 2 runs its own passes: mframe) */
    if (mirror_frame_end()) {
        ui.force = 1;
        st.forced++;
    }
    drain();
    if (mt.loss)
        pix_pass_end();
    st.passes++;
    if (mt.loss) {
        uint32_t k = (uint32_t)(st.passes % (WIN_LOSSY + WIN_CLEAN));
        mt.lossy = k < WIN_LOSSY;
        if (k == WIN_LOSSY + WIN_CLEAN - 1u) {
            st.windows++;
            if (st.lost_tail)
                st.win_tail++;
            else if (!dec.in_sync || mirror_redraw || mr_dirty || ui.force)
                st.win_unsettled++;
            else if (compare_loss("after the losses"))
                st.win_ok++;
            else
                st.win_bad++;
        }
    } else if (mt.check) {
        compare("round trip");
    }
}

/* ---------------------------------------------------------------- part 1 */
static void mirror_start(void)
{
    md_init(&dec);
    memset(screen, 0, sizeof screen);          /* (both black: they start equal) */
    mirror_on = 1;
}
static int run_reference(uint32_t P, uint32_t loss, const char *dir)
{
    char *av[2];
    int saved, fails_ref;
    uint64_t offered0;
    uint32_t bytes0, drops0;
    mt.P = P;
    mt.loss = loss;
    mt.lossy = loss != 0;
    mirror_start();
    offered0 = st.offered;
    bytes0 = mirror_stat.bytes;
    drops0 = mirror_stat.drops;
    fflush(stdout);
    saved = dup(1);
    rep = fdopen(saved, "w");
    setvbuf(rep, 0, _IOLBF, 0);
    if (!freopen("/dev/null", "w", stdout))
        return 1;
    av[0] = (char *)"ui_pages_test";
    av[1] = (char *)dir;
    fails_ref = ui_pages_main(2, av);
    if (loss) {                                /* losses over: the last request served, then the picture */
        uint32_t i;
        mt.lossy = 0;
        frames(4);
        if (st.lost_tail) {                    /* (a loss nothing came after: the viewer cannot know yet) */
            ui.force = 1;
            frames(1);
        }
        for (i = 0; i < 10u; i++)
            frames(1);
        st.windows++;
        if (dec.in_sync && !mirror_redraw && !mr_dirty && compare_loss("at the end"))
            st.win_ok++;
        else
            st.win_bad++;
    }
    big_report();
    if (loss)
        scr_report();
    if (!loss) {
        int ok = !st.bad_passes && mirror_stat.drops == drops0 && st.offered - offered0 + 3u * dec.frames == (uint64_t)(mirror_stat.bytes - bytes0) &&
                 !st.resyncs && !dec.bad && !dec.lost;
        fprintf(rep, "mirror: %-6s P=%4u  passes %llu, compared %llu, mismatched %llu (%llu px), records %u, dropped %u, "
                "stream %u B (counted %llu), packets %llu, decoder bad %u lost %u, frames %u, reference checks failed %d   %s\n",
                "rt", P, (unsigned long long)st.passes, (unsigned long long)st.cmp, (unsigned long long)st.bad_passes,
                (unsigned long long)st.bad_px, mirror_stat.recs, mirror_stat.drops - drops0, mirror_stat.bytes - bytes0,
                (unsigned long long)(st.offered - offered0 + 3u * dec.frames), (unsigned long long)st.pkts, dec.bad, dec.lost, dec.frames,
                fails_ref, ok ? "ok" : "FAIL");
        return !ok;
    } else {
        int ok = st.win_bad == 0u && st.win_ok > 0u;
        fprintf(rep, "mirror: %-6s P=%4u  1/%u lost: passes %llu, packets %llu, lost %llu, resyncs %llu, forced %llu, decoder "
                "lost %u bad %u; clean windows: %u, picture == screen %u, different %u, not settled %u, last packet "
                "lost (undetectable) %u   %s\n",
                "loss", P, loss, (unsigned long long)st.passes, (unsigned long long)st.pkts, (unsigned long long)st.lost,
                (unsigned long long)st.resyncs, (unsigned long long)st.forced, dec.lost, dec.bad, st.windows, st.win_ok,
                st.win_bad, st.win_unsettled, st.win_tail, ok ? "ok" : "FAIL");
        return !ok;
    }
}

/* random fills / blits straight into the encoder, packets of random size */
static int encoder_random(uint32_t rounds)
{
    static uint16_t ref[240 * 240], px[240 * 240];
    uint32_t r, k, bad = 0, drops0 = mirror_stat.drops;
    uint64_t off0 = st.offered;
    uint32_t bytes0 = mirror_stat.bytes;
    md_init(&dec);
    memset(ref, 0, sizeof ref);
    mirror_on = 1;
    for (r = 0; r < rounds; r++) {
        uint32_t n = 1u + mt_rnd(6);
        for (k = 0; k < n; k++) {
            uint32_t w = 1u + mt_rnd(240), h = 1u + mt_rnd(240), x, y, i, j, mode = mt_rnd(6);
            if (mt_rnd(3) == 0) { w = 1u + mt_rnd(8); }
            if (mt_rnd(3) == 0) { h = 1u + mt_rnd(8); }
            x = mt_rnd(241u - w);
            y = mt_rnd(241u - h);
            if (mode == 0) {
                uint16_t c = (uint16_t)mt_rnd(65536);
                mirror_fill(x, y, w, h, c);
                st.offered += 9u;
                for (j = 0; j < h; j++)
                    for (i = 0; i < w; i++)
                        ref[(y + j) * 240u + x + i] = c;
            } else {
                uint32_t len = 0, cols = mode == 1 ? 65536u : mode == 2 ? 2u : 4u;
                uint16_t c = 0;
                for (i = 0; i < w * h; i++) {          /* runs of 1..300, or noise */
                    if (!len) {
                        len = mode == 1 ? 1u : mode == 5 ? 1u + mt_rnd(4) : 1u + mt_rnd(mt_rnd(2) ? 300u : 5u);
                        c = (uint16_t)(mt_rnd(cols) * 0x1357u);
                    }
                    px[i] = c;
                    len--;
                }
                while (blit_size(w, h, px) > MIRROR_RING)   /* (more than the ring in one call: dropped, tested below) */
                    h = (h + 1u) / 2u;
                mirror_blit(x, y, w, h, px);
                st.offered += blit_size(w, h, px);
                for (j = 0; j < h; j++)
                    for (i = 0; i < w; i++)
                        ref[(y + j) * 240u + x + i] = swap16(px[j * w + i]);
            }
            while ((mt.P = 5u + mt_rnd(1019)), usb_take())
                ;
        }
        if (mirror_frame_end())
            bad++;
        while ((mt.P = 5u + mt_rnd(1019)), usb_take())
            ;
        if (memcmp(ref, dec.shadow, sizeof ref))
            bad++;
    }
    {
        int ok = !bad && mirror_stat.drops == drops0 && !dec.bad && !dec.lost && st.offered - off0 + 3u * dec.frames == mirror_stat.bytes - bytes0;
        fprintf(rep, "mirror: encoder: %u rounds of random fills / blits (1..240 wide, runs 1..300, noise), packets 5..1023 B: "
                "%u wrong pictures, decoder bad %u, stream %u B (counted %llu)   %s\n",
                rounds, bad, dec.bad, mirror_stat.bytes - bytes0, (unsigned long long)(st.offered - off0 + 3u * dec.frames), ok ? "ok" : "FAIL");
        if (!ok)
            return 1;
    }
    {   /* the device's timing (1 packet of 1023 B a ms): a 240x240 blit of noise (115 KB) waits for the reader at most
         * the panel's transfer (76 ms), then drops what is left; the repair once the backlog is <= RING / 4; then a
         * viewer that stops reading: one wait of MIRROR_STALL_MS, then none until it reads again; the repair after it;
         * a second repair MIRROR_HOLD passes after the first */
        uint32_t i, d0, w0, pumps, ok = 1, passes;
        uint64_t t0, e1, e2, e3;
        int f1;
        for (i = 0; i < 240u * 240u; i++) {
            px[i] = (uint16_t)mt_rnd(65536);
            ref[i] = swap16(px[i]);
        }
        mt.timed = 1;
        mt.P = 1023u;
        usb_next_us = vt_us + 1000u;
        #define REDRAW() do { uint32_t j_; for (j_ = 0; j_ < 240u; j_ += 4u) { mirror_blit(0, j_, 240, 4, px + j_ * 240u); vt_go(3000u); }                               mirror_frame_end(); vt_go(200000u); } while (0)
        #define PUMP_UNTIL_REPAIR(n_) do { for ((n_) = 0; (n_) < 1000u && !mirror_frame_end(); (n_)++) vt_go(1000u); } while (0)
        d0 = mirror_stat.drops;
        t0 = vt_us;
        mirror_blit(0, 0, 240, 240, px);
        e1 = vt_us - t0;
        f1 = mirror_frame_end();
        PUMP_UNTIL_REPAIR(pumps);
        ok &= e1 <= 77000u && e1 >= 70000u && mirror_stat.drops > d0 && !f1 && pumps < 1000u && mr_wr - mr_rd <= MIRROR_RING / 4u;
        REDRAW();
        ok &= !memcmp(ref, dec.shadow, sizeof ref);
        fprintf(rep, "mirror: encoder: a 240x240 blit of noise, P=1023: waited %.1f ms (the transfer: 76.8), %u records dropped, repair "
                "%u ms later (backlog <= RING/4), picture right after it: %s\n", e1 / 1000.0, mirror_stat.drops - d0, pumps,
                !memcmp(ref, dec.shadow, sizeof ref) ? "yes" : "NO");
        /* the viewer stops reading */
        mt.paused = 1;
        d0 = mirror_stat.drops;
        for (i = 0; i < 240u * 240u; i++) {
            px[i] = (uint16_t)mt_rnd(65536);
            ref[i] = swap16(px[i]);
        }
        t0 = vt_us;
        mirror_blit(0, 0, 240, 240, px);
        e2 = vt_us - t0;
        w0 = mirror_stat.waits;
        t0 = vt_us;
        mirror_blit(0, 0, 240, 240, px);
        e3 = vt_us - t0;
        ok &= e2 <= (MIRROR_STALL_MS + 1u) * 1000u && e3 == 0u && mirror_stat.waits == w0 && mirror_stat.drops > d0;
        for (i = 0; i < 100u; i++) {               /* (still not reading: no repair, no wait) */
            ok &= !mirror_frame_end();
            vt_go(16000u);
        }
        mt.paused = 0;
        PUMP_UNTIL_REPAIR(pumps);
        ok &= pumps < 1000u;
        REDRAW();
        ok &= !memcmp(ref, dec.shadow, sizeof ref);
        fprintf(rep, "mirror: encoder: the reader stops: the first blit waited %.1f ms (stall %u ms), the next %.1f ms (%u more waits), "
                "%u records dropped; it reads again: repair %u ms later, picture right: %s\n", e2 / 1000.0, (unsigned)MIRROR_STALL_MS,
                e3 / 1000.0, mirror_stat.waits - w0, mirror_stat.drops - d0, pumps, !memcmp(ref, dec.shadow, sizeof ref) ? "yes" : "NO");
        /* a second overflow right after a repair: MIRROR_HOLD passes */
        mt.paused = 1;
        mirror_blit(0, 0, 240, 240, px);
        mt.paused = 0;
        for (passes = 1; passes < 200u && !mirror_frame_end(); passes++)
            vt_go(16000u);
        ok &= passes >= MIRROR_HOLD - 2u && passes <= MIRROR_HOLD + 1u;
        fprintf(rep, "mirror: encoder: an overflow right after a repair: repaired %u passes later (MIRROR_HOLD %u)   %s\n", passes,
                (unsigned)MIRROR_HOLD, ok ? "ok" : "FAIL");
        #undef REDRAW
        #undef PUMP_UNTIL_REPAIR
        mt.timed = 0;
        return !ok;
    }
}

/* the decoder fed anything: random packets, valid ones mutated, spliced, cut; its state checked after each */
static int decoder_fuzz(uint32_t n)
{
    enum { NCORP = 4096 };
    static uint8_t corp[NCORP][1024];
    static uint16_t clen[NCORP];
    static uint16_t px[960];
    uint32_t nc = 0, i, k, bad = 0;
    uint8_t b[1100];
    mirror_dec_t *d = malloc(sizeof *d);
    md_init(d);
    md_init(&dec);
    mirror_on = 1;
    while (nc < NCORP) {                       /* the corpus: real packets of every size */
        uint32_t w = 1u + mt_rnd(240), h = 1u + mt_rnd(240 * 240 / w < 240 ? 240 * 240 / w : 240), m = mt_rnd(3);
        if (w * h > 960u) h = 960u / w;
        for (i = 0; i < w * h; i++)
            px[i] = (uint16_t)(m == 0 ? mt_rnd(65536) : m == 1 ? (i / (1u + mt_rnd(40))) * 77u : mt_rnd(2));
        if (mt_rnd(4) == 0)
            mirror_fill(mt_rnd(240 - w + 1), 0, w, 1u + mt_rnd(240), mt_rnd(65536));
        else
            mirror_blit(mt_rnd(241 - w), mt_rnd(241 - h), w, h, px);
        if (mt_rnd(3) == 0)
            mirror_frame_end();
        for (;;) {
            uint32_t P = 5u + mt_rnd(1019), q = mirror_packet(corp[nc], P);
            if (!q)
                break;
            clen[nc] = (uint16_t)q;
            if (++nc == NCORP)
                break;
        }
    }
    while (mirror_packet(b, 1023))
        ;
    for (i = 0; i < n; i++) {
        uint32_t len, op = mt_rnd(8), m;
        uint8_t *q;
        int x, y, w, h;
        if (op == 0) {                             /* noise */
            len = mt_rnd(1100);
            for (k = 0; k < len; k++) b[k] = (uint8_t)mt_rnd(256);
            if (len > 1 && mt_rnd(2)) b[1] = MIRROR_VER;
        } else {
            uint32_t c = mt_rnd(nc);
            len = clen[c];
            memcpy(b, corp[c], len);
            if (op == 1) {                         /* a packet as sent (in order or not) */
            } else if (op == 2) {                  /* bytes changed */
                for (m = 1u + mt_rnd(4); m; m--) b[mt_rnd(len)] ^= (uint8_t)(1u << mt_rnd(8));
            } else if (op == 3) {                  /* the header lies */
                b[0] = (uint8_t)mt_rnd(256);
                b[2] = (uint8_t)mt_rnd(256);
                b[3] = mt_rnd(3) ? (uint8_t)mt_rnd(4) : (uint8_t)mt_rnd(256);
            } else if (op == 4) {                  /* cut short / made longer */
                len = mt_rnd(2) ? mt_rnd(len + 1u) : len + mt_rnd(64);
                for (k = clen[c]; k < len; k++) b[k] = (uint8_t)mt_rnd(256);
            } else if (op == 5) {                  /* a record header with a big or a tiny length, a type or size inside */
                k = 4u + mt_rnd(len > 4u ? len - 4u : 1u);
                if (k + 2u < len) {
                    b[k] = "FBEX"[mt_rnd(4)];
                    b[k + 1u] = (uint8_t)mt_rnd(256);
                    b[k + 2u] = (uint8_t)(mt_rnd(2) ? mt_rnd(9) : mt_rnd(256));
                }
            } else if (op == 6) {                  /* coordinates / sizes at the edges */
                for (m = 1u + mt_rnd(6); m; m--) {
                    static const uint8_t E[] = {0, 1, 127, 128, 129, 239, 240, 241, 255};
                    b[4u + mt_rnd(len > 4u ? len - 4u : 1u)] = E[mt_rnd(sizeof E)];
                }
            } else {                               /* two packets spliced */
                uint32_t c2 = mt_rnd(nc), at = mt_rnd(len + 1u), l2 = clen[c2], from = mt_rnd(l2 + 1u);
                if (at + (l2 - from) > sizeof b) l2 = from + (uint32_t)sizeof b - at;
                memcpy(b + at, corp[c2] + from, l2 - from);
                len = at + l2 - from;
            }
        }
        q = malloc(len ? len : 1u);
        memcpy(q, b, len);
        if (mt_rnd(1000) == 0)
            md_reset(d);
        md_packet(d, q, len);
        free(q);
        if (d->hn > 3u || (d->in_sync && d->hn == 3u && (d->got >= d->len || d->len > MIRROR_REC_MAX)))
            bad++;                                 /* (hn 3: a payload under way) */
        if (d->dx1 > d->dx0 && (d->dx0 < 0 || d->dy0 < 0 || d->dx1 > 240 || d->dy1 > 240 || d->dy1 <= d->dy0))
            bad++;
        if (mt_rnd(8) == 0 && md_take_dirty(d, &x, &y, &w, &h) && (x < 0 || y < 0 || w < 1 || h < 1 || x + w > 240 || y + h > 240))
            bad++;
    }
    fprintf(rep, "mirror: fuzz: %u packets (noise, mutated, cut, spliced) into the decoder: no fault, state out of bounds %u "
            "times (records applied %u, malformed %u, gaps %u)   %s\n", n, bad, d->recs, d->bad, d->lost, bad ? "FAIL" : "ok");
    free(d);
    return bad != 0;
}

/* losses at the worst places, step by step */
static uint32_t take_pkt(uint8_t *b) { return mirror_packet(b, mt.P); }
static int protocol_cases(void)
{
    static uint16_t noise[960];
    uint8_t b[1024];
    uint32_t i, n, k, wrong, asked;
    int fails_here = 0;
    mt.P = 196u;
    for (i = 0; i < 960u; i++)
        noise[i] = (uint16_t)mt_rnd(65536);
    /* A: a packet lost while the viewer is already out of step (after an earlier loss, before a record start):
     * the redraw asked for at the first loss is under way, its first packet is lost too */
    md_init(&dec);
    mirror_on = 1;
    mirror_redraw = 0;
    mirror_fill(0, 0, 240, 240, 0x0000u);
    mirror_frame_end();
    while ((n = take_pkt(b))) deliver(b, n);
    mirror_blit(0, 0, 240, 4, noise);              /* (1927 B: 10 packets, 9 of them with no record start) */
    for (k = 0; (n = take_pkt(b)); k++)
        if (k != 2u)
            deliver(b, n);                         /* the third lost: the viewer asks for a redraw at the fourth */
    asked = mirror_redraw;
    i = (uint32_t)mirror_frame_end();              /* the device: the redraw */
    mirror_redraw = 0;
    mirror_fill(0, 0, 240, 240, 0xF800u);          /* the redraw's first record: its packet lost */
    n = take_pkt(b);
    (void)n;
    mirror_fill(0, 0, 8, 8, 0x001Fu);              /* the rest of it arrives */
    mirror_frame_end();
    while ((n = take_pkt(b))) deliver(b, n);
    for (wrong = 0, k = 0; k < 240u * 240u; k++)
        wrong += dec.shadow[k] != ((k % 240u < 8u && k / 240u < 8u) ? 0x001Fu : 0xF800u);
    if (asked && i && wrong && !mirror_redraw) {
        fprintf(rep, "mirror: BUG decoder: a packet lost while out of step (after a loss, before the next record start) is not "
                "reported (md_packet: the gap is only counted while in_sync): the redraw under way lost its first record, "
                "%u pixels stay wrong, no new request   FAIL\n", wrong);
        fails_here++;
    } else {
        fprintf(rep, "mirror: a loss while out of step: %s   %s\n", wrong ? (mirror_redraw ? "asked again" : "picture wrong") : "picture right",
                wrong && !mirror_redraw ? "FAIL" : "ok");
        fails_here += wrong && !mirror_redraw;
    }
    /* B: the last packet before the screen goes quiet is lost: the beat 'E' (MIRROR_BEAT_MS) shows the gap */
    md_init(&dec);
    mirror_redraw = 0;
    mirror_fill(0, 0, 240, 240, 0x0000u);
    mirror_frame_end();
    while ((n = take_pkt(b))) deliver(b, n);
    fm1_ms += 16u;
    mirror_fill(0, 0, 240, 240, 0x07E0u);
    mirror_frame_end();
    n = take_pkt(b);                               /* (the fill and its 'E': lost) */
    {
        uint32_t t0 = fm1_ms, seen = 0, ok;
        for (k = 0; k < 600u; k++) {               /* a still screen, a UI pass every 16 ms */
            fm1_ms += 16u;
            if (mirror_frame_end()) {              /* the redraw the viewer asked for */
                mirror_fill(0, 0, 240, 240, 0x07E0u);
                mirror_frame_end();
            }
            while ((n = take_pkt(b))) deliver(b, n);
            if (!seen && dec.lost)
                seen = fm1_ms - t0;
            if (seen && dec.shadow[0] == 0x07E0u && dec.shadow[240u * 240u - 1u] == 0x07E0u)
                break;
        }
        ok = seen && seen <= MIRROR_BEAT_MS + 16u && k < 600u;
        fprintf(rep, "mirror: the last packet before a still screen lost: the gap seen %u ms later (the beat 'E', %u ms), picture "
                "right %u ms later   %s\n", seen, (unsigned)MIRROR_BEAT_MS, fm1_ms - t0, ok ? "ok" : "FAIL");
        fails_here += !ok;
    }
    return fails_here;
}
static int proto_child(void *a) { (void)a; rep = stdout; return protocol_cases(); }

/* the 'S' record: what mirror_status sends reaches the decoder, once per change, and touches no pixel */
static int status_cases(void)
{
    uint8_t b[1024], s[MIRROR_STATUS_LEN];
    uint32_t i, n, k, wrong;
    int r, ok = 1;
    mt.P = 196u;
    md_init(&dec);
    mirror_on = 1;
    mirror_redraw = 0;
    mirror_fill(0, 0, 240, 240, 0x07E0u);
    mirror_frame_end();
    while ((n = take_pkt(b))) deliver(b, n);
    for (i = 0; i < MIRROR_STATUS_LEN; i++)
        s[i] = (uint8_t)(0x11u * (i + 1u));
    mirror_status(s);                               /* the first: sent */
    for (r = 0; (n = take_pkt(b));) r |= deliver(b, n);
    ok &= (r & MD_STATUS) && dec.have_status && !memcmp(dec.status, s, MIRROR_STATUS_LEN);
    mirror_status(s);                               /* the same, at once: nothing on the cable */
    ok &= take_pkt(b) == 0u;
    s[1] ^= 1u;
    mirror_status(s);                               /* a change: sent */
    for (r = 0; (n = take_pkt(b));) r |= deliver(b, n);
    ok &= (r & MD_STATUS) && !memcmp(dec.status, s, MIRROR_STATUS_LEN) && !(r & MD_RESYNC);
    for (wrong = 0, k = 0; k < 240u * 240u; k++)
        wrong += dec.shadow[k] != 0x07E0u;
    ok &= !wrong && !dec.bad;
    mirror_on = 0;
    mirror_status(s);                               /* the viewer left: the next one gets it again */
    mirror_on = 1;
    mirror_status(s);
    ok &= take_pkt(b) != 0u;
    fprintf(rep, "mirror: status ('S'): round trip, once per change, again for a new viewer, no pixel touched   %s\n", ok ? "ok" : "FAIL");
    return !ok;
}
static int status_child(void *a) { (void)a; rep = stdout; return status_cases(); }

/* a blit with the interrupts off (fm1_fault's crash screen draws text boxes so, main.c): the clock and the reader
 * stand still; with the ring full mirror_blit must still return */
static int frozen_child(void *a)
{
    static uint16_t px[240 * 20];
    uint32_t i;
    (void)a;
    rep = stdout;
    md_init(&dec);
    mirror_on = 1;
    mt.paused = 1;                                 /* (the ring fills: nobody reads) */
    for (i = 0; i < 240u * 20u; i++)
        px[i] = (uint16_t)mt_rnd(65536);
    for (i = 0; i < 10u; i++)
        mirror_fill(0, 0, 240, 240, i);
    while (MIRROR_RING - (mr_wr - mr_rd) >= MIRROR_REC_MAX + 3u)
        mirror_blit(0, 0, 240, 4, px);
    mt.paused = 0;
    mr_stall = 0;                                  /* (the reader moved since the last stall) */
    mt.frozen = 1;
    if (setjmp(frozen_out)) {
        printf("mirror: BUG encoder: mirror_blit with the clock and the reader stopped (interrupts off, e.g. fm1_fault's crash "
               "screen) and the ring full: mr_room never returns (still waiting after %u spins)   FAIL\n", frozen_spins);
        return 1;
    }
    mirror_blit(10, 40, 220, 20, px);              /* (a draw_text_box line of the crash screen) */
    printf("mirror: a blit with the interrupts off and the ring full returns after %u spins   ok\n", frozen_spins);
    return 0;
}

static int child(int (*fn)(void *), void *arg)
{
    pid_t p;
    int s;
    fflush(stdout);
    p = fork();
    if (p == 0)
        exit(fn(arg));
    if (p < 0 || waitpid(p, &s, 0) < 0)
        return 1;
    if (!WIFEXITED(s)) {
        printf("mirror: a test process died (signal %d)   FAIL\n", WIFSIGNALED(s) ? WTERMSIG(s) : -1);
        return 1;
    }
    return WEXITSTATUS(s) != 0;
}
struct ref_args { uint32_t P, loss; const char *dir; };
static int ref_child(void *a)
{
    struct ref_args *r = a;
    return run_reference(r->P, r->loss, r->dir);
}
static int enc_child(void *a) { rep = stdout; mt_seed = 99u; return encoder_random(*(uint32_t *)a); }
static int fuzz_child(void *a) { rep = stdout; mt_seed = 4242u; return decoder_fuzz(*(uint32_t *)a); }

/* ---------------------------------------------------------------- part 2 */
static uint32_t mt_P_list[3] = {196u, 512u, 1023u};
static void audio_upto(void)
{
    static int32_t o[CTL * 2];
    uint64_t want = vt_us * FS / 1000000u;
    while (aud_smp + CTL <= want) {
        uint32_t i;
        mix_block(o, CTL);
        for (i = 1; i < CTL; i += 2u) {
            scope_bufr[scope_w & (SCOPE_N - 1u)] = vis_tap[2u * i + 1u];
            scope_buf[scope_w++ & (SCOPE_N - 1u)] = vis_tap[2u * i];
        }
        aud_smp += CTL;
    }
}
static void ms_begin(void)
{
    memset(&ms, 0, sizeof ms);
    ms.t0 = vt_us;
    ms.drops0 = mirror_stat.drops;
    ms.waits0 = mirror_stat.waits;
    ms.steady_from = vt_us + 1000000u;
}
/* one UI pass on the device's timing */
static void mframe(void)
{
    uint64_t t0 = vt_us, off0 = st.offered, px0 = st.px, b;
    audio_upto();
    fm1_ms = ms_base + (uint32_t)(vt_us / 1000u) - p1_wait_ms;
    ui_input();
    ui_leds();
    ui_draw();
    if (mirror_frame_end()) {
        ui.force = 1;
        ms.forced++;
    }
    b = st.offered - off0 + (st.px != px0 ? 3u : 0u);   /* (+ the 'E') */
    ms.bytes += b;
    ms.raw += 2u * (st.px - px0);
    ms.passes++;
    if (b > ms.pass_max)
        ms.pass_max = b;
    if (vt_us < t0 + 15000u)
        vt_go(t0 + 15000u - vt_us);
}
static void mframes_for(uint64_t us)
{
    uint64_t end = vt_us + us;
    while (vt_us < end)
        mframe();
}
static void mpress(uint32_t b) { edges_btn |= BT(b); fm1_in.buttons |= BT(b); mframe(); }
static void mrelease(uint32_t b) { fm1_in.buttons &= ~BT(b); mframe(); }
static void mtap(uint32_t b) { mpress(b); mrelease(b); }

typedef struct { double pps, bpp, bps, ratio, over, inside; uint64_t bmax; uint32_t drops, steady, forced, backlog, burst, waits; } row_t;
static row_t ms_end(void)
{
    row_t r;
    double s = (double)(vt_us - ms.t0) / 1e6;
    if (!ms.steady_taken)
        ms.steady_drops0 = mirror_stat.drops;
    r.pps = ms.passes / s;
    r.bpp = ms.passes ? (double)ms.bytes / ms.passes : 0;
    r.bmax = ms.pass_max;
    r.bps = ms.bytes / s;
    r.ratio = ms.bytes ? (double)ms.raw / ms.bytes : 0;
    r.drops = mirror_stat.drops - (uint32_t)ms.drops0;
    r.steady = mirror_stat.drops - (uint32_t)ms.steady_drops0;
    r.forced = (uint32_t)ms.forced;
    r.backlog = ms.backlog;
    r.burst = ms.burst;
    r.waits = mirror_stat.waits - ms.waits0;
    r.over = (double)ms.over_us / 1000.0 / s;
    r.inside = (double)ms.wait_us / 1000.0 / s;   /* ms a second inside mirror_blit (the panel's transfer runs meanwhile) */     /* ms a second the UI waited past the panel */
    return r;
}
static void row_print(const char *name, const row_t *r)
{
    if (!mirror_on)
        printf("%-24s  off %5u | %5.1f | (mirror off: the UI's own pace)\n", name, (unsigned)MIRROR_RING, r->pps);
    else
        printf("%-24s %4u %5u | %5.1f | %7.0f %7llu | %8.0f %4.0f%% | %5.2f | %5u %5u | %3u | %5u %5u | %6u %5.1f %5.1f | %s\n", name,
               mt.P, (unsigned)MIRROR_RING, r->pps, r->bpp, (unsigned long long)r->bmax, r->bps,
               100.0 * r->bps / ((mt.P - 4u) * 1000.0), r->ratio, r->drops, r->steady, r->forced, r->backlog, r->burst, r->waits,
               r->inside, r->over, r->steady ? (mr_dirty ? "NO (repair pending)" : "NO") : "fits");
    fflush(stdout);
}
/* after a scenario: the time until the picture is right again (the ring empty, no repair due, picture == screen) */
/* after a scenario, the same screen going on: the time until the viewer's picture is the screen again (checked each
 * time the ring is empty with no repair due) */
static void settle_report(const char *name)
{
    if (!mirror_on)
        return;
    uint64_t t0 = vt_us;
    settle_ok = 0;
    settle_watch = 1;
    if (mr_wr == mr_rd && dec.in_sync) {
        int a, b, c, d;
        settle_ok = !diff_count(&a, &b, &c, &d);
    }
    while (!settle_ok && vt_us - t0 < 5000000u)
        mframe();
    settle_watch = 0;
    if (!settle_ok)
        printf("    %-22s the viewer's picture still differs from the screen 5 s later (repair due: %s)\n", name,
               mr_dirty ? "yes, never sent" : "no");
    else if (vt_us - t0 > 200000u || getenv("MT_DETAIL"))
        printf("    %-22s the viewer's picture right again %.2f s later%s\n", name, (vt_us - t0) / 1e6,
               mr_dirty ? " (a repair still due)" : "");
}

static void groove(void)
{
    uint32_t j, k;
    static const uint8_t BASS[2] = {38, 34};
    for (j = 0; j < NTRK; j++) { steps_clear(&trk[j]); trk[j].p[P_SLEN] = 16; }
    for (j = 0; j < 16u; j++) {                /* (tests/ui_pages_test.c, the visualiser's groove) */
        step_t *sp;
        if (!(j & 1u)) { sp = &trk[0].step[j]; sp->n = 1; sp->note[0] = (uint8_t)(BASS[j / 8u] + ((j & 2u) ? 12 : 0)); sp->time = ST_NOTE; sp->vel = 100; }
        if ((j % 4u) == 2u) { sp = &trk[1].step[j]; sp->n = 3; sp->note[0] = 65; sp->note[1] = 69; sp->note[2] = 72; sp->time = ST_NOTE; sp->vel = 100; }
        if (!(j % 8u)) { sp = &trk[2].step[j]; sp->n = 2; sp->note[0] = 57; sp->note[1] = 64; sp->time = ST_NOTE; sp->vel = 100; }
        memset(&TDRUM->dstep[j], 0, sizeof(dstep_t));
        if (!(j % 4u)) dstep_set(&TDRUM->dstep[j], 0, LV_NORM, 0);
        if ((j % 8u) == 4u) dstep_set(&TDRUM->dstep[j], 3, LV_NORM, 0);
        dstep_set(&TDRUM->dstep[j], 4, (j & 1u) ? LV_SOFT : LV_NORM, 0);
        for (k = 0; k < 4u; k++) trk[k].seq_active = 1;
    }
}
static void harness_init(void)
{
    uint32_t i;
    panel = PANEL_DEFAULT;
    layers_init();
    settings.palette = 4;
    palette_set(4);
    host_tracks_init();
    for (i = 0; i < NPART; i++) { set_engine_of(&trk[i], TRK_DEF[i][0]); apply_preset_to(&trk[i], TRK_DEF[i][1]); trk[i].engine = trk[i].eng_req; }
    TDRUM->p[P_E0] = DRUM_DEFAULT_KIT;
    groove();
    go_home(); ui.force = 1; frames(20);
}
static void measure_start(uint32_t P)
{
    drain();
    mirror_on = P != 0u;                           /* (0: the mirror off, for the UI's own pace) */
    mt.P = P ? P : 1023u;
    mt.timed = 1;
    ms_base = fm1_ms + p1_wait_ms - (uint32_t)(vt_us / 1000u);   /* (MIRROR_MS() goes on, never back) */
    usb_next_us = (vt_us / 1000u + 1u) * 1000u;
    aud_smp = vt_us * FS / 1000000u;
}

/* a: the static screens */
enum { SA_HOME, SA_PAGE, SA_DRUMGRID, SA_DRUMKIT, SA_SONG, SA_REC, SA_MENU, SA_ABOUT, SA_LAYER };
static void static_screen(uint32_t kind, uint32_t arg, char *name, size_t nn)
{
    if (fm1_in.buttons) { fm1_in.buttons = 0; frames(3); }
    ui.menu = 0; rec_wait = 0; drum_page = 0; song.sel = 0; vis_on = 0;
    go_home(); frames(3);
    switch (kind) {
    case SA_HOME: snprintf(name, nn, "TRACKS"); break;
    case SA_PAGE:
        song.sel = page_for_drum(&PAGES[arg]) ? TRK_DRUM : 0;
        open_family(PAGES[arg].fam); ui.page = (uint8_t)arg; ui.fam_last[PAGES[arg].fam] = (uint8_t)arg; page_entered();
        snprintf(name, nn, "%s", ui.home ? "(home)" : PAGES[ui.page].title);
        break;
    case SA_DRUMGRID: case SA_DRUMKIT:
        song.sel = TRK_DRUM; studio_open(SC_DRUM); drum_page = kind == SA_DRUMKIT;
        snprintf(name, nn, kind == SA_DRUMKIT ? "DRUMS kit" : "DRUMS grid");
        break;
    case SA_SONG: tap(B_SAVE); snprintf(name, nn, "SONG"); break;
    case SA_REC: rec_wait = 1; snprintf(name, nn, "REC ready"); break;
    case SA_MENU: ui.menu = 1; ui.menu_sel = 0; snprintf(name, nn, "menu"); break;
    case SA_ABOUT: ui.menu = 2; snprintf(name, nn, "about"); break;
    case SA_LAYER:
        press(arg); frames(30);
        snprintf(name, nn, "layer %s held", B_NAME[arg]);
        break;
    }
    frames(3);
}
static int measure_static(uint32_t P)
{
    struct { uint32_t kind, arg; } L[NPAGES + 16];
    uint32_t n = 0, i, k;
    static const uint8_t LAYB[] = {B_FX, B_SEQ, B_EDIT, B_ARP, B_SCL, B_GLO, B_SAVE};
    uint64_t sum_first = 0, max_first = 0, sum_idle_bps = 0, worst_idle = 0;
    uint32_t drops = 0, steady = 0, bad_pages = 0, forced = 0, backlog = 0, burst = 0, waits = 0;
    uint64_t tot_passes = 0, tot_us = 0, over_us = 0;
    char worst[40] = "", names_bad[400] = "";
    L[n].kind = SA_HOME; L[n++].arg = 0;
    for (i = 0; i < NPAGES; i++) { L[n].kind = SA_PAGE; L[n++].arg = i; }
    L[n].kind = SA_DRUMGRID; L[n++].arg = 0;
    L[n].kind = SA_DRUMKIT; L[n++].arg = 0;
    L[n].kind = SA_SONG; L[n++].arg = 0;
    L[n].kind = SA_REC; L[n++].arg = 0;
    L[n].kind = SA_MENU; L[n++].arg = 0;
    L[n].kind = SA_ABOUT; L[n++].arg = 0;
    for (k = 0; k < sizeof LAYB; k++) { L[n].kind = SA_LAYER; L[n++].arg = LAYB[k]; }
    song.playing = 0; transport_req = 2; frames(10);
    for (i = 0; i < n; i++) {
        char name[40];
        row_t r;
        uint64_t first, b1, p1, fpx;
        mt.timed = 0;
        static_screen(L[i].kind, L[i].arg, name, sizeof name);
        measure_start(P);
        ms_begin();
        ui.force = 1;
        mframe();
        first = ms.bytes;
        fpx = ms.raw / 2u;
        b1 = ms.bytes; p1 = ms.passes;
        mframes_for(2000000u);
        r = ms_end();
        sum_first += first;
        if (first > max_first) max_first = first;
        {
            uint64_t idle = (ms.bytes - b1) * 1000000u / (vt_us - ms.t0 > 1 ? vt_us - ms.t0 : 1);
            sum_idle_bps += idle;
            if (idle > worst_idle) { worst_idle = idle; snprintf(worst, sizeof worst, "%s", name); }
            if (getenv("MT_DETAIL"))
                printf("  a %-20s P=%4u: first draw %6llu B (%llu px), then %6llu B/s, %llu passes, biggest call %5u B, drops %u (steady %u)\n",
                       name, P, (unsigned long long)first, (unsigned long long)fpx, (unsigned long long)idle,
                       (unsigned long long)(ms.passes - p1), r.burst, r.drops, r.steady);
        }
        drops += r.drops;
        steady += r.steady;
        waits += r.waits;
        over_us += ms.over_us;
        tot_passes += ms.passes;
        tot_us += vt_us - ms.t0;
        forced += r.forced;
        if (r.backlog > backlog) backlog = r.backlog;
        if (r.burst > burst) burst = r.burst;
        if (r.drops) {
            bad_pages++;
            if (strlen(names_bad) + strlen(name) + 3 < sizeof names_bad) { strcat(names_bad, name); strcat(names_bad, ", "); }
        }
        settle_report(name);
    }
    if (!mirror_on) {
        printf("a static (%u screens)       off %5u | %5.1f | (mirror off: the UI's own pace)\n", n, (unsigned)MIRROR_RING,
               tot_passes * 1e6 / (double)tot_us);
        return 0;
    }
    printf("a static (%u screens)      %4u %5u | %5.1f pass/s | first draw: mean %llu B, max %llu B | idle: mean %llu B/s, worst %llu B/s "
           "(%s) | drops %u (steady %u) on %u screens%s%s | forced %u | backlog max %u, biggest call %u B | waits %u, UI waited past "
           "the panel %.1f ms in all | %s\n",
           n, P, (unsigned)MIRROR_RING, tot_passes * 1e6 / (double)tot_us, (unsigned long long)(sum_first / n), (unsigned long long)max_first,
           (unsigned long long)(sum_idle_bps / n), (unsigned long long)worst_idle, worst, drops, steady, bad_pages,
           bad_pages ? ": " : "", names_bad, forced, backlog, burst, waits, over_us / 1000.0, steady ? "NO" : "fits");
    return 0;
}

/* b1: normal use; b2: the reference's random use */
static int measure_use(uint32_t P, int random_use)
{
    row_t r;
    uint32_t seed = 777, held = 0, f = 0;
    static const uint8_t PAGEB[] = {B_ENV, B_LFO, B_FX, B_ARP, B_SEQ, B_GLO, B_SCL, B_EDIT};
    #define RR(n) ((seed = seed * 1664525u + 1013904223u) >> 8) % (n)
    song.sel = 0; go_home(); frames(4);
    transport_req = 1; frames(30);
    measure_start(P);
    ms_begin();
    while (vt_us < 10000000u) {
        if (random_use) {                          /* (ui_pages_test.c, its fuzz) */
            if (RR(6) == 0) {
                uint32_t bt = RR(NB);
                if (bt == B_PLAY || bt == B_REC) bt = B_ENV;   /* (the sequencer keeps running) */
                edges_btn |= 1u << panel.btn[bt];
                if (bt != B_HOME && RR(3) == 0) held ^= 1u << panel.btn[bt];
            }
            if (RR(30) == 0) held = 0;
            fm1_in.buttons = held & ~(1u << panel.btn[B_HOME]);
            if (RR(4) == 0) { uint32_t e = RR(7); encs[e] += (int32_t)RR(5) - 2; }
            if (RR(10) == 0) fm1_in.notes = 1u << RR(27);
            else if (!RR(3)) fm1_in.notes = 0;
            mframe();
            if (ui.menu) { ui.menu = 0; ui.force = 1; }
        } else {                                   /* a person: a page now and then, knobs, a few notes */
            f++;
            if (f % 40u == 0u) {
                uint32_t c = RR(10);
                if (c < 6u) mtap(PAGEB[RR(sizeof PAGEB)]);
                else if (c < 8u) { go_home(); }
                else if (c < 9u) { song.sel = TRK_DRUM; studio_open(SC_DRUM); }
                else { song.sel = (uint8_t)RR(3); go_home(); }
            }
            if (RR(3) == 0) { uint32_t e = panel.enc[EN_K1 + RR(4)]; encs[e] += RR(2) ? 1 : -1; }
            if (RR(25) == 0) { int32_t d = RR(2) ? 1 : -1; encs[panel.enc[EN_SELECT]] += d; }
            if (RR(12) == 0) fm1_in.notes = 1u << RR(27);
            else if (RR(3) == 0) fm1_in.notes = 0;
            mframe();
        }
    }
    #undef RR
    r = ms_end();
    row_print(random_use ? "b2 random use (fuzz)" : "b1 normal use", &r);
    fm1_in.buttons = 0; fm1_in.notes = 0;
    settle_report(random_use ? "b2" : "b1");
    return 0;
}

/* c: every visualiser style, playing */
static int measure_vis(uint32_t P)
{
    uint32_t s;
    song.sel = 0; go_home(); frames(4);
    transport_req = 1; frames(30);
    vis_open(); frames(4);
    measure_start(P);
    for (s = 0; s < VIS_N; s++) {
        char nm[40];
        row_t r;
        uint64_t t0;
        vis_style = (uint8_t)s; vis_name_t = 30; ui.force = 1;
        ms_begin();
        t0 = vt_us;
        while (vt_us - t0 < 5000000u) {
            uint32_t q = (uint32_t)((vt_us - t0) / 15000u);
            fm1_in.notes = (q % 23u) < 4u ? 1u << (5u + (q / 23u) % 14u) : 0u;   /* (live notes too) */
            mframe();
        }
        fm1_in.notes = 0;
        r = ms_end();
        snprintf(nm, sizeof nm, "c vis %2u %s", s + 1u, VIS_NAME[s]);
        row_print(nm, &r);
    }
    settle_report("c");
    return 0;
}

/* d: the Snoop37 intro, as snoop_splash.c mk_frame draws it (5 strips of 240 x 48), then the fade-in passes */
static uint16_t mk_pool[6720], mk_strip[2][240 * MK_SH];
static int measure_intro(uint32_t P)
{
    row_t r;
    uint64_t t0;
    uint32_t k, s;
    int32_t l[32], rr[32], se[32];
    song.playing = 0; transport_req = 2; go_home(); frames(4);
    measure_start(P);
    ms_begin();
    mk_mem(mk_pool);
    mk_ota = 0;
    mkj_start();
    lcd_fill(0, 0, 240, 240, 0);                   /* (snoop_splash_bg) */
    t0 = vt_us;
    for (;;) {
        uint32_t t = (uint32_t)((vt_us - t0) / 1000u), px0 = (uint32_t)st.px;
        uint64_t off0 = st.offered, b;
        if (t >= MK_LEN)
            break;
        while (mkj.on && mkj.s * 10u / 441u < t)
            mkj_render(l, rr, se, 32);
        mk_band_tick();
        mk_prep(t);
        for (s = 0; s < 240u / MK_SH; s++) {
            mk_render(mk_strip[s & 1u], (int32_t)(s * MK_SH), t);
            lcd_blit(0, s * MK_SH, 240, MK_SH, mk_strip[s & 1u]);
        }
        if (mirror_frame_end())
            ms.forced++;
        b = st.offered - off0 + 3u;
        ms.bytes += b;
        ms.raw += 2u * (st.px - px0);
        ms.passes++;
        if (b > ms.pass_max) ms.pass_max = b;
    }
    lcd_fill(0, 0, 240, 240, 0);
    for (k = 1; k <= 4u; k++) {                    /* SLOOP's first screen, four times (lcd_dim not modelled) */
        uint64_t t1 = vt_us;
        ui.force = 1;
        mframe();
        if (vt_us < t1 + 70000u) vt_go(t1 + 70000u - vt_us);
    }
    r = ms_end();
    row_print("d Snoop37 intro", &r);
    settle_report("d");
    return 0;
}

/* the viewer stops reading for a second while the visualiser runs (P=512), then reads again: mirror_blit waits at
 * most MIRROR_STALL_MS once, the UI keeps its pace, the picture is right again soon after */
static int ui_stall(void *a)
{
    uint64_t t0, max_wait;
    double pps1, pps2;
    uint32_t waits, drops, ok;
    (void)a;
    rep = stdout;
    mirror_start();
    mt.check = 0;
    harness_init();
    song.sel = 0; go_home(); frames(4);
    transport_req = 1; frames(30);
    vis_open(); frames(4);
    vis_style = 0;
    measure_start(512u);
    ms_begin();
    mframes_for(1000000u);
    pps1 = ms.passes * 1e6 / (double)(vt_us - ms.t0);
    mt.paused = 1;
    ms_begin();
    mframes_for(1000000u);
    pps2 = ms.passes * 1e6 / (double)(vt_us - ms.t0);
    max_wait = ms.wait_max;
    waits = mirror_stat.waits - ms.waits0;
    drops = mirror_stat.drops - (uint32_t)ms.drops0;
    mt.paused = 0;
    settle_ok = 0;
    settle_watch = 1;
    t0 = vt_us;
    while (!settle_ok && vt_us - t0 < 3000000u)
        mframe();
    settle_watch = 0;
    ok = max_wait <= (MIRROR_STALL_MS + 1u) * 1000u && waits <= 2u && pps2 >= pps1 * 0.97 && settle_ok;
    printf("mirror: the viewer stops reading for 1 s (visualiser, P=512): %u waits, the longest %.1f ms, %u records dropped, "
           "%.1f passes/s (reading: %.1f); it reads again: picture right %.2f s later%s   %s\n", waits, max_wait / 1000.0, drops,
           pps2, pps1, (vt_us - t0) / 1e6, settle_ok ? "" : " (NOT within 3 s)", ok ? "ok" : "FAIL");
    return !ok;
}

struct m_args { int which; uint32_t P; };
static int measure_child(void *a)
{
    struct m_args *m = a;
    rep = stdout;
    mirror_start();
    mt.check = 0;
    harness_init();
    switch (m->which) {
    case 0: return measure_static(m->P);
    case 1: return measure_use(m->P, 0);
    case 2: return measure_use(m->P, 1);
    case 3: return measure_vis(m->P);
    default: return measure_intro(m->P);
    }
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : "build/host/mirror";
    int fails = 0;
    outdir = dir;
    if (argc > 2 && !strcmp(argv[2], "measure")) {
        uint32_t i, w;
        int np = 0;
        for (i = 3; i < (uint32_t)argc && np < 3; i++) mt_P_list[np++] = (uint32_t)atoi(argv[i]);
        if (!np) np = 3;
        printf("scenario                    P  ring | pass/s | B/pass: mean    max |     B/s  USB%% | ratio | drops steady | forced | backlog  call |  waits inside over | verdict\n");
        for (w = 0; w < 5u; w++)
            for (i = 0; i <= (uint32_t)np; i++) {     /* (first the mirror off: the UI's own pace) */
                struct m_args m = {(int)w, i ? mt_P_list[i - 1u] : 0u};
                child(measure_child, &m);
            }
        return 0;
    }
    printf("mirror: the UI harness (FM-1 SLOOP screens) through mirror.c and mirror_decode.c; ring %u B\n", (unsigned)MIRROR_RING);
    {
        static const uint32_t PS[3] = {1023u, 196u, 64u};
        uint32_t i;
        for (i = 0; i < 3u; i++) {
            struct ref_args a = {PS[i], 0, dir};
            fails += child(ref_child, &a);
        }
        for (i = 0; i < 2u; i++) {
            struct ref_args a = {i ? 1023u : 196u, 50u, dir};
            fails += child(ref_child, &a);
        }
    }
    {
        uint32_t n = 3000;
        fails += child(enc_child, &n);
    }
    fails += child(proto_child, 0);
    fails += child(status_child, 0);
    fails += child(ui_stall, 0);
    fails += child(frozen_child, 0);
    {
        uint32_t n = getenv("FUZZ_N") ? (uint32_t)strtoul(getenv("FUZZ_N"), 0, 10) : 3000000u;
        fails += child(fuzz_child, &n);
    }
    printf("mirror: %s\n", fails ? "FAILED" : "round trip (every screen, the random use, packets 1023 / 196 / 64), losses, encoder (waits, stall), beat, decoder fuzz PASS");
    return fails != 0;
}
