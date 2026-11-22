/* SPDX-License-Identifier: GPL-3.0-only */
/* Snoop37 screen mirror (SNOOP_MIRROR=1): what lcd_fill / lcd_blit draw, as a byte stream for a viewer on
 * the USB cable (mirror_proto.h). No hardware here: lcd.c calls mirror_fill / mirror_blit, main.c calls
 * mirror_frame_end after a UI pass, usb.c takes packets with mirror_packet (from the timer ISR).
 *
 * One writer (the main loop) and one reader (the ISR) on a ring: the writer builds a record past
 * mr_wr and publishes it whole (mr_wr moves once, at the end), so the reader never sees half a record.
 *
 * A blit is larger than the ring (a visualiser's canvas packs to 25 KB). lcd_blit calls mirror_blit once
 * its DMA transfer runs: the panel takes w * h / 750 ms to swallow it, and for that long mirror_blit may
 * wait for the reader to make room, band after band, at no cost to the UI (its next draw waits for that
 * transfer anyway). Past that, or when the viewer stops reading, a band that does not fit is dropped and
 * the picture is repaired later by a full redraw (mirror_frame_end returns 1: the caller sets ui.force). */
#include "mirror_proto.h"

#ifndef MIRROR_RING
#define MIRROR_RING 16384u                          /* a power of two (8192: the oscilloscope drops a band now and then) */
#endif
_Static_assert((MIRROR_RING & (MIRROR_RING - 1u)) == 0, "the ring's size is a power of two");
_Static_assert(MIRROR_RING >= 4u * MIRROR_REC_MAX, "the ring holds a few records");
#ifndef MIRROR_MS
static volatile uint32_t fm1_ms;                    /* (core.h, included later: the same object) */
#define MIRROR_MS() fm1_ms                         /* milliseconds, counted by the timer ISR (the reader runs there too) */
#endif
#ifndef MIRROR_WAIT
#define MIRROR_WAIT() ((void)0)                     /* (the host tests: let the time and the reader run) */
#endif
#define MIRROR_PX_PER_MS 750u                       /* the panel's SPI: 12 MHz, 16 bits a pixel */
#define MIRROR_STALL_MS 3u                          /* no packet taken for this long: the viewer is not reading */
#define MIRROR_SPINS 1000000u                       /* waits in a row with the clock not moving (interrupts off): give up */
#define MIRROR_HOLD 30u                             /* UI passes between two repairs */
#define MIRROR_BEAT_MS 250u                         /* an idle screen still says 'E' this often (a lost last packet shows) */

static uint8_t mr_buf[MIRROR_RING];
static volatile uint32_t mr_wr, mr_rd;              /* free-running: published by the writer / taken by the reader */
static uint32_t mr_w;                               /* the writer's place in the record being built */
static uint8_t mr_fail;                             /* that record ran out of room */
static uint8_t mr_dirty;                            /* a record was dropped: the viewer's picture is wrong */
static uint8_t mr_drew;                             /* records were published since the last 'E' */
static uint8_t mr_hold;                             /* UI passes before the next repair */
static uint8_t mr_stall;                            /* the reader took nothing while we waited: no more waits ... */
static uint32_t mr_stall_rd;                        /* ... until it moves from here */
static uint32_t mr_beat;                            /* MIRROR_MS() at the last 'E' */
static volatile uint8_t mirror_on;                  /* the viewer asked for the stream (usb.c) */
static volatile uint8_t mirror_redraw;              /* the viewer asked for a full redraw (usb.c) */
static struct {
    uint32_t recs, drops, bytes, pkts, waits;       /* (the console / the host tests) */
} mirror_stat;

/* the reader's place in the record structure: header bytes seen (0..2), payload bytes to go */
static uint8_t mp_hn, mp_seq;
static uint16_t mp_len, mp_rem;

static void mr_put(uint32_t b)
{
    if (mr_w - mr_rd >= MIRROR_RING) {
        mr_fail = 1;
        return;
    }
    mr_buf[mr_w++ & (MIRROR_RING - 1u)] = (uint8_t)b;
}

static void mr_begin(uint32_t type)
{
    mr_w = mr_wr;
    mr_fail = 0;
    mr_put(type);
    mr_put(0);
    mr_put(0);
}

/* publish the record. Returns 0 when it did not fit (nothing was published). */
static int mr_end(void)
{
    uint32_t n = mr_w - mr_wr - 3u;
    if (mr_fail)
        return 0;
    mr_buf[(mr_wr + 1u) & (MIRROR_RING - 1u)] = (uint8_t)n;
    mr_buf[(mr_wr + 2u) & (MIRROR_RING - 1u)] = (uint8_t)(n >> 8);
    mr_drew = 1;
    mirror_stat.recs++;
    mirror_stat.bytes += n + 3u;
    __asm__ volatile("" ::: "memory");              /* the record's bytes are in the ring before the reader may see it */
    mr_wr = mr_w;
    return 1;
}

static void mr_drop(void)
{
    mr_dirty = 1;
    mirror_stat.drops++;
}

/* c: RGB565 as the UI names colours (lcd_fill's argument, after the clipping and the dimming) */
static void mirror_fill(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t c)
{
    if (!mirror_on)
        return;
    mr_begin('F');
    mr_put(x);
    mr_put(y);
    mr_put(w);
    mr_put(h);
    mr_put(c >> 8);
    mr_put(c);
    if (!mr_end())
        mr_drop();
}

/* n pixels as the canvas stores them (byte-swapped: the panel's byte order in memory) */
static void mr_rle(const uint16_t *p, uint32_t n)
{
    uint32_t i = 0, j, k;
    while (i < n) {
        for (j = i + 1u; j < n && j - i < 127u && p[j] == p[i]; j++)
            ;
        if (j - i >= 2u) {                          /* a run */
            mr_put(j - i);
            mr_put(p[i] & 0xFFu);
            mr_put(p[i] >> 8);
            i = j;
            continue;
        }
        /* single pixels, up to the next run of three (a run of two inside costs no more as pixels) */
        for (j = i + 1u; j < n && j - i < 128u; j++)
            if (j + 2u < n && p[j] == p[j + 1u] && p[j] == p[j + 2u])
                break;
        mr_put(127u + (j - i));
        for (k = i; k < j; k++) {
            mr_put(p[k] & 0xFFu);
            mr_put(p[k] >> 8);
        }
        i = j;
    }
}

/* wait for room for one more record, until `until` (MIRROR_MS). Returns 0 when there is no point in trying. */
static int mr_room(uint32_t t0, uint32_t budget)
{
    uint32_t rd = mr_rd, since = MIRROR_MS(), ms = since, spins = 0;
    if (mr_stall) {
        if (rd == mr_stall_rd)
            return 0;
        mr_stall = 0;
    }
    mirror_stat.waits++;
    while (MIRROR_RING - (mr_wr - mr_rd) < MIRROR_REC_MAX + 3u) {
        uint32_t now = MIRROR_MS();
        if (now != ms) {
            ms = now;
            spins = 0;
        } else if (++spins >= MIRROR_SPINS) {       /* the clock stands still: no reader either (the crash screen) */
            mr_stall = 1;
            mr_stall_rd = rd;
            return 0;
        }
        if (now - t0 >= budget)
            return 0;
        if (mr_rd != rd) {
            rd = mr_rd;
            since = now;
        } else if (now - since >= MIRROR_STALL_MS) {
            mr_stall = 1;                           /* the viewer does not read: stop waiting for it */
            mr_stall_rd = rd;
            return 0;
        }
        MIRROR_WAIT();
    }
    return 1;
}

/* called by lcd_blit once the panel's transfer of px runs (px stays as it is until the next lcd_sync) */
static void mirror_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint16_t *px)
{
    uint32_t rows, r, t0, budget;
    if (!mirror_on || !w || !h || x + w > MIRROR_W || y + h > MIRROR_H)
        return;
    rows = MIRROR_BAND_PX / w;
    t0 = MIRROR_MS();
    budget = w * h / MIRROR_PX_PER_MS;              /* (rounded down: never longer than the transfer) */
    while (h) {                                     /* bands of whole rows */
        r = h < rows ? h : rows;
        for (;;) {
            mr_begin('B');
            mr_put(x);
            mr_put(y);
            mr_put(w);
            mr_put(r);
            mr_rle(px, w * r);
            if (mr_end())
                break;
            if (!mr_room(t0, budget)) {
                mr_drop();
                break;
            }
        }
        px += w * r;
        y += r;
        h -= r;
    }
}

/* after a UI pass. Returns 1 when the caller must redraw everything (ui.force): the viewer asked, or
 * records were dropped and the ring has room again */
static int mirror_frame_end(void)
{
    int force = 0;
    uint32_t now;
    if (!mirror_on)
        return 0;
    now = MIRROR_MS();
    if (mr_drew || now - mr_beat >= MIRROR_BEAT_MS) {
        mr_begin('E');
        if (mr_end()) {                             /* (no room: the next pass tries again) */
            mr_drew = 0;
            mr_beat = now;
        }
    }
    if (mr_hold)
        mr_hold--;
    if (mirror_redraw) {
        mirror_redraw = 0;
        mr_dirty = 0;
        force = 1;
    } else if (mr_dirty && !mr_hold && mr_wr - mr_rd <= MIRROR_RING / 4u) {
        mr_dirty = 0;
        mr_hold = MIRROR_HOLD;                      /* a redraw that overflows too is not repeated at once */
        force = 1;
    }
    return force;
}

/* the next packet, at most max bytes (max > MIRROR_PKT_HDR); 0: nothing to send. The reader's side. */
static uint32_t mirror_packet(uint8_t *dst, uint32_t max)
{
    uint32_t rd = mr_rd, n = mr_wr - rd, pos = 0, first = MIRROR_NO_FIRST, k;
    if (!n)
        return 0;
    if (n > max - MIRROR_PKT_HDR)
        n = max - MIRROR_PKT_HDR;
    while (pos < n) {
        uint32_t b;
        if (mp_rem) {                               /* a payload: no need to look inside */
            k = n - pos < mp_rem ? n - pos : mp_rem;
            mp_rem = (uint16_t)(mp_rem - k);
            while (k--) {
                dst[MIRROR_PKT_HDR + pos] = mr_buf[(rd + pos) & (MIRROR_RING - 1u)];
                pos++;
            }
            continue;
        }
        b = mr_buf[(rd + pos) & (MIRROR_RING - 1u)];
        if (!mp_hn && first == MIRROR_NO_FIRST)
            first = pos;
        if (mp_hn == 1u)
            mp_len = (uint16_t)b;
        else if (mp_hn == 2u)
            mp_len = (uint16_t)(mp_len | b << 8);
        if (++mp_hn == 3u) {
            mp_hn = 0;
            mp_rem = mp_len;
        }
        dst[MIRROR_PKT_HDR + pos++] = (uint8_t)b;
    }
    dst[0] = mp_seq++;
    dst[1] = MIRROR_VER;
    dst[2] = (uint8_t)first;
    dst[3] = (uint8_t)(first >> 8);
    mr_rd = rd + n;
    mirror_stat.pkts++;
    return n + MIRROR_PKT_HDR;
}
