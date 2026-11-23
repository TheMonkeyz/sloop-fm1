/* SPDX-License-Identifier: GPL-3.0-only */
/* Snoop37 screen mirror: the viewer's decoder (mirror_decode.h). Every length and coordinate comes from
 * the cable: nothing here trusts them. */
#include <string.h>
#include "mirror_decode.h"

void md_reset(mirror_dec_t *d)
{
    d->in_sync = 0;
    d->have_seq = 0;
    d->hn = 0;
    d->got = 0;
    d->len = 0;
}

void md_init(mirror_dec_t *d)
{
    memset(d, 0, sizeof *d);
}

static void md_dirty(mirror_dec_t *d, int x, int y, int w, int h)
{
    if (d->dx1 <= d->dx0) {
        d->dx0 = x;
        d->dy0 = y;
        d->dx1 = x + w;
        d->dy1 = y + h;
        return;
    }
    if (x < d->dx0)
        d->dx0 = x;
    if (y < d->dy0)
        d->dy0 = y;
    if (x + w > d->dx1)
        d->dx1 = x + w;
    if (y + h > d->dy1)
        d->dy1 = y + h;
}

int md_take_dirty(mirror_dec_t *d, int *x, int *y, int *w, int *h)
{
    if (d->dx1 <= d->dx0)
        return 0;
    *x = d->dx0;
    *y = d->dy0;
    *w = d->dx1 - d->dx0;
    *h = d->dy1 - d->dy0;
    d->dx0 = d->dx1 = 0;
    return 1;
}

/* a whole record in d->rec. Returns 0 when it is malformed. */
static int md_apply(mirror_dec_t *d, int *res)
{
    const uint8_t *p = d->rec;
    uint32_t x, y, w, h, n, i, k, at;
    if (d->type == 'E') {
        if (d->len)
            return 0;
        d->frames++;
        *res |= MD_FRAME;
        return 1;
    }
    if (d->type == 'S') {
        if (d->len != MIRROR_STATUS_LEN)
            return 0;
        if (!d->have_status || memcmp(d->status, p, MIRROR_STATUS_LEN)) {
            memcpy(d->status, p, MIRROR_STATUS_LEN);
            d->have_status = 1;
            *res |= MD_STATUS;
        }
        return 1;
    }
    if (d->len < 4u)
        return 0;
    x = p[0];
    y = p[1];
    w = p[2];
    h = p[3];
    if (!w || !h || x + w > MIRROR_W || y + h > MIRROR_H)
        return 0;
    if (d->type == 'F') {
        uint16_t c;
        if (d->len != 6u)
            return 0;
        c = (uint16_t)(p[4] << 8 | p[5]);
        for (k = 0; k < h; k++) {
            uint16_t *q = d->shadow + (y + k) * MIRROR_W + x;
            for (i = 0; i < w; i++)
                q[i] = c;
        }
    } else {                                        /* 'B' */
        n = w * h;
        if (n > MIRROR_BAND_PX)
            return 0;
        /* first pass: the rle must unpack to exactly n pixels, before a pixel is written */
        for (at = 4u, i = 0; at < d->len;) {
            uint32_t c = p[at++];
            if (!c)
                return 0;
            if (c <= 127u) {
                at += 2u;
                i += c;
            } else {
                at += (c - 127u) * 2u;
                i += c - 127u;
            }
        }
        if (at != d->len || i != n)
            return 0;
        for (at = 4u, i = 0; i < n;) {
            uint32_t c = p[at++];
            if (c <= 127u) {
                uint16_t v = (uint16_t)(p[at] << 8 | p[at + 1u]);
                at += 2u;
                while (c--) {
                    d->shadow[(y + i / w) * MIRROR_W + x + i % w] = v;
                    i++;
                }
            } else {
                for (c -= 127u; c; c--, at += 2u, i++)
                    d->shadow[(y + i / w) * MIRROR_W + x + i % w] = (uint16_t)(p[at] << 8 | p[at + 1u]);
            }
        }
    }
    md_dirty(d, (int)x, (int)y, (int)w, (int)h);
    d->recs++;
    return 1;
}

int md_packet(mirror_dec_t *d, const uint8_t *pkt, uint32_t len)
{
    int res = 0;
    uint32_t first, pos = MIRROR_PKT_HDR;
    if (len <= MIRROR_PKT_HDR || pkt[1] < 1u || pkt[1] > MIRROR_VER)    /* (version 1: the same stream, no 'S') */
        return 0;
    first = (uint32_t)pkt[2] | (uint32_t)pkt[3] << 8;
    if (d->have_seq && pkt[0] != (uint8_t)(d->seq + 1u)) {
        d->in_sync = 0;                             /* a packet was lost (also one of a redraw already asked for) */
        d->lost++;
        res |= MD_RESYNC;
    }
    d->have_seq = 1;
    d->seq = pkt[0];
    while (pos < len) {
        if (!d->in_sync) {                          /* pick the stream up at a record's first byte */
            if (first == MIRROR_NO_FIRST || MIRROR_PKT_HDR + first >= len)
                return res;
            if (pos <= MIRROR_PKT_HDR + first)
                pos = MIRROR_PKT_HDR + first;
            else
                return res;                         /* (lost again inside this packet: wait for the next) */
            first = MIRROR_NO_FIRST;
            d->hn = 0;
            d->got = 0;
            d->in_sync = 1;
        }
        if (d->hn < 3u) {
            uint8_t b = pkt[pos++];
            if (d->hn == 0u)
                d->type = b;
            else if (d->hn == 1u)
                d->len = b;
            else
                d->len = (uint16_t)(d->len | b << 8);
            if (++d->hn < 3u)
                continue;
            d->got = 0;
            if ((d->type != 'F' && d->type != 'B' && d->type != 'E' && d->type != 'S') || d->len > MIRROR_REC_MAX)
                goto bad;
        }
        if (d->got < d->len) {
            uint32_t k = d->len - d->got;
            if (k > len - pos)
                k = len - pos;
            memcpy(d->rec + d->got, pkt + pos, k);
            d->got = (uint16_t)(d->got + k);
            pos += k;
        }
        if (d->got == d->len) {
            d->hn = 0;
            if (!md_apply(d, &res))
                goto bad;
        }
        continue;
bad:
        d->bad++;
        d->in_sync = 0;
        d->hn = 0;
        res |= MD_RESYNC;
        return res;                                 /* the rest of this packet cannot be trusted */
    }
    return res;
}
