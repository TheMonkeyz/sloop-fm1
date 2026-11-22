/* SPDX-License-Identifier: GPL-3.0-only */
/* Snoop37 screen mirror: the viewer's decoder (firmware/src/mirror_proto.h). Plain C, no hardware:
 * packets in, a 240 x 240 picture out. */
#ifndef MIRROR_DECODE_H
#define MIRROR_DECODE_H
#include <stdint.h>
#include "mirror_proto.h"

typedef struct {
    uint16_t shadow[MIRROR_W * MIRROR_H];           /* RGB565, this machine's byte order, row-major */
    /* what changed since the last md_take_dirty */
    int dx0, dy0, dx1, dy1;                         /* [x0, x1) x [y0, y1); x1 <= x0: nothing */
    uint8_t in_sync, have_seq, seq;
    uint8_t hn, type;                               /* header bytes seen (0..2), the record's type */
    uint16_t len, got;
    uint8_t rec[MIRROR_REC_MAX];
    uint32_t frames, recs, lost, bad;               /* 'E' records, records applied, gaps, malformed records */
} mirror_dec_t;

/* results of md_packet, or-ed */
#define MD_FRAME 1                                  /* an 'E' arrived: show the picture (md_take_dirty) */
#define MD_RESYNC 2                                 /* data was lost: ask the device for a full redraw */

void md_init(mirror_dec_t *d);
/* the device went away / came back: forget the stream's state (the picture stays) */
void md_reset(mirror_dec_t *d);
int md_packet(mirror_dec_t *d, const uint8_t *pkt, uint32_t len);
/* the rectangle that changed, then nothing changed. Returns 0 when nothing did. */
int md_take_dirty(mirror_dec_t *d, int *x, int *y, int *w, int *h);

#endif
