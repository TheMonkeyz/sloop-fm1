/* SPDX-License-Identifier: GPL-3.0-only */
/* Snoop37 screen mirror (SNOOP_MIRROR=1): the wire format, shared by the firmware (mirror.c) and the
 * viewer (tools/mirror/mirror_decode.c, copied into the ESP32-P4 project).
 *
 * The firmware has no framebuffer: every pixel reaches the panel through lcd_fill / lcd_blit (lcd.c).
 * The mirror copies those calls into a byte stream of records:
 *
 *   record = type(1) len(2, little-endian: the payload's bytes) payload(len)
 *
 *   'F' fill   x y w h hi lo          a rectangle of one colour
 *   'B' blit   x y w h rle...         a rectangle of pixels, row-major, w*h pixels once unpacked
 *   'E' end    (no payload)           the UI pass is over: the viewer shows what it has
 *
 * x, y: 0..239. w, h: 1..240. A colour is RGB565, high byte first (the panel's order).
 * A blit never holds more than MIRROR_BAND_PX pixels (the firmware cuts a blit into bands of whole rows),
 * so a record never exceeds MIRROR_REC_MAX bytes.
 *
 * rle: a control byte n, then
 *   n = 1..127    one colour (2 bytes), n times
 *   n = 128..255  n - 127 pixels, each 2 bytes
 *
 * The stream travels in isochronous packets (no retry: a packet may be lost):
 *
 *   packet = seq(1) ver(1) first(2, little-endian) stream bytes...
 *
 * seq counts the packets that carry data. first is the offset, in this packet's stream bytes, of the
 * first record that starts in it (MIRROR_NO_FIRST: none). After a gap in seq the viewer drops what it
 * was assembling, starts again at `first`, and asks the device for a full redraw (MIRROR_REQ_RESYNC). */
#ifndef MIRROR_PROTO_H
#define MIRROR_PROTO_H

#define MIRROR_VER 1u
#define MIRROR_W 240u
#define MIRROR_H 240u
#define MIRROR_PKT_HDR 4u
#define MIRROR_NO_FIRST 0xFFFFu
#define MIRROR_BAND_PX 960u                         /* four full rows */
#define MIRROR_REC_MAX (4u + MIRROR_BAND_PX * 2u + (MIRROR_BAND_PX + 127u) / 128u + 8u)

/* On the cable: USB 1209:0001, a vendor interface (class 0xFF, number 2). SET_INTERFACE alt 1 starts the
 * stream (the device redraws everything) on its isochronous IN endpoint 0x84; alt 0 stops it.
 * One vendor request on EP0 (bmRequestType 0x41: host to device, vendor, interface; wValue 0, wIndex the
 * interface, no data): */
#define MIRROR_REQ_RESYNC 0x62u                     /* the device redraws everything */

#endif
