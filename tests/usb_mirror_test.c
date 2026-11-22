/* SPDX-License-Identifier: GPL-3.0-only */
/* The screen mirror's USB side (firmware/src/usb.c with SNOOP_MIRROR=1, FELUCCA_UAC=0, FELUCCA_CDC=0) on the host:
 *   descriptors  sizeof CFG_DESC == CFG_LEN == wTotalLength; the bLength chain ends at the end; bNumInterfaces = the
 *                highest interface + 1; IF 2 (vendor) alt 0 no endpoint, alt 1 one: 0x84 isochronous, MIRROR_MAXP
 *                bytes, every frame; bcdDevice +0.40
 *   EP0          the requests as a host sends them, through a simulated SIE (fm1_usb.h's functions replaced here):
 *                SET_CONFIGURATION, SET_INTERFACE (2, 1): mirror_on and mirror_redraw; GET_INTERFACE: the alt;
 *                SET_INTERFACE (2, 0): mirror_on off; the vendor request 0x41 / MIRROR_REQ_RESYNC: mirror_redraw;
 *                another vendor request: stalled
 *   EP4          mirror_service (the TIMER5 tick): one packet of at most MIRROR_MAXP bytes, none while one is queued
 * With SNOOP_MIRROR=0 the descriptors are tests/uac_test.c's business (its UAC_DUMP comparisons).
 *   cc -Ifirmware/src -Ifirmware/hal -o build/host/usb_mirror_test tests/usb_mirror_test.c */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define SNOOP_MIRROR 1
#define FELUCCA_UAC 0
#define FELUCCA_CDC 0
#define FELUCCA_OTA 0
#define RING_PUBLISH() __asm__ volatile("" ::: "memory")
static void fm1_delay_ms(uint32_t ms) { (void)ms; }
static uint32_t host_ms;
#define MIRROR_MS() host_ms
#include "../firmware/src/mirror.c"

/* fm1_usb.h's register functions out of the way (#pragma once: usb.c's include of it is then skipped) ... */
#define fm1_usb_reset hw_usb_reset
#define fm1_usb_attach hw_usb_attach
#define fm1_usb_off hw_usb_off
#define fm1_usb_sie_on hw_usb_sie_on
#define fm1_usb_sie_wr_start hw_usb_sie_wr_start
#define fm1_usb_sie_rd_start hw_usb_sie_rd_start
#define fm1_usb_sie_done hw_usb_sie_done
#define fm1_usb_sie_data hw_usb_sie_data
#define fm1_usb_ep0_buf hw_usb_ep0_buf
#define fm1_usb_ep_txbuf hw_usb_ep_txbuf
#define fm1_usb_ep_rxbuf hw_usb_ep_rxbuf
#define fm1_usb_ep0_send hw_usb_ep0_send
#define fm1_usb_ep_send hw_usb_ep_send
#define fm1_usb_ep4_txbuf hw_usb_ep4_txbuf
#define fm1_usb_ep4_send hw_usb_ep4_send
#define fm1_usb_rx_sync hw_usb_rx_sync
#define fm1_usb_ep_enable hw_usb_ep_enable
#define fm1_usb_sof_take hw_usb_sof_take
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#include "../firmware/hal/fm1_usb.h"
#undef fm1_usb_reset
#undef fm1_usb_attach
#undef fm1_usb_off
#undef fm1_usb_sie_on
#undef fm1_usb_sie_wr_start
#undef fm1_usb_sie_rd_start
#undef fm1_usb_sie_done
#undef fm1_usb_sie_data
#undef fm1_usb_ep0_buf
#undef fm1_usb_ep_txbuf
#undef fm1_usb_ep_rxbuf
#undef fm1_usb_ep0_send
#undef fm1_usb_ep_send
#undef fm1_usb_ep4_txbuf
#undef fm1_usb_ep4_send
#undef fm1_usb_rx_sync
#undef fm1_usb_ep_enable
#undef fm1_usb_sof_take
/* ... and a simulated SIE in their place: a register file (indexed registers per S_INDEX), EP0 with a SETUP
 * pending (CSR0 RxPktRdy) until usb.c writes ServicedRxPktRdy, the bytes EP0 and EP4 send */
static uint32_t sie_index, sie_val, sie_reg[16][32], setup_pending, csr0_last;
static uint8_t ep0_out[256], ep4_out[1024];
static uint32_t ep0_out_n, ep4_out_n, ep4_sends;
static void fm1_usb_reset(void) {}
static void fm1_usb_attach(void *ep0) { (void)ep0; }
static void fm1_usb_off(void) {}
static uint32_t fm1_usb_sie_on(void) { return 1; }
static void fm1_usb_sie_wr_start(uint32_t r, uint32_t v)
{
    if (r == 14u) {                                     /* S_INDEX */
        sie_index = v & 15u;
        return;
    }
    if (r == 17u && sie_index == 0u) {                  /* CSR0 */
        csr0_last = v;
        if (v & 0x40u)
            setup_pending = 0;                          /* ServicedRxPktRdy */
        return;
    }
    sie_reg[r >= 16u ? sie_index : 0u][r & 31u] = v & 0xFFu;
}
static void fm1_usb_sie_rd_start(uint32_t r)
{
    if (r == 17u && sie_index == 0u)
        sie_val = setup_pending ? 0x01u : 0u;
    else if (r == 22u && sie_index == 0u)
        sie_val = 8u;                                   /* COUNT0 */
    else
        sie_val = sie_reg[r >= 16u ? sie_index : 0u][r & 31u];
}
static uint32_t fm1_usb_sie_done(void) { return 0x8000u; }
static uint32_t fm1_usb_sie_data(void) { return sie_val & 0xFFu; }
static void fm1_usb_ep0_buf(void *p) { (void)p; }
static void fm1_usb_ep_txbuf(uint32_t ep, void *p) { (void)ep, (void)p; }
static void fm1_usb_ep_rxbuf(uint32_t ep, void *p) { (void)ep, (void)p; }
static void fm1_usb_ep0_send(void *p, uint32_t n)
{
    if (ep0_out_n + n <= sizeof ep0_out)
        memcpy(ep0_out + ep0_out_n, p, n);
    ep0_out_n += n;
}
static void fm1_usb_ep_send(uint32_t ep, void *p, uint32_t n) { (void)ep, (void)p, (void)n; }
static void fm1_usb_ep4_txbuf(void *p) { (void)p; }
static void fm1_usb_ep4_send(void *p, uint32_t n)
{
    if (n <= sizeof ep4_out)
        memcpy(ep4_out, p, n);
    ep4_out_n = n;
    ep4_sends++;
}
static void fm1_usb_rx_sync(void) {}
static void fm1_usb_ep_enable(uint32_t eps) { (void)eps; }
static uint32_t fm1_usb_sof_take(void) { return 0; }
#include "../firmware/src/usb.c"

static int fails;
static int check(const char *what, int ok)
{
    printf("usb mirror: %-90s %s\n", what, ok ? "ok" : "FAIL");
    fails += !ok;
    return ok;
}
static uint32_t le16(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8; }

/* a control request; returns the last CSR0 write: 0x48 ack (no data), 0x0A data sent, 0x60 stall */
static uint32_t request(uint8_t bm, uint8_t req, uint16_t val, uint16_t idx, uint16_t len)
{
    uint8_t s[8] = {bm, req, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8), (uint8_t)len, (uint8_t)(len >> 8)};
    memcpy(ep0buf, s, 8);
    ep0_out_n = 0;
    setup_pending = 1;
    ep0_service();
    while (usb.e0_tx)                                   /* (the IN data stage, 64 bytes at a time) */
        ep0_service();
    return csr0_last;
}

static void test_descriptors(void)
{
    const uint8_t *c, *dv;
    uint16_t cl, dl;
    uint32_t off, max_if = 0, chain = 1, eps_ok = 1, cur_if = 99, cur_alt = 99, cur_neps = 0, got = 0, iso = 0, alt0 = 0, alt1 = 0;
    get_desc(0x0100, &dv, &dl);
    get_desc(0x0200, &c, &cl);
    check("sizeof CFG_DESC == CFG_LEN == wTotalLength == the bytes get_desc gives", sizeof CFG_DESC == CFG_LEN && le16(c + 2) == cl && cl == CFG_LEN);
    check("device descriptor: 18 bytes; bcdDevice +0.40 (a mirror build)", dl == 18u && dv[0] == 18u && (dv[12] & 0x40u) == 0x40u);
    for (off = 0; off < cl;) {
        const uint8_t *d = c + off;
        if (d[0] < 2u || off + d[0] > cl) {
            chain = 0;
            break;
        }
        if (d[1] == 4u) {
            if (cur_if != 99u && got != cur_neps)
                eps_ok = 0;
            cur_if = d[2], cur_alt = d[3], cur_neps = d[4], got = 0;
            if (cur_if > max_if)
                max_if = cur_if;
            if (cur_if == MIRROR_IF && cur_alt == 0u)
                alt0 = d[0] == 9u && d[4] == 0u && d[5] == 0xFFu;
            if (cur_if == MIRROR_IF && cur_alt == 1u)
                alt1 = d[0] == 9u && d[4] == 1u && d[5] == 0xFFu;
        } else if (d[1] == 5u) {
            got++;
            if (d[2] == 0x84u)
                iso += cur_if == MIRROR_IF && cur_alt == 1u && d[0] >= 7u && (d[3] & 3u) == 1u && le16(d + 4) == MIRROR_MAXP &&
                       d[6] == 1u;
        }
        off += d[0];
    }
    if (cur_if != 99u && got != cur_neps)
        eps_ok = 0;
    check("the bLength chain ends exactly at wTotalLength", chain && off == cl);
    check("bNumInterfaces = the highest interface + 1 (3: AC, MIDI, the mirror)", c[4] == max_if + 1u && c[4] == 3u);
    check("every interface setting has its bNumEndpoints endpoints", eps_ok);
    check("IF 2 alt 0: vendor, no endpoint", alt0);
    check("IF 2 alt 1: vendor, one endpoint", alt1);
    check("EP 0x84 (once): isochronous, wMaxPacketSize MIRROR_MAXP (512), bInterval 1, in IF 2 alt 1", iso == 1u);
}

static void test_ep0(void)
{
    uint32_t r;
    r = request(0x00, 0x09, 1, 0, 0);
    check("SET_CONFIGURATION 1: acked, configured, the mirror off", r == 0x48u && usb.config == 1u && !mirror_on);
    mirror_redraw = 0;
    r = request(0x01, 0x0B, 1, MIRROR_IF, 0);
    check("SET_INTERFACE (2, alt 1): acked, mirror_on, mirror_redraw (a whole picture first)", r == 0x48u && mirror_on && mirror_redraw);
    r = request(0x81, 0x0A, 0, MIRROR_IF, 1);
    check("GET_INTERFACE (2): 1", r == 0x0Au && ep0_out_n == 1u && ep0_out[0] == 1u);
    mirror_redraw = 0;
    r = request(0x41, MIRROR_REQ_RESYNC, 0, MIRROR_IF, 0);
    check("vendor request 0x41 / MIRROR_REQ_RESYNC (0x62): acked, mirror_redraw", r == 0x48u && mirror_redraw);
    mirror_redraw = 0;
    r = request(0x41, MIRROR_REQ_RESYNC, 0, MIRROR_IF, 4);
    check("the same with a data stage (wLength 4): stalled, nothing done", r == 0x60u && !mirror_redraw);
    r = request(0x41, 0x61, 0, MIRROR_IF, 0);
    check("another vendor request (0x61): stalled", r == 0x60u && mirror_on);
    {   /* EP4: the TIMER5 tick sends a packet, then none while it is queued, then the next */
        uint16_t px[240 * 4];
        uint32_t i, s0;
        for (i = 0; i < 240u * 4u; i++)
            px[i] = (uint16_t)(i * 2654435761u >> 16);   /* (noise: 1.9 KB) */
        mirror_blit(0, 0, 240, 4, px);
        s0 = ep4_sends;
        mirror_service();
        check("mirror_service: a packet of MIRROR_MAXP bytes at most, the header's version", ep4_sends == s0 + 1u && ep4_out_n <= MIRROR_MAXP &&
              ep4_out_n > MIRROR_PKT_HDR && ep4_out[1] == MIRROR_VER);
        mirror_service();
        check("mirror_service: nothing more while that packet is queued (TxPktRdy)", ep4_sends == s0 + 1u);
        sie_reg[4][S_TXCSR1] &= ~1u;                    /* (the host's IN token took it) */
        mirror_service();
        check("mirror_service: the next packet once it has gone", ep4_sends == s0 + 2u);
    }
    r = request(0x01, 0x0B, 0, MIRROR_IF, 0);
    check("SET_INTERFACE (2, alt 0): acked, mirror_on off", r == 0x48u && !mirror_on);
    r = request(0x81, 0x0A, 0, MIRROR_IF, 1);
    check("GET_INTERFACE (2): 0", r == 0x0Au && ep0_out_n == 1u && ep0_out[0] == 0u);
    r = request(0x01, 0x0B, 2, MIRROR_IF, 0);
    check("SET_INTERFACE (2, alt 2): stalled", r == 0x60u && !mirror_on);
    request(0x01, 0x0B, 1, MIRROR_IF, 0);
    r = request(0x00, 0x09, 0, 0, 0);
    check("SET_CONFIGURATION 0: the mirror off", r == 0x48u && !mirror_on);
    r = request(0x02, 0x01, 0, 0x84, 0);
    check("CLEAR_FEATURE (ENDPOINT_HALT, 0x84): acked (isochronous)", r == 0x48u);
}

int main(void)
{
    test_descriptors();
    test_ep0();
    printf("usb mirror: %s\n", fails ? "FAILED" : "descriptors, EP0 requests, EP4 packets PASS");
    return fails != 0;
}
