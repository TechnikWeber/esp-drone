/*
 * Host unit tests for the captive portal DNS answer builder.
 * Build and run: make -C components/webctrl/host_test
 *
 * Copyright 2026 Philipp Weber
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dns_server.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        checks++;                                                            \
        if (!(cond)) {                                                       \
            failures++;                                                      \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)

/* Build a query for name with the given type; returns its length. */
static size_t makeQuery(uint8_t *buf, const char *name, uint16_t qtype, uint16_t flags)
{
    memset(buf, 0, 512);
    buf[0] = 0xAB;              // ID
    buf[1] = 0xCD;
    buf[2] = (uint8_t)(flags >> 8);
    buf[3] = (uint8_t)flags;
    buf[5] = 1;                 // QDCOUNT
    size_t pos = 12;
    const char *p = name;
    while (*p) {
        const char *dot = strchr(p, '.');
        size_t len = dot ? (size_t)(dot - p) : strlen(p);
        buf[pos++] = (uint8_t)len;
        memcpy(&buf[pos], p, len);
        pos += len;
        p += len;
        if (*p == '.') {
            p++;
        }
    }
    buf[pos++] = 0;
    buf[pos++] = (uint8_t)(qtype >> 8);
    buf[pos++] = (uint8_t)qtype;
    buf[pos++] = 0;
    buf[pos++] = 1;             // class IN
    return pos;
}

int main(void)
{
    uint8_t q[512], r[512];
    const uint8_t ipBytes[4] = {192, 168, 43, 42};
    uint32_t ip;
    memcpy(&ip, ipBytes, 4);    // network byte order as in the firmware

    puts("A query is answered with the drone's address");
    size_t ql = makeQuery(q, "connectivitycheck.gstatic.com", 1, 0x0100);
    size_t rl = dnsBuildResponse(q, ql, ip, r, sizeof(r));
    CHECK(rl == ql + 16);
    CHECK(r[0] == 0xAB && r[1] == 0xCD);          // same ID
    CHECK((r[2] & 0x80) != 0);                    // QR: response
    CHECK((r[2] & 0x04) != 0);                    // AA
    CHECK((r[2] & 0x01) != 0);                    // RD copied
    CHECK((r[3] & 0x0F) == 0);                    // NOERROR
    CHECK(r[5] == 1 && r[7] == 1);                // one question, one answer
    CHECK(r[ql] == 0xC0 && r[ql + 1] == 12);      // name pointer
    CHECK(r[ql + 3] == 1 && r[ql + 5] == 1);      // type A, class IN
    CHECK(r[ql + 11] == 4);                       // RDLENGTH
    CHECK(memcmp(&r[ql + 12], ipBytes, 4) == 0);

    puts("AAAA query gets an empty answer");
    ql = makeQuery(q, "captive.apple.com", 28, 0x0100);
    rl = dnsBuildResponse(q, ql, ip, r, sizeof(r));
    CHECK(rl == ql);
    CHECK(r[7] == 0 && (r[3] & 0x0F) == 0);

    puts("EDNS record of the query is not echoed");
    ql = makeQuery(q, "example.org", 1, 0x0100);
    q[11] = 1;                                    // ARCOUNT 1
    memset(&q[ql], 0, 11);                        // OPT record
    q[ql + 2] = 41;
    rl = dnsBuildResponse(q, ql + 11, ip, r, sizeof(r));
    CHECK(rl == ql + 16);
    CHECK(r[11] == 0);                            // ARCOUNT 0

    puts("non-standard opcode gets NOTIMP");
    ql = makeQuery(q, "example.org", 1, 0x2800);  // opcode 5 (update)
    rl = dnsBuildResponse(q, ql, ip, r, sizeof(r));
    CHECK(rl == ql);
    CHECK((r[3] & 0x0F) == 4);

    puts("malformed queries are ignored");
    ql = makeQuery(q, "example.org", 1, 0x0100);
    CHECK(dnsBuildResponse(q, 11, ip, r, sizeof(r)) == 0);        // short header
    CHECK(dnsBuildResponse(q, ql - 2, ip, r, sizeof(r)) == 0);    // no class
    q[12] = 64;                                                   // label too long
    CHECK(dnsBuildResponse(q, ql, ip, r, sizeof(r)) == 0);
    ql = makeQuery(q, "example.org", 1, 0x8100);                  // a response
    CHECK(dnsBuildResponse(q, ql, ip, r, sizeof(r)) == 0);
    ql = makeQuery(q, "example.org", 1, 0x0100);
    q[5] = 0;                                                     // no question
    CHECK(dnsBuildResponse(q, ql, ip, r, sizeof(r)) == 0);
    memset(q, 0, sizeof(q));                                      // name runs off the end
    q[5] = 1;
    for (int i = 12; i < 512; i++) {
        q[i] = 1;
    }
    CHECK(dnsBuildResponse(q, 512, ip, r, sizeof(r)) == 0);
    ql = makeQuery(q, "example.org", 1, 0x0100);
    CHECK(dnsBuildResponse(q, ql, ip, r, ql + 15) == 0);          // answer does not fit

    printf("\n%d checks, %d failed\n", checks, failures);
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
