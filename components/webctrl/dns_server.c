/*
 * ESP-Drone Firmware
 *
 * Copyright 2026 Philipp Weber
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * dns_server.c - captive portal DNS: every name resolves to the drone.
 *
 * A queries get the drone's IPv4 address, all other types an empty answer
 * (NOERROR, no records), so phones do not wait for AAAA timeouts.
 */

#include <string.h>

#include "dns_server.h"

#define DNS_PORT          53
#define DNS_HEADER_LEN    12
#define DNS_MAX_PACKET    512
#define DNS_TYPE_A        1
#define DNS_CLASS_IN      1
#define DNS_TTL_S         10    /* short: phones must not keep the fake address */

#define DNS_FLAG_QR       0x8000
#define DNS_FLAG_OPCODE   0x7800
#define DNS_FLAG_AA       0x0400
#define DNS_FLAG_RD       0x0100
#define DNS_RCODE_NOTIMP  4

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

size_t dnsBuildResponse(const uint8_t *query, size_t queryLen, uint32_t ipv4,
                        uint8_t *resp, size_t respCap)
{
    if (query == NULL || resp == NULL || queryLen < DNS_HEADER_LEN || queryLen > DNS_MAX_PACKET) {
        return 0;
    }

    const uint16_t flags = rd16(&query[2]);
    const uint16_t qdcount = rd16(&query[4]);
    if ((flags & DNS_FLAG_QR) != 0 || qdcount == 0) {
        return 0;   // a response, or nothing asked
    }

    // Walk the first question name: labels of 1..63 bytes, no compression in
    // queries, at most 255 bytes in total.
    size_t pos = DNS_HEADER_LEN;
    size_t nameLen = 0;
    while (true) {
        if (pos >= queryLen) {
            return 0;
        }
        const uint8_t label = query[pos];
        if (label == 0) {
            pos++;
            break;
        }
        if (label > 63) {
            return 0;
        }
        nameLen += (size_t)label + 1;
        if (nameLen > 255) {
            return 0;
        }
        pos += (size_t)label + 1;
    }
    if (pos + 4 > queryLen) {
        return 0;   // QTYPE and QCLASS missing
    }
    const uint16_t qtype = rd16(&query[pos]);
    const uint16_t qclass = rd16(&query[pos + 2]);
    const size_t questionEnd = pos + 4;

    const bool standardQuery = (flags & DNS_FLAG_OPCODE) == 0;
    const bool answerA = standardQuery && qtype == DNS_TYPE_A && qclass == DNS_CLASS_IN;
    const size_t respLen = questionEnd + (answerA ? 16 : 0);
    if (respLen > respCap) {
        return 0;
    }

    // Header and the first question; further questions and the EDNS record
    // of the query are not echoed.
    memcpy(resp, query, questionEnd);
    uint16_t rflags = (uint16_t)(DNS_FLAG_QR | DNS_FLAG_AA | (flags & (DNS_FLAG_OPCODE | DNS_FLAG_RD)));
    if (!standardQuery) {
        rflags |= DNS_RCODE_NOTIMP;
    }
    wr16(&resp[2], rflags);
    wr16(&resp[4], 1);                  // QDCOUNT
    wr16(&resp[6], answerA ? 1 : 0);    // ANCOUNT
    wr16(&resp[8], 0);                  // NSCOUNT
    wr16(&resp[10], 0);                 // ARCOUNT

    if (answerA) {
        uint8_t *a = &resp[questionEnd];
        wr16(&a[0], 0xC000 | DNS_HEADER_LEN);   // name: pointer to the question
        wr16(&a[2], DNS_TYPE_A);
        wr16(&a[4], DNS_CLASS_IN);
        wr16(&a[6], 0);
        wr16(&a[8], DNS_TTL_S);
        wr16(&a[10], 4);
        memcpy(&a[12], &ipv4, 4);               // already network byte order
    }
    return respLen;
}

#ifndef WEBCTRL_HOST_TEST

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "WEBDNS";
static uint32_t serverIp;

static void dnsTask(void *arg)
{
    uint8_t rx[DNS_MAX_PACKET];
    uint8_t tx[DNS_MAX_PACKET];

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind to port %d failed: errno %d", DNS_PORT, errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "captive DNS running");

    while (true) {
        struct sockaddr_in from;
        socklen_t fromLen = sizeof(from);
        int len = recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&from, &fromLen);
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        size_t out = dnsBuildResponse(rx, (size_t)len, serverIp, tx, sizeof(tx));
        if (out > 0) {
            sendto(sock, tx, out, 0, (struct sockaddr *)&from, fromLen);
        }
    }
}

bool dnsServerStart(uint32_t ipv4)
{
    serverIp = ipv4;
    return xTaskCreatePinnedToCore(dnsTask, WEBCTRL_DNS_TASK_NAME, WEBCTRL_DNS_TASK_STACKSIZE,
                                   NULL, WEBCTRL_DNS_TASK_PRI, NULL, NETWORK_TASK_CORE) == pdPASS;
}

#endif /* WEBCTRL_HOST_TEST */
