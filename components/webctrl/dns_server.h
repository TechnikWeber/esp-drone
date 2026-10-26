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
 * dns_server.h - captive portal DNS: every name resolves to the drone.
 */

#ifndef WEBCTRL_DNS_SERVER_H_
#define WEBCTRL_DNS_SERVER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Start the DNS task. ipv4 is the drone's address in network byte order. */
bool dnsServerStart(uint32_t ipv4);

/*
 * Build the answer for one DNS query (exposed for host tests).
 * Returns the answer length, 0 if the query is ignored.
 */
size_t dnsBuildResponse(const uint8_t *query, size_t queryLen, uint32_t ipv4,
                        uint8_t *resp, size_t respCap);

#endif /* WEBCTRL_DNS_SERVER_H_ */
