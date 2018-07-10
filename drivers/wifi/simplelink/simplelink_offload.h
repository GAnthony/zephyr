/**
 * Copyright (c) 2018 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SIMPLELINK_OFFLOAD_H
#define __SIMPLELINK_OFFLOAD_H

#ifdef __cplusplus
extern "C" {
#endif

#define IS_SOCK_VALID(sd) (sd >=0)
#define INVALID_SOCK (-1)

/* Taken from SimpleLink User Guide, less than SimpleLink MTU size: */
#define SIMPLELINK_MAX_TCP_IPV4_DATA_SIZE 1460

/* Where to send our select_server restart signal: */
#define LOOPBACK_ADDR "127.0.0.1"
#define DISCARD_PORT 9

#define SIGNAL_MSG "POLL"
#define SIGNAL_MSG_SIZE 4

#define SELECT_SERVER_STACKSIZE 1024
#define SELECT_SERVER_PRIORITY K_PRIO_COOP(7)

extern int _simplelink_offload_select_server_init(void);
extern struct net_offload simplelink_offload_fxns;

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLELINK_OFFLOAD_H */
