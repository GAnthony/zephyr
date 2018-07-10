/**
 * Copyright (c) 2018 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_WIFI_LEVEL
#define SYS_LOG_DOMAIN "dev/simplelink"
#include <logging/sys_log.h>

#include <zephyr.h>
/* Define sockaddr, etc, before simplelink.h, which redefines those symbols */
#include <net/net_offload.h>
#include <net/net_pkt.h>
#include <net/net_ip.h>

#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/source/driver.h>

#include "simplelink_offload.h"

struct select_server_data {
	/* Dedicated UDP socket used to unblock the sl_Select() call: */
	int signal_sock;

	/* Set of read file descriptors to await recv/accept actions: */
	SlFdSet_t rfds;
	int max_fd;

	/* Loopback address to signal select server : */
	SlSockAddr_t *loopback_addr;
	SlSockAddrIn_t loopback_addr_in;
	SlSocklen_t loopback_addrlen;

	/* Socket's bind address: */
	SlSockAddr_t *bind_addr;
	SlSockAddrIn_t bind_addr_in;
	SlSocklen_t bind_addrlen;

	/* Dedicated buffer for loopback message: */
	uint8_t rcv_buf[SIGNAL_MSG_SIZE];
};

static struct select_server_data server;

static bool net_context_alloc_flag;

struct socket_callbacks {
	struct net_context *context;

	/* net_context callbacks */
	union {
		net_context_recv_cb_t recv_cb;
		net_tcp_accept_cb_t accept_cb;
	};
	void *user_data;

	/* Receive packet and buf: */
	struct net_pkt *rx_pkt;
	struct net_buf *pkt_buf;
};

/* SimpleLink sockets are 0..SL_MAX_SOCKETS, so we can use sock fd as index: */
struct socket_callbacks socket_callbacks[SL_MAX_SOCKETS];

/* Helper functions: */
static inline bool is_listening(int sock)
{
	struct net_context *context;

	context = socket_callbacks[sock].context;

	return (net_context_get_state(context) == NET_CONTEXT_LISTENING);
}

static SlSockAddr_t *translate_z_to_sl_addrlen(socklen_t addrlen,
					       SlSockAddrIn_t *sl_addr_in,
					       SlSockAddrIn6_t *sl_addr_in6,
					       SlSocklen_t *sl_addrlen)
{
	SlSockAddr_t *sl_addr = NULL;

	if (addrlen == sizeof(struct sockaddr_in)) {
		*sl_addrlen = sizeof(SlSockAddrIn_t);
		sl_addr = (SlSockAddr_t *)sl_addr_in;
	} else if (addrlen == sizeof(struct sockaddr_in6)) {
		*sl_addrlen = sizeof(SlSockAddrIn6_t);
		sl_addr = (SlSockAddr_t *)sl_addr_in6;
	}

	return sl_addr;
}

static SlSockAddr_t *translate_z_to_sl_addrs(const struct sockaddr *addr,
					     socklen_t addrlen,
					     SlSockAddrIn_t *sl_addr_in,
					     SlSockAddrIn6_t *sl_addr_in6,
					     SlSocklen_t *sl_addrlen)
{
	SlSockAddr_t *sl_addr = NULL;

	if (addrlen == sizeof(struct sockaddr_in)) {
		struct sockaddr_in *z_sockaddr_in = (struct sockaddr_in *)addr;

		*sl_addrlen = sizeof(SlSockAddrIn_t);
		sl_addr_in->sin_family = AF_INET;
		sl_addr_in->sin_port = z_sockaddr_in->sin_port;
		sl_addr_in->sin_addr.S_un.S_addr =
			z_sockaddr_in->sin_addr.s_addr;

		sl_addr = (SlSockAddr_t *)sl_addr_in;
	} else if (addrlen == sizeof(struct sockaddr_in6)) {
		struct sockaddr_in6 *z_sockaddr_in6 =
			(struct sockaddr_in6 *)addr;

		*sl_addrlen = sizeof(SlSockAddrIn6_t);
		sl_addr_in6->sin6_family = AF_INET6;
		sl_addr_in6->sin6_port = z_sockaddr_in6->sin6_port;
		memcpy(sl_addr_in6->sin6_addr._S6_un._S6_u32,
		       z_sockaddr_in6->sin6_addr.s6_addr,
		       sizeof(sl_addr_in6->sin6_addr._S6_un._S6_u32));

		sl_addr = (SlSockAddr_t *)sl_addr_in6;
	}

	return sl_addr;
}

static void translate_sl_to_z_addr(SlSockAddr_t *sl_addr,
				   SlSocklen_t sl_addrlen,
				   struct sockaddr *addr,
				   socklen_t *addrlen)
{
	SlSockAddrIn_t *sl_addr_in;
	SlSockAddrIn6_t *sl_addr_in6;

	if (sl_addr->sa_family == SL_AF_INET) {
		if (sl_addrlen == (SlSocklen_t)sizeof(SlSockAddrIn_t)) {
			struct sockaddr_in *z_sockaddr_in =
				(struct sockaddr_in *)addr;

			sl_addr_in = (SlSockAddrIn_t *)sl_addr;
			z_sockaddr_in->sin_family = AF_INET;
			z_sockaddr_in->sin_port = sl_addr_in->sin_port;
			z_sockaddr_in->sin_addr.s_addr =
				sl_addr_in->sin_addr.S_un.S_addr;
			*addrlen = sizeof(struct sockaddr_in);
		} else {
			*addrlen = sl_addrlen;
		}
	} else if (sl_addr->sa_family == SL_AF_INET6) {
		if (sl_addrlen == sizeof(SlSockAddrIn6_t)) {
			struct sockaddr_in6 *z_sockaddr_in6 =
				(struct sockaddr_in6 *)addr;
			sl_addr_in6 = (SlSockAddrIn6_t *)sl_addr;

			z_sockaddr_in6->sin6_family = AF_INET6;
			z_sockaddr_in6->sin6_port = sl_addr_in6->sin6_port;
			z_sockaddr_in6->sin6_scope_id =
				(u8_t)sl_addr_in6->sin6_scope_id;
			memcpy(z_sockaddr_in6->sin6_addr.s6_addr,
			       sl_addr_in6->sin6_addr._S6_un._S6_u32,
			       sizeof(z_sockaddr_in6->sin6_addr.s6_addr));
			*addrlen = sizeof(struct sockaddr_in6);
		} else {
			*addrlen = sl_addrlen;
		}
	}
}

/* Handle accept on the signalled socket: */
static void handle_accept(int sock)
{
	struct socket_callbacks *scb, *new_scb;
	SlSockAddr_t *sl_addr;
	SlSockAddrIn_t sl_addr_in;
	SlSockAddrIn6_t sl_addr_in6;
	SlSocklen_t sl_addrlen;
	struct sockaddr addr;
	socklen_t addrlen;
	int new_sock;
	int retval;

	scb = &socket_callbacks[sock];

	if (scb->accept_cb == NULL) {
		/* Nothing to do: */
		return;
	}

	/* Setup sl_addr and sl_addrlen, assuming family == AF_INET: */
	/* TBD: Save family on _get(), so we don't have to assume: */
	addrlen = sizeof(struct sockaddr_in);
	sl_addr = translate_z_to_sl_addrlen(addrlen, &sl_addr_in, &sl_addr_in6,
					    &sl_addrlen);

	new_sock = sl_Accept(sock, sl_addr, &sl_addrlen);
	if (new_sock < 0) {
		SYS_LOG_ERR("sl_Accept error: %d", new_sock);
	} else {
		SYS_LOG_DBG("sl_Accept(%d,...): %d", sock, new_sock);

		/* Translate returned sl_addr into *addr and set *addrlen: */
		translate_sl_to_z_addr(sl_addr, sl_addrlen, &addr, &addrlen);

		/* Now, create a new net_context for the new socket callback: */
		/* TBD: This call re-offloads to simplelink_get, which creates
		 * another (unnecesary) socket ID!
		 * Ideally, need an internal _net_context_alloc() function which
		 * avoids calling back into the offload driver.
		 * Use a flag to check during simplelink_get for now:
		 */
		net_context_alloc_flag = 1;
		new_scb = &socket_callbacks[new_sock];
		retval = net_context_get(AF_INET, SOCK_STREAM, IPPROTO_TCP,
					 &new_scb->context);

		if (retval < 0) {
			SYS_LOG_ERR("Failed to allocate new context");
		} else {
			new_scb->context->offload_context = (void *)(int)new_sock;

			/* Invoke net_context callback: */
			/* Notes:
			 * net_tcp_accept_cb_t only understands 0 as success;
			 * also, we must set context->remote field here.
			 */
			memcpy(&new_scb->context->remote, &addr, addrlen);
			scb->accept_cb(new_scb->context, &addr, addrlen,
				      (new_sock > 0 ?  0 : new_sock),
				      scb->user_data);
		}
	}
}

/* Initialize a net_pkt for receive: */
static int prepare_pkt(struct socket_callbacks *scb)
{
	/* Get the frame from the buffer */
	scb->rx_pkt = net_pkt_get_reserve_rx(0, K_NO_WAIT);
	if (!scb->rx_pkt) {
		SYS_LOG_ERR("Could not allocate rx packet");
		return -1;
	}

	/* Reserve a data frag to receive the frame */
	scb->pkt_buf = net_pkt_get_frag(scb->rx_pkt, K_NO_WAIT);
	if (!scb->pkt_buf) {
		SYS_LOG_ERR("Could not allocate data frag");
		net_pkt_unref(scb->rx_pkt);
		return -1;
	}

	net_pkt_frag_insert(scb->rx_pkt, scb->pkt_buf);

	return 0;
}

/* Handle receive on the signalled socket: */
static void handle_recv(int sock)
{
	struct socket_callbacks *scb;
	int retval;
	short int size;

	scb = &socket_callbacks[sock];

	if (scb->recv_cb == NULL) {
		/* Nothing to do: */
		return;
	}

	/* Allocate packet, and data buf for recv: */
	retval = prepare_pkt(scb);
	if (retval) {
		SYS_LOG_ERR("Could not reserve packet buffer");
		(void)_SlDrvSetErrno(-ENOMEM);	/* Sets errno */
		return;
	}

	size = sl_Recv(sock, scb->pkt_buf->data,
		       CONFIG_WIFI_SIMPLELINK_MAX_PACKET_SIZE,
		       K_NO_WAIT);

	/* Complete net_pkt.... */
	if (size > 0) {
		net_buf_add(scb->pkt_buf, size);
		net_pkt_set_appdata(scb->rx_pkt, scb->pkt_buf->data);
		net_pkt_set_appdatalen(scb->rx_pkt, size);
	} else {
		/* error, or (size == 0: EOF), set packet to NULL: */
		net_pkt_unref(scb->rx_pkt);
		scb->rx_pkt = NULL;
	}

	/* ... and fire recv callback. */
	if (scb->recv_cb) {
		scb->recv_cb(scb->context, scb->rx_pkt,
			     (size > 0 ? 0 : size),
			     scb->user_data);
	}
}

/* Update server.max_fd to largest socket fd in set: */
static void update_server_max_fd()
{
	int i;

	server.max_fd = server.signal_sock;
	for (i = 0; i < SL_MAX_SOCKETS; i++) {
		if (SL_SOCKET_FD_ISSET(i, &server.rfds)) {
			server.max_fd = (i > server.max_fd ? i : server.max_fd);
		}
	}
}

/* Async socket server thread stack: */
static K_THREAD_STACK_DEFINE(select_server_stack, SELECT_SERVER_STACKSIZE);
static struct k_thread select_server_thread;

/*
 * Await received packets or connections,
 * on each socket notified,
 * lookup the callbacks associated with socket,
 * and call them with previously associated arguments.
 *
 * We also wait on a designated UDP signal_socket which
 * restarts the select() wait with a potentially new set of
 * file descriptors.
 */
void select_server(void *unused1, void *unused2, void *unused3)
{
	SlSocklen_t from_len = sizeof(SlSockAddrIn_t);
	int fd, retval;
	int rcv_len;
	SlFdSet_t rfds;
	SlSockAddr_t from;

	while (1) {
		/* Update array if read fds this for this select call: */
		rfds = server.rfds;

		/* Wait forever until any socket gets signalled: */
		retval = sl_Select(server.max_fd + 1, &rfds, NULL, NULL, NULL);
		if (retval > 0) {
			for (fd = 0; fd < (server.max_fd + 1); fd++) {
				if (SL_SOCKET_FD_ISSET(fd, &rfds)) {
					if (fd != server.signal_sock) {
						/* Call accept/recv callbacks */
						if (is_listening(fd)) {
							handle_accept(fd);
						} else {
							handle_recv(fd);
						}
					}
				}
			}
			if (SL_SOCKET_FD_ISSET(server.signal_sock, &rfds)) {
				/* signal_sock used to break out of select;
				 * just absorb signal message and continue:
				 */
				rcv_len = sl_RecvFrom(server.signal_sock, server.rcv_buf,
						      SIGNAL_MSG_SIZE, 0,
						      &from, &from_len);
				__ASSERT_NO_MSG(rcv_len == SIGNAL_MSG_SIZE);
			}
		} else {
			SYS_LOG_ERR("sl_Select failed: %d", retval);
		}
	}
}

/* Helper function, to restart wait on select: */
static int server_restart(void)
{
	int rc;

	rc = sl_SendTo(server.signal_sock, SIGNAL_MSG, SIGNAL_MSG_SIZE, 0,
		       server.loopback_addr, server.loopback_addrlen);

	return !(rc == SIGNAL_MSG_SIZE);
}

/* Initialize the select server: */
int _simplelink_offload_select_server_init(void)
{
	int rc = 0;
	struct sockaddr_in loopback_addr;
	struct sockaddr_in bind_addr;

	/* Set loopback address, discard port: */
	loopback_addr.sin_family = AF_INET;
	rc = net_addr_pton(AF_INET, LOOPBACK_ADDR, &loopback_addr.sin_addr);
	__ASSERT_NO_MSG(rc == 0);
	loopback_addr.sin_port = htons(DISCARD_PORT);

	/* Convert loopback addr to SimpleLink addr struct: */
	server.loopback_addr =
		translate_z_to_sl_addrs((struct sockaddr *)&loopback_addr,
					sizeof(loopback_addr),
					&server.loopback_addr_in,
					NULL,
					&server.loopback_addrlen);

	/* Bind to any addr: */
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_port = htons(DISCARD_PORT),
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	/* Convert bind addr to SimpleLink addr struct: */
	server.bind_addr =
		translate_z_to_sl_addrs((struct sockaddr *)&bind_addr,
					sizeof(bind_addr),
					&server.bind_addr_in,
					NULL,
					&server.bind_addrlen);

	/* Create a UDP socket to enable unblocking the server's select(): */
	rc  = sl_Socket(SL_AF_INET, SL_SOCK_DGRAM, SL_IPPROTO_UDP);
	if (rc < 0) {
		SYS_LOG_ERR("Unable to create poll server socket: %d", rc);
		server.signal_sock = INVALID_SOCK;
	} else {
		server.signal_sock = rc;
		rc = sl_Bind(server.signal_sock,
			     (SlSockAddr_t *)server.bind_addr,
			     server.bind_addrlen);
		if (rc < 0) {
			SYS_LOG_ERR("Cannot bind poll server socket: %d\n", rc);
		} else {
			/* Setup read fds for select: */
			SL_SOCKET_FD_ZERO(&server.rfds);
			SL_SOCKET_FD_SET(server.signal_sock, &server.rfds);
			server.max_fd = server.signal_sock;

			/* Start the select server: */
			(void)k_thread_create(&select_server_thread,
					      select_server_stack,
					      SELECT_SERVER_STACKSIZE,
					      select_server,
					      NULL, NULL, NULL,
					      SELECT_SERVER_PRIORITY,
					      0, K_NO_WAIT);
		}
	}

	return rc;
}

/* SimpleLink Net Offload functions:
 *
 * Arguments and return codes are mostly consistent with net_context APIs.
 * Error codes should be negative values of codes from error.h.
 * Non-negative return values are success.
 * _SlDrvSetErrno(retval) converts SL error codes to BSD errno.h codes.
 *
 * These functions assume that any usage of timeouts is consisent
 * with calls from the BSD sockets layer; i.e., the sockets are either
 * blocking (timeout = K_FOREVER) or non-blocking (timeout = K_NO_WAIT), and
 * the non-blocking option is set via a setsockopt() call.
 * Any other timeouts passed in via net_context are ignored, consistent with
 * BSD sockets (which don't take timeouts).
 *
 * TBD:
 *  - Add setsockopt() to net_context and net_offload functions.
 */

static int simplelink_get(sa_family_t family,
			  enum net_sock_type type,
			  enum net_ip_protocol proto,
			  struct net_context **context)
{
	short int sock;
	short int domain;

	/* This flag indicates we are being called via simplelink_accept, and
	 * therefore do not need to allocate another socket:
	 */
	if (net_context_alloc_flag) {
		net_context_alloc_flag = 0;
		return 0;
	}

	__ASSERT_NO_MSG(context);
	/* Map Zephyr socket.h AF_INET6 to SimpleLink's: */
	domain = (family == AF_INET6 ? SL_AF_INET6 : family);
	/* Ensure other Zephyr definitions match SimpleLink's: */
	__ASSERT(domain == SL_AF_INET || domain == SL_AF_INET6 \
		 || domain == SL_INADDR_ANY,		       \
		 "unrecognized family: %d", domain);
	__ASSERT(type == SL_SOCK_STREAM || type == SL_SOCK_DGRAM || \
		 type == SL_SOCK_RAW,				    \
		 "unrecognized type: %d", type);
	__ASSERT(proto == SL_IPPROTO_TCP || proto == SL_IPPROTO_UDP || \
		 proto == SL_IPPROTO_RAW,			       \
		 "unrecognized proto: %d", proto);

	/* Allocate socket and store in net_context struct: */
	sock = sl_Socket(domain, type, proto);
	if (sock >= 0) {
		(*context)->offload_context = (void *)(int)sock;
		socket_callbacks[sock].context = *context;
	} else {
		SYS_LOG_ERR("sl_Socket error: %d", sock);
		(*context)->offload_context = (void *)INVALID_SOCK;
	}

	SYS_LOG_DBG("sl_Socket(family:%d, type:%d, proto:%d): %d",
		    family, type, proto, sock);

	return _SlDrvSetErrno(sock);
}

static int simplelink_bind(struct net_context *context,
			   const struct sockaddr *addr,
			   socklen_t addrlen)
{
	short int sock;
	short int retval;
	SlSockAddr_t *sl_addr;
	SlSockAddrIn_t sl_addr_in;
	SlSockAddrIn6_t sl_addr_in6;
	SlSocklen_t sl_addrlen;

	__ASSERT_NO_MSG(context);
	sock = (short int)(int)context->offload_context;
	__ASSERT_NO_MSG(IS_SOCK_VALID(sock));
	__ASSERT_NO_MSG(addr != NULL);

	/* Translate to sl_Bind() parameters: */
	sl_addr = translate_z_to_sl_addrs(addr, addrlen, &sl_addr_in,
					  &sl_addr_in6, &sl_addrlen);
	if (sl_addr == NULL) {
		SYS_LOG_ERR("Invalid addrlen");
		retval = SL_RET_CODE_INVALID_INPUT;
		goto exit;
	}

	retval = sl_Bind(sock, sl_addr, sl_addrlen);
	if (retval < 0) {
		SYS_LOG_ERR("sl_Bind error: %d", retval);
	}

	SYS_LOG_DBG("sl_Bind(%d,...): %d", sock, retval);

exit:
	return _SlDrvSetErrno(retval);
}

static int simplelink_listen(struct net_context *context, int backlog)
{
	short int sock;
	short int retval;

	__ASSERT_NO_MSG(context);
	sock = (short int)(int)context->offload_context;
	__ASSERT_NO_MSG(IS_SOCK_VALID(sock));

	retval = sl_Listen(sock, backlog);
	if (retval < 0) {
		SYS_LOG_ERR("sl_Listen error: %d", retval);
	} else {
		/* Since offload bypasses tcp stack, need to set context state.
		 * Socket layer checks for this.
		 */
		if (net_context_get_ip_proto(context) == IPPROTO_TCP) {
			net_context_set_state(context, NET_CONTEXT_LISTENING);
		}
	}

	SYS_LOG_DBG("sl_Listen(%d, %d): %d", sock, backlog, retval);

	return _SlDrvSetErrno(retval);
}

static int simplelink_connect(struct net_context *context,
			      const struct sockaddr *addr,
			      socklen_t addrlen,
			      net_context_connect_cb_t cb,
			      s32_t timeout,
			      void *user_data)
{
	short int sock;
	short int retval;
	int status;
	SlSockAddr_t *sl_addr;
	SlSockAddrIn_t sl_addr_in;
	SlSockAddrIn6_t sl_addr_in6;
	SlSocklen_t sl_addrlen;

	__ASSERT_NO_MSG(context);
	sock = (short int)(int)context->offload_context;
	__ASSERT_NO_MSG(IS_SOCK_VALID(sock));
	__ASSERT_NO_MSG(addr);
	__ASSERT(timeout == K_FOREVER, "Can only handle timeout = K_FOREVER");

	/* Translate to sl_Connect() parameters: */
	sl_addr = translate_z_to_sl_addrs(addr, addrlen, &sl_addr_in,
					  &sl_addr_in6, &sl_addrlen);

	if (sl_addr == NULL) {
		SYS_LOG_ERR("Invalid addrlen");
		retval = SL_RET_CODE_INVALID_INPUT;
		goto exit;
	}

	retval = sl_Connect(sock, sl_addr, sl_addrlen);
	if (retval < 0) {
		SYS_LOG_ERR("sl_Connect error: %d", retval);
	}

	SYS_LOG_DBG("sl_Connect(%d,...): %d", sock, retval);

exit:
	status = _SlDrvSetErrno(retval);

	/* Invoke connect callback, if any: */
	if (cb != NULL) {
		cb(context, status, user_data);
	}

	return status;
}

static int simplelink_accept(struct net_context *context,
			     net_tcp_accept_cb_t cb,
			     s32_t timeout,
			     void *user_data)
{
	short int sock;
	int retval;

	__ASSERT_NO_MSG(context);
	sock = (short int)(int)context->offload_context;
	__ASSERT_NO_MSG(IS_SOCK_VALID(sock));
	__ASSERT(timeout == K_NO_WAIT, "Timeout must be K_NO_WAIT");
	__ASSERT_NO_MSG(socket_callbacks[sock].context == context);

	if ((socket_callbacks[sock].accept_cb == cb) &&
	    (socket_callbacks[sock].user_data == user_data)) {
		/* No need to update or restart select_server: */
		return 0;
	}

	socket_callbacks[sock].user_data = user_data;
	socket_callbacks[sock].accept_cb = cb;

	if (cb == NULL) {
		/* If callback is NULL, we no longer want select on this fd: */
		SL_SOCKET_FD_CLR(sock, &server.rfds);
	} else {
		/* Otherwise, we do want select on this fd: */
		SL_SOCKET_FD_SET(sock, &server.rfds);
	}
	update_server_max_fd();

	/* Restart select server: */
	retval = server_restart();

	return _SlDrvSetErrno(retval);
}

static int simplelink_sendto(struct net_pkt *pkt,
			     const struct sockaddr *dst_addr,
			     socklen_t addrlen,
			     net_context_send_cb_t cb,
			     s32_t timeout,
			     void *token,
			     void *user_data)
{
	short int retval = 0;
	SlSockAddr_t *sl_addr = NULL;
	short int sock;
	int status;
	struct net_buf *frag;
	struct net_context *context;
	SlSockAddrIn_t sl_addr_in;
	SlSockAddrIn6_t sl_addr_in6;
	SlSocklen_t sl_addrlen;

	__ASSERT_NO_MSG(pkt);
	context = pkt->context;
	__ASSERT_NO_MSG(context);
	sock = (short int)(int)context->offload_context;

	/* Translate dest address, if not NULL: */
	if (dst_addr != NULL) {
		sl_addr = translate_z_to_sl_addrs(dst_addr, addrlen, &sl_addr_in,
					  &sl_addr_in6, &sl_addrlen);
		if (sl_addr == NULL) {
			SYS_LOG_ERR("Invalid addrlen");
			retval = SL_RET_CODE_INVALID_INPUT;
			goto exit;
		}
	}

	/* Loop through fragments: */
	frag = pkt->frags;
	while (frag) {
		/* Verify frag data len within range: 1-1460 bytes allowed by SimpleLink */
		if (frag->len < 1 || frag->len > SIMPLELINK_MAX_TCP_IPV4_DATA_SIZE) {
			SYS_LOG_ERR("Invalid buffer length: must be 1 to 1460 bytes");
			retval = SL_ERROR_BSD_ENOBUFS;
			break;
		}

		if (dst_addr != NULL) {
			retval = sl_SendTo(sock, (void *)frag->data,
					   (u16_t)frag->len, 0,
					   sl_addr, sl_addrlen);
		} else {
			retval = sl_Send(sock, (void *)frag->data,
					 (u16_t)frag->len, 0);
		}
		SYS_LOG_DBG("sl_Send[To](%d): %d", sock, retval);

		frag = frag->frags;
	}

	net_pkt_unref(pkt);

exit:
	status = _SlDrvSetErrno(retval);

	/* Fire user's callback, if any. */
	if (cb) {
		cb(context, status, token, user_data);
	}

	return status;
}


static int simplelink_send(struct net_pkt *pkt,
	    net_context_send_cb_t cb,
	    s32_t timeout,
	    void *token,
	    void *user_data)
{
	return simplelink_sendto(pkt, NULL, 0, cb, timeout, token, user_data);
}

static int simplelink_recv(struct net_context *context,
			   net_context_recv_cb_t cb,
			   s32_t timeout,
			   void *user_data)
{
	int retval;
	short int sock;

	__ASSERT_NO_MSG(context);
	sock = (short int)(int)context->offload_context;
	__ASSERT_NO_MSG(IS_SOCK_VALID(sock));
	__ASSERT(timeout == K_NO_WAIT, "Timeout must be K_NO_WAIT");
	__ASSERT_NO_MSG(socket_callbacks[sock].context == context);

	if ((socket_callbacks[sock].recv_cb == cb) &&
	    (socket_callbacks[sock].user_data == user_data)) {
		/* No need to update or restart select_server: */
		return 0;
	}

	socket_callbacks[sock].user_data = user_data;
	socket_callbacks[sock].recv_cb = cb;

	if (cb == NULL) {
		/* If callback is NULL, we no longer want select on this fd: */
		SL_SOCKET_FD_CLR(sock, &server.rfds);
	} else {
		/* Otherwise, we do want select on this fd: */
		SL_SOCKET_FD_SET(sock, &server.rfds);
	}
	update_server_max_fd();

	/* Restart select server: */
	retval = server_restart();

	return _SlDrvSetErrno(retval);
}

int simplelink_put(struct net_context *context)
{
	short int sock;
	int retval;

	__ASSERT_NO_MSG(context);
	sock = (short int)(int)context->offload_context;

	retval = sl_Close(sock);
	socket_callbacks[sock].context = NULL;
	socket_callbacks[sock].recv_cb = NULL;
	socket_callbacks[sock].user_data = NULL;

	context->connect_cb = NULL;
	context->recv_cb = NULL;
	context->send_cb = NULL;
	net_context_unref(context);

	return _SlDrvSetErrno(retval);
}

struct net_offload simplelink_offload_fxns = {
	.get =	   simplelink_get,
	.bind =	   simplelink_bind,
	.listen =  simplelink_listen,
	.connect = simplelink_connect,
	.accept =  simplelink_accept,
	.send =	   simplelink_send,
	.sendto =  simplelink_sendto,
	.recv =	   simplelink_recv,
	.put =	   simplelink_put,
};
