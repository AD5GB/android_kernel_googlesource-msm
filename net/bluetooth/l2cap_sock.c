/*
   BlueZ - Bluetooth protocol stack for Linux
   Copyright (c) 2000-2001, 2011-2012 The Linux Foundation.  All rights reserved.
   Copyright (C) 2009-2010 Gustavo F. Padovan <gustavo@padovan.org>
   Copyright (C) 2010 Google Inc.

   Written 2000,2001 by Maxim Krasnyansky <maxk@qualcomm.com>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation;

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS.
   IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) AND AUTHOR(S) BE LIABLE FOR ANY
   CLAIM, OR ANY SPECIAL INDIRECT OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES
   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

   ALL LIABILITY, INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PATENTS,
   COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS, RELATING TO USE OF THIS
   SOFTWARE IS DISCLAIMED.
*/

/* Bluetooth L2CAP sockets. */

#include <linux/export.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/l2cap.h>
#include <net/bluetooth/smp.h>
#include <net/bluetooth/amp.h>

static struct bt_sock_list l2cap_sk_list = {
	.lock = __RW_LOCK_UNLOCKED(l2cap_sk_list.lock)
};

static const struct proto_ops l2cap_sock_ops;
static void l2cap_sock_init(struct sock *sk, struct sock *parent);
static struct sock *l2cap_sock_alloc(struct net *net, struct socket *sock,
				     int proto, gfp_t prio);

bool l2cap_is_socket(struct socket *sock)
{
	return sock && sock->ops == &l2cap_sock_ops;
}
EXPORT_SYMBOL(l2cap_is_socket);

static int l2cap_sock_bind(struct socket *sock, struct sockaddr *addr, int alen)
{
	struct sock *sk = sock->sk;
	struct sockaddr_l2 la;
	int len, err = 0;

	BT_DBG("sk %p", sk);

	if (!addr || addr->sa_family != AF_BLUETOOTH)
		return -EINVAL;

	memset(&la, 0, sizeof(la));
	len = min_t(unsigned int, sizeof(la), alen);
	memcpy(&la, addr, len);

	if (la.l2_cid && la.l2_psm)
		return -EINVAL;

	lock_sock(sk);

	if (sk->sk_state != BT_OPEN) {
		err = -EBADFD;
		goto done;
	}

	if (la.l2_psm) {
		__u16 psm = __le16_to_cpu(la.l2_psm);

		/* PSM must be odd and lsb of upper byte must be 0 */
		if ((psm & 0x0101) != 0x0001) {
			err = -EINVAL;
			goto done;
		}

		/* Restrict usage of well-known PSMs */
		if (psm < 0x1001 && !capable(CAP_NET_BIND_SERVICE)) {
			err = -EACCES;
			goto done;
		}
	}

	write_lock_bh(&l2cap_sk_list.lock);

	if (err < 0)
		goto done;

	if (__le16_to_cpu(la.l2_psm) == L2CAP_PSM_SDP ||
	    __le16_to_cpu(la.l2_psm) == L2CAP_PSM_RFCOMM)
		chan->sec_level = BT_SECURITY_SDP;

	if (la.l2_cid)
		l2cap_pi(sk)->scid = la.l2_cid;

	write_unlock_bh(&l2cap_sk_list.lock);

done:
	release_sock(sk);
	return err;
}

static int l2cap_sock_connect(struct socket *sock, struct sockaddr *addr,
			      int alen, int flags)
{
	struct sock *sk = sock->sk;
	struct sockaddr_l2 la;
	int len, err = 0;

	BT_DBG("sk %p type %d mode %d state %d", sk, sk->sk_type,
		l2cap_pi(sk)->mode, sk->sk_state);

	if (!addr || alen < sizeof(addr->sa_family) ||
		addr->sa_family != AF_BLUETOOTH)
		return -EINVAL;

	memset(&la, 0, sizeof(la));
	len = min_t(unsigned int, sizeof(la), alen);
	memcpy(&la, addr, len);

	if (la.l2_cid && la.l2_psm)
		return -EINVAL;

	err = l2cap_chan_connect(chan, la.l2_psm, __le16_to_cpu(la.l2_cid),
				 &la.l2_bdaddr, la.l2_bdaddr_type);
	if (err)
		return err;

	lock_sock(sk);

	if ((sk->sk_type == SOCK_SEQPACKET || sk->sk_type == SOCK_STREAM)
		&& !(la.l2_psm || la.l2_cid || l2cap_pi(sk)->fixed_channel)) {
		err = -EINVAL;
		goto done;
	}

	switch (l2cap_pi(sk)->mode) {
	case L2CAP_MODE_BASIC:
		break;
	case L2CAP_MODE_ERTM:
	case L2CAP_MODE_STREAMING:
		if (!disable_ertm)
			break;
		/* fall through */
	default:
		err = -ENOTSUPP;
		goto done;
	}

	switch (sk->sk_state) {
	case BT_CONNECT:
	case BT_CONNECT2:
	case BT_CONFIG:
		/* Already connecting */
		goto wait;

	case BT_CONNECTED:
		/* Already connected */
		err = -EISCONN;
		goto done;

	case BT_OPEN:
	case BT_BOUND:
		/* Can connect */
		break;

	default:
		err = -EBADFD;
		goto done;
	}

	/* PSM must be odd and lsb of upper byte must be 0 */
	if ((__le16_to_cpu(la.l2_psm) & 0x0101) != 0x0001 &&
		!l2cap_pi(sk)->fixed_channel &&
				sk->sk_type != SOCK_RAW && !la.l2_cid) {
		BT_DBG("Bad PSM 0x%x", (int)__le16_to_cpu(la.l2_psm));
		err = -EINVAL;
		goto done;
	}

	/* Set destination address and psm */
	bacpy(&bt_sk(sk)->dst, &la.l2_bdaddr);
	l2cap_pi(sk)->psm = la.l2_psm;
	l2cap_pi(sk)->dcid = la.l2_cid;

	err = l2cap_do_connect(sk);
	if (err)
		goto done;

wait:
	err = bt_sock_wait_state(sk, BT_CONNECTED,
				 sock_sndtimeo(sk, flags & O_NONBLOCK));

	release_sock(sk);
	return err;
}

static int l2cap_sock_listen(struct socket *sock, int backlog)
{
	struct sock *sk = sock->sk;
	int err = 0;

	BT_DBG("sk %p backlog %d", sk, backlog);

	lock_sock(sk);

	if (sk->sk_state != BT_BOUND) {
		err = -EBADFD;
		goto done;
	}

	if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM) {
		err = -EINVAL;
		goto done;
	}

	switch (chan->mode) {
	case L2CAP_MODE_BASIC:
		break;
	case L2CAP_MODE_ERTM:
	case L2CAP_MODE_STREAMING:
		if (!disable_ertm)
			break;
		/* fall through */
	default:
		err = -ENOTSUPP;
		goto done;
	}

	if (!l2cap_pi(sk)->psm && !l2cap_pi(sk)->scid) {
		bdaddr_t *src = &bt_sk(sk)->src;
		u16 psm;

		err = -EINVAL;

		write_lock_bh(&l2cap_sk_list.lock);

		for (psm = 0x1001; psm < 0x1100; psm += 2)
			if (!__l2cap_get_sock_by_addr(cpu_to_le16(psm), src)) {
				l2cap_pi(sk)->psm   = cpu_to_le16(psm);
				l2cap_pi(sk)->sport = cpu_to_le16(psm);
				err = 0;
				break;
			}

		write_unlock_bh(&l2cap_sk_list.lock);

		if (err < 0)
			goto done;
	}

	sk->sk_max_ack_backlog = backlog;
	sk->sk_ack_backlog = 0;
	sk->sk_state = BT_LISTEN;

done:
	release_sock(sk);
	return err;
}

static int l2cap_sock_accept(struct socket *sock, struct socket *newsock,
			     int flags)
{
	DECLARE_WAITQUEUE(wait, current);
	struct sock *sk = sock->sk, *nsk;
	long timeo;
	int err = 0;

	lock_sock_nested(sk, SINGLE_DEPTH_NESTING);

	if (sk->sk_state != BT_LISTEN) {
		err = -EBADFD;
		goto done;
	}

	timeo = sock_rcvtimeo(sk, flags & O_NONBLOCK);

	BT_DBG("sk %p timeo %ld", sk, timeo);

	/* Wait for an incoming connection. (wake-one). */
	add_wait_queue_exclusive(sk_sleep(sk), &wait);
	while (!(nsk = bt_accept_dequeue(sk, newsock))) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!timeo) {
			err = -EAGAIN;
			break;
		}

		release_sock(sk);
		timeo = schedule_timeout(timeo);
		lock_sock_nested(sk, SINGLE_DEPTH_NESTING);

		if (sk->sk_state != BT_LISTEN) {
			err = -EBADFD;
			break;
		}

		if (signal_pending(current)) {
			err = sock_intr_errno(timeo);
			break;
		}
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(sk_sleep(sk), &wait);

	if (err)
		goto done;

	newsock->state = SS_CONNECTED;

	BT_DBG("new socket %p", nsk);

done:
	release_sock(sk);
	return err;
}

static int l2cap_sock_getname(struct socket *sock, struct sockaddr *addr,
			      int *len, int peer)
{
	struct sockaddr_l2 *la = (struct sockaddr_l2 *) addr;
	struct sock *sk = sock->sk;

	BT_DBG("sock %p, sk %p", sock, sk);

	memset(la, 0, sizeof(struct sockaddr_l2));
	addr->sa_family = AF_BLUETOOTH;
	*len = sizeof(struct sockaddr_l2);

	if (peer) {
		la->l2_psm = l2cap_pi(sk)->psm;
		bacpy(&la->l2_bdaddr, &bt_sk(sk)->dst);
		la->l2_cid = cpu_to_le16(l2cap_pi(sk)->dcid);
	} else {
		la->l2_psm = l2cap_pi(sk)->sport;
		bacpy(&la->l2_bdaddr, &bt_sk(sk)->src);
		la->l2_cid = cpu_to_le16(l2cap_pi(sk)->scid);
	}

	return 0;
}

static int l2cap_sock_getsockopt_old(struct socket *sock, int optname,
				     char __user *optval, int __user *optlen)
{
	struct sock *sk = sock->sk;
	struct l2cap_options opts;
	struct l2cap_conninfo cinfo;
	int len, err = 0;
	u32 opt;

	BT_DBG("sk %p", sk);

	if (get_user(len, optlen))
		return -EFAULT;

	lock_sock(sk);

	switch (optname) {
	case L2CAP_OPTIONS:
		memset(&opts, 0, sizeof(opts));
		opts.imtu     = l2cap_pi(sk)->imtu;
		opts.omtu     = l2cap_pi(sk)->omtu;
		opts.flush_to = l2cap_pi(sk)->flush_to;
		opts.mode     = l2cap_pi(sk)->mode;
		opts.fcs      = l2cap_pi(sk)->fcs;
		opts.max_tx   = l2cap_pi(sk)->max_tx;
		opts.txwin_size = l2cap_pi(sk)->tx_win;

		len = min_t(unsigned int, len, sizeof(opts));
		if (copy_to_user(optval, (char *) &opts, len))
			err = -EFAULT;

		break;

	case L2CAP_LM:
		switch (l2cap_pi(sk)->sec_level) {
		case BT_SECURITY_LOW:
			opt = L2CAP_LM_AUTH;
			break;
		case BT_SECURITY_MEDIUM:
			opt = L2CAP_LM_AUTH | L2CAP_LM_ENCRYPT;
			break;
		case BT_SECURITY_HIGH:
			opt = L2CAP_LM_AUTH | L2CAP_LM_ENCRYPT |
			      L2CAP_LM_SECURE;
			break;
		default:
			opt = 0;
			break;
		}

		if (l2cap_pi(sk)->role_switch)
			opt |= L2CAP_LM_MASTER;

		if (l2cap_pi(sk)->force_reliable)
			opt |= L2CAP_LM_RELIABLE;

		if (l2cap_pi(sk)->flushable)
			opt |= L2CAP_LM_FLUSHABLE;

		if (put_user(opt, (u32 __user *) optval))
			err = -EFAULT;
		break;

	case L2CAP_CONNINFO:
		if (sk->sk_state != BT_CONNECTED &&
		    !(sk->sk_state == BT_CONNECT2 &&
		      test_bit(BT_SK_DEFER_SETUP, &bt_sk(sk)->flags))) {
			err = -ENOTCONN;
			break;
		}

		cinfo.hci_handle = l2cap_pi(sk)->conn->hcon->handle;
		memcpy(cinfo.dev_class, l2cap_pi(sk)->conn->hcon->dev_class, 3);

		len = min_t(unsigned int, len, sizeof(cinfo));
		if (copy_to_user(optval, (char *) &cinfo, len))
			err = -EFAULT;

		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static int l2cap_sock_getsockopt(struct socket *sock, int level, int optname,
				 char __user *optval, int __user *optlen)
{
	struct sock *sk = sock->sk;
	struct bt_security sec;
	struct bt_power pwr;
	int len, err = 0;

	BT_DBG("sk %p", sk);

	if (level == SOL_L2CAP)
		return l2cap_sock_getsockopt_old(sock, optname, optval, optlen);

	if (level != SOL_BLUETOOTH)
		return -ENOPROTOOPT;

	if (get_user(len, optlen))
		return -EFAULT;

	lock_sock(sk);

	switch (optname) {
	case BT_SECURITY:
		if (chan->chan_type != L2CAP_CHAN_CONN_ORIENTED &&
		    chan->chan_type != L2CAP_CHAN_RAW) {
			err = -EINVAL;
			break;
		}

		memset(&sec, 0, sizeof(sec));
		if (chan->conn) {
			sec.level = chan->conn->hcon->sec_level;

			if (sk->sk_state == BT_CONNECTED)
				sec.key_size = chan->conn->hcon->enc_key_size;
		} else {
			sec.level = chan->sec_level;
		}

		len = min_t(unsigned int, len, sizeof(sec));
		if (copy_to_user(optval, (char *) &sec, len))
			err = -EFAULT;

		break;

	case BT_DEFER_SETUP:
		if (sk->sk_state != BT_BOUND && sk->sk_state != BT_LISTEN) {
			err = -EINVAL;
			break;
		}

		if (put_user(test_bit(BT_SK_DEFER_SETUP, &bt_sk(sk)->flags),
			     (u32 __user *) optval))
			err = -EFAULT;

		break;

	case BT_FLUSHABLE:
		if (put_user(test_bit(FLAG_FLUSHABLE, &chan->flags),
			     (u32 __user *) optval))
			err = -EFAULT;

		break;

	case BT_POWER:
		if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM
		    && sk->sk_type != SOCK_RAW) {
			err = -EINVAL;
			break;
		}

		pwr.force_active = l2cap_pi(sk)->force_active;

		len = min_t(unsigned int, len, sizeof(pwr));
		if (copy_to_user(optval, (char *) &pwr, len))
			err = -EFAULT;

		break;

	case BT_AMP_POLICY:
		if (put_user(l2cap_pi(sk)->amp_pref, (u32 __user *) optval))
			err = -EFAULT;
		break;

	case BT_LE_PARAMS:
		if (l2cap_pi(sk)->scid != L2CAP_CID_LE_DATA) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(optval, (char *) &bt_sk(sk)->le_params,
						sizeof(bt_sk(sk)->le_params)))
			err = -EFAULT;
		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static bool l2cap_valid_mtu(struct l2cap_chan *chan, u16 mtu)
{
	switch (chan->scid) {
	case L2CAP_CID_LE_DATA:
		if (mtu < L2CAP_LE_MIN_MTU)
			return false;
		break;

	default:
		if (mtu < L2CAP_DEFAULT_MIN_MTU)
			return false;
	}

	return true;
}

static int l2cap_sock_setsockopt_old(struct socket *sock, int optname,
				     char __user *optval, unsigned int optlen)
{
	struct sock *sk = sock->sk;
	struct l2cap_options opts;
	int len, le_sock, err = 0;
	u32 opt;

	BT_DBG("sk %p", sk);

	lock_sock(sk);

	le_sock = l2cap_pi(sk)->scid == L2CAP_CID_LE_DATA;

	switch (optname) {
	case L2CAP_OPTIONS:
		if (sk->sk_state == BT_CONNECTED && !le_sock) {
			err = -EINVAL;
			break;
		}

		opts.imtu     = l2cap_pi(sk)->imtu;
		opts.omtu     = l2cap_pi(sk)->omtu;
		opts.flush_to = l2cap_pi(sk)->flush_to;
		opts.mode     = l2cap_pi(sk)->mode;
		opts.fcs      = l2cap_pi(sk)->fcs;
		opts.max_tx   = l2cap_pi(sk)->max_tx;
		opts.txwin_size = l2cap_pi(sk)->tx_win;

		len = min_t(unsigned int, sizeof(opts), optlen);
		if (copy_from_user((char *) &opts, optval, len)) {
			err = -EFAULT;
			break;
		}

		if ((opts.imtu || opts.omtu) && le_sock &&
				(sk->sk_state == BT_CONNECTED)) {
			if (opts.imtu >= L2CAP_LE_DEFAULT_MTU)
				l2cap_pi(sk)->imtu = opts.imtu;
			if (opts.omtu >= L2CAP_LE_DEFAULT_MTU)
				l2cap_pi(sk)->omtu = opts.omtu;
			if (opts.imtu < L2CAP_LE_DEFAULT_MTU ||
					opts.omtu < L2CAP_LE_DEFAULT_MTU)
				err = -EINVAL;
			break;
		}

		if (opts.txwin_size < 1 ||
			opts.txwin_size > L2CAP_TX_WIN_MAX_EXTENDED) {
			err = -EINVAL;
			break;
		}

		if (!l2cap_valid_mtu(chan, opts.imtu)) {
			err = -EINVAL;
			break;
		}

		chan->mode = opts.mode;
		switch (chan->mode) {
		case L2CAP_MODE_BASIC:
			l2cap_pi(sk)->conf_state &= ~L2CAP_CONF_STATE2_DEVICE;
			break;
		case L2CAP_MODE_STREAMING:
			if (!disable_ertm) {
				/* No fallback to ERTM or Basic mode */
				l2cap_pi(sk)->conf_state |=
						L2CAP_CONF_STATE2_DEVICE;
				break;
			}
			err = -EINVAL;
			break;
		case L2CAP_MODE_ERTM:
			if (!disable_ertm)
				break;
			/* fall through */
		default:
			err = -EINVAL;
			break;
		}

		chan->imtu = opts.imtu;
		chan->omtu = opts.omtu;
		chan->fcs  = opts.fcs;
		chan->max_tx = opts.max_tx;
		chan->tx_win = opts.txwin_size;
		chan->flush_to = opts.flush_to;
		break;

	case L2CAP_LM:
		if (get_user(opt, (u32 __user *) optval)) {
			err = -EFAULT;
			break;
		}

		if (opt & L2CAP_LM_AUTH)
			l2cap_pi(sk)->sec_level = BT_SECURITY_LOW;
		if (opt & L2CAP_LM_ENCRYPT)
			l2cap_pi(sk)->sec_level = BT_SECURITY_MEDIUM;
		if (opt & L2CAP_LM_SECURE)
			l2cap_pi(sk)->sec_level = BT_SECURITY_HIGH;

		l2cap_pi(sk)->role_switch    = (opt & L2CAP_LM_MASTER);
		l2cap_pi(sk)->force_reliable = (opt & L2CAP_LM_RELIABLE);
		l2cap_pi(sk)->flushable = (opt & L2CAP_LM_FLUSHABLE);
		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static int l2cap_sock_setsockopt(struct socket *sock, int level, int optname,
				 char __user *optval, unsigned int optlen)
{
	struct sock *sk = sock->sk;
	struct bt_security sec;
	struct bt_power pwr;
	struct bt_le_params le_params;
	struct l2cap_conn *conn;
	int len, err = 0;
	u32 opt;

	BT_DBG("sk %p", sk);

	if (level == SOL_L2CAP)
		return l2cap_sock_setsockopt_old(sock, optname, optval, optlen);

	if (level != SOL_BLUETOOTH)
		return -ENOPROTOOPT;

	lock_sock(sk);

	switch (optname) {
	case BT_SECURITY:
		if (chan->chan_type != L2CAP_CHAN_CONN_ORIENTED &&
		    chan->chan_type != L2CAP_CHAN_RAW) {
			err = -EINVAL;
			break;
		}

		sec.level = BT_SECURITY_LOW;

		len = min_t(unsigned int, sizeof(sec), optlen);
		if (copy_from_user((char *) &sec, optval, len)) {
			err = -EFAULT;
			break;
		}

		if (sec.level < BT_SECURITY_LOW ||
		    sec.level > BT_SECURITY_HIGH) {
			err = -EINVAL;
			break;
		}

		l2cap_pi(sk)->sec_level = sec.level;

		conn = l2cap_pi(sk)->conn;
		if (conn && l2cap_pi(sk)->scid == L2CAP_CID_LE_DATA) {
			if (!conn->hcon->out) {
				err = -EINVAL;
				break;
			}

			if (smp_conn_security(conn->hcon, sec.level))
				break;

			err = 0;
			sk->sk_state = BT_CONFIG;
			chan->state = BT_CONFIG;

		/* or for ACL link */
		} else if ((sk->sk_state == BT_CONNECT2 &&
			    test_bit(BT_SK_DEFER_SETUP, &bt_sk(sk)->flags)) ||
			   sk->sk_state == BT_CONNECTED) {
			if (!l2cap_chan_check_security(chan))
				set_bit(BT_SK_SUSPEND, &bt_sk(sk)->flags);
			else
				sk->sk_state_change(sk);
		} else {
			err = -EINVAL;
		}
		break;

	case BT_DEFER_SETUP:
		if (sk->sk_state != BT_BOUND && sk->sk_state != BT_LISTEN) {
			err = -EINVAL;
			break;
		}

		if (get_user(opt, (u32 __user *) optval)) {
			err = -EFAULT;
			break;
		}

		if (opt)
			set_bit(BT_SK_DEFER_SETUP, &bt_sk(sk)->flags);
		else
			clear_bit(BT_SK_DEFER_SETUP, &bt_sk(sk)->flags);
		break;

	case BT_POWER:
		if (sk->sk_type != SOCK_SEQPACKET && sk->sk_type != SOCK_STREAM
				&& sk->sk_type != SOCK_RAW) {
			err = -EINVAL;
			break;
		}

		pwr.force_active = 1;

		len = min_t(unsigned int, sizeof(pwr), optlen);
		if (copy_from_user((char *) &pwr, optval, len)) {
			err = -EFAULT;
			break;
		}
		l2cap_pi(sk)->force_active = pwr.force_active;
		break;

	case BT_AMP_POLICY:
		if (get_user(opt, (u32 __user *) optval)) {
			err = -EFAULT;
			break;
		}

		if (opt)
			set_bit(FLAG_FLUSHABLE, &chan->flags);
		else
			clear_bit(FLAG_FLUSHABLE, &chan->flags);
		break;

	case BT_POWER:
		if (chan->chan_type != L2CAP_CHAN_CONN_ORIENTED &&
		    chan->chan_type != L2CAP_CHAN_RAW) {
			err = -EINVAL;
			break;
		}

		l2cap_pi(sk)->amp_pref = (u8) opt;
		BT_DBG("BT_AMP_POLICY now %d", opt);

		if ((sk->sk_state == BT_CONNECTED) &&
			(l2cap_pi(sk)->amp_move_role == L2CAP_AMP_MOVE_NONE))
			l2cap_amp_move_init(sk);

		break;

	case BT_FLUSHABLE:
		if (get_user(opt, (u32 __user *) optval)) {
			err = -EFAULT;
			break;
		}
		l2cap_pi(sk)->flushable = opt;

		break;

	case BT_LE_PARAMS:
		if (l2cap_pi(sk)->scid != L2CAP_CID_LE_DATA) {
			err = -EINVAL;
			break;
		}

		if (copy_from_user((char *) &le_params, optval,
					sizeof(struct bt_le_params))) {
			err = -EFAULT;
			break;
		}

		conn = l2cap_pi(sk)->conn;
		if (!conn || !conn->hcon ||
				l2cap_pi(sk)->scid != L2CAP_CID_LE_DATA) {
			memcpy(&bt_sk(sk)->le_params, &le_params,
							sizeof(le_params));
			break;
		}

		if (chan->mode != L2CAP_MODE_ERTM &&
		    chan->mode != L2CAP_MODE_STREAMING) {
			err = -EOPNOTSUPP;
			break;
		}

		chan->chan_policy = (u8) opt;

		if (sk->sk_state == BT_CONNECTED &&
		    chan->move_role == L2CAP_MOVE_ROLE_NONE)
			l2cap_move_start(chan);

		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);
	return err;
}

static int l2cap_sock_sendmsg(struct kiocb *iocb, struct socket *sock,
			      struct msghdr *msg, size_t len)
{
	struct sock *sk = sock->sk;
	struct l2cap_pinfo *pi = l2cap_pi(sk);
	struct sk_buff *skb;
	struct sk_buff_head seg_queue;
	int err;
	u8 amp_id;

	BT_DBG("sock %p, sk %p", sock, sk);

	err = sock_error(sk);
	if (err)
		return err;

	if (msg->msg_flags & MSG_OOB)
		return -EOPNOTSUPP;

	if (sk->sk_state != BT_CONNECTED)
		return -ENOTCONN;

	l2cap_chan_lock(chan);
	err = l2cap_chan_send(chan, msg, len, sk->sk_priority);
	l2cap_chan_unlock(chan);

	return err;
}

static int l2cap_sock_recvmsg(struct kiocb *iocb, struct socket *sock,
			      struct msghdr *msg, size_t len, int flags)
{
	struct sock *sk = sock->sk;
	int err;

	lock_sock(sk);

	if (sk->sk_state == BT_CONNECT2 && test_bit(BT_SK_DEFER_SETUP,
						    &bt_sk(sk)->flags)) {
		sk->sk_state = BT_CONFIG;

		rsp.scid   = cpu_to_le16(l2cap_pi(sk)->dcid);
		rsp.dcid   = cpu_to_le16(l2cap_pi(sk)->scid);
		rsp.result = cpu_to_le16(L2CAP_CR_SUCCESS);
		rsp.status = cpu_to_le16(L2CAP_CS_NO_INFO);
		l2cap_send_cmd(l2cap_pi(sk)->conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_RSP, sizeof(rsp), &rsp);

		if (l2cap_pi(sk)->conf_state & L2CAP_CONF_REQ_SENT) {
			release_sock(sk);
			return 0;
		}

		l2cap_pi(sk)->conf_state |= L2CAP_CONF_REQ_SENT;
		l2cap_send_cmd(conn, l2cap_get_ident(conn), L2CAP_CONF_REQ,
				l2cap_build_conf_req(sk, buf), buf);
		l2cap_pi(sk)->num_conf_req++;

		release_sock(sk);
		return 0;
	}

	release_sock(sk);

	if (sock->type == SOCK_STREAM)
		err = bt_sock_stream_recvmsg(iocb, sock, msg, len, flags);
	else
		err = bt_sock_recvmsg(iocb, sock, msg, len, flags);

	if (err >= 0)
		l2cap_ertm_recv_done(sk);

	return err;
}

/* Kill socket (only if zapped and orphan)
 * Must be called on unlocked socket.
 */
void l2cap_sock_kill(struct sock *sk)
{
	if (!sock_flag(sk, SOCK_ZAPPED) || sk->sk_socket)
		return;

	BT_DBG("sk %p state %d", sk, sk->sk_state);

	/* Kill poor orphan */

	l2cap_chan_put(l2cap_pi(sk)->chan);
	sock_set_flag(sk, SOCK_DEAD);
	sock_put(sk);
}

/* Must be called on unlocked socket. */
static void l2cap_sock_close(struct sock *sk)
{
	l2cap_sock_clear_timer(sk);
	lock_sock(sk);
	__l2cap_sock_close(sk, ECONNRESET);
	release_sock(sk);
	l2cap_sock_kill(sk);
}

static void l2cap_sock_cleanup_listen(struct sock *parent)
{
	struct sock *sk;

	BT_DBG("parent %p", parent);

	/* Close not yet accepted channels */
	while ((sk = bt_accept_dequeue(parent, NULL)))
		l2cap_sock_close(sk);

	parent->sk_state = BT_CLOSED;
	sock_set_flag(parent, SOCK_ZAPPED);
}

void __l2cap_sock_close(struct sock *sk, int reason)
{
	struct l2cap_conn *conn = l2cap_pi(sk)->conn;

	BT_DBG("sk %p state %d socket %p", sk, sk->sk_state, sk->sk_socket);

	switch (sk->sk_state) {
	case BT_LISTEN:
		l2cap_sock_cleanup_listen(sk);
		break;

	case BT_CONNECTED:
	case BT_CONFIG:
		if ((sk->sk_type == SOCK_SEQPACKET ||
					sk->sk_type == SOCK_STREAM) &&
					conn->hcon->type == ACL_LINK) {
			l2cap_sock_set_timer(sk, sk->sk_sndtimeo);
			l2cap_send_disconn_req(conn, sk, reason);
		} else
			l2cap_chan_del(sk, reason);
		break;

	case BT_CONNECT2:
		if ((sk->sk_type == SOCK_SEQPACKET ||
					sk->sk_type == SOCK_STREAM) &&
					conn->hcon->type == ACL_LINK) {
			struct l2cap_conn_rsp rsp;
			__u16 result;

			if (bt_sk(sk)->defer_setup)
				result = L2CAP_CR_SEC_BLOCK;
			else
				result = L2CAP_CR_BAD_PSM;
			sk->sk_state = BT_DISCONN;

			rsp.scid   = cpu_to_le16(l2cap_pi(sk)->dcid);
			rsp.dcid   = cpu_to_le16(l2cap_pi(sk)->scid);
			rsp.result = cpu_to_le16(result);
			rsp.status = cpu_to_le16(L2CAP_CS_NO_INFO);
			l2cap_send_cmd(conn, l2cap_pi(sk)->ident,
					L2CAP_CONN_RSP, sizeof(rsp), &rsp);
		}

		l2cap_chan_del(sk, reason);
		break;

	case BT_CONNECT:
	case BT_DISCONN:
		l2cap_chan_del(sk, reason);
		break;

	default:
		sock_set_flag(sk, SOCK_ZAPPED);
		break;
	}
}

static int l2cap_sock_shutdown(struct socket *sock, int how)
{
	struct sock *sk = sock->sk;
	int err = 0;

	BT_DBG("sock %p, sk %p", sock, sk);

	if (!sk)
		return 0;

	lock_sock(sk);
	if (!sk->sk_shutdown) {

		if (l2cap_pi(sk)->mode == L2CAP_MODE_ERTM) {
			err = __l2cap_wait_ack(sk);
			l2cap_ertm_shutdown(sk);
		}

		sk->sk_shutdown = SHUTDOWN_MASK;
		l2cap_sock_clear_timer(sk);
		__l2cap_sock_close(sk, 0);

		if (sock_flag(sk, SOCK_LINGER) && sk->sk_lingertime)
			err = bt_sock_wait_state(sk, BT_CLOSED,
						 sk->sk_lingertime);
	}

	if (!err && sk->sk_err)
		err = -sk->sk_err;

	release_sock(sk);
	return err;
}

static int l2cap_sock_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct sock *sk2 = NULL;
	int err;

	BT_DBG("sock %p, sk %p", sock, sk);

	if (!sk)
		return 0;

	bt_sock_unlink(&l2cap_sk_list, sk);

	err = l2cap_sock_shutdown(sock, 2);

	sock_orphan(sk);
	l2cap_sock_kill(sk);
	return err;
}

static void l2cap_sock_cleanup_listen(struct sock *parent)
{
	struct sock *sk;

	BT_DBG("parent %p", parent);

	/* Close not yet accepted channels */
	while ((sk = bt_accept_dequeue(parent, NULL))) {
		struct l2cap_chan *chan = l2cap_pi(sk)->chan;

		l2cap_chan_lock(chan);
		__clear_chan_timer(chan);
		l2cap_chan_close(chan, ECONNRESET);
		l2cap_chan_unlock(chan);

		l2cap_sock_kill(sk);
	}
}

static struct l2cap_chan *l2cap_sock_new_connection_cb(struct l2cap_chan *chan)
{
	struct sock *sk, *parent = chan->data;

	/* Check for backlog size */
	if (sk_acceptq_is_full(parent)) {
		BT_DBG("backlog full %d", parent->sk_ack_backlog);
		return NULL;
	}

	sk = l2cap_sock_alloc(sock_net(parent), NULL, BTPROTO_L2CAP,
			      GFP_ATOMIC);
	if (!sk)
		return NULL;

	bt_sock_reclassify_lock(sk, BTPROTO_L2CAP);

	l2cap_sock_init(sk, parent);

	bt_accept_enqueue(parent, sk);

	return l2cap_pi(sk)->chan;
}

static int l2cap_sock_recv_cb(struct l2cap_chan *chan, struct sk_buff *skb)
{
	int err;
	struct sock *sk = chan->data;
	struct l2cap_pinfo *pi = l2cap_pi(sk);

	lock_sock(sk);

	if (pi->rx_busy_skb) {
		err = -ENOMEM;
		goto done;
	}

	err = sock_queue_rcv_skb(sk, skb);

	/* For ERTM, handle one skb that doesn't fit into the recv
	 * buffer.  This is important to do because the data frames
	 * have already been acked, so the skb cannot be discarded.
	 *
	 * Notify the l2cap core that the buffer is full, so the
	 * LOCAL_BUSY state is entered and no more frames are
	 * acked and reassembled until there is buffer space
	 * available.
	 */
	if (err < 0 && pi->chan->mode == L2CAP_MODE_ERTM) {
		pi->rx_busy_skb = skb;
		l2cap_chan_busy(pi->chan, 1);
		err = 0;
	}

done:
	release_sock(sk);

	return err;
}

static void l2cap_sock_close_cb(struct l2cap_chan *chan)
{
	struct sock *sk = chan->data;

	l2cap_sock_kill(sk);
}

static void l2cap_sock_teardown_cb(struct l2cap_chan *chan, int err)
{
	struct sock *sk = chan->data;
	struct sock *parent;

	lock_sock(sk);

	parent = bt_sk(sk)->parent;

	sock_set_flag(sk, SOCK_ZAPPED);

	switch (chan->state) {
	case BT_OPEN:
	case BT_BOUND:
	case BT_CLOSED:
		break;
	case BT_LISTEN:
		l2cap_sock_cleanup_listen(sk);
		sk->sk_state = BT_CLOSED;
		chan->state = BT_CLOSED;

		break;
	default:
		sk->sk_state = BT_CLOSED;
		chan->state = BT_CLOSED;

		sk->sk_err = err;

		if (parent) {
			bt_accept_unlink(sk);
			parent->sk_data_ready(parent, 0);
		} else {
			sk->sk_state_change(sk);
		}

		break;
	}

	release_sock(sk);
}

static void l2cap_sock_state_change_cb(struct l2cap_chan *chan, int state)
{
	struct sock *sk = chan->data;

	sk->sk_state = state;
}

static struct sk_buff *l2cap_sock_alloc_skb_cb(struct l2cap_chan *chan,
					       unsigned long len, int nb)
{
	struct sk_buff *skb;
	int err;

	l2cap_chan_unlock(chan);
	skb = bt_skb_send_alloc(chan->sk, len, nb, &err);
	l2cap_chan_lock(chan);

	if (!skb)
		return ERR_PTR(err);

	return skb;
}

static void l2cap_sock_ready_cb(struct l2cap_chan *chan)
{
	struct sock *sk = chan->data;
	struct sock *parent;

	lock_sock(sk);

	parent = bt_sk(sk)->parent;

	BT_DBG("sk %p, parent %p", sk, parent);

	sk->sk_state = BT_CONNECTED;
	sk->sk_state_change(sk);

	if (parent)
		parent->sk_data_ready(parent, 0);

	release_sock(sk);
}

static void l2cap_sock_defer_cb(struct l2cap_chan *chan)
{
	struct sock *sk = chan->data;
	struct sock *parent = bt_sk(sk)->parent;

	if (parent)
		parent->sk_data_ready(parent, 0);
}

static struct l2cap_ops l2cap_chan_ops = {
	.name		= "L2CAP Socket Interface",
	.new_connection	= l2cap_sock_new_connection_cb,
	.recv		= l2cap_sock_recv_cb,
	.close		= l2cap_sock_close_cb,
	.teardown	= l2cap_sock_teardown_cb,
	.state_change	= l2cap_sock_state_change_cb,
	.ready		= l2cap_sock_ready_cb,
	.defer		= l2cap_sock_defer_cb,
	.alloc_skb	= l2cap_sock_alloc_skb_cb,
};

static void l2cap_sock_destruct(struct sock *sk)
{
	BT_DBG("sk %p", sk);

	if (l2cap_pi(sk)->chan)
		l2cap_chan_put(l2cap_pi(sk)->chan);
	if (l2cap_pi(sk)->rx_busy_skb) {
		kfree_skb(l2cap_pi(sk)->rx_busy_skb);
		l2cap_pi(sk)->rx_busy_skb = NULL;
	}

	skb_queue_purge(&sk->sk_receive_queue);
	skb_queue_purge(&sk->sk_write_queue);

	l2cap_ertm_destruct(sk);
}

static void set_default_config(struct l2cap_conf_prm *conf_prm)
{
	conf_prm->fcs = L2CAP_FCS_CRC16;
	conf_prm->flush_to = L2CAP_DEFAULT_FLUSH_TO;
}

void l2cap_sock_init(struct sock *sk, struct sock *parent)
{
	struct l2cap_pinfo *pi = l2cap_pi(sk);

	BT_DBG("sk %p parent %p", sk, parent);

	if (parent) {
		sk->sk_type = parent->sk_type;
		bt_sk(sk)->flags = bt_sk(parent)->flags;

		pi->imtu = l2cap_pi(parent)->imtu;
		pi->omtu = l2cap_pi(parent)->omtu;
		pi->conf_state = l2cap_pi(parent)->conf_state;
		pi->mode = l2cap_pi(parent)->mode;
		pi->fcs  = l2cap_pi(parent)->fcs;
		pi->max_tx = l2cap_pi(parent)->max_tx;
		pi->tx_win = l2cap_pi(parent)->tx_win;
		pi->sec_level = l2cap_pi(parent)->sec_level;
		pi->role_switch = l2cap_pi(parent)->role_switch;
		pi->force_reliable = l2cap_pi(parent)->force_reliable;
		pi->flushable = l2cap_pi(parent)->flushable;
		pi->force_active = l2cap_pi(parent)->force_active;
		pi->amp_pref = l2cap_pi(parent)->amp_pref;
	} else {
		pi->imtu = L2CAP_DEFAULT_MTU;
		pi->omtu = 0;
		if (!disable_ertm && sk->sk_type == SOCK_STREAM) {
			pi->mode = L2CAP_MODE_ERTM;
			pi->conf_state |= L2CAP_CONF_STATE2_DEVICE;
		} else {
			pi->mode = L2CAP_MODE_BASIC;
		}

		l2cap_chan_set_defaults(chan);
	}

	/* Default config options */
	sk->sk_backlog_rcv = l2cap_data_channel;
	pi->ampcon = NULL;
	pi->ampchan = NULL;
	pi->conf_len = 0;
	pi->flush_to = L2CAP_DEFAULT_FLUSH_TO;
	pi->scid = 0;
	pi->dcid = 0;
	pi->tx_win_max = L2CAP_TX_WIN_MAX_ENHANCED;
	pi->ack_win = pi->tx_win;
	pi->extended_control = 0;

	pi->local_conf.fcs = pi->fcs;
	pi->local_conf.flush_to = pi->flush_to;

	set_default_config(&pi->remote_conf);

	skb_queue_head_init(TX_QUEUE(sk));
	skb_queue_head_init(SREJ_QUEUE(sk));
}

static struct proto l2cap_proto = {
	.name		= "L2CAP",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct l2cap_pinfo)
};

static struct sock *l2cap_sock_alloc(struct net *net, struct socket *sock,
				     int proto, gfp_t prio)
{
	struct sock *sk;

	sk = sk_alloc(net, PF_BLUETOOTH, prio, &l2cap_proto);
	if (!sk)
		return NULL;

	sock_init_data(sock, sk);
	INIT_LIST_HEAD(&bt_sk(sk)->accept_q);

	sk->sk_destruct = l2cap_sock_destruct;
	sk->sk_sndtimeo = msecs_to_jiffies(L2CAP_CONN_TIMEOUT);

	sock_reset_flag(sk, SOCK_ZAPPED);

	sk->sk_protocol = proto;
	sk->sk_state = BT_OPEN;

	chan = l2cap_chan_create();
	if (!chan) {
		sk_free(sk);
		return NULL;
	}

	l2cap_chan_hold(chan);

	chan->sk = sk;

	l2cap_pi(sk)->chan = chan;

	bt_sock_link(&l2cap_sk_list, sk);
	return sk;
}

static int l2cap_sock_create(struct net *net, struct socket *sock, int protocol,
			     int kern)
{
	struct sock *sk;

	BT_DBG("sock %p", sock);

	sock->state = SS_UNCONNECTED;

	if (sock->type != SOCK_SEQPACKET && sock->type != SOCK_STREAM &&
	    sock->type != SOCK_DGRAM && sock->type != SOCK_RAW)
		return -ESOCKTNOSUPPORT;

	if (sock->type == SOCK_RAW && !kern && !capable(CAP_NET_RAW))
		return -EPERM;

	sock->ops = &l2cap_sock_ops;

	sk = l2cap_sock_alloc(net, sock, protocol, GFP_ATOMIC);
	if (!sk)
		return -ENOMEM;

	l2cap_sock_init(sk, NULL);
	bt_sock_link(&l2cap_sk_list, sk);
	return 0;
}

const struct proto_ops l2cap_sock_ops = {
	.family		= PF_BLUETOOTH,
	.owner		= THIS_MODULE,
	.release	= l2cap_sock_release,
	.bind		= l2cap_sock_bind,
	.connect	= l2cap_sock_connect,
	.listen		= l2cap_sock_listen,
	.accept		= l2cap_sock_accept,
	.getname	= l2cap_sock_getname,
	.sendmsg	= l2cap_sock_sendmsg,
	.recvmsg	= l2cap_sock_recvmsg,
	.poll		= bt_sock_poll,
	.ioctl		= bt_sock_ioctl,
	.mmap		= sock_no_mmap,
	.socketpair	= sock_no_socketpair,
	.shutdown	= l2cap_sock_shutdown,
	.setsockopt	= l2cap_sock_setsockopt,
	.getsockopt	= l2cap_sock_getsockopt
};

static const struct net_proto_family l2cap_sock_family_ops = {
	.family	= PF_BLUETOOTH,
	.owner	= THIS_MODULE,
	.create	= l2cap_sock_create,
};

int __init l2cap_init_sockets(void)
{
	int err;

	err = proto_register(&l2cap_proto, 0);
	if (err < 0)
		return err;

	err = bt_sock_register(BTPROTO_L2CAP, &l2cap_sock_family_ops);
	if (err < 0) {
		BT_ERR("L2CAP socket registration failed");
		goto error;
	}

	err = bt_procfs_init(&init_net, "l2cap", &l2cap_sk_list,
			     NULL);
	if (err < 0) {
		BT_ERR("Failed to create L2CAP proc file");
		bt_sock_unregister(BTPROTO_L2CAP);
		goto error;
	}

	BT_INFO("L2CAP socket layer initialized");

	return 0;

error:
	proto_unregister(&l2cap_proto);
	return err;
}

void l2cap_cleanup_sockets(void)
{
	bt_procfs_cleanup(&init_net, "l2cap");
	bt_sock_unregister(BTPROTO_L2CAP);
	proto_unregister(&l2cap_proto);
}
