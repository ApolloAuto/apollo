/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtBluetooth module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef BLUEZ_DATA_P_H
#define BLUEZ_DATA_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qglobal.h>
#include <QtCore/qendian.h>
#include <sys/socket.h>
#include <QtBluetooth/QBluetoothUuid>

#define BTPROTO_L2CAP   0
#define BTPROTO_HCI     1
#define BTPROTO_RFCOMM  3

#define SOL_HCI     0
#define SOL_L2CAP   6
#define SOL_RFCOMM  18
#ifndef SOL_BLUETOOTH
#define SOL_BLUETOOTH   274
#endif

#define RFCOMM_LM   0x03

#define RFCOMM_LM_AUTH      0x0002
#define RFCOMM_LM_ENCRYPT   0x0004
#define RFCOMM_LM_TRUSTED   0x0008
#define RFCOMM_LM_SECURE    0x0020

#define L2CAP_LM            0x03
#define L2CAP_LM_AUTH       0x0002
#define L2CAP_LM_ENCRYPT    0x0004
#define L2CAP_LM_TRUSTED    0x0008
#define L2CAP_LM_SECURE     0x0020

#define BT_SECURITY 4
struct bt_security {
    quint8 level;
    quint8 key_size;
};
#define BT_SECURITY_SDP     0
#define BT_SECURITY_LOW     1
#define BT_SECURITY_MEDIUM  2
#define BT_SECURITY_HIGH    3

#define BDADDR_LE_PUBLIC    0x01
#define BDADDR_LE_RANDOM    0x02


/* Byte order conversions */
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define htobs(d)  (d)
#define htobl(d)  (d)
#define htobll(d) (d)
#define btohs(d)  (d)
#define btohl(d)  (d)
#define btohll(d) (d)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define htobs(d)  qbswap((quint16)(d))
#define htobl(d)  qbswap((quint32)(d))
#define htobll(d) qbswap((quint64)(d))
#define btohs(d)  qbswap((quint16)(d))
#define btohl(d)  qbswap((quint32)(d))
#define btohll(d) qbswap((quint64)(d))
#else
#error "Unknown byte order"
#endif

// Bluetooth address
typedef struct {
    quint8 b[6];
} __attribute__((packed)) bdaddr_t;

// L2CP socket
struct sockaddr_l2 {
    sa_family_t     l2_family;
    unsigned short  l2_psm;
    bdaddr_t        l2_bdaddr;
    unsigned short  l2_cid;
#if !defined(QT_BLUEZ_NO_BTLE)
    quint8          l2_bdaddr_type;
#endif
};

// RFCOMM socket
struct sockaddr_rc {
    sa_family_t rc_family;
    bdaddr_t    rc_bdaddr;
    quint8      rc_channel;
};

// Bt Low Energy related

#define bt_get_unaligned(ptr)           \
({                                      \
    struct __attribute__((packed)) {    \
        __typeof__(*(ptr)) __v;         \
    } *__p = (__typeof__(__p)) (ptr);   \
    __p->__v;                           \
})

#define bt_put_unaligned(val, ptr)      \
do {                                    \
    struct __attribute__((packed)) {    \
        __typeof__(*(ptr)) __v;         \
    } *__p = (__typeof__(__p)) (ptr);   \
    __p->__v = (val);                   \
} while (0)

#if __BYTE_ORDER == __LITTLE_ENDIAN

static inline void btoh128(const quint128 *src, quint128 *dst)
{
    memcpy(dst, src, sizeof(quint128));
}

static inline void ntoh128(const quint128 *src, quint128 *dst)
{
    int i;

    for (i = 0; i < 16; i++)
        dst->data[15 - i] = src->data[i];
}

static inline quint16 bt_get_le16(const void *ptr)
{
    return bt_get_unaligned((const quint16 *) ptr);
}
#elif __BYTE_ORDER == __BIG_ENDIAN
static inline quint16 bt_get_le16(const void *ptr)
{
    return qbswap(bt_get_unaligned((const quint16 *) ptr));
}

static inline void btoh128(const quint128 *src, quint128 *dst)
{
    int i;

    for (i = 0; i < 16; i++)
        dst->data[15 - i] = src->data[i];
}

static inline void ntoh128(const quint128 *src, quint128 *dst)
{
    memcpy(dst, src, sizeof(quint128));
}
#else
#error "Unknown byte order"
#endif

#define hton128(x, y) ntoh128(x, y)

// HCI related

#define HCI_MAX_DEV 16

#define HCI_MAX_EVENT_SIZE 260

// HCI sockopts
#define HCI_FILTER 2

// HCI packet types
#define HCI_EVENT_PKT   0x04
#define HCI_VENDOR_PKT  0xff

#define HCI_FLT_TYPE_BITS  31
#define HCI_FLT_EVENT_BITS 63

struct sockaddr_hci {
    sa_family_t hci_family;
    unsigned short hci_dev;
    unsigned short hci_channel;
};

struct hci_dev_req {
    quint16 dev_id;
    quint32 dev_opt;
};

struct hci_dev_list_req {
    quint16 dev_num;
    struct hci_dev_req dev_req[0];
};

struct hci_dev_stats {
    quint32 err_rx;
    quint32 err_tx;
    quint32 cmd_tx;
    quint32 evt_rx;
    quint32 acl_tx;
    quint32 acl_rx;
    quint32 sco_tx;
    quint32 sco_rx;
    quint32 byte_rx;
    quint32 byte_tx;
};

struct hci_dev_info {
    quint16 dev_id;
    char     name[8];

    bdaddr_t bdaddr;

    quint32 flags;
    quint8  type;

    quint8  features[8];

    quint32 pkt_type;
    quint32 link_policy;
    quint32 link_mode;

    quint16 acl_mtu;
    quint16 acl_pkts;
    quint16 sco_mtu;
    quint16 sco_pkts;

    struct   hci_dev_stats stat;
};

struct hci_conn_info {
    quint16 handle;
    bdaddr_t bdaddr;
    quint8  type;
    quint8  out;
    quint16 state;
    quint32 link_mode;
};

struct hci_conn_list_req {
    quint16 dev_id;
    quint16 conn_num;
    struct hci_conn_info conn_info[0];
};

struct hci_filter {
    quint32 type_mask;
    quint32 event_mask[2];
    quint16 opcode;
};

static inline void hci_set_bit(int nr, void *addr)
{
    *((quint32 *) addr + (nr >> 5)) |= (1 << (nr & 31));
}
static inline void hci_clear_bit(int nr, void *addr)
{
    *((quint32 *) addr + (nr >> 5)) &= ~(1 << (nr & 31));
}
static inline void hci_filter_clear(struct hci_filter *f)
{
    memset(f, 0, sizeof(*f));
}
static inline void hci_filter_set_ptype(int t, struct hci_filter *f)
{
    hci_set_bit((t == HCI_VENDOR_PKT) ? 0 : (t & HCI_FLT_TYPE_BITS), &f->type_mask);
}
static inline void hci_filter_clear_ptype(int t, struct hci_filter *f)
{
    hci_clear_bit((t == HCI_VENDOR_PKT) ? 0 : (t & HCI_FLT_TYPE_BITS), &f->type_mask);
}
static inline void hci_filter_set_event(int e, struct hci_filter *f)
{
    hci_set_bit((e & HCI_FLT_EVENT_BITS), &f->event_mask);
}
static inline void hci_filter_clear_event(int e, struct hci_filter *f)
{
    hci_clear_bit((e & HCI_FLT_EVENT_BITS), &f->event_mask);
}
static inline void hci_filter_all_ptypes(struct hci_filter *f)
{
    memset((void *) &f->type_mask, 0xff, sizeof(f->type_mask));
}
static inline void hci_filter_all_events(struct hci_filter *f)
{
    memset((void *) f->event_mask, 0xff, sizeof(f->event_mask));
}

typedef struct {
    quint8 evt;
    quint8 plen;
} __attribute__ ((packed)) hci_event_hdr;
#define HCI_EVENT_HDR_SIZE 2

#define EVT_ENCRYPT_CHANGE 0x08
typedef struct {
    quint8  status;
    quint16 handle;
    quint8  encrypt;
} __attribute__ ((packed)) evt_encrypt_change;
#define EVT_ENCRYPT_CHANGE_SIZE 4

#endif // BLUEZ_DATA_P_H
