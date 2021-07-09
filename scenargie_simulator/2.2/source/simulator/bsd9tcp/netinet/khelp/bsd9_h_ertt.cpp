/*-
 * Copyright (c) 2009-2010
 *  Swinburne University of Technology, Melbourne, Australia
 * Copyright (c) 2010 Lawrence Stewart <lstewart@freebsd.org>
 * Copyright (c) 2010-2011 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed at the Centre for Advanced Internet
 * Architectures, Swinburne University of Technology, by David Hayes, made
 * possible in part by a grant from the Cisco University Research Program Fund
 * at Community Foundation Silicon Valley.
 *
 * Portions of this software were developed at the Centre for Advanced
 * Internet Architectures, Swinburne University of Technology, Melbourne,
 * Australia by David Hayes under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

//ScenSim-Port//#include <sys/cdefs.h>
//ScenSim-Port//__FBSDID("$FreeBSD$");

//ScenSim-Port//#include <sys/param.h>
//ScenSim-Port//#include <sys/kernel.h>
//ScenSim-Port//#include <sys/mbuf.h>
//ScenSim-Port//#include <sys/module.h>
//ScenSim-Port//#include <sys/hhook.h>
//ScenSim-Port//#include <sys/khelp.h>
//ScenSim-Port//#include <sys/module_khelp.h>
//ScenSim-Port//#include <sys/socket.h>
//ScenSim-Port//#include <sys/sockopt.h>

//ScenSim-Port//#include <net/vnet.h>

//ScenSim-Port//#include <netinet/in.h>
//ScenSim-Port//#include <netinet/in_pcb.h>
//ScenSim-Port//#include <netinet/tcp_seq.h>
//ScenSim-Port//#include <netinet/tcp_var.h>

//ScenSim-Port//#include <netinet/khelp/h_ertt.h>

//ScenSim-Port//#include <vm/uma.h>
#include "tcp_porting.h"                     //ScenSim-Port//
namespace FreeBsd9Port {                                        //ScenSim-Port//

//ScenSim-Port//uma_zone_t txseginfo_zone;
#define txseginfo_zone SCENSIMGLOBAL(txseginfo_zone)            //ScenSim-Port//

/* Smoothing factor for delayed ack guess. */
#define DLYACK_SMOOTH   5

/* Max number of time stamp errors allowed in a session. */
#define MAX_TS_ERR  10

//ScenSim-Port//static int ertt_packet_measurement_hook(int hhook_type, int hhook_id,
//ScenSim-Port//    void *udata, void *ctx_data, void *hdata, struct osd *hosd);
//ScenSim-Port//static int ertt_add_tx_segment_info_hook(int hhook_type, int hhook_id,
//ScenSim-Port//    void *udata, void *ctx_data, void *hdata, struct osd *hosd);
//ScenSim-Port//static int ertt_mod_init(void);
//ScenSim-Port//static int ertt_mod_destroy(void);
//ScenSim-Port//static int ertt_uma_ctor(void *mem, int size, void *arg, int flags);
//ScenSim-Port//static void ertt_uma_dtor(void *mem, int size, void *arg);

/*
 * Contains information about the sent segment for comparison with the
 * corresponding ack.
 */
struct txseginfo {
    /* Segment length. */
    long        len;
    /* Segment sequence number. */
    tcp_seq     seq;
    /* Time stamp indicating when the packet was sent. */
    uint32_t    tx_ts;
    /* Last received receiver ts (if the TCP option is used). */
    uint32_t    rx_ts;
    uint32_t    flags;
    TAILQ_ENTRY (txseginfo) txsegi_lnk;
};

/* Flags for struct txseginfo. */
//ScenSim-Port//#define TXSI_TSO        0x01 /* TSO was used for this entry. */
#define TXSI_RTT_MEASURE_START  0x02 /* Start a per RTT measurement. */
//ScenSim-Port//#define TXSI_RX_MEASURE_END 0x04 /* Measure the rx rate until this txsi. */

//ScenSim-Port//struct helper ertt_helper = {
//ScenSim-Port//    .mod_init = ertt_mod_init,
//ScenSim-Port//    .mod_destroy = ertt_mod_destroy,
//ScenSim-Port//    .h_flags = HELPER_NEEDS_OSD,
//ScenSim-Port//    .h_classes = HELPER_CLASS_TCP
//ScenSim-Port//};

/* Define the helper hook info required by ERTT. */
//ScenSim-Port//struct hookinfo ertt_hooks[] = {
//ScenSim-Port//    {
//ScenSim-Port//        .hook_type = HHOOK_TYPE_TCP,
//ScenSim-Port//        .hook_id = HHOOK_TCP_EST_IN,
//ScenSim-Port//        .hook_udata = NULL,
//ScenSim-Port//        .hook_func = &ertt_packet_measurement_hook
//ScenSim-Port//    },
//ScenSim-Port//    {
//ScenSim-Port//        .hook_type = HHOOK_TYPE_TCP,
//ScenSim-Port//        .hook_id = HHOOK_TCP_EST_OUT,
//ScenSim-Port//        .hook_udata = NULL,
//ScenSim-Port//        .hook_func = &ertt_add_tx_segment_info_hook
//ScenSim-Port//    }
//ScenSim-Port//};

/* Flags to indicate how marked_packet_rtt should handle this txsi. */
#define MULTI_ACK       0x01 /* More than this txsi is acked. */
#define OLD_TXSI        0x02 /* TXSI is old according to timestamps. */
#define CORRECT_ACK     0X04 /* Acks this TXSI. */
#define FORCED_MEASUREMENT  0X08 /* Force an RTT measurement. */

/*
 * This fuction measures the RTT of a particular segment/ack pair, or the next
 * closest if this will yield an inaccurate result due to delayed acking or
 * other issues.
 */
static void inline
marked_packet_rtt(struct txseginfo *txsi, struct ertt *e_t, struct tcpcb *tp,
    uint32_t *pmeasurenext, int *pmeasurenext_len, int *prtt_bytes_adjust,
    int mflag)
{

    /*
     * If we can't measure this one properly due to delayed acking adjust
     * byte counters and flag to measure next txsi. Note that since the
     * marked packet's transmitted bytes are measured we need to subtract the
     * transmitted bytes. Then pretend the next txsi was marked.
     */
    if (mflag & (MULTI_ACK|OLD_TXSI)) {
        *pmeasurenext = txsi->tx_ts;
        *pmeasurenext_len = txsi->len;
        *prtt_bytes_adjust += *pmeasurenext_len;
    } else {
        if (mflag & FORCED_MEASUREMENT) {
            e_t->markedpkt_rtt = ticks - *pmeasurenext + 1;
            e_t->bytes_tx_in_marked_rtt = e_t->bytes_tx_in_rtt +
                *pmeasurenext_len - *prtt_bytes_adjust;
        } else {
            e_t->markedpkt_rtt = ticks - txsi->tx_ts + 1;
            e_t->bytes_tx_in_marked_rtt = e_t->bytes_tx_in_rtt -
                *prtt_bytes_adjust;
        }
        e_t->marked_snd_cwnd = tp->snd_cwnd;

        /*
         * Reset the ERTT_MEASUREMENT_IN_PROGRESS flag to indicate to
         * add_tx_segment_info that a new measurement should be started.
         */
        e_t->flags &= ~ERTT_MEASUREMENT_IN_PROGRESS;
        /*
         * Set ERTT_NEW_MEASUREMENT to tell the congestion control
         * algorithm that a new marked RTT measurement has has been made
         * and is available for use.
         */
        e_t->flags |= ERTT_NEW_MEASUREMENT;

//ScenSim-Port//        if (tp->t_flags & TF_TSO) {
//ScenSim-Port//            /* Temporarily disable TSO to aid a new measurment. */
//ScenSim-Port//            tp->t_flags &= ~TF_TSO;
//ScenSim-Port//            /* Keep track that we've disabled it. */
//ScenSim-Port//            e_t->flags |= ERTT_TSO_DISABLED;
//ScenSim-Port//        }
    }
}

/*
 * Ertt_packet_measurements uses a small amount of state kept on each packet
 * sent to match incoming acknowledgements. This enables more accurate and
 * secure round trip time measurements. The resulting measurement is used for
 * congestion control algorithms which require a more accurate time.
 * Ertt_packet_measurements is called via the helper hook in tcp_input.c
 */
//ScenSim-Port//static int
//ScenSim-Port//ertt_packet_measurement_hook(int hhook_type, int hhook_id, void *udata,
//ScenSim-Port//    void *ctx_data, void *hdata, struct osd *hosd)
int ertt_packet_measurement_hook(                               //ScenSim-Port//
    struct tcp_hhook_data *ctx_data, struct ertt *hdata)        //ScenSim-Port//
{
    struct ertt *e_t;
    struct tcpcb *tp;
    struct tcphdr *th;
    struct tcpopt *to;
    struct tcp_hhook_data *thdp;
    struct txseginfo *txsi;
    int acked, measurenext_len, multiack, new_sacked_bytes, rtt_bytes_adjust;
    uint32_t measurenext, rts;
    tcp_seq ack;

//ScenSim-Port//    KASSERT(ctx_data != NULL, ("%s: ctx_data is NULL!", __func__));
//ScenSim-Port//    KASSERT(hdata != NULL, ("%s: hdata is NULL!", __func__));

    e_t = (struct ertt *)hdata;
    thdp = ctx_data;
    tp = thdp->tp;
    th = thdp->th;
    to = thdp->to;
    new_sacked_bytes = (tp->sackhint.last_sack_ack != 0);
    measurenext = measurenext_len = multiack = rts = rtt_bytes_adjust = 0;
    acked = th->th_ack - tp->snd_una;

//ScenSim-Port//    INP_WLOCK_ASSERT(tp->t_inpcb);

    /* Packet has provided new acknowledgements. */
    if (acked > 0 || new_sacked_bytes) {
        if (acked == 0 && new_sacked_bytes) {
            /* Use last sacked data. */
            ack = tp->sackhint.last_sack_ack;
        } else
            ack = th->th_ack;

        txsi = TAILQ_FIRST(&e_t->txsegi_q);
        while (txsi != NULL) {
            rts = 0;

            /* Acknowledgement is acking more than this txsi. */
            if (SEQ_GT(ack, txsi->seq + txsi->len)) {
                if (txsi->flags & TXSI_RTT_MEASURE_START ||
                    measurenext) {
                    marked_packet_rtt(txsi, e_t, tp,
                        &measurenext, &measurenext_len,
                        &rtt_bytes_adjust, MULTI_ACK);
                }
                TAILQ_REMOVE(&e_t->txsegi_q, txsi, txsegi_lnk);
                uma_zfree(txseginfo_zone, txsi);
                txsi = TAILQ_FIRST(&e_t->txsegi_q);
                continue;
            }

            /*
             * Guess if delayed acks are being used by the receiver.
             *
             * XXXDH: A simple heuristic that could be improved
             */
            if (!new_sacked_bytes) {
                if (acked > tp->t_maxseg) {
                    e_t->dlyack_rx +=
                        (e_t->dlyack_rx < DLYACK_SMOOTH) ?
                        1 : 0;
                    multiack = 1;
                } else if (acked > txsi->len) {
                    multiack = 1;
                    e_t->dlyack_rx +=
                        (e_t->dlyack_rx < DLYACK_SMOOTH) ?
                        1 : 0;
                } else if ((u_int)acked == tp->t_maxseg ||
                       acked == txsi->len) {
                    e_t->dlyack_rx -=
                        (e_t->dlyack_rx > 0) ? 1 : 0;
                }
                /* Otherwise leave dlyack_rx the way it was. */
            }

            /*
             * Time stamps are only to help match the txsi with the
             * received acknowledgements.
             */
            if (e_t->timestamp_errors < MAX_TS_ERR &&
                (to->to_flags & TOF_TS) != 0 && to->to_tsecr) {
                /*
                 * Note: All packets sent with the offload will
                 * have the same time stamp. If we are sending
                 * on a fast interface and the t_maxseg is much
                 * smaller than one tick, this will be fine. The
                 * time stamp would be the same whether we were
                 * using tso or not. However, if the interface
                 * is slow, this will cause problems with the
                 * calculations. If the interface is slow, there
                 * is not reason to be using tso, and it should
                 * be turned off.
                 */
                /*
                 * If there are too many time stamp errors, time
                 * stamps won't be trusted
                 */
                rts = to->to_tsecr;
                /* Before this packet. */
                if (!e_t->dlyack_rx && TSTMP_LT(rts, txsi->tx_ts))
                    /* When delayed acking is used, the
                     * reflected time stamp is of the first
                     * packet and thus may be before
                     * txsi->tx_ts.
                     */
                    break;
                if (TSTMP_GT(rts, txsi->tx_ts)) {
                    /*
                     * If reflected time stamp is later than
                     * tx_tsi, then this txsi is old.
                     */
                    if (txsi->flags & TXSI_RTT_MEASURE_START
                        || measurenext) {
                        marked_packet_rtt(txsi, e_t, tp,
                            &measurenext, &measurenext_len,
                            &rtt_bytes_adjust, OLD_TXSI);
                    }
                    TAILQ_REMOVE(&e_t->txsegi_q, txsi,
                        txsegi_lnk);
                    uma_zfree(txseginfo_zone, txsi);
                    txsi = TAILQ_FIRST(&e_t->txsegi_q);
                    continue;
                }
                if (rts == txsi->tx_ts &&
                    TSTMP_LT(to->to_tsval, txsi->rx_ts)) {
                    /*
                     * Segment received before sent!
                     * Something is wrong with the received
                     * timestamps so increment errors. If
                     * this keeps up we will ignore
                     * timestamps.
                     */
                    e_t->timestamp_errors++;
                }
            }
            /*
             * Acknowledging a sequence number before this txsi.
             * If it is an old txsi that may have had the same seq
             * numbers, it should have been removed if time stamps
             * are being used.
             */
            if (SEQ_LEQ(ack, txsi->seq))
                break; /* Before first packet in txsi. */

            /*
             * Only ack > txsi->seq and ack <= txsi->seq+txsi->len
             * past this point.
             *
             * If delayed acks are being used, an acknowledgement
             * for a single segment will have been delayed by the
             * receiver and will yield an inaccurate measurement. In
             * this case, we only make the measurement if more than
             * one segment is being acknowledged or sack is
             * currently being used.
             */
            if (!e_t->dlyack_rx || multiack || new_sacked_bytes) {
                /* Make an accurate new measurement. */
                e_t->rtt = ticks - txsi->tx_ts + 1;
                if (e_t->rtt < e_t->minrtt || e_t->minrtt == 0)
                    e_t->minrtt = e_t->rtt;

                if (e_t->rtt > e_t->maxrtt || e_t->maxrtt == 0)
                    e_t->maxrtt = e_t->rtt;
            }

            if (txsi->flags & TXSI_RTT_MEASURE_START || measurenext)
                marked_packet_rtt(txsi, e_t, tp,
                    &measurenext, &measurenext_len,
                    &rtt_bytes_adjust, CORRECT_ACK);

//ScenSim-Port//            if (txsi->flags & TXSI_TSO) {
//ScenSim-Port//                txsi->len -= acked;
//ScenSim-Port//                if (txsi->len > 0) {
//ScenSim-Port//                    /*
//ScenSim-Port//                     * This presumes ack for first bytes in
//ScenSim-Port//                     * txsi, this may not be true but it
//ScenSim-Port//                     * shouldn't cause problems for the
//ScenSim-Port//                     * timing.
//ScenSim-Port//                     *
//ScenSim-Port//                     * We remeasure RTT even though we only
//ScenSim-Port//                     * have a single txsi. The rationale
//ScenSim-Port//                     * behind this is that it is better to
//ScenSim-Port//                     * have a slightly inaccurate
//ScenSim-Port//                     * measurement than no additional
//ScenSim-Port//                     * measurement for the rest of the bulk
//ScenSim-Port//                     * transfer. Since TSO is only used on
//ScenSim-Port//                     * high speed interface cards, so the
//ScenSim-Port//                     * packets should be transmitted at line
//ScenSim-Port//                     * rate back to back with little
//ScenSim-Port//                     * difference in transmission times (in
//ScenSim-Port//                     * ticks).
//ScenSim-Port//                     */
//ScenSim-Port//                    txsi->seq += acked;
//ScenSim-Port//                    /*
//ScenSim-Port//                     * Reset txsi measure flag so we don't
//ScenSim-Port//                     * use it for another RTT measurement.
//ScenSim-Port//                     */
//ScenSim-Port//                    txsi->flags &= ~TXSI_RTT_MEASURE_START;
//ScenSim-Port//                    /*
//ScenSim-Port//                     * There is still more data to be acked
//ScenSim-Port//                     * from tso bulk transmission, so we
//ScenSim-Port//                     * won't remove it from the TAILQ yet.
//ScenSim-Port//                     */
//ScenSim-Port//                    break;
//ScenSim-Port//                }
//ScenSim-Port//            }

            TAILQ_REMOVE(&e_t->txsegi_q, txsi, txsegi_lnk);
            uma_zfree(txseginfo_zone, txsi);
            break;
        }

        if (measurenext) {
            /*
             * We need to do a RTT measurement. It won't be the best
             * if we do it here.
             */
            marked_packet_rtt(txsi, e_t, tp,
                &measurenext, &measurenext_len,
                &rtt_bytes_adjust, FORCED_MEASUREMENT);
        }
    }

    return (0);
}

/*
 * Add information about a transmitted segment to a list.
 * This is called via the helper hook in tcp_output.c
 */
//ScenSim-Port//static int
//ScenSim-Port//ertt_add_tx_segment_info_hook(int hhook_type, int hhook_id, void *udata,
//ScenSim-Port//    void *ctx_data, void *hdata, struct osd *hosd)
int ertt_add_tx_segment_info_hook(                              //ScenSim-Port//
    struct tcp_hhook_data *ctx_data, struct ertt *hdata)        //ScenSim-Port//
{
    struct ertt *e_t;
    struct tcpcb *tp;
    struct tcphdr *th;
    struct tcpopt *to;
    struct tcp_hhook_data *thdp;
    struct txseginfo *txsi;
    long len;
//ScenSim-Port//    int tso;

//ScenSim-Port//    KASSERT(ctx_data != NULL, ("%s: ctx_data is NULL!", __func__));
//ScenSim-Port//    KASSERT(hdata != NULL, ("%s: hdata is NULL!", __func__));

    e_t = (struct ertt *)hdata;
//ScenSim-Port//    thdp = ctx_data;
    thdp = (struct tcp_hhook_data *)ctx_data;                   //ScenSim-Port//
    tp = thdp->tp;
    th = thdp->th;
    to = thdp->to;
    len = thdp->len;
//ScenSim-Port//    tso = thdp->tso;

//ScenSim-Port//    INP_WLOCK_ASSERT(tp->t_inpcb);

    if (len > 0) {
//ScenSim-Port//        txsi = uma_zalloc(txseginfo_zone, M_NOWAIT);
        txsi = (struct txseginfo *)uma_zalloc(                  //ScenSim-Port//
            txseginfo_zone, M_NOWAIT);                          //ScenSim-Port//
        if (txsi != NULL) {
            /* Construct txsi setting the necessary flags. */
            txsi->flags = 0; /* Needs to be initialised. */
            txsi->seq = ntohl(th->th_seq);
            txsi->len = len;
//ScenSim-Port//            if (tso)
//ScenSim-Port//                txsi->flags |= TXSI_TSO;
//ScenSim-Port//            else if (e_t->flags & ERTT_TSO_DISABLED) {
//ScenSim-Port//                tp->t_flags |= TF_TSO;
//ScenSim-Port//                e_t->flags &= ~ERTT_TSO_DISABLED;
//ScenSim-Port//            }

            if (e_t->flags & ERTT_MEASUREMENT_IN_PROGRESS) {
                e_t->bytes_tx_in_rtt += len;
            } else {
                txsi->flags |= TXSI_RTT_MEASURE_START;
                e_t->flags |= ERTT_MEASUREMENT_IN_PROGRESS;
                e_t->bytes_tx_in_rtt = len;
            }

            if (((tp->t_flags & TF_NOOPT) == 0) &&
                (to->to_flags & TOF_TS)) {
                txsi->tx_ts = ntohl(to->to_tsval) -
                    tp->ts_offset;
                txsi->rx_ts = ntohl(to->to_tsecr);
            } else {
                txsi->tx_ts = ticks;
                txsi->rx_ts = 0; /* No received time stamp. */
            }
            TAILQ_INSERT_TAIL(&e_t->txsegi_q, txsi, txsegi_lnk);
        }
    }

    return (0);
}

//ScenSim-Port//static int
//ScenSim-Port//ertt_mod_init(void)
int ertt_mod_init(void)                                         //ScenSim-Port//
{

    txseginfo_zone = uma_zcreate("ertt_txseginfo", sizeof(struct txseginfo),
        NULL, NULL, NULL, NULL, 0, 0);

    return (0);
}

//ScenSim-Port//static int
//ScenSim-Port//ertt_mod_destroy(void)
int ertt_mod_destroy(void)                                      //ScenSim-Port//
{

    uma_zdestroy(txseginfo_zone);

    return (0);
}

//ScenSim-Port//static int
//ScenSim-Port//ertt_uma_ctor(void *mem, int size, void *arg, int flags)
int ertt_uma_ctor(struct ertt *e_t)                             //ScenSim-Port//
{
//ScenSim-Port//    struct ertt *e_t;

//ScenSim-Port//    e_t = mem;

    TAILQ_INIT(&e_t->txsegi_q);
    e_t->timestamp_errors = 0;
    e_t->minrtt = 0;
    e_t->maxrtt = 0;
    e_t->rtt = 0;
    e_t->flags = 0;
    e_t->dlyack_rx = 0;
    e_t->bytes_tx_in_rtt = 0;
    e_t->markedpkt_rtt = 0;

    return (0);
}

//ScenSim-Port//static void
//ScenSim-Port//ertt_uma_dtor(void *mem, int size, void *arg)
void ertt_uma_dtor(struct ertt *e_t)                            //ScenSim-Port//
{
//ScenSim-Port//    struct ertt *e_t;
    struct txseginfo *n_txsi, *txsi;

//ScenSim-Port//    e_t = mem;
    txsi = TAILQ_FIRST(&e_t->txsegi_q);
    while (txsi != NULL) {
        n_txsi = TAILQ_NEXT(txsi, txsegi_lnk);
        uma_zfree(txseginfo_zone, txsi);
        txsi = n_txsi;
    }
}

//ScenSim-Port//KHELP_DECLARE_MOD_UMA(ertt, &ertt_helper, ertt_hooks, 1, sizeof(struct ertt),
//ScenSim-Port//    ertt_uma_ctor, ertt_uma_dtor);
}//namespace//                                                  //ScenSim-Port//
