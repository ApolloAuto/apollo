/*********************************************************************************
 * The RTKLIB software package is distributed under the following BSD 2-clause
 * license (http://opensource.org/licenses/BSD-2-Clause) and additional two
 * exclusive clauses. Users are permitted to develop, produce or sell their own
 * non-commercial or commercial products utilizing, linking or including RTKLIB
 *as long as they comply with the license.
 *
 *           Copyright (c) 2007-2013, T. Takasu, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice,
 *this list of conditions and the following disclaimer in the documentation
 *and/or other materials provided with the distribution.
 *
 * - The software package includes some companion executive binaries or shared
 *   libraries necessary to execute APs on Windows. These licenses succeed to
 *the original ones of these software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************************************/

/*------------------------------------------------------------------------------
 * rtcm3.c : rtcm ver.3 message decorder functions
 *
 *          Copyright (C) 2009-2015 by T.TAKASU, All rights reserved.
 *
 * options :
 *     -DSSR_QZSS_DRAFT_V05: qzss ssr messages based on ref [16]
 *
 * references :
 *     see rtcm.c
 *
 * version : $Revision:$ $Date:$
 * history : 2012/05/14 1.0  separated from rtcm.c
 *           2012/12/12 1.1  support gal/qzs ephemeris, gal/qzs ssr, msm
 *                           add station id consistency test for obs data
 *           2012/12/25 1.2  change compass msm id table
 *           2013/01/31 1.3  change signal id by the latest draft (ref [13])
 *           2013/02/23 1.4  change reference for rtcm 3 message (ref [14])
 *           2013/05/19 1.5  gpst -> bdt of time-tag in beidou msm message
 *           2014/05/02 1.6  fix bug on dropping last field of ssr message
 *                           comply with rtcm 3.2 with amendment 1/2 (ref[15])
 *                           delete MT 1046 according to ref [15]
 *           2014/09/14 1.7  add receiver option -RT_INP
 *           2014/12/06 1.8  support SBAS/BeiDou SSR messages (ref [16])
 *           2015/03/22 1.9  add handling of iodcrc for beidou/sbas ssr messages
 *-----------------------------------------------------------------------------*/

/**
 * file: rtcm3.c
 * version: rtklib ver.2.4.2
 * Copy from
 * https://github.com/tomojitakasu/RTKLIB/tree/76b9c97257f304aedad38b5a6bbbac444724aab3/src/rtcm3.c
 */

#include "rtklib.h"

static const char rcsid[] = "$Id:$";

/* constants -----------------------------------------------------------------*/

#define PRUNIT_GPS 299792.458 /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO 599584.916 /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS (CLIGHT * 0.001) /* range in 1 ms */

#define P2_10 0.0009765625          /* 2^-10 */
#define P2_34 5.820766091346740E-11 /* 2^-34 */
#define P2_46 1.421085471520200E-14 /* 2^-46 */
#define P2_59 1.734723475976810E-18 /* 2^-59 */

/* type definition -----------------------------------------------------------*/

typedef struct {              /* multi-signal-message header type */
  unsigned char iod;          /* issue of data station */
  unsigned char time_s;       /* cumulative session transmitting time */
  unsigned char clk_str;      /* clock steering indicator */
  unsigned char clk_ext;      /* external clock indicator */
  unsigned char smooth;       /* divergence free smoothing indicator */
  unsigned char tint_s;       /* soothing interval */
  unsigned char nsat, nsig;   /* number of satellites/signals */
  unsigned char sats[64];     /* satellites */
  unsigned char sigs[32];     /* signals */
  unsigned char cellmask[64]; /* cell mask */
} msm_h_t;

/* msm signal id table -------------------------------------------------------*/
const char* msm_sig_gps[32] = {
    /* GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 */
    "",   "1C", "1P", "1W", "1Y", "1M", "",   "2C",
    "2P", "2W", "2Y", "2M", /*  1-12 */
    "",   "",   "2S", "2L", "2X", "",   "",   "",
    "",   "5I", "5Q", "5X",                        /* 13-24 */
    "",   "",   "",   "",   "",   "1S", "1L", "1X" /* 25-32 */
};
const char* msm_sig_glo[32] = {
    /* GLONASS: ref [13] table 3.5-93, ref [14][15] table 3.5-97 */
    "",   "1C", "1P", "", "", "", "", "2C", "2P", "", "3I",
    "3Q", "3X", "",   "", "", "", "", "",   "",   "", "",
    "",   "",   "",   "", "", "", "", "",   "",   ""};
const char* msm_sig_gal[32] = {
    /* Galileo: ref [15] table 3.5-100 */
    "",   "1C", "1A", "1B", "1X", "1Z", "",   "6C", "6A", "6B", "6X",
    "6Z", "",   "7I", "7Q", "7X", "",   "8I", "8Q", "8X", "",   "5I",
    "5Q", "5X", "",   "",   "",   "",   "",   "",   "",   ""};
const char* msm_sig_qzs[32] = {
    /* QZSS: ref [15] table 3.5-103 */
    "",   "1C", "", "",   "",   "",   "", "",   "6S", "6L", "6X",
    "",   "",   "", "2S", "2L", "2X", "", "",   "",   "",   "5I",
    "5Q", "5X", "", "",   "",   "",   "", "1S", "1L", "1X"};
const char* msm_sig_sbs[32] = {
    /* SBAS: ref [13] table 3.5-T+005 */
    "", "1C", "", "", "", "",   "",   "",   "", "", "", "", "", "", "", "",
    "", "",   "", "", "", "5I", "5Q", "5X", "", "", "", "", "", "", "", ""};
const char* msm_sig_cmp[32] = {
    /* BeiDou: ref [15] table 3.5-106 */
    "", "1I", "1Q", "1X", "",   "", "", "6I", "6Q", "6X", "",
    "", "",   "7I", "7Q", "7X", "", "", "",   "",   "",   "",
    "", "",   "",   "",   "",   "", "", "",   "",   ""};
/* ssr update intervals ------------------------------------------------------*/
static const double ssrudint[16] = {
    1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800};
/* get sign-magnitude bits ---------------------------------------------------*/
static double getbitg(const unsigned char* buff, int pos, int len) {
  double value = getbitu(buff, pos + 1, len - 1);
  return getbitu(buff, pos, 1) ? -value : value;
}
/* adjust weekly rollover of gps time ----------------------------------------*/
static void adjweek(rtcm_t* rtcm, double tow) {
  double tow_p;
  int week;

  /* if no time, get cpu time */
  if (rtcm->time.time == 0) {
    rtcm->time = utc2gpst(timeget());
  }
  tow_p = time2gpst(rtcm->time, &week);
  if (tow < tow_p - 302400.0) {
    tow += 604800.0;
  } else if (tow > tow_p + 302400.0) {
    tow -= 604800.0;
  }
  rtcm->time = gpst2time(week, tow);
}
/* adjust weekly rollover of bdt time ----------------------------------------*/
static int adjbdtweek(int week) {
  int w;
  (void)time2bdt(gpst2bdt(utc2gpst(timeget())), &w);
  if (w < 1) {
    w = 1; /* use 2006/1/1 if time is earlier than 2006/1/1 */
  }
  return week + (w - week + 512) / 1024 * 1024;
}
/* adjust daily rollover of glonass time -------------------------------------*/
static void adjday_glot(rtcm_t* rtcm, double tod) {
  gtime_t time;
  double tow, tod_p;
  int week;

  if (rtcm->time.time == 0) {
    rtcm->time = utc2gpst(timeget());
  }
  time = timeadd(gpst2utc(rtcm->time), 10800.0); /* glonass time */
  tow = time2gpst(time, &week);
  tod_p = fmod(tow, 86400.0);
  tow -= tod_p;
  if (tod < tod_p - 43200.0) {
    tod += 86400.0;
  } else if (tod > tod_p + 43200.0) {
    tod -= 86400.0;
  }
  time = gpst2time(week, tow + tod);
  rtcm->time = utc2gpst(timeadd(time, -10800.0));
}
/* adjust carrier-phase rollover ---------------------------------------------*/
static double adjcp(rtcm_t* rtcm, int sat, int freq, double cp) {
  if (rtcm->cp[sat - 1][freq] == 0.0) {
    ;
  } else if (cp < rtcm->cp[sat - 1][freq] - 750.0) {
    cp += 1500.0;
  } else if (cp > rtcm->cp[sat - 1][freq] + 750.0) {
    cp -= 1500.0;
  }
  rtcm->cp[sat - 1][freq] = cp;
  return cp;
}
/* loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_t* rtcm, int sat, int freq, int lock) {
  int lli =
      (!lock && !rtcm->lock[sat - 1][freq]) || lock < rtcm->lock[sat - 1][freq];
  rtcm->lock[sat - 1][freq] = lock;
  return lli;
}
/* s/n ratio -----------------------------------------------------------------*/
static unsigned char snratio(double snr) {
  return (unsigned char)(snr <= 0.0 || 255.5 <= snr ? 0.0 : snr * 4.0 + 0.5);
}
/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t* obs, gtime_t time, int sat) {
  int i, j;

  for (i = 0; i < obs->n; ++i) {
    if (obs->data[i].sat == sat) {
      return i; /* field already exists */
    }
  }
  if (i >= MAXOBS) {
    return -1; /* overflow */
  }

  /* add new field */
  obs->data[i].time = time;
  obs->data[i].sat = sat;
  for (j = 0; j < NFREQ + NEXOBS; ++j) {
    obs->data[i].L[j] = obs->data[i].P[j] = 0.0;
    obs->data[i].D[j] = 0.0;
    obs->data[i].SNR[j] = obs->data[i].LLI[j] = obs->data[i].code[j] = 0;
  }
  obs->n++;
  return i;
}
/* test station id consistency -----------------------------------------------*/
static int test_staid(rtcm_t* rtcm, int staid) {
  char* p;
  int type, id;

  /* test station id option */
  if ((p = strstr(rtcm->opt, "-STA=")) && sscanf(p, "-STA=%d", &id) == 1) {
    if (staid != id) {
      return 0;
    }
  }
  /* save station id */
  if (rtcm->staid == 0 || rtcm->obsflag) {
    rtcm->staid = staid;
  } else if (staid != rtcm->staid) {
    type = getbitu(rtcm->buff, 24, 12);
    trace(2, "rtcm3 %d staid invalid id=%d %d\n", type, staid, rtcm->staid);

    /* reset station id if station id error */
    rtcm->staid = 0;
    return 0;
  }
  return 1;
}
/* decode type 1001-1004 message header --------------------------------------*/
static int decode_head1001(rtcm_t* rtcm, int* sync) {
  double tow;
  char* msg;
  int i = 24, staid, nsat, type;

  type = getbitu(rtcm->buff, i, 12);
  i += 12;

  if (i + 52 <= rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12;
    tow = getbitu(rtcm->buff, i, 30) * 0.001;
    i += 30;
    *sync = getbitu(rtcm->buff, i, 1);
    i += 1;
    nsat = getbitu(rtcm->buff, i, 5);
  } else {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  adjweek(rtcm, tow);

  trace(4, "decode_head1001: time=%s nsat=%d sync=%d\n",
        time_str(rtcm->time, 2), nsat, *sync);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " %s nsat=%2d sync=%d", time_str(rtcm->time, 2), nsat, *sync);
  }
  return nsat;
}
/* decode type 1001: L1-only gps rtk observation -----------------------------*/
static int decode_type1001(rtcm_t* rtcm) {
  int sync;
  if (decode_head1001(rtcm, &sync) < 0) {
    return -1;
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode type 1002: extended L1-only gps rtk observables --------------------*/
static int decode_type1002(rtcm_t* rtcm) {
  double pr1, cnr1, tt, cp1;
  int i = 24 + 64, j, index, nsat, sync, prn, code, sat, ppr1, lock1, amb, sys;

  if ((nsat = decode_head1001(rtcm, &sync)) < 0) {
    return -1;
  }

  for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 74 <= rtcm->len * 8;
       ++j) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    code = getbitu(rtcm->buff, i, 1);
    i += 1;
    pr1 = getbitu(rtcm->buff, i, 24);
    i += 24;
    ppr1 = getbits(rtcm->buff, i, 20);
    i += 20;
    lock1 = getbitu(rtcm->buff, i, 7);
    i += 7;
    amb = getbitu(rtcm->buff, i, 8);
    i += 8;
    cnr1 = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (prn < 40) {
      sys = SYS_GPS;
    } else {
      sys = SYS_SBS;
      prn += 80;
    }
    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 1002 satellite number error: prn=%d\n", prn);
      continue;
    }
    tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    if ((index = obsindex(&rtcm->obs, rtcm->time, sat)) < 0) {
      continue;
    }
    pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
    if (ppr1 != (int)0xFFF80000) {
      rtcm->obs.data[index].P[0] = pr1;
      cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam_carr[0]);
      rtcm->obs.data[index].L[0] = pr1 / lam_carr[0] + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code ? CODE_L1P : CODE_L1C;
  }
  return sync ? 0 : 1;
}
/* decode type 1003: L1&L2 gps rtk observables -------------------------------*/
static int decode_type1003(rtcm_t* rtcm) {
  int sync;
  if (decode_head1001(rtcm, &sync) < 0) {
    return -1;
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode type 1004: extended L1&L2 gps rtk observables ----------------------*/
static int decode_type1004(rtcm_t* rtcm) {
  const int L2codes[] = {CODE_L2C, CODE_L2P, CODE_L2W, CODE_L2W};
  double pr1, cnr1, cnr2, tt, cp1, cp2;
  int i = 24 + 64, j, index, nsat, sync, prn, sat, code1, code2, pr21, ppr1,
      ppr2;
  int lock1, lock2, amb, sys;

  if ((nsat = decode_head1001(rtcm, &sync)) < 0) {
    return -1;
  }

  for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 125 <= rtcm->len * 8;
       ++j) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    code1 = getbitu(rtcm->buff, i, 1);
    i += 1;
    pr1 = getbitu(rtcm->buff, i, 24);
    i += 24;
    ppr1 = getbits(rtcm->buff, i, 20);
    i += 20;
    lock1 = getbitu(rtcm->buff, i, 7);
    i += 7;
    amb = getbitu(rtcm->buff, i, 8);
    i += 8;
    cnr1 = getbitu(rtcm->buff, i, 8);
    i += 8;
    code2 = getbitu(rtcm->buff, i, 2);
    i += 2;
    pr21 = getbits(rtcm->buff, i, 14);
    i += 14;
    ppr2 = getbits(rtcm->buff, i, 20);
    i += 20;
    lock2 = getbitu(rtcm->buff, i, 7);
    i += 7;
    cnr2 = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (prn < 40) {
      sys = SYS_GPS;
    } else {
      sys = SYS_SBS;
      prn += 80;
    }
    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 1004 satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    if ((index = obsindex(&rtcm->obs, rtcm->time, sat)) < 0) {
      continue;
    }
    pr1 = pr1 * 0.02 + amb * PRUNIT_GPS;
    if (ppr1 != (int)0xFFF80000) {
      rtcm->obs.data[index].P[0] = pr1;
      cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam_carr[0]);
      rtcm->obs.data[index].L[0] = pr1 / lam_carr[0] + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

    if (pr21 != (int)0xFFFFE000) {
      rtcm->obs.data[index].P[1] = pr1 + pr21 * 0.02;
    }
    if (ppr2 != (int)0xFFF80000) {
      cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 / lam_carr[1]);
      rtcm->obs.data[index].L[1] = pr1 / lam_carr[1] + cp2;
    }
    rtcm->obs.data[index].LLI[1] = lossoflock(rtcm, sat, 1, lock2);
    rtcm->obs.data[index].SNR[1] = snratio(cnr2 * 0.25);
    rtcm->obs.data[index].code[1] = L2codes[code2];
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* get signed 38bit field ----------------------------------------------------*/
static double getbits_38(const unsigned char* buff, int pos) {
  return (double)getbits(buff, pos, 32) * 64.0 + getbitu(buff, pos + 32, 6);
}
/* decode type 1005: stationary rtk reference station arp --------------------*/
static int decode_type1005(rtcm_t* rtcm) {
  double rr[3];
  char* msg;
  int i = 24 + 12, j, staid, itrf;

  if (i + 140 == rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12;
    itrf = getbitu(rtcm->buff, i, 6);
    i += 6 + 4;
    rr[0] = getbits_38(rtcm->buff, i);
    i += 38 + 2;
    rr[1] = getbits_38(rtcm->buff, i);
    i += 38 + 2;
    rr[2] = getbits_38(rtcm->buff, i);
  } else {
    trace(2, "rtcm3 1005 length error: len=%d\n", rtcm->len);
    return -1;
  }
  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " staid=%4d", staid);
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  rtcm->sta.deltype = 0; /* xyz */
  for (j = 0; j < 3; ++j) {
    rtcm->sta.pos[j] = rr[j] * 0.0001;
    rtcm->sta.del[j] = 0.0;
  }
  rtcm->sta.hgt = 0.0;
  rtcm->sta.itrf = itrf;
  return 5;
}
/* decode type 1006: stationary rtk reference station arp with height --------*/
static int decode_type1006(rtcm_t* rtcm) {
  double rr[3], anth;
  char* msg;
  int i = 24 + 12, j, staid, itrf;

  if (i + 156 <= rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12;
    itrf = getbitu(rtcm->buff, i, 6);
    i += 6 + 4;
    rr[0] = getbits_38(rtcm->buff, i);
    i += 38 + 2;
    rr[1] = getbits_38(rtcm->buff, i);
    i += 38 + 2;
    rr[2] = getbits_38(rtcm->buff, i);
    i += 38;
    anth = getbitu(rtcm->buff, i, 16);
  } else {
    trace(2, "rtcm3 1006 length error: len=%d\n", rtcm->len);
    return -1;
  }
  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " staid=%4d", staid);
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  rtcm->sta.deltype = 1; /* xyz */
  for (j = 0; j < 3; ++j) {
    rtcm->sta.pos[j] = rr[j] * 0.0001;
    rtcm->sta.del[j] = 0.0;
  }
  rtcm->sta.hgt = anth * 0.0001;
  rtcm->sta.itrf = itrf;
  return 5;
}
/* decode type 1007: antenna descriptor --------------------------------------*/
static int decode_type1007(rtcm_t* rtcm) {
  char des[32] = "";
  char* msg;
  int i = 24 + 12, j, staid, n, setup;

  n = getbitu(rtcm->buff, i + 12, 8);

  if (i + 28 + 8 * n <= rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12 + 8;
    for (j = 0; j < n && j < 31; ++j) {
      des[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
    setup = getbitu(rtcm->buff, i, 8);
  } else {
    trace(2, "rtcm3 1007 length error: len=%d\n", rtcm->len);
    return -1;
  }
  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " staid=%4d", staid);
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  strncpy(rtcm->sta.antdes, des, n);
  rtcm->sta.antdes[n] = '\0';
  rtcm->sta.antsetup = setup;
  rtcm->sta.antsno[0] = '\0';
  return 5;
}
/* decode type 1008: antenna descriptor & serial number ----------------------*/
static int decode_type1008(rtcm_t* rtcm) {
  char des[32] = "", sno[32] = "";
  char* msg;
  int i = 24 + 12, j, staid, n, m, setup;

  n = getbitu(rtcm->buff, i + 12, 8);
  m = getbitu(rtcm->buff, i + 28 + 8 * n, 8);

  if (i + 36 + 8 * (n + m) <= rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12 + 8;
    for (j = 0; j < n && j < 31; ++j) {
      des[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
    setup = getbitu(rtcm->buff, i, 8);
    i += 8 + 8;
    for (j = 0; j < m && j < 31; ++j) {
      sno[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
  } else {
    trace(2, "rtcm3 1008 length error: len=%d\n", rtcm->len);
    return -1;
  }
  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " staid=%4d", staid);
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  strncpy(rtcm->sta.antdes, des, n);
  rtcm->sta.antdes[n] = '\0';
  rtcm->sta.antsetup = setup;
  strncpy(rtcm->sta.antsno, sno, m);
  rtcm->sta.antsno[m] = '\0';
  return 5;
}
/* decode type 1009-1012 message header --------------------------------------*/
static int decode_head1009(rtcm_t* rtcm, int* sync) {
  double tod;
  char* msg;
  int i = 24, staid, nsat, type;

  type = getbitu(rtcm->buff, i, 12);
  i += 12;

  if (i + 49 <= rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12;
    tod = getbitu(rtcm->buff, i, 27) * 0.001;
    i += 27; /* sec in a day */
    *sync = getbitu(rtcm->buff, i, 1);
    i += 1;
    nsat = getbitu(rtcm->buff, i, 5);
  } else {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  adjday_glot(rtcm, tod);

  trace(4, "decode_head1009: time=%s nsat=%d sync=%d\n",
        time_str(rtcm->time, 2), nsat, *sync);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " %s nsat=%2d sync=%d", time_str(rtcm->time, 2), nsat, *sync);
  }
  return nsat;
}
/* decode type 1009: L1-only glonass rtk observables -------------------------*/
static int decode_type1009(rtcm_t* rtcm) {
  int sync;
  if (decode_head1009(rtcm, &sync) < 0) {
    return -1;
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode type 1010: extended L1-only glonass rtk observables ----------------*/
static int decode_type1010(rtcm_t* rtcm) {
  double pr1, cnr1, tt, cp1, lam1;
  int i = 24 + 61, j, index, nsat, sync, prn, sat, code, freq, ppr1, lock1, amb,
      sys = SYS_GLO;

  if ((nsat = decode_head1009(rtcm, &sync)) < 0) {
    return -1;
  }

  for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 79 <= rtcm->len * 8;
       ++j) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    code = getbitu(rtcm->buff, i, 1);
    i += 1;
    freq = getbitu(rtcm->buff, i, 5);
    i += 5;
    pr1 = getbitu(rtcm->buff, i, 25);
    i += 25;
    ppr1 = getbits(rtcm->buff, i, 20);
    i += 20;
    lock1 = getbitu(rtcm->buff, i, 7);
    i += 7;
    amb = getbitu(rtcm->buff, i, 7);
    i += 7;
    cnr1 = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 1010 satellite number error: prn=%d\n", prn);
      continue;
    }
    tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    if ((index = obsindex(&rtcm->obs, rtcm->time, sat)) < 0) {
      continue;
    }
    pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
    if (ppr1 != (int)0xFFF80000) {
      rtcm->obs.data[index].P[0] = pr1;
      lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
      cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam1);
      rtcm->obs.data[index].L[0] = pr1 / lam1 + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code ? CODE_L1P : CODE_L1C;
  }
  return sync ? 0 : 1;
}
/* decode type 1011: L1&L2 glonass rtk observables ---------------------------*/
static int decode_type1011(rtcm_t* rtcm) {
  int sync;
  if (decode_head1009(rtcm, &sync) < 0) {
    return -1;
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode type 1012: extended L1&L2 glonass rtk observables ------------------*/
static int decode_type1012(rtcm_t* rtcm) {
  double pr1, cnr1, cnr2, tt, cp1, cp2, lam1, lam2;
  int i = 24 + 61, j, index, nsat, sync, prn, sat, freq, code1, code2, pr21,
      ppr1, ppr2;
  int lock1, lock2, amb, sys = SYS_GLO;

  if ((nsat = decode_head1009(rtcm, &sync)) < 0) {
    return -1;
  }

  for (j = 0; j < nsat && rtcm->obs.n < MAXOBS && i + 130 <= rtcm->len * 8;
       ++j) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    code1 = getbitu(rtcm->buff, i, 1);
    i += 1;
    freq = getbitu(rtcm->buff, i, 5);
    i += 5;
    pr1 = getbitu(rtcm->buff, i, 25);
    i += 25;
    ppr1 = getbits(rtcm->buff, i, 20);
    i += 20;
    lock1 = getbitu(rtcm->buff, i, 7);
    i += 7;
    amb = getbitu(rtcm->buff, i, 7);
    i += 7;
    cnr1 = getbitu(rtcm->buff, i, 8);
    i += 8;
    code2 = getbitu(rtcm->buff, i, 2);
    i += 2;
    pr21 = getbits(rtcm->buff, i, 14);
    i += 14;
    ppr2 = getbits(rtcm->buff, i, 20);
    i += 20;
    lock2 = getbitu(rtcm->buff, i, 7);
    i += 7;
    cnr2 = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 1012 satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    tt = timediff(rtcm->obs.data[0].time, rtcm->time);
    if (rtcm->obsflag || fabs(tt) > 1E-9) {
      rtcm->obs.n = rtcm->obsflag = 0;
    }
    if ((index = obsindex(&rtcm->obs, rtcm->time, sat)) < 0) {
      continue;
    }
    pr1 = pr1 * 0.02 + amb * PRUNIT_GLO;
    if (ppr1 != (int)0xFFF80000) {
      lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
      rtcm->obs.data[index].P[0] = pr1;
      cp1 = adjcp(rtcm, sat, 0, ppr1 * 0.0005 / lam1);
      rtcm->obs.data[index].L[0] = pr1 / lam1 + cp1;
    }
    rtcm->obs.data[index].LLI[0] = lossoflock(rtcm, sat, 0, lock1);
    rtcm->obs.data[index].SNR[0] = snratio(cnr1 * 0.25);
    rtcm->obs.data[index].code[0] = code1 ? CODE_L1P : CODE_L1C;

    if (pr21 != (int)0xFFFFE000) {
      rtcm->obs.data[index].P[1] = pr1 + pr21 * 0.02;
    }
    if (ppr2 != (int)0xFFF80000) {
      lam2 = CLIGHT / (FREQ2_GLO + DFRQ2_GLO * (freq - 7));
      cp2 = adjcp(rtcm, sat, 1, ppr2 * 0.0005 / lam2);
      rtcm->obs.data[index].L[1] = pr1 / lam2 + cp2;
    }
    rtcm->obs.data[index].LLI[1] = lossoflock(rtcm, sat, 1, lock2);
    rtcm->obs.data[index].SNR[1] = snratio(cnr2 * 0.25);
    rtcm->obs.data[index].code[1] = code2 ? CODE_L2P : CODE_L2C;
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode type 1013: system parameters ---------------------------------------*/
static int decode_type1013(rtcm_t* rtcm) { return 0; }
/* decode type 1019: gps ephemerides -----------------------------------------*/
static int decode_type1019(rtcm_t* rtcm) {
  eph_t eph = {0};
  double toc, sqrtA;
  char* msg;
  int i = 24 + 12, prn, sat, week, sys = SYS_GPS;

  if (i + 476 <= rtcm->len * 8) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    week = getbitu(rtcm->buff, i, 10);
    i += 10;
    eph.sva = getbitu(rtcm->buff, i, 4);
    i += 4;
    eph.code = getbitu(rtcm->buff, i, 2);
    i += 2;
    eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
    i += 14;
    eph.iode = getbitu(rtcm->buff, i, 8);
    i += 8;
    toc = getbitu(rtcm->buff, i, 16) * 16.0;
    i += 16;
    eph.f2 = getbits(rtcm->buff, i, 8) * P2_55;
    i += 8;
    eph.f1 = getbits(rtcm->buff, i, 16) * P2_43;
    i += 16;
    eph.f0 = getbits(rtcm->buff, i, 22) * P2_31;
    i += 22;
    eph.iodc = getbitu(rtcm->buff, i, 10);
    i += 10;
    eph.crs = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
    i += 16;
    eph.M0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.e = getbitu(rtcm->buff, i, 32) * P2_33;
    i += 32;
    eph.cus = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    sqrtA = getbitu(rtcm->buff, i, 32) * P2_19;
    i += 32;
    eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;
    i += 16;
    eph.cic = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.OMG0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cis = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.crc = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.OMGd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
    i += 24;
    eph.tgd[0] = getbits(rtcm->buff, i, 8) * P2_31;
    i += 8;
    eph.svh = getbitu(rtcm->buff, i, 6);
    i += 6;
    eph.flag = getbitu(rtcm->buff, i, 1);
    i += 1;
    eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */
  } else {
    trace(2, "rtcm3 1019 length error: len=%d\n", rtcm->len);
    return -1;
  }
  if (prn >= 40) {
    sys = SYS_SBS;
    prn += 80;
  }
  trace(4, "decode_type1019: prn=%d iode=%d toe=%.0f\n", prn, eph.iode,
        eph.toes);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg,
            " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
            prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
  }
  if (!(sat = satno(sys, prn))) {
    trace(2, "rtcm3 1019 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjgpsweek(week);
  eph.tocs = toc;
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1].iode) {
      return 0; /* unchanged */
    }
  }
  rtcm->nav.eph[sat - 1] = eph;
  rtcm->ephsat = sat;
  return 2;
}
/* decode type 1020: glonass ephemerides -------------------------------------*/
static int decode_type1020(rtcm_t* rtcm) {
  geph_t geph = {0};
  double tk_h, tk_m, tk_s, toe, tow, tod, tof;
  char* msg;
  int i = 24 + 12, prn, sat, week, tb, bn, sys = SYS_GLO;

  if (i + 348 <= rtcm->len * 8) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    geph.frq = getbitu(rtcm->buff, i, 5) - 7;
    i += 5 + 2 + 2;
    tk_h = getbitu(rtcm->buff, i, 5);
    i += 5;
    tk_m = getbitu(rtcm->buff, i, 6);
    i += 6;
    tk_s = getbitu(rtcm->buff, i, 1) * 30.0;
    i += 1;
    bn = getbitu(rtcm->buff, i, 1);
    i += 1 + 1;
    tb = getbitu(rtcm->buff, i, 7);
    i += 7;
    geph.vel[0] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3;
    i += 24;
    geph.pos[0] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3;
    i += 27;
    geph.acc[0] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3;
    i += 5;
    geph.vel[1] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3;
    i += 24;
    geph.pos[1] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3;
    i += 27;
    geph.acc[1] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3;
    i += 5;
    geph.vel[2] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3;
    i += 24;
    geph.pos[2] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3;
    i += 27;
    geph.acc[2] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3;
    i += 5 + 1;
    geph.gamn = getbitg(rtcm->buff, i, 11) * P2_40;
    i += 11 + 3;
    geph.taun = getbitg(rtcm->buff, i, 22) * P2_30;
  } else {
    trace(2, "rtcm3 1020 length error: len=%d\n", rtcm->len);
    return -1;
  }
  if (!(sat = satno(sys, prn))) {
    trace(2, "rtcm3 1020 satellite number error: prn=%d\n", prn);
    return -1;
  }
  trace(4, "decode_type1020: prn=%d tk=%02.0f:%02.0f:%02.0f\n", prn, tk_h, tk_m,
        tk_s);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " prn=%2d tk=%02.0f:%02.0f:%02.0f frq=%2d bn=%d tb=%d", prn,
            tk_h, tk_m, tk_s, geph.frq, bn, tb);
  }
  geph.sat = sat;
  geph.svh = bn;
  geph.iode = tb & 0x7F;
  if (rtcm->time.time == 0) {
    rtcm->time = utc2gpst(timeget());
  }
  tow = time2gpst(gpst2utc(rtcm->time), &week);
  tod = fmod(tow, 86400.0);
  tow -= tod;
  tof = tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0; /* lt->utc */
  if (tof < tod - 43200.0) {
    tof += 86400.0;
  } else if (tof > tod + 43200.0) {
    tof -= 86400.0;
  }
  geph.tof = utc2gpst(gpst2time(week, tow + tof));
  toe = tb * 900.0 - 10800.0; /* lt->utc */
  if (toe < tod - 43200.0) {
    toe += 86400.0;
  } else if (toe > tod + 43200.0) {
    toe -= 86400.0;
  }
  geph.toe = utc2gpst(gpst2time(week, tow + toe)); /* utc->gpst */

  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (fabs(timediff(geph.toe, rtcm->nav.geph[sat - 1].toe)) < 1.0 &&
        geph.svh == rtcm->nav.geph[sat - 1].svh) {
      return 0; /* unchanged */
    }
  }
  rtcm->nav.geph[sat - 1] = geph;
  rtcm->ephsat = sat;
  return 2;
}
/* decode type 1021: helmert/abridged molodenski -----------------------------*/
static int decode_type1021(rtcm_t* rtcm) {
  trace(2, "rtcm3 1021: not supported message\n");
  return 0;
}
/* decode type 1022: moledenski-badekas transfromation -----------------------*/
static int decode_type1022(rtcm_t* rtcm) {
  trace(2, "rtcm3 1022: not supported message\n");
  return 0;
}
/* decode type 1023: residual, ellipoidal grid representation ----------------*/
static int decode_type1023(rtcm_t* rtcm) {
  trace(2, "rtcm3 1023: not supported message\n");
  return 0;
}
/* decode type 1024: residual, plane grid representation ---------------------*/
static int decode_type1024(rtcm_t* rtcm) {
  trace(2, "rtcm3 1024: not supported message\n");
  return 0;
}
/* decode type 1025: projection (types except LCC2SP,OM) ---------------------*/
static int decode_type1025(rtcm_t* rtcm) {
  trace(2, "rtcm3 1025: not supported message\n");
  return 0;
}
/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -----*/
static int decode_type1026(rtcm_t* rtcm) {
  trace(2, "rtcm3 1026: not supported message\n");
  return 0;
}
/* decode type 1027: projection (type OM - oblique mercator) -----------------*/
static int decode_type1027(rtcm_t* rtcm) {
  trace(2, "rtcm3 1027: not supported message\n");
  return 0;
}
/* decode type 1030: network rtk residual ------------------------------------*/
static int decode_type1030(rtcm_t* rtcm) {
  trace(2, "rtcm3 1030: not supported message\n");
  return 0;
}
/* decode type 1031: glonass network rtk residual ----------------------------*/
static int decode_type1031(rtcm_t* rtcm) {
  trace(2, "rtcm3 1031: not supported message\n");
  return 0;
}
/* decode type 1032: physical reference station position information ---------*/
static int decode_type1032(rtcm_t* rtcm) {
  trace(2, "rtcm3 1032: not supported message\n");
  return 0;
}
/* decode type 1033: receiver and antenna descriptor -------------------------*/
static int decode_type1033(rtcm_t* rtcm) {
  char des[32] = "", sno[32] = "", rec[32] = "", ver[32] = "", rsn[32] = "";
  char* msg;
  int i = 24 + 12, j, staid, n, m, n1, n2, n3, setup;

  n = getbitu(rtcm->buff, i + 12, 8);
  m = getbitu(rtcm->buff, i + 28 + 8 * n, 8);
  n1 = getbitu(rtcm->buff, i + 36 + 8 * (n + m), 8);
  n2 = getbitu(rtcm->buff, i + 44 + 8 * (n + m + n1), 8);
  n3 = getbitu(rtcm->buff, i + 52 + 8 * (n + m + n1 + n2), 8);

  if (i + 60 + 8 * (n + m + n1 + n2 + n3) <= rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12 + 8;
    for (j = 0; j < n && j < 31; ++j) {
      des[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
    setup = getbitu(rtcm->buff, i, 8);
    i += 8 + 8;
    for (j = 0; j < m && j < 31; ++j) {
      sno[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
    i += 8;
    for (j = 0; j < n1 && j < 31; ++j) {
      rec[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
    i += 8;
    for (j = 0; j < n2 && j < 31; ++j) {
      ver[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
    i += 8;
    for (j = 0; j < n3 && j < 31; ++j) {
      rsn[j] = (char)getbitu(rtcm->buff, i, 8);
      i += 8;
    }
  } else {
    trace(2, "rtcm3 1033 length error: len=%d\n", rtcm->len);
    return -1;
  }
  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " staid=%4d", staid);
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  strncpy(rtcm->sta.antdes, des, n);
  rtcm->sta.antdes[n] = '\0';
  rtcm->sta.antsetup = setup;
  strncpy(rtcm->sta.antsno, sno, m);
  rtcm->sta.antsno[m] = '\0';
  strncpy(rtcm->sta.rectype, rec, n1);
  rtcm->sta.rectype[n1] = '\0';
  strncpy(rtcm->sta.recver, ver, n2);
  rtcm->sta.recver[n2] = '\0';
  strncpy(rtcm->sta.recsno, rsn, n3);
  rtcm->sta.recsno[n3] = '\0';

  trace(3, "rtcm3 1033: ant=%s:%s rec=%s:%s:%s\n", des, sno, rec, ver, rsn);
  return 5;
}
/* decode type 1034: gps network fkp gradient --------------------------------*/
static int decode_type1034(rtcm_t* rtcm) {
  trace(2, "rtcm3 1034: not supported message\n");
  return 0;
}
/* decode type 1035: glonass network fkp gradient ----------------------------*/
static int decode_type1035(rtcm_t* rtcm) {
  trace(2, "rtcm3 1035: not supported message\n");
  return 0;
}
/* decode type 1037: glonass network rtk ionospheric correction difference ---*/
static int decode_type1037(rtcm_t* rtcm) {
  trace(2, "rtcm3 1037: not supported message\n");
  return 0;
}
/* decode type 1038: glonass network rtk geometic correction difference ------*/
static int decode_type1038(rtcm_t* rtcm) {
  trace(2, "rtcm3 1038: not supported message\n");
  return 0;
}
/* decode type 1039: glonass network rtk combined correction difference ------*/
static int decode_type1039(rtcm_t* rtcm) {
  trace(2, "rtcm3 1039: not supported message\n");
  return 0;
}
/* decode type 1044: qzss ephemerides (ref [15]) -----------------------------*/
static int decode_type1044(rtcm_t* rtcm) {
  eph_t eph = {0};
  double toc, sqrtA;
  char* msg;
  int i = 24 + 12, prn, sat, week, sys = SYS_QZS;

  if (i + 473 <= rtcm->len * 8) {
    prn = getbitu(rtcm->buff, i, 4) + 192;
    i += 4;
    toc = getbitu(rtcm->buff, i, 16) * 16.0;
    i += 16;
    eph.f2 = getbits(rtcm->buff, i, 8) * P2_55;
    i += 8;
    eph.f1 = getbits(rtcm->buff, i, 16) * P2_43;
    i += 16;
    eph.f0 = getbits(rtcm->buff, i, 22) * P2_31;
    i += 22;
    eph.iode = getbitu(rtcm->buff, i, 8);
    i += 8;
    eph.crs = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
    i += 16;
    eph.M0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.e = getbitu(rtcm->buff, i, 32) * P2_33;
    i += 32;
    eph.cus = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    sqrtA = getbitu(rtcm->buff, i, 32) * P2_19;
    i += 32;
    eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;
    i += 16;
    eph.cic = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.OMG0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cis = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.crc = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.OMGd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
    i += 24;
    eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
    i += 14;
    eph.code = getbitu(rtcm->buff, i, 2);
    i += 2;
    week = getbitu(rtcm->buff, i, 10);
    i += 10;
    eph.sva = getbitu(rtcm->buff, i, 4);
    i += 4;
    eph.svh = getbitu(rtcm->buff, i, 6);
    i += 6;
    eph.tgd[0] = getbits(rtcm->buff, i, 8) * P2_31;
    i += 8;
    eph.iodc = getbitu(rtcm->buff, i, 10);
    i += 10;
    eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 2.0; /* 0:2hr,1:>2hr */
  } else {
    trace(2, "rtcm3 1044 length error: len=%d\n", rtcm->len);
    return -1;
  }
  trace(4, "decode_type1044: prn=%d iode=%d toe=%.0f\n", prn, eph.iode,
        eph.toes);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg,
            " prn=%3d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
            prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
  }
  if (!(sat = satno(sys, prn))) {
    trace(2, "rtcm3 1044 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjgpsweek(week);
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1].iode &&
        eph.iodc == rtcm->nav.eph[sat - 1].iodc) {
      return 0; /* unchanged */
    }
  }
  rtcm->nav.eph[sat - 1] = eph;
  rtcm->ephsat = sat;
  return 2;
}
/* decode type 1045: galileo satellite ephemerides (ref [15]) ----------------*/
static int decode_type1045(rtcm_t* rtcm) {
  eph_t eph = {0};
  double toc, sqrtA;
  char* msg;
  //int i = 24 + 12, prn, sat, week, e5a_hs, e5a_dvs, rsv, sys = SYS_GAL;
  int i = 24 + 12, prn, sat, week, e5a_hs, e5a_dvs, sys = SYS_GAL;

  if (i + 484 <= rtcm->len * 8) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    week = getbitu(rtcm->buff, i, 12);
    i += 12;
    eph.iode = getbitu(rtcm->buff, i, 10);
    i += 10;
    eph.sva = getbitu(rtcm->buff, i, 8);
    i += 8;
    eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
    i += 14;
    toc = getbitu(rtcm->buff, i, 14) * 60.0;
    i += 14;
    eph.f2 = getbits(rtcm->buff, i, 6) * P2_59;
    i += 6;
    eph.f1 = getbits(rtcm->buff, i, 21) * P2_46;
    i += 21;
    eph.f0 = getbits(rtcm->buff, i, 31) * P2_34;
    i += 31;
    eph.crs = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
    i += 16;
    eph.M0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.e = getbitu(rtcm->buff, i, 32) * P2_33;
    i += 32;
    eph.cus = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    sqrtA = getbitu(rtcm->buff, i, 32) * P2_19;
    i += 32;
    eph.toes = getbitu(rtcm->buff, i, 14) * 60.0;
    i += 14;
    eph.cic = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.OMG0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cis = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.crc = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.OMGd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
    i += 24;
    eph.tgd[0] = getbits(rtcm->buff, i, 10) * P2_32;
    i += 10; /* E5a/E1 */
    e5a_hs = getbitu(rtcm->buff, i, 2);
    i += 2; /* OSHS */
    e5a_dvs = getbitu(rtcm->buff, i, 1);
    i += 1; /* OSDVS */
    //rsv = getbitu(rtcm->buff, i, 7);
    getbitu(rtcm->buff, i, 7);
  } else {
    trace(2, "rtcm3 1045 length error: len=%d\n", rtcm->len);
    return -1;
  }
  trace(4, "decode_type1045: prn=%d iode=%d toe=%.0f\n", prn, eph.iode,
        eph.toes);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d",
            prn, eph.iode, week, eph.toes, toc, e5a_hs, e5a_dvs);
  }
  if (!(sat = satno(sys, prn))) {
    trace(2, "rtcm3 1045 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjgpsweek(week % 1024);
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = gpst2time(eph.week, toc);
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  eph.svh = (e5a_hs << 4) + (e5a_dvs << 3);
  eph.code = 2; /* data source = f/nav e5a */
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1].iode) {
      return 0; /* unchanged */
    }
  }
  rtcm->nav.eph[sat - 1] = eph;
  rtcm->ephsat = sat;
  return 2;
}
/* decode type 1047: beidou ephemerides (tentative mt and format) ------------*/
static int decode_type1047(rtcm_t* rtcm) {
  eph_t eph = {0};
  double toc, sqrtA;
  char* msg;
  int i = 24 + 12, prn, sat, week, sys = SYS_CMP;

  if (i + 476 <= rtcm->len * 8) {
    prn = getbitu(rtcm->buff, i, 6);
    i += 6;
    week = getbitu(rtcm->buff, i, 10);
    i += 10;
    eph.sva = getbitu(rtcm->buff, i, 4);
    i += 4;
    eph.code = getbitu(rtcm->buff, i, 2);
    i += 2;
    eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD;
    i += 14;
    eph.iode = getbitu(rtcm->buff, i, 8);
    i += 8;
    toc = getbitu(rtcm->buff, i, 16) * 16.0;
    i += 16;
    eph.f2 = getbits(rtcm->buff, i, 8) * P2_55;
    i += 8;
    eph.f1 = getbits(rtcm->buff, i, 16) * P2_43;
    i += 16;
    eph.f0 = getbits(rtcm->buff, i, 22) * P2_31;
    i += 22;
    eph.iodc = getbitu(rtcm->buff, i, 10);
    i += 10;
    eph.crs = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD;
    i += 16;
    eph.M0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.e = getbitu(rtcm->buff, i, 32) * P2_33;
    i += 32;
    eph.cus = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    sqrtA = getbitu(rtcm->buff, i, 32) * P2_19;
    i += 32;
    eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;
    i += 16;
    eph.cic = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.OMG0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.cis = getbits(rtcm->buff, i, 16) * P2_29;
    i += 16;
    eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.crc = getbits(rtcm->buff, i, 16) * P2_5;
    i += 16;
    eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD;
    i += 32;
    eph.OMGd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD;
    i += 24;
    eph.tgd[0] = getbits(rtcm->buff, i, 8) * P2_31;
    i += 8;
    eph.svh = getbitu(rtcm->buff, i, 6);
    i += 6;
    eph.flag = getbitu(rtcm->buff, i, 1);
    i += 1;
    eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */
  } else {
    trace(2, "rtcm3 1047 length error: len=%d\n", rtcm->len);
    return -1;
  }
  trace(4, "decode_type1047: prn=%d iode=%d toe=%.0f\n", prn, eph.iode,
        eph.toes);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg,
            " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
            prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
  }
  if (!(sat = satno(sys, prn))) {
    trace(2, "rtcm3 1047 satellite number error: prn=%d\n", prn);
    return -1;
  }
  eph.sat = sat;
  eph.week = adjbdtweek(week);
  eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
  eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
  eph.ttr = rtcm->time;
  eph.A = sqrtA * sqrtA;
  if (!strstr(rtcm->opt, "-EPHALL")) {
    if (eph.iode == rtcm->nav.eph[sat - 1].iode) {
      return 0; /* unchanged */
    }
  }
  rtcm->nav.eph[sat - 1] = eph;
  rtcm->ephsat = sat;
  return 2;
}
/* decode ssr 1,4 message header ---------------------------------------------*/
static int decode_ssr1_head(rtcm_t* rtcm, int sys, int* sync, int* iod,
                            double* udint, int* refd, int* hsize) {
  double tod, tow;
  char* msg;
  int i = 24 + 12, nsat, udi, provid = 0, solid = 0, ns = 6;

#ifndef SSR_QZSS_DRAFT_V05
  ns = sys == SYS_QZS ? 4 : 6;
#endif
  if (i + (sys == SYS_GLO ? 53 : 50 + ns) > rtcm->len * 8) {
    return -1;
  }

  if (sys == SYS_GLO) {
    tod = getbitu(rtcm->buff, i, 17);
    i += 17;
    adjday_glot(rtcm, tod);
  } else {
    tow = getbitu(rtcm->buff, i, 20);
    i += 20;
    adjweek(rtcm, tow);
  }
  udi = getbitu(rtcm->buff, i, 4);
  i += 4;
  *sync = getbitu(rtcm->buff, i, 1);
  i += 1;
  *refd = getbitu(rtcm->buff, i, 1);
  i += 1; /* satellite ref datum */
  *iod = getbitu(rtcm->buff, i, 4);
  i += 4; /* iod */
  provid = getbitu(rtcm->buff, i, 16);
  i += 16; /* provider id */
  solid = getbitu(rtcm->buff, i, 4);
  i += 4; /* solution id */
  nsat = getbitu(rtcm->buff, i, ns);
  i += ns;
  *udint = ssrudint[udi];

  trace(4,
        "decode_ssr1_head: time=%s sys=%d nsat=%d sync=%d iod=%d provid=%d "
        "solid=%d\n",
        time_str(rtcm->time, 2), sys, nsat, *sync, *iod, provid, solid);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " %s nsat=%2d iod=%2d udi=%2d sync=%d",
            time_str(rtcm->time, 2), nsat, *iod, udi, *sync);
  }
  *hsize = i;
  return nsat;
}
/* decode ssr 2,3,5,6 message header -----------------------------------------*/
static int decode_ssr2_head(rtcm_t* rtcm, int sys, int* sync, int* iod,
                            double* udint, int* hsize) {
  double tod, tow;
  char* msg;
  int i = 24 + 12, nsat, udi, provid = 0, solid = 0, ns = 6;

#ifndef SSR_QZSS_DRAFT_V05
  ns = sys == SYS_QZS ? 4 : 6;
#endif
  if (i + (sys == SYS_GLO ? 52 : 49 + ns) > rtcm->len * 8) {
    return -1;
  }

  if (sys == SYS_GLO) {
    tod = getbitu(rtcm->buff, i, 17);
    i += 17;
    adjday_glot(rtcm, tod);
  } else {
    tow = getbitu(rtcm->buff, i, 20);
    i += 20;
    adjweek(rtcm, tow);
  }
  udi = getbitu(rtcm->buff, i, 4);
  i += 4;
  *sync = getbitu(rtcm->buff, i, 1);
  i += 1;
  *iod = getbitu(rtcm->buff, i, 4);
  i += 4;
  provid = getbitu(rtcm->buff, i, 16);
  i += 16; /* provider id */
  solid = getbitu(rtcm->buff, i, 4);
  i += 4; /* solution id */
  nsat = getbitu(rtcm->buff, i, ns);
  i += ns;
  *udint = ssrudint[udi];

  trace(4,
        "decode_ssr2_head: time=%s sys=%d nsat=%d sync=%d iod=%d provid=%d "
        "solid=%d\n",
        time_str(rtcm->time, 2), sys, nsat, *sync, *iod, provid, solid);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " %s nsat=%2d iod=%2d udi=%2d sync=%d",
            time_str(rtcm->time, 2), nsat, *iod, udi, *sync);
  }
  *hsize = i;
  return nsat;
}
/* decode ssr 1: orbit corrections -------------------------------------------*/
static int decode_ssr1(rtcm_t* rtcm, int sys) {
  double udint, deph[3], ddeph[3];
  int i, j, k, type, sync, iod, nsat, prn, sat, iode, iodcrc, refd = 0, np, ni,
                                                              nj, offp;

  type = getbitu(rtcm->buff, 24, 12);

  if ((nsat = decode_ssr1_head(rtcm, sys, &sync, &iod, &udint, &refd, &i)) <
      0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  switch (sys) {
    case SYS_GPS:
      np = 6;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      ni = 10;
      nj = 0;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      ni = 8;
      nj = 0;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      ni = 10;
      nj = 24;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      ni = 9;
      nj = 24;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  for (j = 0; j < nsat && i + 121 + np + ni + nj <= rtcm->len * 8; ++j) {
    prn = getbitu(rtcm->buff, i, np) + offp;
    i += np;
    iode = getbitu(rtcm->buff, i, ni);
    i += ni;
    iodcrc = getbitu(rtcm->buff, i, nj);
    i += nj;
    deph[0] = getbits(rtcm->buff, i, 22) * 1E-4;
    i += 22;
    deph[1] = getbits(rtcm->buff, i, 20) * 4E-4;
    i += 20;
    deph[2] = getbits(rtcm->buff, i, 20) * 4E-4;
    i += 20;
    ddeph[0] = getbits(rtcm->buff, i, 21) * 1E-6;
    i += 21;
    ddeph[1] = getbits(rtcm->buff, i, 19) * 4E-6;
    i += 19;
    ddeph[2] = getbits(rtcm->buff, i, 19) * 4E-6;
    i += 19;

    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[0] = rtcm->time;
    rtcm->ssr[sat - 1].udi[0] = udint;
    rtcm->ssr[sat - 1].iod[0] = iod;
    rtcm->ssr[sat - 1].iode = iode;     /* sbas/bds: toe/t0 modulo */
    rtcm->ssr[sat - 1].iodcrc = iodcrc; /* sbas/bds: iod crc */
    rtcm->ssr[sat - 1].refd = refd;

    for (k = 0; k < 3; ++k) {
      rtcm->ssr[sat - 1].deph[k] = deph[k];
      rtcm->ssr[sat - 1].ddeph[k] = ddeph[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* decode ssr 2: clock corrections -------------------------------------------*/
static int decode_ssr2(rtcm_t* rtcm, int sys) {
  double udint, dclk[3];
  int i, j, k, type, sync, iod, nsat, prn, sat, np, offp;

  type = getbitu(rtcm->buff, 24, 12);

  if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  for (j = 0; j < nsat && i + 70 + np <= rtcm->len * 8; ++j) {
    prn = getbitu(rtcm->buff, i, np) + offp;
    i += np;
    dclk[0] = getbits(rtcm->buff, i, 22) * 1E-4;
    i += 22;
    dclk[1] = getbits(rtcm->buff, i, 21) * 1E-6;
    i += 21;
    dclk[2] = getbits(rtcm->buff, i, 27) * 2E-8;
    i += 27;

    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[1] = rtcm->time;
    rtcm->ssr[sat - 1].udi[1] = udint;
    rtcm->ssr[sat - 1].iod[1] = iod;

    for (k = 0; k < 3; ++k) {
      rtcm->ssr[sat - 1].dclk[k] = dclk[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* decode ssr 3: satellite code biases ---------------------------------------*/
static int decode_ssr3(rtcm_t* rtcm, int sys) {
  const int codes_gps[] = {CODE_L1C, CODE_L1P, CODE_L1W, CODE_L1Y, CODE_L1M,
                           CODE_L2C, CODE_L2D, CODE_L2S, CODE_L2L, CODE_L2X,
                           CODE_L2P, CODE_L2W, CODE_L2Y, CODE_L2M, CODE_L5I,
                           CODE_L5Q, CODE_L5X};
  const int codes_glo[] = {CODE_L1C, CODE_L1P, CODE_L2C, CODE_L2P};
  const int codes_gal[] = {CODE_L1A, CODE_L1B, CODE_L1C, CODE_L1X, CODE_L1Z,
                           CODE_L5I, CODE_L5Q, CODE_L5X, CODE_L7I, CODE_L7Q,
                           CODE_L7X, CODE_L8I, CODE_L8Q, CODE_L8X, CODE_L6A,
                           CODE_L6B, CODE_L6C, CODE_L6X, CODE_L6Z};
  const int codes_qzs[] = {CODE_L1C, CODE_L1S, CODE_L1L, CODE_L2S, CODE_L2L,
                           CODE_L2X, CODE_L5I, CODE_L5Q, CODE_L5X, CODE_L6S,
                           CODE_L6L, CODE_L6X, CODE_L1X};
  const int codes_bds[] = {CODE_L1I, CODE_L1Q, CODE_L1X, CODE_L7I, CODE_L7Q,
                           CODE_L7X, CODE_L6I, CODE_L6Q, CODE_L6X};
  const int codes_sbs[] = {CODE_L1C, CODE_L5I, CODE_L5Q, CODE_L5X};
  const int* codes;
  double udint, bias, cbias[MAXCODE];
  int i, j, k, type, mode, sync, iod, nsat, prn, sat, nbias, np, offp, ncode;

  type = getbitu(rtcm->buff, 24, 12);

  if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      codes = codes_gps;
      ncode = 16;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      codes = codes_glo;
      ncode = 3;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      codes = codes_gal;
      ncode = 18;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      codes = codes_qzs;
      ncode = 12;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      codes = codes_bds;
      ncode = 8;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      codes = codes_sbs;
      ncode = 3;
      break;
    default:
      return sync ? 0 : 10;
  }
  for (j = 0; j < nsat && i + 5 + np <= rtcm->len * 8; ++j) {
    prn = getbitu(rtcm->buff, i, np) + offp;
    i += np;
    nbias = getbitu(rtcm->buff, i, 5);
    i += 5;

    for (k = 0; k < MAXCODE; ++k) {
      cbias[k] = 0.0;
    }
    for (k = 0; k < nbias && i + 19 < rtcm->len * 8; ++k) {
      mode = getbitu(rtcm->buff, i, 5);
      i += 5;
      bias = getbits(rtcm->buff, i, 14) * 0.01;
      i += 14;
      if (mode <= ncode) {
        cbias[codes[mode] - 1] = (float)bias;
      } else {
        trace(2, "rtcm3 %d not supported mode: mode=%d\n", type, mode);
      }
    }
    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[4] = rtcm->time;
    rtcm->ssr[sat - 1].udi[4] = udint;
    rtcm->ssr[sat - 1].iod[4] = iod;

    for (k = 0; k < MAXCODE; ++k) {
      rtcm->ssr[sat - 1].cbias[k] = (float)cbias[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* decode ssr 4: combined orbit and clock corrections ------------------------*/
static int decode_ssr4(rtcm_t* rtcm, int sys) {
  double udint, deph[3], ddeph[3], dclk[3];
  int i, j, k, type, nsat, sync, iod, prn, sat, iode, iodcrc, refd = 0, np, ni,
                                                              nj, offp;

  type = getbitu(rtcm->buff, 24, 12);

  if ((nsat = decode_ssr1_head(rtcm, sys, &sync, &iod, &udint, &refd, &i)) <
      0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  switch (sys) {
    case SYS_GPS:
      np = 6;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      ni = 8;
      nj = 0;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      ni = 10;
      nj = 0;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      ni = 8;
      nj = 0;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      ni = 10;
      nj = 24;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      ni = 9;
      nj = 24;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  for (j = 0; j < nsat && i + 191 + np + ni + nj <= rtcm->len * 8; ++j) {
    prn = getbitu(rtcm->buff, i, np) + offp;
    i += np;
    iode = getbitu(rtcm->buff, i, ni);
    i += ni;
    iodcrc = getbitu(rtcm->buff, i, nj);
    i += nj;
    deph[0] = getbits(rtcm->buff, i, 22) * 1E-4;
    i += 22;
    deph[1] = getbits(rtcm->buff, i, 20) * 4E-4;
    i += 20;
    deph[2] = getbits(rtcm->buff, i, 20) * 4E-4;
    i += 20;
    ddeph[0] = getbits(rtcm->buff, i, 21) * 1E-6;
    i += 21;
    ddeph[1] = getbits(rtcm->buff, i, 19) * 4E-6;
    i += 19;
    ddeph[2] = getbits(rtcm->buff, i, 19) * 4E-6;
    i += 19;

    dclk[0] = getbits(rtcm->buff, i, 22) * 1E-4;
    i += 22;
    dclk[1] = getbits(rtcm->buff, i, 21) * 1E-6;
    i += 21;
    dclk[2] = getbits(rtcm->buff, i, 27) * 2E-8;
    i += 27;

    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[0] = rtcm->ssr[sat - 1].t0[1] = rtcm->time;
    rtcm->ssr[sat - 1].udi[0] = rtcm->ssr[sat - 1].udi[1] = udint;
    rtcm->ssr[sat - 1].iod[0] = rtcm->ssr[sat - 1].iod[1] = iod;
    rtcm->ssr[sat - 1].iode = iode;
    rtcm->ssr[sat - 1].iodcrc = iodcrc;
    rtcm->ssr[sat - 1].refd = refd;

    for (k = 0; k < 3; ++k) {
      rtcm->ssr[sat - 1].deph[k] = deph[k];
      rtcm->ssr[sat - 1].ddeph[k] = ddeph[k];
      rtcm->ssr[sat - 1].dclk[k] = dclk[k];
    }
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* decode ssr 5: ura ---------------------------------------------------------*/
static int decode_ssr5(rtcm_t* rtcm, int sys) {
  double udint;
  int i, j, type, nsat, sync, iod, prn, sat, ura, np, offp;

  type = getbitu(rtcm->buff, 24, 12);

  if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  for (j = 0; j < nsat && i + 6 + np <= rtcm->len * 8; ++j) {
    prn = getbitu(rtcm->buff, i, np) + offp;
    i += np;
    ura = getbitu(rtcm->buff, i, 6);
    i += 6;

    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[3] = rtcm->time;
    rtcm->ssr[sat - 1].udi[3] = udint;
    rtcm->ssr[sat - 1].iod[3] = iod;
    rtcm->ssr[sat - 1].ura = ura;
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* decode ssr 6: high rate clock correction ----------------------------------*/
static int decode_ssr6(rtcm_t* rtcm, int sys) {
  double udint, hrclk;
  int i, j, type, nsat, sync, iod, prn, sat, np, offp;

  type = getbitu(rtcm->buff, 24, 12);

  if ((nsat = decode_ssr2_head(rtcm, sys, &sync, &iod, &udint, &i)) < 0) {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  switch (sys) {
    case SYS_GPS:
      np = 6;
      offp = 0;
      break;
    case SYS_GLO:
      np = 5;
      offp = 0;
      break;
    case SYS_GAL:
      np = 6;
      offp = 0;
      break;
    case SYS_QZS:
      np = 4;
      offp = 192;
      break;
    case SYS_CMP:
      np = 6;
      offp = 1;
      break;
    case SYS_SBS:
      np = 6;
      offp = 120;
      break;
    default:
      return sync ? 0 : 10;
  }
  for (j = 0; j < nsat && i + 22 + np <= rtcm->len * 8; ++j) {
    prn = getbitu(rtcm->buff, i, np) + offp;
    i += np;
    hrclk = getbits(rtcm->buff, i, 22) * 1E-4;
    i += 22;

    if (!(sat = satno(sys, prn))) {
      trace(2, "rtcm3 %d satellite number error: prn=%d\n", type, prn);
      continue;
    }
    rtcm->ssr[sat - 1].t0[2] = rtcm->time;
    rtcm->ssr[sat - 1].udi[2] = udint;
    rtcm->ssr[sat - 1].iod[2] = iod;
    rtcm->ssr[sat - 1].hrclk = hrclk;
    rtcm->ssr[sat - 1].update = 1;
  }
  return sync ? 0 : 10;
}
/* get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const unsigned char* code, const int* freq, int n,
                     const char* opt, int* ind) {
  int i, nex, pri, pri_h[8] = {0}, index[8] = {0}, ex[32] = {0};

  /* test code priority */
  for (i = 0; i < n; ++i) {
    if (!code[i]) {
      continue;
    }

    if (freq[i] > NFREQ) { /* save as extended signal if freq > NFREQ */
      ex[i] = 1;
      continue;
    }
    /* code priority */
    pri = getcodepri(sys, code[i], opt);

    /* select highest priority signal */
    if (pri > pri_h[freq[i] - 1]) {
      if (index[freq[i] - 1]) {
        ex[index[freq[i] - 1] - 1] = 1;
      }
      pri_h[freq[i] - 1] = pri;
      index[freq[i] - 1] = i + 1;
    } else {
      ex[i] = 1;
    }
  }
  /* signal index in obs data */
  for (i = nex = 0; i < n; ++i) {
    if (ex[i] == 0) {
      ind[i] = freq[i] - 1;
    } else if (nex < NEXOBS) {
      ind[i] = NFREQ + nex++;
    } else { /* no space in obs data */
      trace(2, "rtcm msm: no space in obs data sys=%d code=%d\n", sys, code[i]);
      ind[i] = -1;
    }
#if 0
    trace(2, "sig pos: sys=%d code=%d ex=%d ind=%d\n", sys, code[i], ex[i], ind[i]);
#endif
  }
}
/* save obs data in msm message ----------------------------------------------*/
static void save_msm_obs(rtcm_t* rtcm, int sys, msm_h_t* h, const double* r,
                         const double* pr, const double* cp, const double* rr,
                         const double* rrf, const double* cnr, const int* lock,
                         const int* ex, const int* half) {
  const char* sig[32];
  double tt, wl;
  unsigned char code[32];
  char *msm_type = "", *q = NULL;
  int i, j, k, type, prn, sat, fn, index = 0, freq[32], ind[32];

  type = getbitu(rtcm->buff, 24, 12);

  switch (sys) {
    case SYS_GPS:
      msm_type = q = rtcm->msmtype[0];
      break;
    case SYS_GLO:
      msm_type = q = rtcm->msmtype[1];
      break;
    case SYS_GAL:
      msm_type = q = rtcm->msmtype[2];
      break;
    case SYS_QZS:
      msm_type = q = rtcm->msmtype[3];
      break;
    case SYS_SBS:
      msm_type = q = rtcm->msmtype[4];
      break;
    case SYS_CMP:
      msm_type = q = rtcm->msmtype[5];
      break;
  }
  /* id to signal */
  for (i = 0; i < h->nsig; ++i) {
    switch (sys) {
      case SYS_GPS:
        sig[i] = msm_sig_gps[h->sigs[i] - 1];
        break;
      case SYS_GLO:
        sig[i] = msm_sig_glo[h->sigs[i] - 1];
        break;
      case SYS_GAL:
        sig[i] = msm_sig_gal[h->sigs[i] - 1];
        break;
      case SYS_QZS:
        sig[i] = msm_sig_qzs[h->sigs[i] - 1];
        break;
      case SYS_SBS:
        sig[i] = msm_sig_sbs[h->sigs[i] - 1];
        break;
      case SYS_CMP:
        sig[i] = msm_sig_cmp[h->sigs[i] - 1];
        break;
      default:
        sig[i] = "";
        break;
    }
    /* signal to rinex obs type */
    code[i] = obs2code(sig[i], freq + i);

    /* freqency index for beidou */
    if (sys == SYS_CMP) {
      if (freq[i] == 5) {
        freq[i] = 2; /* B2 */
      } else if (freq[i] == 4) {
        freq[i] = 3; /* B3 */
      }
    }
    if (code[i] != CODE_NONE) {
      if (q) {
        q += sprintf(q, "L%s%s", sig[i], i < h->nsig - 1 ? "," : "");
      }
    } else {
      if (q) {
        q += sprintf(q, "(%d)%s", h->sigs[i], i < h->nsig - 1 ? "," : "");
      }

      trace(2, "rtcm3 %d: unknown signal id=%2d\n", type, h->sigs[i]);
    }
  }
  trace(3, "rtcm3 %d: signals=%s\n", type, msm_type);

  /* get signal index */
  sigindex(sys, code, freq, h->nsig, rtcm->opt, ind);

  for (i = j = 0; i < h->nsat; ++i) {
    prn = h->sats[i];
    if (sys == SYS_QZS) {
      prn += MINPRNQZS - 1;
    } else if (sys == SYS_SBS) {
      prn += MINPRNSBS - 1;
    }

    if ((sat = satno(sys, prn))) {
      tt = timediff(rtcm->obs.data[0].time, rtcm->time);
      if (rtcm->obsflag || fabs(tt) > 1E-9) {
        rtcm->obs.n = rtcm->obsflag = 0;
      }
      index = obsindex(&rtcm->obs, rtcm->time, sat);
    } else {
      trace(2, "rtcm3 %d satellite error: prn=%d\n", type, prn);
    }
    for (k = 0; k < h->nsig; ++k) {
      if (!h->cellmask[k + i * h->nsig]) {
        continue;
      }

      if (sat && index >= 0 && ind[k] >= 0) {
        /* satellite carrier wave length */
        wl = satwavelen(sat, freq[k] - 1, &rtcm->nav);

        /* glonass wave length by extended info */
        if (sys == SYS_GLO && ex && ex[i] <= 13) {
          fn = ex[i] - 7;
          wl = CLIGHT / ((freq[k] == 2 ? FREQ2_GLO : FREQ1_GLO) +
                         (freq[k] == 2 ? DFRQ2_GLO : DFRQ1_GLO) * fn);
        }
        /* pseudorange (m) */
        if (r[i] != 0.0 && pr[j] > -1E12) {
          rtcm->obs.data[index].P[ind[k]] = r[i] + pr[j];
        }
        /* carrier-phase (cycle) */
        if (r[i] != 0.0 && cp[j] > -1E12 && wl > 0.0) {
          rtcm->obs.data[index].L[ind[k]] = (r[i] + cp[j]) / wl;
        }
        /* doppler (hz) */
        if (rr && rrf && rrf[j] > -1E12 && wl > 0.0) {
          rtcm->obs.data[index].D[ind[k]] = (float)(-(rr[i] + rrf[j]) / wl);
        }
        rtcm->obs.data[index].LLI[ind[k]] =
            lossoflock(rtcm, sat, ind[k], lock[j]) + (half[j] ? 3 : 0);
        rtcm->obs.data[index].SNR[ind[k]] = (unsigned char)(cnr[j] * 4.0);
        rtcm->obs.data[index].code[ind[k]] = code[k];
      }
      ++j;
    }
  }
}
/* decode type msm message header --------------------------------------------*/
static int decode_msm_head(rtcm_t* rtcm, int sys, int* sync, int* iod,
                           msm_h_t* h, int* hsize) {
  msm_h_t h0 = {0};
  double tow, tod;
  char* msg;
  //int i = 24, j, dow, mask, staid, type, ncell = 0;
  int i = 24, j, mask, staid, type, ncell = 0;

  type = getbitu(rtcm->buff, i, 12);
  i += 12;

  *h = h0;
  if (i + 157 <= rtcm->len * 8) {
    staid = getbitu(rtcm->buff, i, 12);
    i += 12;

    if (sys == SYS_GLO) {
      //dow = getbitu(rtcm->buff, i, 3);
      getbitu(rtcm->buff, i, 3);
      i += 3;
      tod = getbitu(rtcm->buff, i, 27) * 0.001;
      i += 27;
      adjday_glot(rtcm, tod);
    } else if (sys == SYS_CMP) {
      tow = getbitu(rtcm->buff, i, 30) * 0.001;
      i += 30;
      tow += 14.0; /* BDT -> GPST */
      adjweek(rtcm, tow);
    } else {
      tow = getbitu(rtcm->buff, i, 30) * 0.001;
      i += 30;
      adjweek(rtcm, tow);
    }
    *sync = getbitu(rtcm->buff, i, 1);
    i += 1;
    *iod = getbitu(rtcm->buff, i, 3);
    i += 3;
    h->time_s = getbitu(rtcm->buff, i, 7);
    i += 7;
    h->clk_str = getbitu(rtcm->buff, i, 2);
    i += 2;
    h->clk_ext = getbitu(rtcm->buff, i, 2);
    i += 2;
    h->smooth = getbitu(rtcm->buff, i, 1);
    i += 1;
    h->tint_s = getbitu(rtcm->buff, i, 3);
    i += 3;
    for (j = 1; j <= 64; ++j) {
      mask = getbitu(rtcm->buff, i, 1);
      i += 1;
      if (mask) {
        h->sats[h->nsat++] = j;
      }
    }
    for (j = 1; j <= 32; ++j) {
      mask = getbitu(rtcm->buff, i, 1);
      i += 1;
      if (mask) {
        h->sigs[h->nsig++] = j;
      }
    }
  } else {
    trace(2, "rtcm3 %d length error: len=%d\n", type, rtcm->len);
    return -1;
  }
  /* test station id */
  if (!test_staid(rtcm, staid)) {
    return -1;
  }

  if (h->nsat * h->nsig > 64) {
    trace(2, "rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n", type,
          h->nsat, h->nsig);
    return -1;
  }
  if (i + h->nsat * h->nsig > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: len=%d nsat=%d nsig=%d\n", type, rtcm->len,
          h->nsat, h->nsig);
    return -1;
  }
  for (j = 0; j < h->nsat * h->nsig; ++j) {
    h->cellmask[j] = getbitu(rtcm->buff, i, 1);
    i += 1;
    if (h->cellmask[j]) {
      ncell++;
    }
  }
  *hsize = i;

  trace(4,
        "decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d "
        "iod=%d ncell=%d\n",
        time_str(rtcm->time, 2), sys, staid, h->nsat, h->nsig, *sync, *iod,
        ncell);

  if (rtcm->outtype) {
    msg = rtcm->msgtype + strlen(rtcm->msgtype);
    sprintf(msg, " %s staid=%3d nsat=%2d nsig=%2d iod=%2d ncell=%2d sync=%d",
            time_str(rtcm->time, 2), staid, h->nsat, h->nsig, *iod, ncell,
            *sync);
  }
  return ncell;
}
/* decode unsupported msm message --------------------------------------------*/
static int decode_msm0(rtcm_t* rtcm, int sys) {
  msm_h_t h = {0};
  int i, sync, iod;
  if (decode_msm_head(rtcm, sys, &sync, &iod, &h, &i) < 0) {
    return -1;
  }
  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode msm 4: full pseudorange and phaserange plus cnr --------------------*/
static int decode_msm4(rtcm_t* rtcm, int sys) {
  msm_h_t h = {0};
  double r[64], pr[64], cp[64], cnr[64];
  int i, j, type, sync, iod, ncell, rng, rng_m, prv, cpv, lock[64], half[64];

  type = getbitu(rtcm->buff, 24, 12);

  /* decode msm header */
  if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) {
    return -1;
  }

  if (i + h.nsat * 18 + ncell * 48 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
          ncell, rtcm->len);
    return -1;
  }
  for (j = 0; j < h.nsat; ++j) {
    r[j] = 0.0;
  }
  for (j = 0; j < ncell; ++j) {
    pr[j] = cp[j] = -1E16;
  }

  /* decode satellite data */
  for (j = 0; j < h.nsat; ++j) { /* range */
    rng = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (rng != 255) {
      r[j] = rng * RANGE_MS;
    }
  }
  for (j = 0; j < h.nsat; ++j) {
    rng_m = getbitu(rtcm->buff, i, 10);
    i += 10;
    if (r[j] != 0.0) {
      r[j] += rng_m * P2_10 * RANGE_MS;
    }
  }
  /* decode signal data */
  for (j = 0; j < ncell; ++j) { /* pseudorange */
    prv = getbits(rtcm->buff, i, 15);
    i += 15;
    if (prv != -16384) {
      pr[j] = prv * P2_24 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* phaserange */
    cpv = getbits(rtcm->buff, i, 22);
    i += 22;
    if (cpv != -2097152) {
      cp[j] = cpv * P2_29 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* lock time */
    lock[j] = getbitu(rtcm->buff, i, 4);
    i += 4;
  }
  for (j = 0; j < ncell; ++j) { /* half-cycle ambiguity */
    half[j] = getbitu(rtcm->buff, i, 1);
    i += 1;
  }
  for (j = 0; j < ncell; ++j) { /* cnr */
    cnr[j] = getbitu(rtcm->buff, i, 6) * 1.0;
    i += 6;
  }
  /* save obs data in msm message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr --------*/
static int decode_msm5(rtcm_t* rtcm, int sys) {
  msm_h_t h = {0};
  double r[64], rr[64], pr[64], cp[64], rrf[64], cnr[64];
  int i, j, type, sync, iod, ncell, rng, rng_m, rate, prv, cpv, rrv, lock[64];
  int ex[64], half[64];

  type = getbitu(rtcm->buff, 24, 12);

  /* decode msm header */
  if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) {
    return -1;
  }

  if (i + h.nsat * 36 + ncell * 63 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
          ncell, rtcm->len);
    return -1;
  }
  for (j = 0; j < h.nsat; ++j) {
    r[j] = rr[j] = 0.0;
    ex[j] = 15;
  }
  for (j = 0; j < ncell; ++j) {
    pr[j] = cp[j] = rrf[j] = -1E16;
  }

  /* decode satellite data */
  for (j = 0; j < h.nsat; ++j) { /* range */
    rng = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (rng != 255) {
      r[j] = rng * RANGE_MS;
    }
  }
  for (j = 0; j < h.nsat; ++j) { /* extended info */
    ex[j] = getbitu(rtcm->buff, i, 4);
    i += 4;
  }
  for (j = 0; j < h.nsat; ++j) {
    rng_m = getbitu(rtcm->buff, i, 10);
    i += 10;
    if (r[j] != 0.0) {
      r[j] += rng_m * P2_10 * RANGE_MS;
    }
  }
  for (j = 0; j < h.nsat; ++j) { /* phaserangerate */
    rate = getbits(rtcm->buff, i, 14);
    i += 14;
    if (rate != -8192) {
      rr[j] = rate * 1.0;
    }
  }
  /* decode signal data */
  for (j = 0; j < ncell; ++j) { /* pseudorange */
    prv = getbits(rtcm->buff, i, 15);
    i += 15;
    if (prv != -16384) {
      pr[j] = prv * P2_24 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* phaserange */
    cpv = getbits(rtcm->buff, i, 22);
    i += 22;
    if (cpv != -2097152) {
      cp[j] = cpv * P2_29 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* lock time */
    lock[j] = getbitu(rtcm->buff, i, 4);
    i += 4;
  }
  for (j = 0; j < ncell; ++j) { /* half-cycle ambiguity */
    half[j] = getbitu(rtcm->buff, i, 1);
    i += 1;
  }
  for (j = 0; j < ncell; ++j) { /* cnr */
    cnr[j] = getbitu(rtcm->buff, i, 6) * 1.0;
    i += 6;
  }
  for (j = 0; j < ncell; ++j) { /* phaserangerate */
    rrv = getbits(rtcm->buff, i, 15);
    i += 15;
    if (rrv != -16384) {
      rrf[j] = rrv * 0.0001;
    }
  }
  /* save obs data in msm message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ---------*/
static int decode_msm6(rtcm_t* rtcm, int sys) {
  msm_h_t h = {0};
  double r[64], pr[64], cp[64], cnr[64];
  int i, j, type, sync, iod, ncell, rng, rng_m, prv, cpv, lock[64], half[64];

  type = getbitu(rtcm->buff, 24, 12);

  /* decode msm header */
  if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) {
    return -1;
  }

  if (i + h.nsat * 18 + ncell * 65 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
          ncell, rtcm->len);
    return -1;
  }
  for (j = 0; j < h.nsat; ++j) {
    r[j] = 0.0;
  }
  for (j = 0; j < ncell; ++j) {
    pr[j] = cp[j] = -1E16;
  }

  /* decode satellite data */
  for (j = 0; j < h.nsat; ++j) { /* range */
    rng = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (rng != 255) {
      r[j] = rng * RANGE_MS;
    }
  }
  for (j = 0; j < h.nsat; ++j) {
    rng_m = getbitu(rtcm->buff, i, 10);
    i += 10;
    if (r[j] != 0.0) {
      r[j] += rng_m * P2_10 * RANGE_MS;
    }
  }
  /* decode signal data */
  for (j = 0; j < ncell; ++j) { /* pseudorange */
    prv = getbits(rtcm->buff, i, 20);
    i += 20;
    if (prv != -524288) {
      pr[j] = prv * P2_29 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* phaserange */
    cpv = getbits(rtcm->buff, i, 24);
    i += 24;
    if (cpv != -8388608) {
      cp[j] = cpv * P2_31 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* lock time */
    lock[j] = getbitu(rtcm->buff, i, 10);
    i += 10;
  }
  for (j = 0; j < ncell; ++j) { /* half-cycle ambiguity */
    half[j] = getbitu(rtcm->buff, i, 1);
    i += 1;
  }
  for (j = 0; j < ncell; ++j) { /* cnr */
    cnr[j] = getbitu(rtcm->buff, i, 10) * 0.0625;
    i += 10;
  }
  /* save obs data in msm message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) */
static int decode_msm7(rtcm_t* rtcm, int sys) {
  msm_h_t h = {0};
  double r[64], rr[64], pr[64], cp[64], rrf[64], cnr[64];
  int i, j, type, sync, iod, ncell, rng, rng_m, rate, prv, cpv, rrv, lock[64];
  int ex[64], half[64];

  type = getbitu(rtcm->buff, 24, 12);

  /* decode msm header */
  if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) {
    return -1;
  }

  if (i + h.nsat * 36 + ncell * 80 > rtcm->len * 8) {
    trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\n", type, h.nsat,
          ncell, rtcm->len);
    return -1;
  }
  for (j = 0; j < h.nsat; ++j) {
    r[j] = rr[j] = 0.0;
    ex[j] = 15;
  }
  for (j = 0; j < ncell; ++j) {
    pr[j] = cp[j] = rrf[j] = -1E16;
  }

  /* decode satellite data */
  for (j = 0; j < h.nsat; ++j) { /* range */
    rng = getbitu(rtcm->buff, i, 8);
    i += 8;
    if (rng != 255) {
      r[j] = rng * RANGE_MS;
    }
  }
  for (j = 0; j < h.nsat; ++j) { /* extended info */
    ex[j] = getbitu(rtcm->buff, i, 4);
    i += 4;
  }
  for (j = 0; j < h.nsat; ++j) {
    rng_m = getbitu(rtcm->buff, i, 10);
    i += 10;
    if (r[j] != 0.0) {
      r[j] += rng_m * P2_10 * RANGE_MS;
    }
  }
  for (j = 0; j < h.nsat; ++j) { /* phaserangerate */
    rate = getbits(rtcm->buff, i, 14);
    i += 14;
    if (rate != -8192) {
      rr[j] = rate * 1.0;
    }
  }
  /* decode signal data */
  for (j = 0; j < ncell; ++j) { /* pseudorange */
    prv = getbits(rtcm->buff, i, 20);
    i += 20;
    if (prv != -524288) {
      pr[j] = prv * P2_29 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* phaserange */
    cpv = getbits(rtcm->buff, i, 24);
    i += 24;
    if (cpv != -8388608) {
      cp[j] = cpv * P2_31 * RANGE_MS;
    }
  }
  for (j = 0; j < ncell; ++j) { /* lock time */
    lock[j] = getbitu(rtcm->buff, i, 10);
    i += 10;
  }
  for (j = 0; j < ncell; ++j) { /* half-cycle amiguity */
    half[j] = getbitu(rtcm->buff, i, 1);
    i += 1;
  }
  for (j = 0; j < ncell; ++j) { /* cnr */
    cnr[j] = getbitu(rtcm->buff, i, 10) * 0.0625;
    i += 10;
  }
  for (j = 0; j < ncell; ++j) { /* phaserangerate */
    rrv = getbits(rtcm->buff, i, 15);
    i += 15;
    if (rrv != -16384) {
      rrf[j] = rrv * 0.0001;
    }
  }
  /* save obs data in msm message */
  save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

  rtcm->obsflag = !sync;
  return sync ? 0 : 1;
}
/* decode type 1230: glonass L1 and L2 code-phase biases ---------------------*/
static int decode_type1230(rtcm_t* rtcm) {
  trace(2, "rtcm3 1230: not supported message\n");
  return 0;
}
/* decode rtcm ver.3 message -------------------------------------------------*/
extern int decode_rtcm3(rtcm_t* rtcm) {
  int ret = 0, type = getbitu(rtcm->buff, 24, 12);

  trace(3, "decode_rtcm3: len=%3d type=%d\n", rtcm->len, type);

  if (rtcm->outtype) {
    sprintf(rtcm->msgtype, "RTCM %4d (%4d):", type, rtcm->len);
  }

  rtcm->message_type = type;

  /* real-time input option */
  if (strstr(rtcm->opt, "-RT_INP")) {
    rtcm->time = utc2gpst(timeget());
  }
  switch (type) {
    case 1001:
      ret = decode_type1001(rtcm);
      break; /* not supported */
    case 1002:
      ret = decode_type1002(rtcm);
      break;
    case 1003:
      ret = decode_type1003(rtcm);
      break; /* not supported */
    case 1004:
      ret = decode_type1004(rtcm);
      break;
    case 1005:
      ret = decode_type1005(rtcm);
      break;
    case 1006:
      ret = decode_type1006(rtcm);
      break;
    case 1007:
      ret = decode_type1007(rtcm);
      break;
    case 1008:
      ret = decode_type1008(rtcm);
      break;
    case 1009:
      ret = decode_type1009(rtcm);
      break; /* not supported */
    case 1010:
      ret = decode_type1010(rtcm);
      break;
    case 1011:
      ret = decode_type1011(rtcm);
      break; /* not supported */
    case 1012:
      ret = decode_type1012(rtcm);
      break;
    case 1013:
      ret = decode_type1013(rtcm);
      break; /* not supported */
    case 1019:
      ret = decode_type1019(rtcm);
      break;
    case 1020:
      ret = decode_type1020(rtcm);
      break;
    case 1021:
      ret = decode_type1021(rtcm);
      break; /* not supported */
    case 1022:
      ret = decode_type1022(rtcm);
      break; /* not supported */
    case 1023:
      ret = decode_type1023(rtcm);
      break; /* not supported */
    case 1024:
      ret = decode_type1024(rtcm);
      break; /* not supported */
    case 1025:
      ret = decode_type1025(rtcm);
      break; /* not supported */
    case 1026:
      ret = decode_type1026(rtcm);
      break; /* not supported */
    case 1027:
      ret = decode_type1027(rtcm);
      break; /* not supported */
    case 1030:
      ret = decode_type1030(rtcm);
      break; /* not supported */
    case 1031:
      ret = decode_type1031(rtcm);
      break; /* not supported */
    case 1032:
      ret = decode_type1032(rtcm);
      break; /* not supported */
    case 1033:
      ret = decode_type1033(rtcm);
      break;
    case 1034:
      ret = decode_type1034(rtcm);
      break; /* not supported */
    case 1035:
      ret = decode_type1035(rtcm);
      break; /* not supported */
    case 1037:
      ret = decode_type1037(rtcm);
      break; /* not supported */
    case 1038:
      ret = decode_type1038(rtcm);
      break; /* not supported */
    case 1039:
      ret = decode_type1039(rtcm);
      break; /* not supported */
    case 1044:
      ret = decode_type1044(rtcm);
      break;
    case 1045:
      ret = decode_type1045(rtcm);
      break;
    case 1047:
      ret = decode_type1047(rtcm);
      break; /* tentative mt */
    case 1057:
      ret = decode_ssr1(rtcm, SYS_GPS);
      break;
    case 1058:
      ret = decode_ssr2(rtcm, SYS_GPS);
      break;
    case 1059:
      ret = decode_ssr3(rtcm, SYS_GPS);
      break;
    case 1060:
      ret = decode_ssr4(rtcm, SYS_GPS);
      break;
    case 1061:
      ret = decode_ssr5(rtcm, SYS_GPS);
      break;
    case 1062:
      ret = decode_ssr6(rtcm, SYS_GPS);
      break;
    case 1063:
      ret = decode_ssr1(rtcm, SYS_GLO);
      break;
    case 1064:
      ret = decode_ssr2(rtcm, SYS_GLO);
      break;
    case 1065:
      ret = decode_ssr3(rtcm, SYS_GLO);
      break;
    case 1066:
      ret = decode_ssr4(rtcm, SYS_GLO);
      break;
    case 1067:
      ret = decode_ssr5(rtcm, SYS_GLO);
      break;
    case 1068:
      ret = decode_ssr6(rtcm, SYS_GLO);
      break;
    case 1071:
      ret = decode_msm0(rtcm, SYS_GPS);
      break; /* not supported */
    case 1072:
      ret = decode_msm0(rtcm, SYS_GPS);
      break; /* not supported */
    case 1073:
      ret = decode_msm0(rtcm, SYS_GPS);
      break; /* not supported */
    case 1074:
      ret = decode_msm4(rtcm, SYS_GPS);
      break;
    case 1075:
      ret = decode_msm5(rtcm, SYS_GPS);
      break;
    case 1076:
      ret = decode_msm6(rtcm, SYS_GPS);
      break;
    case 1077:
      ret = decode_msm7(rtcm, SYS_GPS);
      break;
    case 1081:
      ret = decode_msm0(rtcm, SYS_GLO);
      break; /* not supported */
    case 1082:
      ret = decode_msm0(rtcm, SYS_GLO);
      break; /* not supported */
    case 1083:
      ret = decode_msm0(rtcm, SYS_GLO);
      break; /* not supported */
    case 1084:
      ret = decode_msm4(rtcm, SYS_GLO);
      break;
    case 1085:
      ret = decode_msm5(rtcm, SYS_GLO);
      break;
    case 1086:
      ret = decode_msm6(rtcm, SYS_GLO);
      break;
    case 1087:
      ret = decode_msm7(rtcm, SYS_GLO);
      break;
    case 1091:
      ret = decode_msm0(rtcm, SYS_GAL);
      break; /* not supported */
    case 1092:
      ret = decode_msm0(rtcm, SYS_GAL);
      break; /* not supported */
    case 1093:
      ret = decode_msm0(rtcm, SYS_GAL);
      break; /* not supported */
    case 1094:
      ret = decode_msm4(rtcm, SYS_GAL);
      break;
    case 1095:
      ret = decode_msm5(rtcm, SYS_GAL);
      break;
    case 1096:
      ret = decode_msm6(rtcm, SYS_GAL);
      break;
    case 1097:
      ret = decode_msm7(rtcm, SYS_GAL);
      break;
    case 1101:
      ret = decode_msm0(rtcm, SYS_SBS);
      break; /* not supported */
    case 1102:
      ret = decode_msm0(rtcm, SYS_SBS);
      break; /* not supported */
    case 1103:
      ret = decode_msm0(rtcm, SYS_SBS);
      break; /* not supported */
    case 1104:
      ret = decode_msm4(rtcm, SYS_SBS);
      break;
    case 1105:
      ret = decode_msm5(rtcm, SYS_SBS);
      break;
    case 1106:
      ret = decode_msm6(rtcm, SYS_SBS);
      break;
    case 1107:
      ret = decode_msm7(rtcm, SYS_SBS);
      break;
    case 1111:
      ret = decode_msm0(rtcm, SYS_QZS);
      break; /* not supported */
    case 1112:
      ret = decode_msm0(rtcm, SYS_QZS);
      break; /* not supported */
    case 1113:
      ret = decode_msm0(rtcm, SYS_QZS);
      break; /* not supported */
    case 1114:
      ret = decode_msm4(rtcm, SYS_QZS);
      break;
    case 1115:
      ret = decode_msm5(rtcm, SYS_QZS);
      break;
    case 1116:
      ret = decode_msm6(rtcm, SYS_QZS);
      break;
    case 1117:
      ret = decode_msm7(rtcm, SYS_QZS);
      break;
    case 1121:
      ret = decode_msm0(rtcm, SYS_CMP);
      break; /* not supported */
    case 1122:
      ret = decode_msm0(rtcm, SYS_CMP);
      break; /* not supported */
    case 1123:
      ret = decode_msm0(rtcm, SYS_CMP);
      break; /* not supported */
    case 1124:
      ret = decode_msm4(rtcm, SYS_CMP);
      break;
    case 1125:
      ret = decode_msm5(rtcm, SYS_CMP);
      break;
    case 1126:
      ret = decode_msm6(rtcm, SYS_CMP);
      break;
    case 1127:
      ret = decode_msm7(rtcm, SYS_CMP);
      break;
    case 1230:
      ret = decode_type1230(rtcm);
      break; /* not supported */
    case 1240:
      ret = decode_ssr1(rtcm, SYS_GAL);
      break;
    case 1241:
      ret = decode_ssr2(rtcm, SYS_GAL);
      break;
    case 1242:
      ret = decode_ssr3(rtcm, SYS_GAL);
      break;
    case 1243:
      ret = decode_ssr4(rtcm, SYS_GAL);
      break;
    case 1244:
      ret = decode_ssr5(rtcm, SYS_GAL);
      break;
    case 1245:
      ret = decode_ssr6(rtcm, SYS_GAL);
      break;
    case 1246:
      ret = decode_ssr1(rtcm, SYS_QZS);
      break;
    case 1247:
      ret = decode_ssr2(rtcm, SYS_QZS);
      break;
    case 1248:
      ret = decode_ssr3(rtcm, SYS_QZS);
      break;
    case 1249:
      ret = decode_ssr4(rtcm, SYS_QZS);
      break;
    case 1250:
      ret = decode_ssr5(rtcm, SYS_QZS);
      break;
    case 1251:
      ret = decode_ssr6(rtcm, SYS_QZS);
      break;
    case 1252:
      ret = decode_ssr1(rtcm, SYS_SBS);
      break;
    case 1253:
      ret = decode_ssr2(rtcm, SYS_SBS);
      break;
    case 1254:
      ret = decode_ssr3(rtcm, SYS_SBS);
      break;
    case 1255:
      ret = decode_ssr4(rtcm, SYS_SBS);
      break;
    case 1256:
      ret = decode_ssr5(rtcm, SYS_SBS);
      break;
    case 1257:
      ret = decode_ssr6(rtcm, SYS_SBS);
      break;
    case 1258:
      ret = decode_ssr1(rtcm, SYS_CMP);
      break;
    case 1259:
      ret = decode_ssr2(rtcm, SYS_CMP);
      break;
    case 1260:
      ret = decode_ssr3(rtcm, SYS_CMP);
      break;
    case 1261:
      ret = decode_ssr4(rtcm, SYS_CMP);
      break;
    case 1262:
      ret = decode_ssr5(rtcm, SYS_CMP);
      break;
    case 1263:
      ret = decode_ssr6(rtcm, SYS_CMP);
      break;
  }
  if (ret >= 0) {
    type -= 1000;
    if (1 <= type && type <= 299) {
      rtcm->nmsg3[type]++;
    } else {
      rtcm->nmsg3[0]++;
    }
  }
  return ret;
}
