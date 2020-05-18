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
 * notvatel.c : NovAtel OEM6/OEM5/OEM4/OEM3 receiver functions
 *
 *          Copyright (C) 2007-2014 by T.TAKASU, All rights reserved.
 *
 * reference :
 *     [1] NovAtel, OM-20000094 Rev6 OEMV Family Firmware Reference Manual, 2008
 *     [2] NovAtel, OM-20000053 Rev2 MiLLennium GPSCard Software Versions 4.503
 *         and 4.52 Command Descriptions Manual, 2001
 *     [3] NovAtel, OM-20000129 Rev2 OEM6 Family Firmware Reference Manual, 2011
 *     [4] NovAtel, OM-20000127 Rev1 OEMStar Firmware Reference Manual, 2009
 *     [5] NovAtel, OM-20000129 Rev6 OEM6 Family Firmware Reference Manual, 2014
 *
 * version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
 * history : 2007/10/08 1.0 new
 *           2008/05/09 1.1 fix bug lli flag outage
 *           2008/06/16 1.2 separate common functions to rcvcmn.c
 *           2009/04/01 1.3 add prn number check for raw obs data
 *           2009/04/10 1.4 refactored
 *                          add oem3, oem4 rangeb support
 *           2009/06/06 1.5 fix bug on numerical exception with illegal snr
 *                          support oem3 regd message
 *           2009/12/09 1.6 support oem4 gloephemerisb message
 *                          invalid if parity unknown in GLONASS range
 *                          fix bug of dopper polarity inversion for oem3 regd
 *           2010/04/29 1.7 add tod field in geph_t
 *           2011/05/27 1.8 support RAWALM for oem4/v
 *                          add almanac decoding
 *                          add -EPHALL option
 *                          fix problem on ARM compiler
 *           2012/05/02 1.9 support OEM6,L5,QZSS
 *           2012/10/18 1.10 change obs codes
 *                           support Galileo
 *                           support rawsbasframeb,galephemerisb,galalmanacb,
 *                           galclockb,galionob
 *           2012/11/08 1.11 support galfnavrawpageb, galinavrawword
 *           2012/11/19 1.12 fix bug on decodeing rangeb
 *           2013/02/23 1.13 fix memory access violation problem on arm
 *           2013/03/28 1.14 fix invalid phase if glonass wavelen unavailable
 *           2013/06/02 1.15 fix bug on reading galephemrisb,galalmanacb,
 *                           galclockb,galionob
 *                           fix bug on decoding rawwaasframeb for qzss-saif
 *           2014/05/24 1.16 support beidou
 *           2014/07/01 1.17 fix problem on decoding of bdsephemerisb
 *                           fix bug on beidou tracking codes
 *           2014/10/20 1.11 fix bug on receiver option -GL*,-RL*,-EL*
 *-----------------------------------------------------------------------------*/
/**
 * file: novatel.c
 * version: rtklib ver.2.4.2
 * Copy from
 * https://github.com/tomojitakasu/RTKLIB/tree/76b9c97257f304aedad38b5a6bbbac444724aab3/src/rcv/novatel.c
 */
#include <stdio.h>
#include "rtklib.h"

/* static const char rcsid[] =
    "$Id: novatel.c,v 1.2 2008/07/14 00:05:05 TTAKA Exp $"; */

#define OEM4SYNC1 0xAA /* oem4 message start sync code 1 */
#define OEM4SYNC2 0x44 /* oem4 message start sync code 2 */
#define OEM4SYNC3 0x12 /* oem4 message start sync code 3 */
#define OEM3SYNC1 0xAA /* oem3 message start sync code 1 */
#define OEM3SYNC2 0x44 /* oem3 message start sync code 2 */
#define OEM3SYNC3 0x11 /* oem3 message start sync code 3 */

#define OEM4HLEN 28 /* oem4 message header length (bytes) */
#define OEM3HLEN 12 /* oem3 message header length (bytes) */

#define ID_ALMANAC 73       /* message id: oem4 decoded almanac */
#define ID_GLOALMANAC 718   /* message id: oem4 glonass decoded almanac */
#define ID_GLOEPHEMERIS 723 /* message id: oem4 glonass ephemeris */
#define ID_IONUTC 8         /* message id: oem4 iono and utc data */
#define ID_RANGE 43         /* message id: oem4 range measurement */
#define ID_RANGECMP 140     /* message id: oem4 range compressed */
#define ID_RAWALM 74        /* message id: oem4 raw almanac */
#define ID_RAWEPHEM 41      /* message id: oem4 raw ephemeris */
#define ID_RAWWAASFRAME 287 /* message id: oem4 raw waas frame */

#define ID_QZSSIONUTC 1347      /* message id: oem6 qzss ion/utc parameters */
#define ID_QZSSRAWEPHEM 1330    /* message id: oem6 qzss raw ephemeris */
#define ID_QZSSRAWSUBFRAME 1331 /* message id: oem6 qzss raw subframe */
#define ID_RAWSBASFRAME 973     /* message id: oem6 raw sbas frame */
#define ID_GALEPHEMERIS 1122    /* message id: oem6 decoded galileo ephemeris */
#define ID_GALALMANAC 1120      /* message id: oem6 decoded galileo almanac */
#define ID_GALCLOCK 1121        /* message id: oem6 galileo clockinformation */
#define ID_GALIONO                                          \
  1127 /* message id: oem6 decoded galileo iono corrections \
        */
#define ID_GALFNAVRAWPAGE \
  1413 /* message id: oem6 raw galileo f/nav paga data */
#define ID_GALINAVRAWWORD \
  1414                       /* message id: oem6 raw galileo i/nav word data */
#define ID_RAWCNAVFRAME 1066 /* message id: oem6 raw cnav frame data */
#define ID_BDSEPHEMERIS 1696 /* message id: oem6 decoded bds ephemeris */

#define ID_ALMB 18 /* message id: oem3 decoded almanac */
#define ID_IONB 16 /* message id: oem3 iono parameters */
#define ID_UTCB 17 /* message id: oem3 utc parameters */
#define ID_FRMB 54 /* message id: oem3 framed raw navigation data */
#define ID_RALB 15 /* message id: oem3 raw almanac */
#define ID_RASB 66 /* message id: oem3 raw almanac set */
#define ID_REPB 14 /* message id: oem3 raw ephemeris */
#define ID_RGEB 32 /* message id: oem3 range measurement */
#define ID_RGED 65 /* message id: oem3 range compressed */

#define WL1 0.1902936727984
#define WL2 0.2442102134246
#define MAXVAL 8388608.0

#define OFF_FRQNO -7 /* F/W ver.3.620 */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char*)(p)))
#define I1(p) (*((char*)(p)))
static unsigned short U2(unsigned char* p) {
  unsigned short u = 0;
  memcpy(&u, p, 2);
  return u;
}
static unsigned int U4(unsigned char* p) {
  unsigned int u = 0;
  memcpy(&u, p, 4);
  return u;
}
static int I4(unsigned char* p) {
  int i = 0;
  memcpy(&i, p, 4);
  return i;
}
static float R4(unsigned char* p) {
  float r = 0.0;
  memcpy(&r, p, 4);
  return r;
}
static double R8(unsigned char* p) {
  double r = 0.0;
  memcpy(&r, p, 8);
  return r;
}

/* extend sign ---------------------------------------------------------------*/
static int exsign(unsigned int v, int bits) {
  return (int)(v & (1 << (bits - 1)) ? v | (~0u << bits) : v);
}
/* checksum ------------------------------------------------------------------*/
static unsigned char chksum(const unsigned char* buff, int len) {
  unsigned char sum = 0;
  int i = 0;
  for (i = 0; i < len; ++i) {
    sum ^= buff[i];
  }
  return sum;
}
/* adjust weekly rollover of gps time ----------------------------------------*/
static gtime_t adjweek(gtime_t time, double tow) {
  double tow_p = 0.0;
  int week = 0;
  tow_p = time2gpst(time, &week);
  if (tow < tow_p - 302400.0) {
    tow += 604800.0;
  } else if (tow > tow_p + 302400.0) {
    tow -= 604800.0;
  }
  return gpst2time(week, tow);
}
/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t* obs, gtime_t time, int sat) {
  int i = 0;
  int j = 0;

  if (obs->n >= MAXOBS) {
    return -1;
  }
  for (i = 0; i < obs->n; ++i) {
    if (obs->data[i].sat == sat) {
      return i;
    }
  }
  obs->data[i].time = time;
  obs->data[i].sat = sat;
  for (j = 0; j < NFREQ + NEXOBS; ++j) {
    obs->data[i].L[j] = obs->data[i].P[j] = 0.0;
    obs->data[i].D[j] = 0.0;
    obs->data[i].SNR[j] = obs->data[i].LLI[j] = 0;
    obs->data[i].code[j] = CODE_NONE;
  }
  obs->n++;
  return i;
}
/* ura value (m) to ura index ------------------------------------------------*/
static int uraindex(double value) {
  static const double ura_eph[] = {2.4,    3.4,    4.85,   6.85,  9.65,  13.65,
                                   24.0,   48.0,   96.0,   192.0, 384.0, 768.0,
                                   1536.0, 3072.0, 6144.0, 0.0};
  int i = 0;
  for (i = 0; i < 15; ++i) {
    if (ura_eph[i] >= value) {
      break;
    }
  }
  return i;
}
/* decode oem4 tracking status -------------------------------------------------
 * deocode oem4 tracking status
 * args   : unsigned int stat I  tracking status field
 *          int    *sys   O      system (SYS_???)
 *          int    *code  O      signal code (CODE_L??)
 *          int    *track O      tracking state
 *                         (oem4/5)
 *                         0=L1 idle                   8=L2 idle
 *                         1=L1 sky search             9=L2 p-code align
 *                         2=L1 wide freq pull-in     10=L2 search
 *                         3=L1 narrow freq pull-in   11=L2 pll
 *                         4=L1 pll                   12=L2 steering
 *                         5=L1 reacq
 *                         6=L1 steering
 *                         7=L1 fll
 *                         (oem6)
 *                         0=idle                      7=freq-lock loop
 *                         2=wide freq band pull-in    9=channel alignment
 *                         3=narrow freq band pull-in 10=code search
 *                         4=phase lock loop          11=aided phase lock loop
 *          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
 *          int    *clock O      code-lock flag    (0=not locked, 1=locked)
 *          int    *parity O     parity known flag (0=not known,  1=known)
 *          int    *halfc O      phase measurement (0=half-cycle not added,
 *                                                  1=added)
 * return : signal frequency (0:L1,1:L2,2:L5,3:L6,4:L7,5:L8,-1:error)
 * notes  : refer [1][3]
 *-----------------------------------------------------------------------------*/
static int decode_trackstat(unsigned int stat, int* sys, int* code, int* track,
                            int* plock, int* clock, int* parity, int* halfc) {
  int satsys = 0;
  int sigtype = 0;
  int freq = 0;

  *track = stat & 0x1F;
  *plock = (stat >> 10) & 1;
  *parity = (stat >> 11) & 1;
  *clock = (stat >> 12) & 1;
  satsys = (stat >> 16) & 7;
  *halfc = (stat >> 28) & 1;
  sigtype = (stat >> 21) & 0x1F;

  switch (satsys) {
    case 0:
      *sys = SYS_GPS;
      break;
    case 1:
      *sys = SYS_GLO;
      break;
    case 2:
      *sys = SYS_SBS;
      break;
    case 3:
      *sys = SYS_GAL;
      break; /* OEM6 */
    case 4:
      *sys = SYS_CMP;
      break; /* OEM6 F/W 6.400 */
    case 5:
      *sys = SYS_QZS;
      break; /* OEM6 */
    default:
      trace(2, "oem4 unknown system: sys=%d\n", satsys);
      return -1;
  }
  if (*sys == SYS_GPS || *sys == SYS_QZS) {
    switch (sigtype) {
      case 0:
        freq = 0;
        *code = CODE_L1C;
        break; /* L1C/A */
      case 5:
        freq = 0;
        *code = CODE_L1P;
        break; /* L1P */
      case 9:
        freq = 1;
        *code = CODE_L2D;
        break; /* L2Pcodeless */
      case 14:
        freq = 2;
        *code = CODE_L5Q;
        break; /* L5Q (OEM6) */
      case 17:
        freq = 1;
        *code = CODE_L2X;
        break; /* L2C(M+L) */
      default:
        freq = -1;
        break;
    }
  } else if (*sys == SYS_GLO) {
    switch (sigtype) {
      case 0:
        freq = 0;
        *code = CODE_L1C;
        break; /* L1C/A */
      case 1:
        freq = 1;
        *code = CODE_L2C;
        break; /* L2C/A (OEM6) */
      case 5:
        freq = 1;
        *code = CODE_L2P;
        break; /* L2P */
      default:
        freq = -1;
        break;
    }
  } else if (*sys == SYS_GAL) {
    switch (sigtype) {
      case 1:
        freq = 0;
        *code = CODE_L1B;
        break; /* E1B  (OEM6) */
      case 2:
        freq = 0;
        *code = CODE_L1C;
        break; /* E1C  (OEM6) */
      case 12:
        freq = 2;
        *code = CODE_L5Q;
        break; /* E5aQ (OEM6) */
      case 17:
        freq = 4;
        *code = CODE_L7Q;
        break; /* E5bQ (OEM6) */
      case 20:
        freq = 5;
        *code = CODE_L8Q;
        break; /* AltBOCQ (OEM6) */
      default:
        freq = -1;
        break;
    }
  } else if (*sys == SYS_CMP) {
    switch (sigtype) {
      case 0:
        freq = 0;
        *code = CODE_L1I;
        break; /* B1 with D1 (OEM6) */
      case 1:
        freq = 1;
        *code = CODE_L7I;
        break; /* B2 with D1 (OEM6) */
      case 4:
        freq = 0;
        *code = CODE_L1I;
        break; /* B1 with D2 (OEM6) */
      case 5:
        freq = 1;
        *code = CODE_L7I;
        break; /* B2 with D2 (OEM6) */
      default:
        freq = -1;
        break;
    }
  } else if (*sys == SYS_SBS) {
    switch (sigtype) {
      case 0:
        freq = 0;
        *code = CODE_L1C;
        break; /* L1C/A */
      case 6:
        freq = 2;
        *code = CODE_L5I;
        break; /* L5I (OEM6) */
      default:
        freq = -1;
        break;
    }
  }
  if (freq < 0) {
    trace(2, "oem4 signal type error: sys=%d sigtype=%d\n", *sys, sigtype);
    return -1;
  }
  return freq;
}
/* check code priority and return obs position -------------------------------*/
static int checkpri(const char* opt, int sys, int code, int freq) {
  int nex = NEXOBS; /* number of extended obs data */

  if (sys == SYS_GPS) {
    if (strstr(opt, "-GL1P") && freq == 0) {
      return code == CODE_L1P ? 0 : -1;
    }
    if (strstr(opt, "-GL2X") && freq == 1) {
      return code == CODE_L2X ? 1 : -1;
    }
    if (code == CODE_L1P) {
      return nex < 1 ? -1 : NFREQ;
    }
    if (code == CODE_L2X) {
      return nex < 2 ? -1 : NFREQ + 1;
    }
  } else if (sys == SYS_GLO) {
    if (strstr(opt, "-RL2C") && freq == 1) {
      return code == CODE_L2C ? 1 : -1;
    }
    if (code == CODE_L2C) {
      return nex < 1 ? -1 : NFREQ;
    }
  } else if (sys == SYS_GAL) {
    if (strstr(opt, "-EL1B") && freq == 0) {
      return code == CODE_L1B ? 0 : -1;
    }
    if (code == CODE_L1B) {
      return nex < 1 ? -1 : NFREQ;
    }
    if (code == CODE_L7Q) {
      return nex < 2 ? -1 : NFREQ + 1;
    }
    if (code == CODE_L8Q) {
      return nex < 3 ? -1 : NFREQ + 2;
    }
  }
  return freq < NFREQ ? freq : -1;
}
/* decode rangecmpb ----------------------------------------------------------*/
static int decode_rangecmpb(raw_t* raw) {
  double psr = 0.0;
  double adr = 0.0;
  double adr_rolls = 0.0;
  double lockt = 0.0;
  double tt = 0.0;
  double dop = 0.0;
  double snr = 0.0;
  double wavelen = 0.0;
  int i = 0;
  int index = 0;
  int nobs = 0;
  int prn = 0;
  int sat = 0;
  int sys = 0;
  int code = 0;
  int freq = 0;
  int pos = 0;

  int track = 0;
  int plock = 0;
  int clock = 0;
  int parity = 0;
  int halfc = 0;
  int lli = 0;
  char* msg = NULL;
  unsigned char* p = raw->buff + OEM4HLEN;

  trace(3, "decode_rangecmpb: len=%d\n", raw->len);

  nobs = U4(p);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " nobs=%2d", nobs);
  }
  if (raw->len < OEM4HLEN + 4 + nobs * 24) {
    trace(2, "oem4 rangecmpb length error: len=%d nobs=%d\n", raw->len, nobs);
    return -1;
  }
  for (i = 0, p += 4; i < nobs; ++i, p += 24) {
    /* decode tracking status */
    if ((freq = decode_trackstat(U4(p), &sys, &code, &track, &plock, &clock,
                                 &parity, &halfc)) < 0) {
      continue;
    }

    /* obs position */
    if ((pos = checkpri(raw->opt, sys, code, freq)) < 0) {
      continue;
    }

    prn = U1(p + 17);
    if (sys == SYS_GLO) {
      prn -= 37;
    }

    if (!(sat = satno(sys, prn))) {
      trace(3, "oem4 rangecmpb satellite number error: sys=%d,prn=%d\n", sys,
            prn);
      continue;
    }
    if (sys == SYS_GLO && !parity) {
      continue; /* invalid if GLO parity unknown */
    }

    dop = exsign(U4(p + 4) & 0xFFFFFFF, 28) / 256.0;
    psr = (U4(p + 7) >> 4) / 128.0 + U1(p + 11) * 2097152.0;

    if ((wavelen = satwavelen(sat, freq, &raw->nav)) <= 0.0) {
      if (sys == SYS_GLO) {
        wavelen = CLIGHT / (freq == 0 ? FREQ1_GLO : FREQ2_GLO);
      } else {
        wavelen = lam_carr[freq];
      }
    }
    adr = I4(p + 12) / 256.0;
    adr_rolls = (psr / wavelen + adr) / MAXVAL;
    adr = -adr + MAXVAL * floor(adr_rolls + (adr_rolls <= 0 ? -0.5 : 0.5));

    lockt = (U4(p + 18) & 0x1FFFFF) / 32.0; /* lock time */

    tt = timediff(raw->time, raw->tobs);
    if (raw->tobs.time != 0) {
      lli = (lockt < 65535.968 &&
             lockt - raw->lockt[sat - 1][pos] + 0.05 <= tt) ||
            halfc != raw->halfc[sat - 1][pos];
    } else {
      lli = 0;
    }
    if (!parity) {
      lli |= 2;
    }
    raw->lockt[sat - 1][pos] = lockt;
    raw->halfc[sat - 1][pos] = halfc;

    snr = ((U2(p + 20) & 0x3FF) >> 5) + 20.0;
    if (!clock) {
      psr = 0.0; /* code unlock */
    }
    if (!plock) {
      adr = dop = 0.0; /* phase unlock */
    }

    if (fabs(timediff(raw->obs.data[0].time, raw->time)) > 1E-9) {
      raw->obs.n = 0;
    }
    if ((index = obsindex(&raw->obs, raw->time, sat)) >= 0) {
      raw->obs.data[index].L[pos] = adr;
      raw->obs.data[index].P[pos] = psr;
      raw->obs.data[index].D[pos] = (float)dop;
      raw->obs.data[index].SNR[pos] =
          0.0 <= snr && snr < 255.0 ? (unsigned char)(snr * 4.0 + 0.5) : 0;
      raw->obs.data[index].LLI[pos] = (unsigned char)lli;
      raw->obs.data[index].code[pos] = code;
#if 0
      /* L2C phase shift correction (L2C->L2P) */
      if (code == CODE_L2X) {
        raw->obs.data[index].L[pos] += 0.25;
        trace(3, "oem4 L2C phase shift corrected: prn=%2d\n", prn);
      }
#endif
    }
  }
  raw->tobs = raw->time;
  return 1;
}
/* decode rangeb -------------------------------------------------------------*/
static int decode_rangeb(raw_t* raw) {
  double psr = 0.0;
  double adr = 0.0;
  double dop = 0.0;
  double snr = 0.0;
  double lockt = 0.0;
  double tt = 0.0;
  char* msg = NULL;
  int i = 0;
  int index = 0;
  int nobs = 0;
  int prn = 0;
  int sat = 0;
  int sys = 0;
  int code = 0;
  int freq = 0;
  int pos = 0;
  ;

  int track = 0;
  int plock = 0;
  int clock = 0;
  int parity = 0;
  int halfc = 0;
  int lli = 0;
  int gfrq = 0;
  ;
  unsigned char* p = raw->buff + OEM4HLEN;

  trace(3, "decode_rangeb: len=%d\n", raw->len);

  nobs = U4(p);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " nobs=%2d", nobs);
  }
  if (raw->len < OEM4HLEN + 4 + nobs * 44) {
    trace(2, "oem4 rangeb length error: len=%d nobs=%d\n", raw->len, nobs);
    return -1;
  }
  for (i = 0, p += 4; i < nobs; ++i, p += 44) {
    /* decode tracking status */
    if ((freq = decode_trackstat(U4(p + 40), &sys, &code, &track, &plock,
                                 &clock, &parity, &halfc)) < 0) {
      continue;
    }

    /* obs position */
    if ((pos = checkpri(raw->opt, sys, code, freq)) < 0) {
      continue;
    }

    prn = U2(p);
    if (sys == SYS_GLO) {
      prn -= 37;
    }

    if (!(sat = satno(sys, prn))) {
      trace(3, "oem4 rangeb satellite number error: sys=%d,prn=%d\n", sys, prn);
      continue;
    }
    if (sys == SYS_GLO && !parity) {
      continue; /* invalid if GLO parity unknown */
    }

    gfrq = U2(p + 2);
    psr = R8(p + 4);
    adr = R8(p + 16);
    dop = R4(p + 28);
    snr = R4(p + 32);
    lockt = R4(p + 36);

    /* set glonass frequency channel number */
    if (sys == SYS_GLO && raw->nav.geph[prn - 1].sat != sat) {
      raw->nav.geph[prn - 1].frq = gfrq - 7;
    }
    tt = timediff(raw->time, raw->tobs);
    if (raw->tobs.time != 0) {
      lli = lockt - raw->lockt[sat - 1][pos] + 0.05 <= tt ||
            halfc != raw->halfc[sat - 1][pos];
    } else {
      lli = 0;
    }
    if (!parity) {
      lli |= 2;
    }
    raw->lockt[sat - 1][pos] = lockt;
    raw->halfc[sat - 1][pos] = halfc;
    if (!clock) {
      psr = 0.0; /* code unlock */
    }
    if (!plock) {
      adr = dop = 0.0; /* phase unlock */
    }

    if (fabs(timediff(raw->obs.data[0].time, raw->time)) > 1E-9) {
      raw->obs.n = 0;
    }
    if ((index = obsindex(&raw->obs, raw->time, sat)) >= 0) {
      raw->obs.data[index].L[pos] = -adr;
      raw->obs.data[index].P[pos] = psr;
      raw->obs.data[index].D[pos] = (float)dop;
      raw->obs.data[index].SNR[pos] =
          0.0 <= snr && snr < 255.0 ? (unsigned char)(snr * 4.0 + 0.5) : 0;
      raw->obs.data[index].LLI[pos] = (unsigned char)lli;
      raw->obs.data[index].code[pos] = code;
#if 0
      /* L2C phase shift correction */
      if (code == CODE_L2X) {
        raw->obs.data[index].L[pos] += 0.25;
        trace(3, "oem4 L2C phase shift corrected: prn=%2d\n", prn);
      }
#endif
    }
  }
  raw->tobs = raw->time;
  return 1;
}
/* decode rawephemb ----------------------------------------------------------*/
static int decode_rawephemb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  eph_t eph = {0};
  int prn = 0;
  int sat = 0;

  trace(3, "decode_rawephemb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 102) {
    trace(2, "oem4 rawephemb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);
  if (!(sat = satno(SYS_GPS, prn))) {
    trace(2, "oem4 rawephemb satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (decode_frame(p + 12, &eph, NULL, NULL, NULL, NULL) != 1 ||
      decode_frame(p + 42, &eph, NULL, NULL, NULL, NULL) != 2 ||
      decode_frame(p + 72, &eph, NULL, NULL, NULL, NULL) != 3) {
    trace(2, "oem4 rawephemb subframe error: prn=%d\n", prn);
    return -1;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1].iode) {
      return 0; /* unchanged */
    }
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  trace(4, "decode_rawephemb: sat=%2d\n", sat);
  return 2;
}
///* decode ionutcb
///------------------------------------------------------------*/
static int decode_ionutcb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  int i = 0;

  trace(3, "decode_ionutcb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 108) {
    trace(2, "oem4 ionutcb length error: len=%d\n", raw->len);
    return -1;
  }
  for (i = 0; i < 8; ++i) {
    raw->nav.ion_gps[i] = R8(p + i * 8);
  }
  raw->nav.utc_gps[0] = R8(p + 72);
  raw->nav.utc_gps[1] = R8(p + 80);
  raw->nav.utc_gps[2] = U4(p + 68);
  raw->nav.utc_gps[3] = U4(p + 64);
  raw->nav.leaps = I4(p + 96);
  return 9;
}
///* decode rawwaasframeb
///------------------------------------------------------*/
static int decode_rawwaasframeb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  int i = 0;
  int prn = 0;

  trace(3, "decode_rawwaasframeb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 48) {
    trace(2, "oem4 rawwaasframeb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p + 4);

  if (MINPRNQZS_S <= prn && prn <= MAXPRNQZS_S) {
    prn += 10; /* QZSS SAIF PRN -> QZSS PRN */
  } else if (prn < MINPRNSBS || MAXPRNSBS < prn) {
    return 0;
  }

  raw->sbsmsg.tow = (int)time2gpst(raw->time, &raw->sbsmsg.week);
  raw->sbsmsg.prn = prn;
  for (i = 0, p += 12; i < 29; ++i, ++p) {
    raw->sbsmsg.msg[i] = *p;
  }
  return 3;
}
///* decode rawsbasframeb
///------------------------------------------------------*/
static int decode_rawsbasframeb(raw_t* raw) {
  trace(3, "decode_rawsbasframeb: len=%d\n", raw->len);

  /* format same as rawwaasframeb */
  return decode_rawwaasframeb(raw);
}
/* decode gloephemerisb ------------------------------------------------------*/
static int decode_gloephemerisb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  geph_t geph = {0};
  char* msg = NULL;
  double tow = 0.0;
  double tof = 0.0;
  double toff = 0.0;
  int prn = 0;
  int sat = 0;
  int week = 0;

  trace(3, "decode_gloephemerisb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 144) {
    trace(2, "oem4 gloephemerisb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U2(p) - 37;

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d", prn);
  }
  if (!(sat = satno(SYS_GLO, prn))) {
    trace(2, "oem4 gloephemerisb prn error: prn=%d\n", prn);
    return -1;
  }
  geph.frq = U2(p + 2) + OFF_FRQNO;
  week = U2(p + 6);
  tow = floor(U4(p + 8) / 1000.0 + 0.5); /* rounded to integer sec */
  toff = U4(p + 12);
  geph.iode = U4(p + 20) & 0x7F;
  geph.svh = U4(p + 24);
  geph.pos[0] = R8(p + 28);
  geph.pos[1] = R8(p + 36);
  geph.pos[2] = R8(p + 44);
  geph.vel[0] = R8(p + 52);
  geph.vel[1] = R8(p + 60);
  geph.vel[2] = R8(p + 68);
  geph.acc[0] = R8(p + 76);
  geph.acc[1] = R8(p + 84);
  geph.acc[2] = R8(p + 92);
  geph.taun = R8(p + 100);
  geph.gamn = R8(p + 116);
  tof = U4(p + 124) - toff; /* glonasst->gpst */
  geph.age = U4(p + 136);
  geph.toe = gpst2time(week, tow);
  tof += floor(tow / 86400.0) * 86400;
  if (tof < tow - 43200.0) {
    tof += 86400.0;
  } else if (tof > tow + 43200.0) {
    tof -= 86400.0;
  }
  geph.tof = gpst2time(week, tof);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (fabs(timediff(geph.toe, raw->nav.geph[prn - 1].toe)) < 1.0 &&
        geph.svh == raw->nav.geph[prn - 1].svh) {
      return 0; /* unchanged */
    }
  }
  geph.sat = sat;
  raw->nav.geph[prn - 1] = geph;
  raw->ephsat = sat;
  return 2;
}
///* decode qzss rawephemb
///-----------------------------------------------------*/
static int decode_qzssrawephemb(raw_t* raw) {
  unsigned char *p = raw->buff + OEM4HLEN, *q;
  eph_t eph = {0};
  char* msg = NULL;
  int i = 0;
  int prn = 0;
  int id = 0;
  int sat = 0;

  trace(3, "decode_qzssrawephemb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 44) {
    trace(2, "oem4 qzssrawephemb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);
  id = U4(p + 4);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d id=%d", prn, id);
  }
  if (!(sat = satno(SYS_QZS, prn))) {
    trace(2, "oem4 qzssrawephemb satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (id < 1 || 3 < id) {
    return 0;
  }

  q = raw->subfrm[sat - 1] + (id - 1) * 30;
  for (i = 0; i < 30; ++i) {
    *q++ = p[8 + i];
  }

  if (id < 3) {
    return 0;
  }
  if (decode_frame(raw->subfrm[sat - 1], &eph, NULL, NULL, NULL, NULL) != 1 ||
      decode_frame(raw->subfrm[sat - 1] + 30, &eph, NULL, NULL, NULL, NULL) !=
          2 ||
      decode_frame(raw->subfrm[sat - 1] + 60, &eph, NULL, NULL, NULL, NULL) !=
          3) {
    return 0;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iodc == raw->nav.eph[sat - 1].iodc &&
        eph.iode == raw->nav.eph[sat - 1].iode) {
      return 0; /* unchanged */
    }
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  trace(4, "decode_qzssrawephemb: sat=%2d\n", sat);
  return 2;
}
///* decode qzss rawsubframeb
///--------------------------------------------------*/
static int decode_qzssrawsubframeb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  eph_t eph = {0};
  char* msg = NULL;
  int prn = 0;
  int sat = 0;

  trace(3, "decode_qzssrawsubframeb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 44) {
    trace(2, "oem4 qzssrawsubframeb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d", prn);
  }
  if (!(sat = satno(SYS_QZS, prn))) {
    trace(2, "oem4 qzssrawephemb satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (decode_frame(p + 12, &eph, NULL, NULL, NULL, NULL) != 1 ||
      decode_frame(p + 42, &eph, NULL, NULL, NULL, NULL) != 2 ||
      decode_frame(p + 72, &eph, NULL, NULL, NULL, NULL) != 3) {
    return 0;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iodc == raw->nav.eph[sat - 1].iodc &&
        eph.iode == raw->nav.eph[sat - 1].iode) {
      return 0; /* unchanged */
    }
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  trace(4, "decode_qzssrawsubframeb: sat=%2d\n", sat);
  return 2;
}
///* decode qzssionutcb
///--------------------------------------------------------*/
static int decode_qzssionutcb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  int i = 0;

  trace(3, "decode_qzssionutcb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 108) {
    trace(2, "oem4 qzssionutcb length error: len=%d\n", raw->len);
    return -1;
  }
  for (i = 0; i < 8; ++i) {
    raw->nav.ion_qzs[i] = R8(p + i * 8);
  }
  raw->nav.utc_qzs[0] = R8(p + 72);
  raw->nav.utc_qzs[1] = R8(p + 80);
  raw->nav.utc_qzs[2] = U4(p + 68);
  raw->nav.utc_qzs[3] = U4(p + 64);
  raw->nav.leaps = I4(p + 96);
  return 9;
}
/* decode galephemerisb ------------------------------------------------------*/
static int decode_galephemerisb(raw_t* raw) {
  eph_t eph = {0};
  unsigned char* p = raw->buff + OEM4HLEN;
  double tow = 0.0;
  double sqrtA = 0.0;
  double af0_fnav = 0.0;
  double af1_fnav = 0.0;
  double af2_fnav = 0.0;
  double af0_inav = 0.0;
  double af1_inav = 0.0;
  double af2_inav = 0.0;
  double tt = 0.0;
  char* msg = NULL;
  int prn;
  int rcv_fnav = 0;
  int rcv_inav = 0;
  int svh_e1b = 0;
  int svh_e5a = 0;
  int svh_e5b = 0;
  int dvs_e1b = 0;
  int dvs_e5a = 0;
  int dvs_e5b = 0;

  int toc_fnav = 0;
  int toc_inav = 0;
  int week = 0;

  trace(3, "decode_galephemerisb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 220) {
    trace(2, "oem4 galephemrisb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);
  p += 4;
  rcv_fnav = U4(p) & 1;
  p += 4;
  rcv_inav = U4(p) & 1;
  (void)rcv_inav;
  p += 4;
  svh_e1b = U1(p) & 3;
  p += 1;
  svh_e5a = U1(p) & 3;
  p += 1;
  svh_e5b = U1(p) & 3;
  p += 1;
  dvs_e1b = U1(p) & 1;
  p += 1;
  dvs_e5a = U1(p) & 1;
  p += 1;
  dvs_e5b = U1(p) & 1;
  p += 1;
  eph.sva = U1(p);
  p += 1 + 1; /* SISA */
  eph.iode = U4(p);
  p += 4; /* IODNav */
  eph.toes = U4(p);
  p += 4;
  sqrtA = R8(p);
  p += 8;
  eph.deln = R8(p);
  p += 8;
  eph.M0 = R8(p);
  p += 8;
  eph.e = R8(p);
  p += 8;
  eph.omg = R8(p);
  p += 8;
  eph.cuc = R8(p);
  p += 8;
  eph.cus = R8(p);
  p += 8;
  eph.crc = R8(p);
  p += 8;
  eph.crs = R8(p);
  p += 8;
  eph.cic = R8(p);
  p += 8;
  eph.cis = R8(p);
  p += 8;
  eph.i0 = R8(p);
  p += 8;
  eph.idot = R8(p);
  p += 8;
  eph.OMG0 = R8(p);
  p += 8;
  eph.OMGd = R8(p);
  p += 8;
  toc_fnav = U4(p);
  p += 4;
  af0_fnav = R8(p);
  p += 8;
  af1_fnav = R8(p);
  p += 8;
  af2_fnav = R8(p);
  p += 8;
  toc_inav = U4(p);
  p += 4;
  af0_inav = R8(p);
  p += 8;
  af1_inav = R8(p);
  p += 8;
  af2_inav = R8(p);
  p += 8;
  eph.tgd[0] = R8(p);
  p += 8;             /* BGD: E5A-E1 (s) */
  eph.tgd[1] = R8(p); /* BGD: E5B-E1 (s) */
  eph.iodc = eph.iode;
  eph.svh = (svh_e5b << 7) | (dvs_e5b << 6) | (svh_e5a << 4) | (dvs_e5a << 3) |
            (svh_e1b << 1) | dvs_e1b;
  eph.code = rcv_fnav ? 1 : 0; /* 0:INAV,1:FNAV */
  eph.A = sqrtA * sqrtA;
  eph.f0 = rcv_fnav ? af0_fnav : af0_inav;
  eph.f1 = rcv_fnav ? af1_fnav : af1_inav;
  eph.f2 = rcv_fnav ? af2_fnav : af2_inav;

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d iod=%3d toes=%6.0f", prn, eph.iode, eph.toes);
  }
  if (!(eph.sat = satno(SYS_GAL, prn))) {
    trace(2, "oemv galephemeris satellite error: prn=%d\n", prn);
    return -1;
  }
  tow = time2gpst(raw->time, &week);
  eph.week = week; /* gps week */
  eph.toe = gpst2time(eph.week, eph.toes);

  /* for week-handover problem */
  tt = timediff(eph.toe, raw->time);
  if (tt < -302400.0) {
    eph.week++;
  } else if (tt > 302400.0) {
    eph.week--;
  }
  eph.toe = gpst2time(eph.week, eph.toes);
  eph.toc = adjweek(eph.toe, rcv_fnav ? toc_fnav : toc_inav);
  eph.ttr = adjweek(eph.toe, tow);

  if (!strstr(raw->opt, "-EPHALL")) {
    if (raw->nav.eph[eph.sat - 1].iode == eph.iode &&
        raw->nav.eph[eph.sat - 1].code == eph.code) {
      return 0; /* unchanged */
    }
  }
  raw->nav.eph[eph.sat - 1] = eph;
  raw->ephsat = eph.sat;
  return 2;
}
///* decode galalmanacb
///--------------------------------------------------------*/
static int decode_galalmanacb(raw_t* raw) {
  alm_t alm = {0};
  unsigned char* p = raw->buff + OEM4HLEN;
  double dsqrtA = 0.0;
  double sqrtA = sqrt(29601297.0);
  int prn = 0;
  int rcv_fnav = 0;
  int rcv_inav = 0;
  int svh_e1b = 0;
  int svh_e5a = 0;
  int svh_e5b = 0;
  int ioda = 0;

  trace(3, "decode_galalmanacb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 100) {
    trace(2, "oem4 galephemrisb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);
  p += 4;
  rcv_fnav = U4(p) & 1;
  (void)rcv_fnav; 
  p += 4;
  rcv_inav = U4(p) & 1;
  (void)rcv_inav;
  p += 4;
  svh_e1b = U1(p) & 3;
  p += 1;
  svh_e5a = U1(p) & 3;
  p += 1;
  svh_e5b = U1(p) & 3;
  p += 1 + 1;
  ioda = U4(p);
  (void)ioda;
  p += 4;
  alm.week = U4(p);
  p += 4; /* gst week */
  alm.toas = U4(p);
  p += 4;
  alm.e = R8(p);
  p += 8;
  alm.OMGd = R8(p);
  p += 8;
  alm.OMG0 = R8(p);
  p += 8;
  alm.omg = R8(p);
  p += 8;
  alm.M0 = R8(p);
  p += 8;
  alm.f0 = R8(p);
  p += 8;
  alm.f1 = R8(p);
  p += 8;
  dsqrtA = R8(p);
  p += 8;
  alm.i0 = (R8(p) + 56.0) * D2R;
  alm.svh = (svh_e5b << 7) | (svh_e5a << 4) | (svh_e1b << 1);
  alm.A = (sqrtA + dsqrtA) * (sqrtA + dsqrtA);

  if (!(alm.sat = satno(SYS_GAL, prn))) {
    trace(2, "oemv galalmanac satellite error: prn=%d\n", prn);
    return -1;
  }
  alm.toa = gst2time(alm.week, alm.toas);
  raw->nav.alm[alm.sat - 1] = alm;
  return 0;
}
///* decode galclockb
///----------------------------------------------------------*/
static int decode_galclockb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  double a0 = 0.0;
  double a1 = 0.0;
  //double a0g = 0.0;
  //double a1g = 0.0;

  //int leaps = 0;
  int tot = 0;
  int wnt = 0;
  //int wnlsf = 0;
  //int dn = 0;
  //int dtlsf = 0;
  //int t0g = 0;
  //int wn0g = 0;

  trace(3, "decode_galclockb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 64) {
    trace(2, "oem4 galclockb length error: len=%d\n", raw->len);
    return -1;
  }
  a0 = R8(p);
  p += 8;
  a1 = R8(p);
  p += 8;
  //leaps = I4(p);
  I4(p);
  p += 4;
  tot = U4(p);
  p += 4;
  wnt = U4(p);
  p += 4;
  //wnlsf = U4(p);
  U4(p);
  p += 4;
  //dn = U4(p);
  U4(p);
  p += 4;
  //dtlsf = U4(p);
  U4(p);
  p += 4;
  //a0g = R8(p);
  R8(p);
  p += 8;
  //a1g = R8(p);
  R8(p);
  p += 8;
  //t0g = U4(p);
  U4(p);
  p += 4;
  //wn0g = U4(p);
  U4(p);

  raw->nav.utc_gal[0] = a0;
  raw->nav.utc_gal[1] = a1;
  raw->nav.utc_gal[2] = tot; /* utc reference tow (s) */
  raw->nav.utc_gal[3] = wnt; /* utc reference week */
  return 9;
}
///* decode galionob
///-----------------------------------------------------------*/
static int decode_galionob(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  double ai[3];
  int i = 0;
  int sf[5];

  trace(3, "decode_galionob: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 29) {
    trace(2, "oem4 galionob length error: len=%d\n", raw->len);
    return -1;
  }
  ai[0] = R8(p);
  p += 8;
  ai[1] = R8(p);
  p += 8;
  ai[2] = R8(p);
  p += 8;
  sf[0] = U1(p);
  p += 1;
  sf[1] = U1(p);
  p += 1;
  sf[2] = U1(p);
  p += 1;
  sf[3] = U1(p);
  p += 1;
  sf[4] = U1(p);
  (void)sf;

  for (i = 0; i < 3; ++i) {
    raw->nav.ion_gal[i] = ai[i];
  }
  return 9;
}
///* decode galfnavrawpageb
///----------------------------------------------------*/
static int decode_galfnavrawpageb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  unsigned char buff[27];
  int i = 0;
  //int sigch = 0;
  int satid = 0;
  int page = 0;

  trace(3, "decode_galfnavrawpageb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 35) {
    trace(2, "oem4 galfnavrawpageb length error: len=%d\n", raw->len);
    return -1;
  }
  //sigch = U4(p);
  U4(p);
  p += 4;
  satid = U4(p);
  p += 4;
  for (i = 0; i < 27; ++i) {
    buff[i] = U1(p);
    p += 1;
  }
  page = getbitu(buff, 0, 6);

  trace(3, "%s E%2d FNAV     (%2d) ", time_str(raw->time, 0), satid, page);
  traceb(3, buff, 27);

  return 0;
}
///* decode galinavrawwordb
///----------------------------------------------------*/
static int decode_galinavrawwordb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  unsigned char buff[16];
  gtime_t time = raw->time;
  char* sig = NULL;
  int i = 0;
  //int sigch = 0;
  int satid = 0;
  int sigtype = 0;
  int type = 0;
  int week = 0;
  int tow = 0;

  trace(3, "decode_galinavrawwordb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 28) {
    trace(2, "oem4 galinavrawwordb length error: len=%d\n", raw->len);
    return -1;
  }
  //sigch = U4(p);
  U4(p);
  p += 4;
  satid = U4(p);
  p += 4;
  sigtype = U4(p);
  p += 4;

  switch (sigtype) {
    case 10433:
      sig = "E1 ";
      break;
    case 10466:
      sig = "E5A";
      break;
    case 10499:
      sig = "E5B";
      break;
    default:
      sig = "???";
      break;
  }
  for (i = 0; i < 16; ++i) {
    buff[i] = U1(p);
    p += 1;
  }
  type = getbitu(buff, 0, 6);
  if (type == 0 && getbitu(buff, 6, 2) == 2) {
    week = getbitu(buff, 96, 12); /* gst week */
    tow = getbitu(buff, 108, 20);
    time = gst2time(week, tow);
  }
  trace(3, "%s E%2d INAV-%s (%2d) ", time_str(time, 0), satid, sig, type);
  traceb(3, buff, 16);

  return 0;
}
///* decode rawcnavframeb
///------------------------------------------------------*/
static int decode_rawcnavframeb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM4HLEN;
  unsigned char buff[38];
  int i = 0;
  int sigch = 0;
  int prn = 0;
  int frmid = 0;

  trace(3, "decode_rawcnavframeb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 50) {
    trace(2, "oem4 rawcnavframeb length error: len=%d\n", raw->len);
    return -1;
  }
  sigch = U4(p);
  (void)sigch;
  p += 4;
  prn = U4(p);
  p += 4;
  frmid = U4(p);
  p += 4;

  for (i = 0; i < 38; ++i) {
    buff[i] = U1(p);
    p += 1;
  }
  trace(3, "%s PRN=%3d FRMID=%2d ", time_str(raw->time, 0), prn, frmid);
  traceb(3, buff, 38);

  return 0;
}
///* decode bdsephemerisb
///------------------------------------------------------*/
static int decode_bdsephemerisb(raw_t* raw) {
  eph_t eph = {0};
  unsigned char* p = raw->buff + OEM4HLEN;
  double ura = 0.0;
  double sqrtA = 0.0;
  char* msg = NULL;
  int prn = 0;
  int toc = 0;

  trace(3, "decode_bdsephemerisb: len=%d\n", raw->len);

  if (raw->len < OEM4HLEN + 196) {
    trace(2, "oem4 bdsephemrisb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);
  p += 4;
  eph.week = U4(p);
  p += 4;
  ura = R8(p);
  p += 8;
  eph.svh = U4(p) & 1;
  p += 4;
  eph.tgd[0] = R8(p);
  p += 8; /* TGD1 for B1 (s) */
  eph.tgd[1] = R8(p);
  p += 8; /* TGD2 for B2 (s) */
  eph.iodc = U4(p);
  p += 4; /* AODC */
  toc = U4(p);
  p += 4;
  eph.f0 = R8(p);
  p += 8;
  eph.f1 = R8(p);
  p += 8;
  eph.f2 = R8(p);
  p += 8;
  eph.iode = U4(p);
  p += 4; /* AODE */
  eph.toes = U4(p);
  p += 4;
  sqrtA = R8(p);
  p += 8;
  eph.e = R8(p);
  p += 8;
  eph.omg = R8(p);
  p += 8;
  eph.deln = R8(p);
  p += 8;
  eph.M0 = R8(p);
  p += 8;
  eph.OMG0 = R8(p);
  p += 8;
  eph.OMGd = R8(p);
  p += 8;
  eph.i0 = R8(p);
  p += 8;
  eph.idot = R8(p);
  p += 8;
  eph.cuc = R8(p);
  p += 8;
  eph.cus = R8(p);
  p += 8;
  eph.crc = R8(p);
  p += 8;
  eph.crs = R8(p);
  p += 8;
  eph.cic = R8(p);
  p += 8;
  eph.cis = R8(p);
  eph.A = sqrtA * sqrtA;
  eph.sva = uraindex(ura);

  if (raw->outtype) {
    msg = raw->msgtype + strlen(raw->msgtype);
    sprintf(msg, " prn=%3d iod=%3d toes=%6.0f", prn, eph.iode, eph.toes);
  }
  if (!(eph.sat = satno(SYS_CMP, prn))) {
    trace(2, "oemv bdsephemeris satellite error: prn=%d\n", prn);
    return -1;
  }
  eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
  eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
  eph.ttr = raw->time;

  if (!strstr(raw->opt, "-EPHALL")) {
    if (timediff(raw->nav.eph[eph.sat - 1].toe, eph.toe) == 0.0) {
      return 0; /* unchanged */
    }
  }
  raw->nav.eph[eph.sat - 1] = eph;
  raw->ephsat = eph.sat;
  return 2;
}
///* decode rgeb
///---------------------------------------------------------------*/
static int decode_rgeb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM3HLEN;
  double tow = 0.0;
  double psr = 0.0;
  double adr = 0.0;
  double tt = 0.0;
  double lockt = 0.0;
  double dop = 0.0;
  double snr = 0.0;
  int i = 0;
  int week = 0;
  int nobs = 0;
  int prn = 0;
  int sat = 0;
  int stat = 0;
  int sys = 0;
  int parity = 0;
  int lli = 0;
  int index = 0;
  int freq = 0;

  trace(3, "decode_rgeb: len=%d\n", raw->len);

  week = adjgpsweek(U4(p));
  tow = R8(p + 4);
  nobs = U4(p + 12);
  raw->time = gpst2time(week, tow);

  if (raw->len != OEM3HLEN + 20 + nobs * 44) {
    trace(2, "oem3 regb length error: len=%d nobs=%d\n", raw->len, nobs);
    return -1;
  }
  for (i = 0, p += 20; i < nobs; ++i, p += 44) {
    prn = U4(p);
    psr = R8(p + 4);
    adr = R8(p + 16);
    dop = R4(p + 28);
    snr = R4(p + 32);
    lockt = R4(p + 36);        /* lock time (s) */
    stat = I4(p + 40);         /* tracking status */
    freq = (stat >> 20) & 1;   /* L1:0,L2:1 */
    sys = (stat >> 15) & 7;    /* satellite sys (0:GPS,1:GLONASS,2:WAAS) */
    parity = (stat >> 10) & 1; /* parity known */
    if (!(sat = satno(sys == 1 ? SYS_GLO : (sys == 2 ? SYS_SBS : SYS_GPS),
                      prn))) {
      trace(2, "oem3 regb satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    tt = timediff(raw->time, raw->tobs);
    if (raw->tobs.time != 0) {
      lli = lockt - raw->lockt[sat - 1][freq] + 0.05 < tt ||
            parity != raw->halfc[sat - 1][freq];
    } else {
      lli = 0;
    }
    if (!parity) {
      lli |= 2;
    }
    raw->lockt[sat - 1][freq] = lockt;
    raw->halfc[sat - 1][freq] = parity;

    if (fabs(timediff(raw->obs.data[0].time, raw->time)) > 1E-9) {
      raw->obs.n = 0;
    }
    if ((index = obsindex(&raw->obs, raw->time, sat)) >= 0) {
      raw->obs.data[index].L[freq] = -adr; /* flip sign */
      raw->obs.data[index].P[freq] = psr;
      raw->obs.data[index].D[freq] = (float)dop;
      raw->obs.data[index].SNR[freq] =
          0.0 <= snr && snr < 255.0 ? (unsigned char)(snr * 4.0 + 0.5) : 0;
      raw->obs.data[index].LLI[freq] = (unsigned char)lli;
      raw->obs.data[index].code[freq] = freq == 0 ? CODE_L1C : CODE_L2P;
    }
  }
  raw->tobs = raw->time;
  return 1;
}
///* decode rged
///---------------------------------------------------------------*/
static int decode_rged(raw_t* raw) {
  unsigned int word = 0;
  unsigned char* p = raw->buff + OEM3HLEN;
  double tow = 0.0;
  double psrh = 0.0;
  double psrl = 0.0;
  double psr = 0.0;
  double adr = 0.0;
  double adr_rolls = 0.0;
  double tt = 0.0;
  double lockt = 0.0;
  double dop = 0.0;
  int i = 0;
  int week = 0;
  int nobs = 0;
  int prn = 0;
  int sat = 0;
  int stat = 0;
  int sys = 0;
  int parity = 0;
  int lli = 0;
  int index = 0;
  int freq = 0;
  int snr = 0;

  trace(3, "decode_rged: len=%d\n", raw->len);

  nobs = U2(p);
  week = adjgpsweek(U2(p + 2));
  tow = U4(p + 4) / 100.0;
  raw->time = gpst2time(week, tow);
  if (raw->len != OEM3HLEN + 12 + nobs * 20) {
    trace(2, "oem3 regd length error: len=%d nobs=%d\n", raw->len, nobs);
    return -1;
  }
  for (i = 0, p += 12; i < nobs; ++i, p += 20) {
    word = U4(p);
    prn = word & 0x3F;
    snr = ((word >> 6) & 0x1F) + 20;
    lockt = (word >> 11) / 32.0;
    adr = -I4(p + 4) / 256.0;
    word = U4(p + 8);
    psrh = word & 0xF;
    dop = exsign(word >> 4, 28) / 256.0;
    psrl = U4(p + 12);
    stat = U4(p + 16) >> 8;
    freq = (stat >> 20) & 1;   /* L1:0,L2:1 */
    sys = (stat >> 15) & 7;    /* satellite sys (0:GPS,1:GLONASS,2:WAAS) */
    parity = (stat >> 10) & 1; /* parity known */
    if (!(sat = satno(sys == 1 ? SYS_GLO : (sys == 2 ? SYS_SBS : SYS_GPS),
                      prn))) {
      trace(2, "oem3 regd satellite number error: sys=%d prn=%d\n", sys, prn);
      continue;
    }
    tt = timediff(raw->time, raw->tobs);
    psr = (psrh * 4294967296.0 + psrl) / 128.0;
    adr_rolls = floor((psr / (freq == 0 ? WL1 : WL2) - adr) / MAXVAL + 0.5);
    adr = adr + MAXVAL * adr_rolls;

    if (raw->tobs.time != 0) {
      lli = lockt - raw->lockt[sat - 1][freq] + 0.05 < tt ||
            parity != raw->halfc[sat - 1][freq];
    } else {
      lli = 0;
    }
    if (!parity) {
      lli |= 2;
    }
    raw->lockt[sat - 1][freq] = lockt;
    raw->halfc[sat - 1][freq] = parity;

    if (fabs(timediff(raw->obs.data[0].time, raw->time)) > 1E-9) {
      raw->obs.n = 0;
    }
    if ((index = obsindex(&raw->obs, raw->time, sat)) >= 0) {
      raw->obs.data[index].L[freq] = adr;
      raw->obs.data[index].P[freq] = psr;
      raw->obs.data[index].D[freq] = (float)dop;
      raw->obs.data[index].SNR[freq] = (unsigned char)(snr * 4.0 + 0.5);
      raw->obs.data[index].LLI[freq] = (unsigned char)lli;
      raw->obs.data[index].code[freq] = freq == 0 ? CODE_L1C : CODE_L2P;
    }
  }
  raw->tobs = raw->time;
  return 1;
}
///* decode repb
///---------------------------------------------------------------*/
static int decode_repb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM3HLEN;
  eph_t eph = {0};
  int prn = 0;
  int sat = 0;

  trace(3, "decode_repb: len=%d\n", raw->len);

  if (raw->len != OEM3HLEN + 96) {
    trace(2, "oem3 repb length error: len=%d\n", raw->len);
    return -1;
  }
  prn = U4(p);
  if (!(sat = satno(SYS_GPS, prn))) {
    trace(2, "oem3 repb satellite number error: prn=%d\n", prn);
    return -1;
  }
  if (decode_frame(p + 4, &eph, NULL, NULL, NULL, NULL) != 1 ||
      decode_frame(p + 34, &eph, NULL, NULL, NULL, NULL) != 2 ||
      decode_frame(p + 64, &eph, NULL, NULL, NULL, NULL) != 3) {
    trace(2, "oem3 repb subframe error: prn=%d\n", prn);
    return -1;
  }
  if (!strstr(raw->opt, "-EPHALL")) {
    if (eph.iode == raw->nav.eph[sat - 1].iode) {
      return 0; /* unchanged */
    }
  }
  eph.sat = sat;
  raw->nav.eph[sat - 1] = eph;
  raw->ephsat = sat;
  return 2;
}
///* decode frmb
///--------------------------------------------------------------*/
static int decode_frmb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM3HLEN;
  double tow = 0.0;
  int i = 0;
  int week = 0;
  int prn = 0;
  int nbit = 0;

  trace(3, "decode_frmb: len=%d\n", raw->len);

  week = adjgpsweek(U4(p));
  tow = R8(p + 4);
  prn = U4(p + 12);
  nbit = U4(p + 20);
  raw->time = gpst2time(week, tow);
  if (nbit != 250) {
    return 0;
  }
  if (prn < MINPRNSBS || MAXPRNSBS < prn) {
    trace(2, "oem3 frmb satellite number error: prn=%d\n", prn);
    return -1;
  }
  raw->sbsmsg.week = week;
  raw->sbsmsg.tow = (int)tow;
  raw->sbsmsg.prn = prn;
  for (i = 0; i < 29; ++i) {
    raw->sbsmsg.msg[i] = p[24 + i];
  }
  return 3;
}
///* decode ionb
///---------------------------------------------------------------*/
static int decode_ionb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM3HLEN;
  int i = 0;

  if (raw->len != 64 + OEM3HLEN) {
    trace(2, "oem3 ionb length error: len=%d\n", raw->len);
    return -1;
  }
  for (i = 0; i < 8; ++i) {
    raw->nav.ion_gps[i] = R8(p + i * 8);
  }
  return 9;
}
///* decode utcb
///---------------------------------------------------------------*/
static int decode_utcb(raw_t* raw) {
  unsigned char* p = raw->buff + OEM3HLEN;

  trace(3, "decode_utcb: len=%d\n", raw->len);

  if (raw->len != 40 + OEM3HLEN) {
    trace(2, "oem3 utcb length error: len=%d\n", raw->len);
    return -1;
  }
  raw->nav.utc_gps[0] = R8(p);
  raw->nav.utc_gps[1] = R8(p + 8);
  raw->nav.utc_gps[2] = U4(p + 16);
  raw->nav.utc_gps[3] = adjgpsweek(U4(p + 20));
  raw->nav.leaps = I4(p + 28);
  return 9;
}
/* decode oem4 message -------------------------------------------------------*/
static int decode_oem4(raw_t* raw) {
  double tow = 0.0;
  int msg = 0;
  int week = 0;
  int type = U2(raw->buff + 4);

  trace(3, "decode_oem4: type=%3d len=%d\n", type, raw->len);

  /* check crc32 */
  //    printf("before crc23\n");
  //    if (crc32(raw->buff,raw->len)!=U4(raw->buff+raw->len)) {
  //        printf("check crc23 failed\n");
  //        trace(2,"oem4 crc error: type=%3d len=%d\n",type,raw->len);
  //        printf("check crc error: type=%3d len=%d\n", type, raw->len);
  //        return -1;
  //    }
  msg = (U1(raw->buff + 6) >> 4) & 0x3;
  week = adjgpsweek(U2(raw->buff + 14));
  tow = U4(raw->buff + 16) * 0.001;
  raw->time = gpst2time(week, tow);

  if (raw->outtype) {
    sprintf(raw->msgtype, "OEM4 %4d (%4d): msg=%d %s", type, raw->len, msg,
            time_str(gpst2time(week, tow), 2));
  }
  if (msg != 0) {
    return 0; /* message type: 0=binary,1=ascii */
  }
  switch (type) {
    case ID_RANGECMP:
      return decode_rangecmpb(raw);
    case ID_RANGE:
      return decode_rangeb(raw);
    case ID_RAWEPHEM:
      return decode_rawephemb(raw);
    case ID_RAWWAASFRAME:
      return decode_rawwaasframeb(raw);
    case ID_RAWSBASFRAME:
      return decode_rawsbasframeb(raw);
    case ID_IONUTC:
      return decode_ionutcb(raw);
    case ID_GLOEPHEMERIS:
      return decode_gloephemerisb(raw);
    case ID_QZSSRAWEPHEM:
      return decode_qzssrawephemb(raw);
    case ID_QZSSRAWSUBFRAME:
      return decode_qzssrawsubframeb(raw);
    case ID_QZSSIONUTC:
      return decode_qzssionutcb(raw);
    case ID_GALEPHEMERIS:
      return decode_galephemerisb(raw);
    case ID_GALALMANAC:
      return decode_galalmanacb(raw);
    case ID_GALCLOCK:
      return decode_galclockb(raw);
    case ID_GALIONO:
      return decode_galionob(raw);
    case ID_GALFNAVRAWPAGE:
      return decode_galfnavrawpageb(raw);
    case ID_GALINAVRAWWORD:
      return decode_galinavrawwordb(raw);
    case ID_RAWCNAVFRAME:
      return decode_rawcnavframeb(raw);
    case ID_BDSEPHEMERIS:
      return decode_bdsephemerisb(raw);
  }
  return 0;
}
/* decode oem3 message -------------------------------------------------------*/
static int decode_oem3(raw_t* raw) {
  int type = U4(raw->buff + 4);

  trace(3, "decode_oem3: type=%3d len=%d\n", type, raw->len);

  /* checksum */
  if (chksum(raw->buff, raw->len)) {
    trace(2, "oem3 checksum error: type=%3d len=%d\n", type, raw->len);
    return -1;
  }
  if (raw->outtype) {
    sprintf(raw->msgtype, "OEM3 %4d (%4d):", type, raw->len);
  }
  switch (type) {
    case ID_RGEB:
      return decode_rgeb(raw);
    case ID_RGED:
      return decode_rged(raw);
    case ID_REPB:
      return decode_repb(raw);
    case ID_FRMB:
      return decode_frmb(raw);
    case ID_IONB:
      return decode_ionb(raw);
    case ID_UTCB:
      return decode_utcb(raw);
  }
  return 0;
}
/* sync header ---------------------------------------------------------------*/
static int sync_oem4(unsigned char* buff, unsigned char data) {
  buff[0] = buff[1];
  buff[1] = buff[2];
  buff[2] = data;
  return buff[0] == OEM4SYNC1 && buff[1] == OEM4SYNC2 && buff[2] == OEM4SYNC3;
}
static int sync_oem3(unsigned char* buff, unsigned char data) {
  buff[0] = buff[1];
  buff[1] = buff[2];
  buff[2] = data;
  return buff[0] == OEM3SYNC1 && buff[1] == OEM3SYNC2 && buff[2] == OEM3SYNC3;
}
/* input oem4/oem3 raw data from stream ----------------------------------------
 * fetch next novatel oem4/oem3 raw data and input a mesasge from stream
 * args   : raw_t *raw   IO     receiver raw data control struct
 *          unsigned char data I stream data (1 byte)
 * return : status (-1: error message, 0: no message, 1: input observation data,
 *                  2: input ephemeris, 3: input sbas message,
 *                  9: input ion/utc parameter)
 *
 * notes  : to specify input options for oem4, set raw->opt to the following
 *          option strings separated by spaces.
 *
 *          -EPHALL : input all ephemerides
 *          -GL1P   : select 1P for GPS L1 (default 1C)
 *          -GL2X   : select 2X for GPS L2 (default 2W)
 *          -RL2C   : select 2C for GLO L2 (default 2P)
 *          -EL2C   : select 2C for GAL L2 (default 2C)
 *
 *-----------------------------------------------------------------------------*/
extern int input_oem4(raw_t* raw, unsigned char data) {
  trace(5, "input_oem4: data=%02x\n", data);
  /* synchronize frame */
  if (raw->nbyte == 0) {
    if (sync_oem4(raw->buff, data)) {
      raw->nbyte = 3;
    }
    return 0;
  }
  raw->buff[raw->nbyte++] = data;

  if (raw->nbyte == 10 &&
      (raw->len = U2(raw->buff + 8) + OEM4HLEN) > MAXRAWLEN - 4) {
    trace(2, "oem4 length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }

  if (raw->nbyte < 10 || raw->nbyte < raw->len + 4) {
    return 0;
  }
  raw->nbyte = 0;
  /* decode oem4 message */
  return decode_oem4(raw);
}
extern int input_oem3(raw_t* raw, unsigned char data) {
  trace(5, "input_oem3: data=%02x\n", data);

  /* synchronize frame */
  if (raw->nbyte == 0) {
    if (sync_oem3(raw->buff, data)) {
      raw->nbyte = 3;
    }
    return 0;
  }
  raw->buff[raw->nbyte++] = data;

  if (raw->nbyte == 12 && (raw->len = U4(raw->buff + 8)) > MAXRAWLEN) {
    trace(2, "oem3 length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }
  if (raw->nbyte < 12 || raw->nbyte < raw->len) {
    return 0;
  }
  raw->nbyte = 0;

  /* decode oem3 message */
  return decode_oem3(raw);
}
///* input oem4/oem3 raw data from file
///------------------------------------------
//* fetch next novatel oem4/oem3 raw data and input a message from file
//* args   : raw_t  *raw   IO     receiver raw data control struct
//*          int    format I      receiver raw data format (STRFMT_???)
//*          FILE   *fp    I      file pointer
//* return : status(-2: end of file, -1...9: same as above)
//*-----------------------------------------------------------------------------*/
extern int input_oem4f(raw_t* raw, FILE* fp) {
  int i = 0;
  int data = 0;

  trace(4, "input_oem4f:\n");

  /* synchronize frame */
  if (raw->nbyte == 0) {
    for (i = 0;; ++i) {
      if ((data = fgetc(fp)) == EOF) {
        return -2;
      }
      if (sync_oem4(raw->buff, (unsigned char)data)) {
        break;
      }
      if (i >= 4096) {
        return 0;
      }
    }
  }
  if (fread(raw->buff + 3, 7, 1, fp) < 1) {
    return -2;
  }
  raw->nbyte = 10;

  if ((raw->len = U2(raw->buff + 8) + OEM4HLEN) > MAXRAWLEN - 4) {
    trace(2, "oem4 length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }
  if (fread(raw->buff + 10, raw->len - 6, 1, fp) < 1) {
    return -2;
  }
  raw->nbyte = 0;

  /* decode oem4 message */
  return decode_oem4(raw);
}
extern int input_oem3f(raw_t* raw, FILE* fp) {
  int i = 0;
  int data = 0;

  trace(4, "input_oem3f:\n");

  /* synchronize frame */
  if (raw->nbyte == 0) {
    for (i = 0;; ++i) {
      if ((data = fgetc(fp)) == EOF) {
        return -2;
      }
      if (sync_oem3(raw->buff, (unsigned char)data)) {
        break;
      }
      if (i >= 4096) {
        return 0;
      }
    }
  }
  if (fread(raw->buff + 3, 1, 9, fp) < 9) {
    return -2;
  }
  raw->nbyte = 12;

  if ((raw->len = U4(raw->buff + 8)) > MAXRAWLEN) {
    trace(2, "oem3 length error: len=%d\n", raw->len);
    raw->nbyte = 0;
    return -1;
  }
  if (fread(raw->buff + 12, 1, raw->len - 12, fp) < (size_t)(raw->len - 12)) {
    return -2;
  }
  raw->nbyte = 0;

  /* decode oem3 message */
  return decode_oem3(raw);
}
