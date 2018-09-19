/*********************************************************************************
* The RTKLIB software package is distributed under the following BSD 2-clause
* license (http://opensource.org/licenses/BSD-2-Clause) and additional two
* exclusive clauses. Users are permitted to develop, produce or sell their own
* non-commercial or commercial products utilizing, linking or including RTKLIB
*as
* long as they comply with the license.
*
*           Copyright (c) 2007-2013, T. Takasu, All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
*modification,
* are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* - Redistributions in binary form must reproduce the above copyright notice,
*this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* - The software package includes some companion executive binaries or shared
*   libraries necessary to execute APs on Windows. These licenses succeed to the
*   original ones of these software.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
*OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************************/

/*------------------------------------------------------------------------------
* rcvraw.c : receiver raw data functions
*
*          Copyright (C) 2009-2014 by T.TAKASU, All rights reserved.
*          Copyright (C) 2014 by T.SUZUKI, All rights reserved.
*
* references :
*     [1] IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
*         7 March, 2006
*     [2] Global navigation satellite system GLONASS interface control document
*         navigation radiosignal in bands L1,L2 (version 5.1), 2008
*     [3] BeiDou satellite navigation system signal in space interface control
*         document open service signal (version 2.0), December 2013
*     [4] Quasi-Zenith Satellite System Navigation Service Interface
*         Specification for QZSS (IS-QZSS) V.1.5, March 27, 2014
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2009/04/10 1.0  new
*           2009/06/02 1.1  support glonass
*           2010/07/31 1.2  support eph_t struct change
*           2010/12/06 1.3  add almanac decoding, support of GW10
*                           change api decode_frame()
*           2013/04/11 1.4  fix bug on decode fit interval
*           2014/01/31 1.5  fix bug on decode fit interval
*           2014/06/22 1.6  add api decode_glostr()
*           2014/06/22 1.7  add api decode_bds_d1(), decode_bds_d2()
*           2014/08/14 1.8  add test_glostr()
*                           add support input format rt17
*           2014/08/31 1.9  suppress warning
*           2014/11/07 1.10 support qzss navigation subframes
*-----------------------------------------------------------------------------*/
/**
* file: rcvraw.c
* version: rtklib ver.2.4.2
* Copy from
* https://github.com/tomojitakasu/RTKLIB/tree/76b9c97257f304aedad38b5a6bbbac444724aab3/src/rcvraw.c
*/
#include <stdint.h>
#include "rtklib.h"

static const char rcsid[] = "$Id:$";

#define P2_66 1.355252715606881E-20 /* 2^-66 for BeiDou ephemeris */

/* get two component bits ----------------------------------------------------*/
static unsigned int getbitu2(const unsigned char* buff, int p1, int l1, int p2,
                             int l2) {
  return (getbitu(buff, p1, l1) << l2) + getbitu(buff, p2, l2);
}
static int getbits2(const unsigned char* buff, int p1, int l1, int p2, int l2) {
  if (getbitu(buff, p1, 1)) {
    return (int)((getbits(buff, p1, l1) << l2) + getbitu(buff, p2, l2));
  } else {
    return (int)getbitu2(buff, p1, l1, p2, l2);
  }
}
/* get three component bits --------------------------------------------------*/
static unsigned int getbitu3(const unsigned char* buff, int p1, int l1, int p2,
                             int l2, int p3, int l3) {
  return (getbitu(buff, p1, l1) << (l2 + l3)) + (getbitu(buff, p2, l2) << l3) +
         getbitu(buff, p3, l3);
}
static int getbits3(const unsigned char* buff, int p1, int l1, int p2, int l2,
                    int p3, int l3) {
  if (getbitu(buff, p1, 1)) {
    return (int)((getbits(buff, p1, l1) << (l2 + l3)) +
                 (getbitu(buff, p2, l2) << l3) + getbitu(buff, p3, l3));
  } else {
    return (int)getbitu3(buff, p1, l1, p2, l2, p3, l3);
  }
}
/* merge two components ------------------------------------------------------*/
static unsigned int merge_two_u(unsigned int a, unsigned int b, int n) {
  return (a << n) + b;
}
static int merge_two_s(int a, unsigned int b, int n) {
  return (int)((a << n) + b);
}
/* get sign-magnitude bits ---------------------------------------------------*/
static double getbitg(const unsigned char* buff, int pos, int len) {
  double value = getbitu(buff, pos + 1, len - 1);
  return getbitu(buff, pos, 1) ? -value : value;
}
/* decode BeiDou D1 ephemeris --------------------------------------------------
* decode BeiDou D1 ephemeris (IGSO/MEO satellites) (ref [3] 5.2)
* args   : unsigned char *buff I beidou D1 subframe bits
*                                  buff[ 0- 37]: subframe 1 (300 bits)
*                                  buff[38- 75]: subframe 2
*                                  buff[76-113]: subframe 3
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int decode_bds_d1(const unsigned char* buff, eph_t* eph) {
  double toc_bds = 0.0;
  double sqrtA = 0.0;
  unsigned int toe1 = 0;
  unsigned int toe2 = 0;
  unsigned int sow1 = 0;
  unsigned int sow2 = 0;
  unsigned int sow3 = 0;
  int i = 0;
  int frn1 = 0;
  int frn2 = 0;
  int frn3 = 0;

  trace(3, "decode_bds_d1:\n");

  i = 8 * 38 * 0; /* subframe 1 */
  frn1 = getbitu(buff, i + 15, 3);
  sow1 = getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->svh = getbitu(buff, i + 42, 1);  /* SatH1 */
  eph->iodc = getbitu(buff, i + 43, 5); /* AODC */
  eph->sva = getbitu(buff, i + 48, 4);
  eph->week = getbitu(buff, i + 60, 13); /* week in BDT */
  toc_bds = getbitu2(buff, i + 73, 9, i + 90, 8) * 8.0;
  eph->tgd[0] = getbits(buff, i + 98, 10) * 0.1 * 1E-9;
  eph->tgd[1] = getbits2(buff, i + 108, 4, i + 120, 6) * 0.1 * 1E-9;
  eph->f2 = getbits(buff, i + 214, 11) * P2_66;
  eph->f0 = getbits2(buff, i + 225, 7, i + 240, 17) * P2_33;
  eph->f1 = getbits2(buff, i + 257, 5, i + 270, 17) * P2_50;
  eph->iode = getbitu(buff, i + 287, 5); /* AODE */

  i = 8 * 38 * 1; /* subframe 2 */
  frn2 = getbitu(buff, i + 15, 3);
  sow2 = getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->deln = getbits2(buff, i + 42, 10, i + 60, 6) * P2_43 * SC2RAD;
  eph->cuc = getbits2(buff, i + 66, 16, i + 90, 2) * P2_31;
  eph->M0 = getbits2(buff, i + 92, 20, i + 120, 12) * P2_31 * SC2RAD;
  eph->e = getbitu2(buff, i + 132, 10, i + 150, 22) * P2_33;
  eph->cus = getbits(buff, i + 180, 18) * P2_31;
  eph->crc = getbits2(buff, i + 198, 4, i + 210, 14) * P2_6;
  eph->crs = getbits2(buff, i + 224, 8, i + 240, 10) * P2_6;
  sqrtA = getbitu2(buff, i + 250, 12, i + 270, 20) * P2_19;
  toe1 = getbitu(buff, i + 290, 2); /* TOE 2-MSB */
  eph->A = sqrtA * sqrtA;

  i = 8 * 38 * 2; /* subframe 3 */
  frn3 = getbitu(buff, i + 15, 3);
  sow3 = getbitu2(buff, i + 18, 8, i + 30, 12);
  toe2 = getbitu2(buff, i + 42, 10, i + 60, 5); /* TOE 5-LSB */
  eph->i0 = getbits2(buff, i + 65, 17, i + 90, 15) * P2_31 * SC2RAD;
  eph->cic = getbits2(buff, i + 105, 7, i + 120, 11) * P2_31;
  eph->OMGd = getbits2(buff, i + 131, 11, i + 150, 13) * P2_43 * SC2RAD;
  eph->cis = getbits2(buff, i + 163, 9, i + 180, 9) * P2_31;
  eph->idot = getbits2(buff, i + 189, 13, i + 210, 1) * P2_43 * SC2RAD;
  eph->OMG0 = getbits2(buff, i + 211, 21, i + 240, 11) * P2_31 * SC2RAD;
  eph->omg = getbits2(buff, i + 251, 11, i + 270, 21) * P2_31 * SC2RAD;
  eph->toes = merge_two_u(toe1, toe2, 15) * 8.0;

  /* check consistency of subframe numbers, sows and toe/toc */
  if (frn1 != 1 || frn2 != 2 || frn3 != 3) {
    trace(3, "decode_bds_d1 error: frn=%d %d %d\n", frn1, frn2, frn3);
    return 0;
  }
  if (sow2 != sow1 + 6 || sow3 != sow2 + 6) {
    trace(3, "decode_bds_d1 error: sow=%d %d %d\n", sow1, sow2, sow3);
    return 0;
  }
  if (toc_bds != eph->toes) {
    trace(3, "decode_bds_d1 error: toe=%.0f toc=%.0f\n", eph->toes, toc_bds);
    return 0;
  }
  eph->ttr = bdt2gpst(bdt2time(eph->week, sow1)); /* bdt -> gpst */
  if (eph->toes > sow1 + 302400.0) {
    eph->week++;
  } else if (eph->toes < sow1 - 302400.0) {
    eph->week--;
  }
  eph->toe = bdt2gpst(bdt2time(eph->week, eph->toes)); /* bdt -> gpst */
  eph->toc = bdt2gpst(bdt2time(eph->week, toc_bds));   /* bdt -> gpst */
  return 1;
}
/* decode BeiDou D2 ephemeris --------------------------------------------------
* decode BeiDou D2 ephemeris (GEO satellites) (ref [3] 5.3)
* args   : unsigned char *buff I beidou D2 subframe 1 page bits
*                                  buff[  0- 37]: page 1 (300 bits)
*                                  buff[ 38- 75]: page 2
*                                  ...
*                                  buff[342-379]: page 10
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int decode_bds_d2(const unsigned char* buff, eph_t* eph) {
  double toc_bds = 0.0;
  double sqrtA = 0.0;
  unsigned int f1p4 = 0;
  unsigned int cucp5 = 0;
  unsigned int ep6 = 0;
  unsigned int cicp7 = 0;
  unsigned int i0p8 = 0;
  unsigned int OMGdp9 = 0;
  unsigned int omgp10 = 0;
  unsigned int sow1 = 0;
  unsigned int sow3 = 0;
  unsigned int sow4 = 0;
  unsigned int sow5 = 0;
  unsigned int sow6 = 0;
  unsigned int sow7 = 0;
  unsigned int sow8 = 0;
  unsigned int sow9 = 0;
  unsigned int sow10 = 0;
  int i = 0;
  int f1p3 = 0;
  int cucp4 = 0;
  int ep5 = 0;
  int cicp6 = 0;
  int i0p7 = 0;
  int OMGdp8 = 0;
  int omgp9 = 0;
  int pgn1 = 0;
  int pgn3 = 0;
  int pgn4 = 0;
  int pgn5 = 0;
  int pgn6 = 0;
  int pgn7 = 0;
  int pgn8 = 0;
  int pgn9 = 0;
  int pgn10 = 0;

  trace(3, "decode_bds_d2:\n");

  i = 8 * 38 * 0; /* page 1 */
  pgn1 = getbitu(buff, i + 42, 4);
  sow1 = getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->svh = getbitu(buff, i + 46, 1);  /* SatH1 */
  eph->iodc = getbitu(buff, i + 47, 5); /* AODC */
  eph->sva = getbitu(buff, i + 60, 4);
  eph->week = getbitu(buff, i + 64, 13); /* week in BDT */
  toc_bds = getbitu2(buff, i + 77, 5, i + 90, 12) * 8.0;
  eph->tgd[0] = getbits(buff, i + 102, 10) * 0.1 * 1E-9;
  eph->tgd[1] = getbits(buff, i + 120, 10) * 0.1 * 1E-9;

  i = 8 * 38 * 2; /* page 3 */
  pgn3 = getbitu(buff, i + 42, 4);
  sow3 = getbitu2(buff, i + 18, 8, i + 30, 12);
  eph->f0 = getbits2(buff, i + 100, 12, i + 120, 12) * P2_33;
  f1p3 = getbits(buff, i + 132, 4);

  i = 8 * 38 * 3; /* page 4 */
  pgn4 = getbitu(buff, i + 42, 4);
  sow4 = getbitu2(buff, i + 18, 8, i + 30, 12);
  f1p4 = getbitu2(buff, i + 46, 6, i + 60, 12);
  eph->f2 = getbits2(buff, i + 72, 10, i + 90, 1) * P2_66;
  eph->iode = getbitu(buff, i + 91, 5); /* AODE */
  eph->deln = getbits(buff, i + 96, 16) * P2_43 * SC2RAD;
  cucp4 = getbits(buff, i + 120, 14);

  i = 8 * 38 * 4; /* page 5 */
  pgn5 = getbitu(buff, i + 42, 4);
  sow5 = getbitu2(buff, i + 18, 8, i + 30, 12);
  cucp5 = getbitu(buff, i + 46, 4);
  eph->M0 = getbits3(buff, i + 50, 2, i + 60, 22, i + 90, 8) * P2_31 * SC2RAD;
  eph->cus = getbits2(buff, i + 98, 14, i + 120, 4) * P2_31;
  ep5 = getbits(buff, i + 124, 10);

  i = 8 * 38 * 5; /* page 6 */
  pgn6 = getbitu(buff, i + 42, 4);
  sow6 = getbitu2(buff, i + 18, 8, i + 30, 12);
  ep6 = getbitu2(buff, i + 46, 6, i + 60, 16);
  sqrtA = getbitu3(buff, i + 76, 6, i + 90, 22, i + 120, 4) * P2_19;
  cicp6 = getbits(buff, i + 124, 10);
  eph->A = sqrtA * sqrtA;

  i = 8 * 38 * 6; /* page 7 */
  pgn7 = getbitu(buff, i + 42, 4);
  sow7 = getbitu2(buff, i + 18, 8, i + 30, 12);
  cicp7 = getbitu2(buff, i + 46, 6, i + 60, 2);
  eph->cis = getbits(buff, i + 62, 18) * P2_31;
  eph->toes = getbitu2(buff, i + 80, 2, i + 90, 15) * 8.0;
  i0p7 = getbits2(buff, i + 105, 7, i + 120, 14);

  i = 8 * 38 * 7; /* page 8 */
  pgn8 = getbitu(buff, i + 42, 4);
  sow8 = getbitu2(buff, i + 18, 8, i + 30, 12);
  i0p8 = getbitu2(buff, i + 46, 6, i + 60, 5);
  eph->crc = getbits2(buff, i + 65, 17, i + 90, 1) * P2_6;
  eph->crs = getbits(buff, i + 91, 18) * P2_6;
  OMGdp8 = getbits2(buff, i + 109, 3, i + 120, 16);

  i = 8 * 38 * 8; /* page 9 */
  pgn9 = getbitu(buff, i + 42, 4);
  sow9 = getbitu2(buff, i + 18, 8, i + 30, 12);
  OMGdp9 = getbitu(buff, i + 46, 5);
  eph->OMG0 = getbits3(buff, i + 51, 1, i + 60, 22, i + 90, 9) * P2_31 * SC2RAD;
  omgp9 = getbits2(buff, i + 99, 13, i + 120, 14);

  i = 8 * 38 * 9; /* page 10 */
  pgn10 = getbitu(buff, i + 42, 4);
  sow10 = getbitu2(buff, i + 18, 8, i + 30, 12);
  omgp10 = getbitu(buff, i + 46, 5);
  eph->idot = getbits2(buff, i + 51, 1, i + 60, 13) * P2_43 * SC2RAD;

  /* check consistency of page numbers, sows and toe/toc */
  if (pgn1 != 1 || pgn3 != 3 || pgn4 != 4 || pgn5 != 5 || pgn6 != 6 ||
      pgn7 != 7 || pgn8 != 8 || pgn9 != 9 || pgn10 != 10) {
    trace(3, "decode_bds_d2 error: pgn=%d %d %d %d %d %d %d %d %d\n", pgn1,
          pgn3, pgn4, pgn5, pgn6, pgn7, pgn8, pgn9, pgn10);
    return 0;
  }
  if (sow3 != sow1 + 6 || sow4 != sow3 + 3 || sow5 != sow4 + 3 ||
      sow6 != sow5 + 3 || sow7 != sow6 + 3 || sow8 != sow7 + 3 ||
      sow9 != sow8 + 3 || sow10 != sow9 + 3) {
    trace(3, "decode_bds_d2 error: sow=%d %d %d %d %d %d %d %d %d\n", sow1,
          sow3, sow4, sow5, sow6, sow7, sow8, sow9, sow10);
    return 0;
  }
  if (toc_bds != eph->toes) {
    trace(3, "decode_bds_d2 error: toe=%.0f toc=%.0f\n", eph->toes, toc_bds);
    return 0;
  }
  eph->f1 = merge_two_s(f1p3, f1p4, 18) * P2_50;
  eph->cuc = merge_two_s(cucp4, cucp5, 4) * P2_31;
  eph->e = merge_two_s(ep5, ep6, 22) * P2_33;
  eph->cic = merge_two_s(cicp6, cicp7, 8) * P2_31;
  eph->i0 = merge_two_s(i0p7, i0p8, 11) * P2_31 * SC2RAD;
  eph->OMGd = merge_two_s(OMGdp8, OMGdp9, 5) * P2_43 * SC2RAD;
  eph->omg = merge_two_s(omgp9, omgp10, 5) * P2_31 * SC2RAD;

  eph->ttr = bdt2gpst(bdt2time(eph->week, sow1)); /* bdt -> gpst */
  if (eph->toes > sow1 + 302400.0) {
    eph->week++;
  } else if (eph->toes < sow1 - 302400.0) {
    eph->week--;
  }
  eph->toe = bdt2gpst(bdt2time(eph->week, eph->toes)); /* bdt -> gpst */
  eph->toc = bdt2gpst(bdt2time(eph->week, toc_bds));   /* bdt -> gpst */
  return 1;
}
/* test hamming code of glonass ephemeris string -------------------------------
* test hamming code of glonass ephemeris string (ref [2] 4.7)
* args   : unsigned char *buff I glonass navigation data string bits in frame
*                                with hamming
*                                  buff[ 0]: string bit 85-78
*                                  buff[ 1]: string bit 77-70
*                                  ...
*                                  buff[10]: string bit  5- 1 (0 padded)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int test_glostr(const unsigned char* buff) {
  static const unsigned char xor_8bit[256] = {
    /* xor of 8 bits */
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0
  };
  static const unsigned char mask_hamming[][12] = {
    /* mask of hamming codes */
    {0x55, 0x55, 0x5A, 0xAA, 0xAA, 0xAA, 0xB5, 0x55, 0x6A, 0xD8, 0x08},
    {0x66, 0x66, 0x6C, 0xCC, 0xCC, 0xCC, 0xD9, 0x99, 0xB3, 0x68, 0x10},
    {0x87, 0x87, 0x8F, 0x0F, 0x0F, 0x0F, 0x1E, 0x1E, 0x3C, 0x70, 0x20},
    {0x07, 0xF8, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0xE0, 0x3F, 0x80, 0x40},
    {0xF8, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x80},
    {0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0x00},
    {0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8}
  };
  unsigned char cs = 0;
  int i = 0;
  int j = 0;
  int n = 0;

  for (i = 0; i < 8; ++i) {
    for (j = 0, cs = 0; j < 11; ++j) {
      cs ^= xor_8bit[buff[j] & mask_hamming[i][j]];
    }
    if (cs) {
      n++;
    }
  }
  return n == 0 || (n == 2 && cs);
}
/* decode glonass ephemeris strings --------------------------------------------
* decode glonass ephemeris string (ref [2])
* args   : unsigned char *buff I glonass navigation data string bits in frames
*                                (without hamming and time mark)
*                                  buff[ 0- 9]: string #1 (77 bits)
*                                  buff[10-19]: string #2
*                                  buff[20-29]: string #3
*                                  buff[30-39]: string #4
*          geph_t *geph  IO     glonass ephemeris message
* return : status (1:ok,0:error)
* notes  : geph->tof should be set to frame time witin 1/2 day before calling
*          geph->frq is set to 0
*-----------------------------------------------------------------------------*/
extern int decode_glostr(const unsigned char* buff, geph_t* geph) {
  double tow = 0.0;
  double tod = 0.0;
  double tof = 0.0;
  double toe = 0.0;
//  int P = 0;
//  int P1 = 0;
//  int P2 = 0;
//  int P3 = 0;
//  int P4 = 0;
  int tk_h = 0;
  int tk_m = 0;
  int tk_s = 0;
  int tb = 0;
  int slot = 0;
  int week = 0;
  int i = 1;
  int frn1 = 0;
  int frn2 = 0;
  int frn3 = 0;
  int frn4 = 0;

  trace(3, "decode_glostr:\n");

  /* frame 1 */
  frn1 = getbitu(buff, i, 4);
  i += 4 + 2;
  //P1 = getbitu(buff, i, 2);
  getbitu(buff, i, 2);
  i += 2;
  tk_h = getbitu(buff, i, 5);
  i += 5;
  tk_m = getbitu(buff, i, 6);
  i += 6;
  tk_s = getbitu(buff, i, 1) * 30;
  i += 1;
  geph->vel[0] = getbitg(buff, i, 24) * P2_20 * 1E3;
  i += 24;
  geph->acc[0] = getbitg(buff, i, 5) * P2_30 * 1E3;
  i += 5;
  geph->pos[0] = getbitg(buff, i, 27) * P2_11 * 1E3;
  i += 27 + 4;

  /* frame 2 */
  frn2 = getbitu(buff, i, 4);
  i += 4;
  geph->svh = getbitu(buff, i, 3);
  i += 3;
  //P2 = getbitu(buff, i, 1);
  getbitu(buff, i, 1);
  i += 1;
  tb = getbitu(buff, i, 7);
  i += 7 + 5;
  geph->vel[1] = getbitg(buff, i, 24) * P2_20 * 1E3;
  i += 24;
  geph->acc[1] = getbitg(buff, i, 5) * P2_30 * 1E3;
  i += 5;
  geph->pos[1] = getbitg(buff, i, 27) * P2_11 * 1E3;
  i += 27 + 4;

  /* frame 3 */
  frn3 = getbitu(buff, i, 4);
  i += 4;
  //P3 = getbitu(buff, i, 1);
  getbitu(buff, i, 1);
  i += 1;
  geph->gamn = getbitg(buff, i, 11) * P2_40;
  i += 11 + 1;
  //P = getbitu(buff, i, 2);
  getbitu(buff, i, 2);
  i += 2;
  //int ln = getbitu(buff, i, 1);
  getbitu(buff, i, 1);
  i += 1;
  geph->vel[2] = getbitg(buff, i, 24) * P2_20 * 1E3;
  i += 24;
  geph->acc[2] = getbitg(buff, i, 5) * P2_30 * 1E3;
  i += 5;
  geph->pos[2] = getbitg(buff, i, 27) * P2_11 * 1E3;
  i += 27 + 4;

  /* frame 4 */
  frn4 = getbitu(buff, i, 4);
  i += 4;
  geph->taun = getbitg(buff, i, 22) * P2_30;
  i += 22;
  geph->dtaun = getbitg(buff, i, 5) * P2_30;
  i += 5;
  geph->age = getbitu(buff, i, 5);
  i += 5 + 14;
  //P4 = getbitu(buff, i, 1);
  getbitu(buff, i, 1);
  i += 1;
  geph->sva = getbitu(buff, i, 4);
  i += 4 + 3;
  //int NT = getbitu(buff, i, 11);
  getbitu(buff, i, 11);
  i += 11;
  slot = getbitu(buff, i, 5);
  i += 5;
  //int M = getbitu(buff, i, 2);
  getbitu(buff, i, 2);

  if (frn1 != 1 || frn2 != 2 || frn3 != 3 || frn4 != 4) {
    trace(3, "decode_glostr error: frn=%d %d %d %d %d\n", frn1, frn2, frn3,
          frn4);
    return 0;
  }
  if (!(geph->sat = satno(SYS_GLO, slot))) {
    trace(2, "decode_glostr error: slot=%d\n", slot);
    return 0;
  }
  geph->frq = 0;
  geph->iode = tb;
  tow = time2gpst(gpst2utc(geph->tof), &week);
  tod = fmod(tow, 86400.0);
  tow -= tod;
  tof = tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0; /* lt->utc */
  if (tof < tod - 43200.0) {
    tof += 86400.0;
  } else if (tof > tod + 43200.0) {
    tof -= 86400.0;
  }
  geph->tof = utc2gpst(gpst2time(week, tow + tof));
  toe = tb * 900.0 - 10800.0; /* lt->utc */
  if (toe < tod - 43200.0) {
    toe += 86400.0;
  } else if (toe > tod + 43200.0) {
    toe -= 86400.0;
  }
  geph->toe = utc2gpst(gpst2time(week, tow + toe)); /* utc->gpst */
  return 1;
}
/* decode gps/qzss navigation data subframe 1 --------------------------------*/
static int decode_subfrm1(const unsigned char* buff, eph_t* eph) {
  double tow = 0.0;
  double toc = 0.0;
  int i = 48;
  int week = 0;
  int iodc0 = 0;
  int iodc1 = 0;
  int tgd = 0;

  trace(4, "decode_subfrm1:\n");
  trace(5, "decode_subfrm1: buff=");
  traceb(5, buff, 30);

  tow = getbitu(buff, 24, 17) * 6.0; /* transmission time */
  week = getbitu(buff, i, 10);
  i += 10;
  eph->code = getbitu(buff, i, 2);
  i += 2;
  eph->sva = getbitu(buff, i, 4);
  i += 4; /* ura index */
  eph->svh = getbitu(buff, i, 6);
  i += 6;
  iodc0 = getbitu(buff, i, 2);
  i += 2;
  eph->flag = getbitu(buff, i, 1);
  i += 1 + 87;
  tgd = getbits(buff, i, 8);
  i += 8;
  iodc1 = getbitu(buff, i, 8);
  i += 8;
  toc = getbitu(buff, i, 16) * 16.0;
  i += 16;
  eph->f2 = getbits(buff, i, 8) * P2_55;
  i += 8;
  eph->f1 = getbits(buff, i, 16) * P2_43;
  i += 16;
  eph->f0 = getbits(buff, i, 22) * P2_31;

  eph->tgd[0] = tgd == -128 ? 0.0 : tgd * P2_31; /* ref [4] */
  eph->iodc = (iodc0 << 8) + iodc1;
  eph->week = adjgpsweek(week); /* week of tow */
  eph->ttr = gpst2time(eph->week, tow);
  eph->toc = gpst2time(eph->week, toc);

  return 1;
}
/* decode gps/qzss navigation data subframe 2 --------------------------------*/
static int decode_subfrm2(const unsigned char* buff, eph_t* eph) {
  double sqrtA = 0.0;
  int i = 48;

  trace(4, "decode_subfrm2:\n");
  trace(5, "decode_subfrm2: buff=");
  traceb(5, buff, 30);

  eph->iode = getbitu(buff, i, 8);
  i += 8;
  eph->crs = getbits(buff, i, 16) * P2_5;
  i += 16;
  eph->deln = getbits(buff, i, 16) * P2_43 * SC2RAD;
  i += 16;
  eph->M0 = getbits(buff, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph->cuc = getbits(buff, i, 16) * P2_29;
  i += 16;
  eph->e = getbitu(buff, i, 32) * P2_33;
  i += 32;
  eph->cus = getbits(buff, i, 16) * P2_29;
  i += 16;
  sqrtA = getbitu(buff, i, 32) * P2_19;
  i += 32;
  eph->toes = getbitu(buff, i, 16) * 16.0;
  i += 16;
  eph->fit = getbitu(buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */

  eph->A = sqrtA * sqrtA;

  return 2;
}
/* decode gps/qzss navigation data subframe 3 --------------------------------*/
static int decode_subfrm3(const unsigned char* buff, eph_t* eph) {
  double tow = 0.0;
  double toc = 0.0;
  int i = 48;
  int iode = 0;

  trace(4, "decode_subfrm3:\n");
  trace(5, "decode_subfrm3: buff=");
  traceb(5, buff, 30);

  eph->cic = getbits(buff, i, 16) * P2_29;
  i += 16;
  eph->OMG0 = getbits(buff, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph->cis = getbits(buff, i, 16) * P2_29;
  i += 16;
  eph->i0 = getbits(buff, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph->crc = getbits(buff, i, 16) * P2_5;
  i += 16;
  eph->omg = getbits(buff, i, 32) * P2_31 * SC2RAD;
  i += 32;
  eph->OMGd = getbits(buff, i, 24) * P2_43 * SC2RAD;
  i += 24;
  iode = getbitu(buff, i, 8);
  i += 8;
  eph->idot = getbits(buff, i, 14) * P2_43 * SC2RAD;

  /* check iode and iodc consistency */
  if (iode != eph->iode || iode != (eph->iodc & 0xFF)) {
    return 0;
  }

  /* adjustment for week handover */
  tow = time2gpst(eph->ttr, &eph->week);
  toc = time2gpst(eph->toc, NULL);
  if (eph->toes < tow - 302400.0) {
    eph->week++;
    tow -= 604800.0;
  } else if (eph->toes > tow + 302400.0) {
    eph->week--;
    tow += 604800.0;
  }
  eph->toe = gpst2time(eph->week, eph->toes);
  eph->toc = gpst2time(eph->week, toc);
  eph->ttr = gpst2time(eph->week, tow);

  return 3;
}
/* decode gps/qzss almanac ---------------------------------------------------*/
static void decode_almanac(const unsigned char* buff, int sat, alm_t* alm) {
  gtime_t toa;
  double deltai = 0.0;
  double sqrtA = 0.0;
  double tt = 0.0;
  int i = 50;
  int f0 = 0;

  trace(4, "decode_almanac: sat=%2d\n", sat);

  if (!alm || alm[sat - 1].week == 0) {
    return;
  }

  alm[sat - 1].sat = sat;
  alm[sat - 1].e = getbits(buff, i, 16) * P2_21;
  i += 16;
  alm[sat - 1].toas = getbitu(buff, i, 8) * 4096.0;
  i += 8;
  deltai = getbits(buff, i, 16) * P2_19 * SC2RAD;
  i += 16;
  alm[sat - 1].OMGd = getbits(buff, i, 16) * P2_38 * SC2RAD;
  i += 16;
  alm[sat - 1].svh = getbitu(buff, i, 8);
  i += 8;
  sqrtA = getbitu(buff, i, 24) * P2_11;
  i += 24;
  alm[sat - 1].OMG0 = getbits(buff, i, 24) * P2_23 * SC2RAD;
  i += 24;
  alm[sat - 1].omg = getbits(buff, i, 24) * P2_23 * SC2RAD;
  i += 24;
  alm[sat - 1].M0 = getbits(buff, i, 24) * P2_23 * SC2RAD;
  i += 24;
  f0 = getbitu(buff, i, 8);
  i += 8;
  alm[sat - 1].f1 = getbits(buff, i, 11) * P2_38;
  i += 11;
  alm[sat - 1].f0 = getbits(buff, i, 3) * P2_17 + f0 * P2_20;
  alm[sat - 1].A = sqrtA * sqrtA;
  alm[sat - 1].i0 = 0.3 * SC2RAD + deltai;

  toa = gpst2time(alm[sat - 1].week, alm[sat - 1].toas);
  tt = timediff(toa, alm[sat - 1].toa);
  if (tt < 302400.0) {
    alm[sat - 1].week--;
  } else if (tt > 302400.0) {
    alm[sat - 1].week++;
  }
  alm[sat - 1].toa = gpst2time(alm[sat - 1].week, alm[sat - 1].toas);
}
/* decode gps navigation data subframe 4 -------------------------------------*/
static void decode_gps_subfrm4(const unsigned char* buff, alm_t* alm,
                               double* ion, double* utc, int* leaps) {
  int i = 0;
  int sat = 0;
  int svid = getbitu(buff, 50, 6);

  if (25 <= svid && svid <= 32) { /* page 2,3,4,5,7,8,9,10 */
    /* decode almanac */
    sat = getbitu(buff, 50, 6);
    if (1 <= sat && sat <= 32) {
      decode_almanac(buff, sat, alm);
    }
  } else if (svid == 63) { /* page 25 */
    /* decode as and sv config */
    i = 56;
    for (sat = 1; sat <= 32; sat++) {
      if (alm) {
        alm[sat - 1].svconf = getbitu(buff, i, 4);
      }
      i += 4;
    }
    /* decode sv health */
    i = 186;
    for (sat = 25; sat <= 32; sat++) {
      if (alm) {
        alm[sat - 1].svh = getbitu(buff, i, 6);
      }
      i += 6;
    }
  } else if (svid == 56) { /* page 18 */
    /* decode ion/utc parameters */
    if (ion) {
      i = 56;
      ion[0] = getbits(buff, i, 8) * P2_30;
      i += 8;
      ion[1] = getbits(buff, i, 8) * P2_27;
      i += 8;
      ion[2] = getbits(buff, i, 8) * P2_24;
      i += 8;
      ion[3] = getbits(buff, i, 8) * P2_24;
      i += 8;
      ion[4] = getbits(buff, i, 8) * pow(2, 11);
      i += 8;
      ion[5] = getbits(buff, i, 8) * pow(2, 14);
      i += 8;
      ion[6] = getbits(buff, i, 8) * pow(2, 16);
      i += 8;
      ion[7] = getbits(buff, i, 8) * pow(2, 16);
    }
    if (utc) {
      i = 120;
      utc[1] = getbits(buff, i, 24) * P2_50;
      i += 24;
      utc[0] = getbits(buff, i, 32) * P2_30;
      i += 32;
      utc[2] = getbits(buff, i, 8) * pow(2, 12);
      i += 8;
      utc[3] = getbitu(buff, i, 8);
    }
    if (leaps) {
      i = 192;
      *leaps = getbits(buff, i, 8);
    }
  }
}
/* decode gps navigation data subframe 5 -------------------------------------*/
static void decode_gps_subfrm5(const unsigned char* buff, alm_t* alm) {
  double toas = 0.0;
  int i = 0;
  int sat = 0;
  int week = 0;
  int svid = getbitu(buff, 50, 6);

  if (1 <= svid && svid <= 24) { /* page 1-24 */
    /* decode almanac */
    sat = getbitu(buff, 50, 6);
    if (1 <= sat && sat <= 32) {
      decode_almanac(buff, sat, alm);
    }
  } else if (svid == 51) { /* page 25 */
    if (alm) {
      i = 56;
      toas = getbitu(buff, i, 8) * 4096;
      i += 8;
      week = getbitu(buff, i, 8);
      i += 8;
      week = adjgpsweek(week);

      /* decode sv health */
      for (sat = 1; sat <= 24; sat++) {
        alm[sat - 1].svh = getbitu(buff, i, 6);
        i += 6;
      }
      for (sat = 1; sat <= 32; sat++) {
        alm[sat - 1].toas = toas;
        alm[sat - 1].week = week;
        alm[sat - 1].toa = gpst2time(week, toas);
      }
    }
  }
}
/* decode qzss navigation data subframe 4/5 ----------------------------------*/
static void decode_qzs_subfrm45(const unsigned char* buff, alm_t* alm,
                                double* ion, double* utc, int* leaps) {
  int i = 0;
  int j = 0;
  int sat = 0;
  int toas = 0;
  int week = 0;
  int svid = getbitu(buff, 50, 6);

  if (1 <= svid && svid <= 5) { /* qzss almanac */
    if (!(sat = satno(SYS_QZS, 192 + svid))) {
      return;
    }
    decode_almanac(buff, sat, alm);
  } else if (svid == 51) { /* qzss health */
    if (alm) {
      i = 56;
      toas = getbitu(buff, i, 8) * 4096;
      i += 8;
      week = getbitu(buff, i, 8);
      i += 8;
      week = adjgpsweek(week);

      for (j = 0; j < 5; ++j) {
        if (!(sat = satno(SYS_QZS, 193 + j))) {
          continue;
        }
        alm[sat - 1].toas = toas;
        alm[sat - 1].week = week;
        alm[sat - 1].toa = gpst2time(week, toas);
        alm[sat - 1].svh = getbitu(buff, i, 6);
        i += 6;
      }
    }
  } else if (svid == 56) { /* ion/utc parameters */
    if (ion) {
      i = 56;
      ion[0] = getbits(buff, i, 8) * P2_30;
      i += 8;
      ion[1] = getbits(buff, i, 8) * P2_27;
      i += 8;
      ion[2] = getbits(buff, i, 8) * P2_24;
      i += 8;
      ion[3] = getbits(buff, i, 8) * P2_24;
      i += 8;
      ion[4] = getbits(buff, i, 8) * pow(2, 11);
      i += 8;
      ion[5] = getbits(buff, i, 8) * pow(2, 14);
      i += 8;
      ion[6] = getbits(buff, i, 8) * pow(2, 16);
      i += 8;
      ion[7] = getbits(buff, i, 8) * pow(2, 16);
    }
    if (utc) {
      i = 120;
      utc[1] = getbits(buff, i, 24) * P2_50;
      i += 24;
      utc[0] = getbits(buff, i, 32) * P2_30;
      i += 32;
      utc[2] = getbits(buff, i, 8) * pow(2, 12);
      i += 8;
      utc[3] = getbitu(buff, i, 8);
    }
  }
}
/* decode gps/qzss navigation data subframe 4 --------------------------------*/
static int decode_subfrm4(const unsigned char* buff, alm_t* alm, double* ion,
                          double* utc, int* leaps) {
  int dataid = getbitu(buff, 48, 2);

  trace(4, "decode_subfrm4: dataid=%d\n", dataid);
  trace(5, "decode_subfrm4: buff=");
  traceb(5, buff, 30);

  if (dataid == 1) { /* gps */
    decode_gps_subfrm4(buff, alm, ion, utc, leaps);
  } else if (dataid == 3) { /* qzss */
    decode_qzs_subfrm45(buff, alm, ion, utc, leaps);
  }
  return 4;
}
/* decode gps/qzss navigation data subframe 5 --------------------------------*/
static int decode_subfrm5(const unsigned char* buff, alm_t* alm, double* ion,
                          double* utc, int* leaps) {
  int dataid = getbitu(buff, 48, 2);

  trace(4, "decode_subfrm5: dataid=%d\n", dataid);
  trace(5, "decode_subfrm5: buff=");
  traceb(5, buff, 30);

  if (dataid == 1) { /* gps */
    decode_gps_subfrm5(buff, alm);
  } else if (dataid == 3) { /* qzss */
    decode_qzs_subfrm45(buff, alm, ion, utc, leaps);
  }
  return 5;
}
/* decode gps/qzss navigation data frame ---------------------------------------
* decode navigation data frame and extract ephemeris and ion/utc parameters
* args   : unsigned char *buff I gps navigation data frame (without parity)
*                                  buff[0-29]: 24 bits x 10 words
*          eph_t *eph    IO     ephemeris message      (NULL: no input)
*          alm_t *alm    IO     almanac                (NULL: no input)
*          double *ion   IO     ionospheric parameters (NULL: no input)
*          double *utc   IO     delta-utc parameters   (NULL: no input)
*          int   *leaps  IO     leap seconds (s)       (NULL: no input)
* return : status (0:no valid, 1-5:subframe id)
* notes  : use cpu time to resolve modulo 1024 ambiguity of the week number
*          see ref [1]
*          utc[3] reference week for utc parameter is truncated in 8 bits
*          ion and utc parameters by qzss indicate local iono and qzst-utc
*          parameters.
*-----------------------------------------------------------------------------*/
extern int decode_frame(const unsigned char* buff, eph_t* eph, alm_t* alm,
                        double* ion, double* utc, int* leaps) {
  int id = getbitu(buff, 43, 3); /* subframe id */

  trace(3, "decodefrm: id=%d\n", id);

  switch (id) {
  case 1:
    return decode_subfrm1(buff, eph);
  case 2:
    return decode_subfrm2(buff, eph);
  case 3:
    return decode_subfrm3(buff, eph);
  case 4:
    return decode_subfrm4(buff, alm, ion, utc, leaps);
  case 5:
    return decode_subfrm5(buff, alm, ion, utc, leaps);
  }
  return 0;
}
/* initialize receiver raw data control ----------------------------------------
* initialize receiver raw data control struct and reallocate obsevation and
* epheris buffer
* args   : raw_t  *raw   IO     receiver raw data control struct
* return : status (1:ok,0:memory allocation error)
*-----------------------------------------------------------------------------*/
extern int init_raw(raw_t* raw) {
  const double lam_glo[NFREQ] = {CLIGHT / FREQ1_GLO, CLIGHT / FREQ2_GLO};
  gtime_t time0 = {0};
  obsd_t data0 = {{0}};
  eph_t eph0 = {0, -1, -1};
  alm_t alm0 = {0, -1};
  geph_t geph0 = {0, -1};
  seph_t seph0 = {0};
  sbsmsg_t sbsmsg0 = {0};
  lexmsg_t lexmsg0 = {0};
  int i = 0;
  int j = 0;
  int sys = 0;

  trace(3, "init_raw:\n");

  raw->time = raw->tobs = time0;
  raw->ephsat = 0;
  raw->sbsmsg = sbsmsg0;
  raw->msgtype[0] = '\0';
  for (i = 0; i < MAXSAT; ++i) {
    for (j = 0; j < 380; ++j) {
      raw->subfrm[i][j] = 0;
    }
    for (j = 0; j < NFREQ; ++j) {
      raw->lockt[i][j] = 0.0;
    }
    for (j = 0; j < NFREQ; ++j) {
      raw->halfc[i][j] = 0;
    }
    raw->icpp[i] = raw->off[i] = raw->prCA[i] = raw->dpCA[i] = 0.0;
  }
  for (i = 0; i < MAXOBS; ++i) {
    raw->freqn[i] = 0;
  }
  raw->lexmsg = lexmsg0;
  raw->icpc = 0.0;
  raw->nbyte = raw->len = 0;
  raw->iod = raw->flag = raw->tbase = raw->outtype = 0;
  raw->tod = -1;
  for (i = 0; i < MAXRAWLEN; ++i) {
    raw->buff[i] = 0;
  }
  raw->opt[0] = '\0';
  raw->receive_time = 0.0;
  raw->plen = raw->pbyte = raw->page = raw->reply = 0;
  raw->week = 0;

  raw->obs.data = NULL;
  raw->obuf.data = NULL;
  raw->nav.eph = NULL;
  raw->nav.alm = NULL;
  raw->nav.geph = NULL;
  raw->nav.seph = NULL;

  if (!(raw->obs.data = (obsd_t*)malloc(sizeof(obsd_t) * MAXOBS)) ||
      !(raw->obuf.data = (obsd_t*)malloc(sizeof(obsd_t) * MAXOBS)) ||
      !(raw->nav.eph = (eph_t*)malloc(sizeof(eph_t) * MAXSAT)) ||
      !(raw->nav.alm = (alm_t*)malloc(sizeof(alm_t) * MAXSAT)) ||
      !(raw->nav.geph = (geph_t*)malloc(sizeof(geph_t) * NSATGLO)) ||
      !(raw->nav.seph = (seph_t*)malloc(sizeof(seph_t) * NSATSBS * 2))) {
    free_raw(raw);
    return 0;
  }
  raw->obs.n = 0;
  raw->obuf.n = 0;
  raw->nav.n = MAXSAT;
  raw->nav.na = MAXSAT;
  raw->nav.ng = NSATGLO;
  raw->nav.ns = NSATSBS * 2;
  for (i = 0; i < MAXOBS; ++i) {
    raw->obs.data[i] = data0;
  }
  for (i = 0; i < MAXOBS; ++i) {
    raw->obuf.data[i] = data0;
  }
  for (i = 0; i < MAXSAT; ++i) {
    raw->nav.eph[i] = eph0;
  }
  for (i = 0; i < MAXSAT; ++i) {
    raw->nav.alm[i] = alm0;
  }
  for (i = 0; i < NSATGLO; ++i) {
    raw->nav.geph[i] = geph0;
  }
  for (i = 0; i < NSATSBS * 2; ++i) {
    raw->nav.seph[i] = seph0;
  }
  for (i = 0; i < MAXSAT; ++i) {
    for (j = 0; j < NFREQ; ++j) {
      if (!(sys = satsys(i + 1, NULL))) {
        continue;
      }
      raw->nav.lam[i][j] = sys == SYS_GLO ? lam_glo[j] : lam_carr[j];
    }
  }
  raw->sta.name[0] = raw->sta.marker[0] = '\0';
  raw->sta.antdes[0] = raw->sta.antsno[0] = '\0';
  raw->sta.rectype[0] = raw->sta.recver[0] = raw->sta.recsno[0] = '\0';
  raw->sta.antsetup = raw->sta.itrf = raw->sta.deltype = 0;
  for (i = 0; i < 3; ++i) {
    raw->sta.pos[i] = raw->sta.del[i] = 0.0;
  }
  raw->sta.hgt = 0.0;
  return 1;
}
/* free receiver raw data control ----------------------------------------------
* free observation and ephemeris buffer in receiver raw data control struct
* args   : raw_t  *raw   IO     receiver raw data control struct
* return : none
*-----------------------------------------------------------------------------*/
extern void free_raw(raw_t* raw) {
  trace(3, "free_raw:\n");

  free(raw->obs.data);
  raw->obs.data = NULL;
  raw->obs.n = 0;
  free(raw->obuf.data);
  raw->obuf.data = NULL;
  raw->obuf.n = 0;
  free(raw->nav.eph);
  raw->nav.eph = NULL;
  raw->nav.n = 0;
  free(raw->nav.alm);
  raw->nav.alm = NULL;
  raw->nav.na = 0;
  free(raw->nav.geph);
  raw->nav.geph = NULL;
  raw->nav.ng = 0;
  free(raw->nav.seph);
  raw->nav.seph = NULL;
  raw->nav.ns = 0;
}
/* input receiver raw data from stream -----------------------------------------
* fetch next receiver raw data and input a message from stream
* args   : raw_t  *raw   IO     receiver raw data control struct
*          int    format I      receiver raw data format (STRFMT_???)
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter, 31: input lex message)
*-----------------------------------------------------------------------------*/
extern int input_raw(raw_t* raw, int format, unsigned char data) {
  trace(5, "input_raw: format=%d data=0x%02x\n", format, data);

  switch (format) {
  case STRFMT_OEM4:
    return input_oem4(raw, data);
  case STRFMT_OEM3:
    return input_oem3(raw, data);
    //        case STRFMT_UBX  : return input_ubx  (raw,data);
    //        case STRFMT_SS2  : return input_ss2  (raw,data);
    //        case STRFMT_CRES : return input_cres (raw,data);
    //        case STRFMT_STQ  : return input_stq  (raw,data);
    //        case STRFMT_GW10 : return input_gw10 (raw,data);
    //        case STRFMT_JAVAD: return input_javad(raw,data);
    //        case STRFMT_NVS  : return input_nvs  (raw,data);
    //        case STRFMT_BINEX: return input_bnx  (raw,data);
    //        case STRFMT_RT17 : return input_rt17 (raw,data);
    //        case STRFMT_LEXR : return input_lexr (raw,data);
  }
  return 0;
}
/* input receiver raw data from file -------------------------------------------
* fetch next receiver raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          int    format I      receiver raw data format (STRFMT_???)
*          FILE   *fp    I      file pointer
* return : status(-2: end of file/format error, -1...31: same as above)
*-----------------------------------------------------------------------------*/
extern int input_rawf(raw_t* raw, int format, FILE* fp) {
  trace(4, "input_rawf: format=%d\n", format);

  switch (format) {
  case STRFMT_OEM4:
    return input_oem4f(raw, fp);
  case STRFMT_OEM3:
    return input_oem3f(raw, fp);
    //        case STRFMT_UBX  : return input_ubxf  (raw,fp);
    //        case STRFMT_SS2  : return input_ss2f  (raw,fp);
    //        case STRFMT_CRES : return input_cresf (raw,fp);
    //        case STRFMT_STQ  : return input_stqf  (raw,fp);
    //        case STRFMT_GW10 : return input_gw10f (raw,fp);
    //        case STRFMT_JAVAD: return input_javadf(raw,fp);
    //        case STRFMT_NVS  : return input_nvsf  (raw,fp);
    //        case STRFMT_BINEX: return input_bnxf  (raw,fp);
    //        case STRFMT_RT17 : return input_rt17f (raw,fp);
    //        case STRFMT_LEXR : return input_lexrf (raw,fp);
  }
  return -2;
}
