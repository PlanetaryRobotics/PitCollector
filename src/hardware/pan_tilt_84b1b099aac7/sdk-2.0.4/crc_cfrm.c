/*****************************************************************************
*****              (C)2014 FLIR Commercial Systems, Inc.                 *****
*****                       All Rights Reserved.                         *****
*****                                                                    *****
*****     This source data and code (the "Code") may NOT be distributed  *****
*****     without the express prior written permission consent from of   *****
*****     FLIR Commercial Systems, Inc. ("FLIR").  FLIR PROVIDES THIS    *****
*****     CODE ON AN "AS IS" BASIS FOR USE BY RECIPIENT AT ITS OWN       *****
*****     RISK.  FLIR DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, IMPLIED *****
*****     OR STATUTORY, INCLUDING WITHOUT LIMITATION ANY IMPLIED         *****
*****     WARRANTIES OF TITLE, NON-INFRINGEMENT OF THIRD PARTY RIGHTS,   *****
*****     MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.          *****
*****     FLIR Commercial Systems, Inc. reserves the right to make       *****
*****     changes without further notice to the Code or any content      *****
*****     herein including to improve reliability, function or design.   *****
*****     FLIR Commercial Systems, Inc. shall not assume any liability   *****
*****     arising from the application or use of this code, data or      *****
*****     function.                                                      *****
*****                                                                    *****
*****     FLIR Commercial Systems, Inc.                                  *****
*****     Motion Control Systems                                         *****
*****     www.flir.com/mcs                                               *****
*****     mcs-support@flir.com                                           *****
*****************************************************************************/

#include "crc_cfrm.h"

/**
 * \file crc_cfrm.c
 * \brief CRC-16 used in the cfrm module
 *
 * Research into 16bit CRCs found "industry standard" polynomials as less than
 * ideal for protecting messages. This CRC is based off research from Philip
 * Koopman and extensive testing of a PTU on RS-232 and RS-485 serial lines.
 *
 * Below is a "Rocksoft Model" of this custom CRC.
 *
 * <pre>
 * NAME    : "CRC-CFRM"
 * Width   : 16
 * Poly    : 755B
 * Init    : 0341
 * RefIn   : False
 * RefOut  : False
 * XorOut  : 0000
 * Check   : 748F
 * </pre>
 */
#ifdef CRC_CFRM_CHECK
#include <stdio.h>
#endif

/** Inital CRC-CFRM value. */
#define CRC_CFRM_INIT 0x0341L

/** CRC-CFRM lookup table */
static uint16_t crc_cfrm_table[256] = {
    0x0000, 0x755B, 0xEAB6, 0x9FED, 0xA037, 0xD56C, 0x4A81, 0x3FDA,
    0x3535, 0x406E, 0xDF83, 0xAAD8, 0x9502, 0xE059, 0x7FB4, 0x0AEF,
    0x6A6A, 0x1F31, 0x80DC, 0xF587, 0xCA5D, 0xBF06, 0x20EB, 0x55B0,
    0x5F5F, 0x2A04, 0xB5E9, 0xC0B2, 0xFF68, 0x8A33, 0x15DE, 0x6085,
    0xD4D4, 0xA18F, 0x3E62, 0x4B39, 0x74E3, 0x01B8, 0x9E55, 0xEB0E,
    0xE1E1, 0x94BA, 0x0B57, 0x7E0C, 0x41D6, 0x348D, 0xAB60, 0xDE3B,
    0xBEBE, 0xCBE5, 0x5408, 0x2153, 0x1E89, 0x6BD2, 0xF43F, 0x8164,
    0x8B8B, 0xFED0, 0x613D, 0x1466, 0x2BBC, 0x5EE7, 0xC10A, 0xB451,
    0xDCF3, 0xA9A8, 0x3645, 0x431E, 0x7CC4, 0x099F, 0x9672, 0xE329,
    0xE9C6, 0x9C9D, 0x0370, 0x762B, 0x49F1, 0x3CAA, 0xA347, 0xD61C,
    0xB699, 0xC3C2, 0x5C2F, 0x2974, 0x16AE, 0x63F5, 0xFC18, 0x8943,
    0x83AC, 0xF6F7, 0x691A, 0x1C41, 0x239B, 0x56C0, 0xC92D, 0xBC76,
    0x0827, 0x7D7C, 0xE291, 0x97CA, 0xA810, 0xDD4B, 0x42A6, 0x37FD,
    0x3D12, 0x4849, 0xD7A4, 0xA2FF, 0x9D25, 0xE87E, 0x7793, 0x02C8,
    0x624D, 0x1716, 0x88FB, 0xFDA0, 0xC27A, 0xB721, 0x28CC, 0x5D97,
    0x5778, 0x2223, 0xBDCE, 0xC895, 0xF74F, 0x8214, 0x1DF9, 0x68A2,
    0xCCBD, 0xB9E6, 0x260B, 0x5350, 0x6C8A, 0x19D1, 0x863C, 0xF367,
    0xF988, 0x8CD3, 0x133E, 0x6665, 0x59BF, 0x2CE4, 0xB309, 0xC652,
    0xA6D7, 0xD38C, 0x4C61, 0x393A, 0x06E0, 0x73BB, 0xEC56, 0x990D,
    0x93E2, 0xE6B9, 0x7954, 0x0C0F, 0x33D5, 0x468E, 0xD963, 0xAC38,
    0x1869, 0x6D32, 0xF2DF, 0x8784, 0xB85E, 0xCD05, 0x52E8, 0x27B3,
    0x2D5C, 0x5807, 0xC7EA, 0xB2B1, 0x8D6B, 0xF830, 0x67DD, 0x1286,
    0x7203, 0x0758, 0x98B5, 0xEDEE, 0xD234, 0xA76F, 0x3882, 0x4DD9,
    0x4736, 0x326D, 0xAD80, 0xD8DB, 0xE701, 0x925A, 0x0DB7, 0x78EC,
    0x104E, 0x6515, 0xFAF8, 0x8FA3, 0xB079, 0xC522, 0x5ACF, 0x2F94,
    0x257B, 0x5020, 0xCFCD, 0xBA96, 0x854C, 0xF017, 0x6FFA, 0x1AA1,
    0x7A24, 0x0F7F, 0x9092, 0xE5C9, 0xDA13, 0xAF48, 0x30A5, 0x45FE,
    0x4F11, 0x3A4A, 0xA5A7, 0xD0FC, 0xEF26, 0x9A7D, 0x0590, 0x70CB,
    0xC49A, 0xB1C1, 0x2E2C, 0x5B77, 0x64AD, 0x11F6, 0x8E1B, 0xFB40,
    0xF1AF, 0x84F4, 0x1B19, 0x6E42, 0x5198, 0x24C3, 0xBB2E, 0xCE75,
    0xAEF0, 0xDBAB, 0x4446, 0x311D, 0x0EC7, 0x7B9C, 0xE471, 0x912A,
    0x9BC5, 0xEE9E, 0x7173, 0x0428, 0x3BF2, 0x4EA9, 0xD144, 0xA41F
};


/** Run CRC-CFRM on \c data for \c len bytes.
 *
 * \param data  Input data to calculate
 * \param len   Length of the input data
 *
 * \returns a 16 bit CRC of \c data.
 */
uint16_t crc_cfrm(const void *data, int len){
   unsigned char *d = (unsigned char *)data;
   uint16_t crc = CRC_CFRM_INIT;

   while(len--){
       crc = (crc<<8) ^ crc_cfrm_table[(crc>>8) ^ *d++];
   }

   return crc;
}

#ifdef CRC_CFRM_CHECK
int main(void){
   uint16_t crc;

   crc = crc_cfrm("123456789", 9);
   printf("%s [%04lX]\n",
       crc == 0x748F ? "PASS" : "FAIL",
       (long)crc);

   return 0;
}
#endif

