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

/**
 * \file stdint.h
 * Visual Studio C99 layer: <stdint.h>
 * This is an incomplete but sufficient implementation of stdint.h.
 */

#ifndef STDINT_H
#define STDINT_H

typedef __int8 int8_t;
typedef unsigned __int8 uint8_t;
typedef __int8 int_least8_t;
typedef unsigned __int8 uint_least8_t;
typedef __int8 int_fast8_t;
typedef unsigned __int8 uint_fast8_t;
#define INT8_MIN                -128
#define INT8_MAX                127
#define UINT8_MAX               255
#define INT_LEAST8_MIN          INT8_MIN
#define INT_LEAST8_MAX          INT8_MAX
#define UINT_LEAST8_MAX         UINT8_MAX
#define INT_FAST8_MIN           INT8_MIN
#define INT_FAST8_MAX           INT8_MAX
#define UINT_FAST8_MAX          UINT8_MAX

typedef __int16 int16_t;
typedef unsigned __int16 uint16_t;
typedef __int16 int_least16_t;
typedef unsigned __int16 uint_least16_t;
typedef __int16 int_fast16_t;
typedef unsigned __int16 uint_fast16_t;
#define INT16_MIN               -32768
#define INT16_MAX               32767
#define UINT16_MAX              65535
#define INT_LEAST16_MIN         INT16_MIN
#define INT_LEAST16_MAX         INT16_MAX
#define UINT_LEAST16_MAX        UINT16_MAX
#define INT_FAST16_MIN          INT16_MIN
#define INT_FAST16_MAX          INT16_MAX
#define UINT_FAST16_MAX         UINT16_MAX

typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef __int32 int_least32_t;
typedef unsigned __int32 uint_least32_t;
typedef __int32 int_fast32_t;
typedef unsigned __int32 uint_fast32_t;
#define INT32_MIN               -2147483648L
#define INT32_MAX               2147483647L
#define UINT32_MAX              4294967295UL
#define INT_LEAST32_MIN         INT32_MIN
#define INT_LEAST32_MAX         INT32_MAX
#define UINT_LEAST32_MAX        UINT32_MAX
#define INT_FAST32_MIN          INT32_MIN
#define INT_FAST32_MAX          INT32_MAX
#define UINT_FAST32_MAX         UINT32_MAX

typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
typedef __int64 int_least64_t;
typedef unsigned __int64 uint_least64_t;
typedef __int64 int_fast64_t;
typedef unsigned __int64 uint_fast64_t;
#define INT64_MIN               -9223372036854775808LL
#define INT64_MAX               9223372036854775807LL
#define UINT64_MAX              18446744073709551615ULL
#define INT_LEAST64_MIN         INT64_MIN
#define INT_LEAST64_MAX         INT64_MAX
#define UINT_LEAST64_MAX        UINT64_MAX
#define INT_FAST64_MIN          INT64_MIN
#define INT_FAST64_MAX          INT64_MAX
#define UINT_FAST64_MAX         UINT64_MAX

#ifdef _WIN64
typedef signed __int64 intptr_t;
typedef unsigned __int64 uintptr_t;
#define INTPTR_MIN  INT64_MIN
#define INTPTR_MAX  INT64_MAX
#define UINTPTR_MAX UINT64_MAX
#else
typedef __w64 signed int intptr_t;
typedef __w64 unsigned int uintptr_t;
#define INTPTR_MIN  INT32_MIN
#define INTPTR_MAX  INT32_MAX
#define UINTPTR_MAX UINT32_MAX
#endif

#endif /* STDINT_H */

