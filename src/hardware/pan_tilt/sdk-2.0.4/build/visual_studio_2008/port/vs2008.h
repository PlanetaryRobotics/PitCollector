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
 * \file vs2008.h
 * Visual Studio 2008 porting header
 *
 * VS2008 does not comply with many standards, like C99 and POSIX. This file
 * implements or redirects these missing calls for VS2008.
 */

#ifndef VS2008_H
#define VS2008_H

#include <stddef.h>

/* POSIX.1-2001: strcasecmp */
#define strcasecmp(a,b)         _stricmp(a,b)
/* POSIX.1-2001: strncasecmp */
#define strncasecmp(a,b,c)      _strnicmp(a,b,c)
/* C99: snprintf */
#define snprintf(s,n,fmt,...)   _snprintf(s,n,fmt,__VA_ARGS__)

#endif /* VS2008_H */

