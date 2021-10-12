/*-------------------------------------------------------------------------
  Fast CRC16 Calculation

   Written By - Dipl.-Ing. (FH) Michael Schmitt
    michael.schmitt@t-online.de

    Initial Version Mai 2000

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation; either version 2, or (at your option) any
   later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

   In other words, you are welcome to use, share and improve this program.
   You are forbidden to forbid anyone else to use, share and improve
   what you give them.   Help stamp out software-hoarding!
-------------------------------------------------------------------------*/

/********************************************************************/
/* Function    : CRC16 Calculation Header File                 FILE */
/*------------------------------------------------------------------*/
/* Description :                                                    */
/*------------------------------------------------------------------*/
/* Author      : Michael Schmitt                                    */
/*------------------------------------------------------------------*/
/* Input       : none                                               */
/*------------------------------------------------------------------*/
/* Returnvalue : none                                               */
/*------------------------------------------------------------------*/
/* History     : 00/05    V1.0 Initial Version                      */
/*                                                                  */
/********************************************************************/

// INCLUDES & DEFINES ===============================================

#include <stdint.h>

#ifndef __FILE_CRC16_H
#define __FILE_CRC16_H

/* CRC-TABLE CRCINIT=0x0000 POLY=0x1021 CCITT V.41 */
//#define CRC16_CCITT

/* CRC-TABLE CRCINIT=0x0000 POLY=0x8005 */
//#undef CRC16_CCITT

uint16_t CRC16Calc(uint16_t Crc16Value, uint16_t DataByte);
uint16_t CRC16Init(void);
uint16_t ComputeCRC16(uint16_t size, uint16_t * buf);

#endif
