/*
 * v4sa.h - V4SA specific functions
 *
 * Copyright (C) 2013-2019 The EmuTOS development team
 *
 * Authors:
 *  PeP   Peter Persson
 *
 * This file is distributed under the GPL, version 2 or at your
 * option any later version.  See doc/license.txt for details.
 */

#ifndef V4SA_H
#define V4SA_H

#ifdef MACHINE_V4SA

/* Custom registers */
#define INTENA  *(volatile UWORD*)0xdff09a
#define INTREQ  *(volatile UWORD*)0xdff09c

/* CIA A registers */
#define CIAAPRA    *(volatile UBYTE*)0xbfe001
#define CIAAPRB    *(volatile UBYTE*)0xbfe101
#define CIAADDRA   *(volatile UBYTE*)0xbfe201
#define CIAADDRB   *(volatile UBYTE*)0xbfe301
#define CIAATALO   *(volatile UBYTE*)0xbfe401
#define CIAATAHI   *(volatile UBYTE*)0xbfe501
#define CIAATBLO   *(volatile UBYTE*)0xbfe601
#define CIAATBHI   *(volatile UBYTE*)0xbfe701
#define CIAATODLO  *(volatile UBYTE*)0xbfe801
#define CIAATODMID *(volatile UBYTE*)0xbfe901
#define CIAATODHI  *(volatile UBYTE*)0xbfea01
#define CIAASDR    *(volatile UBYTE*)0xbfec01
#define CIAAICR    *(volatile UBYTE*)0xbfed01
#define CIAACRA    *(volatile UBYTE*)0xbfee01
#define CIAACRB    *(volatile UBYTE*)0xbfef01

/* CIA B registers */
#define CIABPRA    *(volatile UBYTE*)0xbfd000
#define CIABPRB    *(volatile UBYTE*)0xbfd100
#define CIABDDRA   *(volatile UBYTE*)0xbfd200
#define CIABDDRB   *(volatile UBYTE*)0xbfd300
#define CIABTALO   *(volatile UBYTE*)0xbfd400
#define CIABTAHI   *(volatile UBYTE*)0xbfd500
#define CIABTBLO   *(volatile UBYTE*)0xbfd600
#define CIABTBHI   *(volatile UBYTE*)0xbfd700
#define CIABTODLO  *(volatile UBYTE*)0xbfd800
#define CIABTODMID *(volatile UBYTE*)0xbfd900
#define CIABTODHI  *(volatile UBYTE*)0xbfda00
#define CIABSDR    *(volatile UBYTE*)0xbfdc00
#define CIABICR    *(volatile UBYTE*)0xbfdd00
#define CIABCRA    *(volatile UBYTE*)0xbfde00
#define CIABCRB    *(volatile UBYTE*)0xbfdf00

/* Generic Set/Clear bit */
#define SETBITS  (1U << 15)
#define CLRBITS  (0U << 15)

/* INTREQ / INTENA flags */
#define INTEN    (1U << 14)
#define EXTER    (1U << 13)

#define COREREV *(volatile UWORD*)0xDFF3EA

/* IDE configuration */
#define IDECONF *(volatile UWORD*)0xDD1020

const char *v4sa_machine_name(void);
void v4sa_machine_init(void);

void v4sa_int_1(void);
void v4sa_int_2(void);
void v4sa_int_3(void);
void v4sa_int_4(void);
void v4sa_int_5(void);
void v4sa_int_6(void);
void v4sa_ikbd_writeb(UBYTE b);

#endif /* MACHINE_V4SA */

#endif /* V4SA_H */
