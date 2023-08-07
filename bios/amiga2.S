/*
 * amiga2.S - Amiga specific assembler functions
 *
 * Copyright (C) 2013-2022 The EmuTOS development team
 *
 * Authors:
 *  VRI   Vincent Rivière
 *
 * This file is distributed under the GPL, version 2 or at your
 * option any later version.  See doc/license.txt for details.
 */

#include "asmdefs.h"

#ifdef MACHINE_AMIGA

        .extern _amiga_rs232_rbf_interrupt
        .extern _amiga_screen_width_in_bytes
        .extern _amiga_screenbase
        .extern _int_vbl
        .extern _kbdvecs
        .extern _longframe
        .extern _scancode_atari_from_amiga
        .extern _amiga_mouse_poll
        .extern _amiga_joystick_poll
        
/******************************************************************************/
/* Startup                                                                    */
/******************************************************************************/

        .globl  amiga_startup
amiga_startup:
        move.l  #0x020000,D0
delay:  subq.l  #1,D0                   // Delay loop.
        jgt     delay

        // Set up port A on the first CIA (8520-A).

        move.b  #3,0xbfe201             // Set low two bits for output.
        move.b  #2,0xbfe001             // Set boot ROM off, power light dim.

        // Disable interrupts and DMA.

        lea     0xdff000,a0             // Base address of custom chip area.
        move.w  #0x7fff,d0
        move.w  d0,0x9a(a0)             // Disable all interrupts.
        move.w  d0,0x9c(a0)             // Clear all pending interrupts.
        move.w  d0,0x96(a0)             // Disable all DMA.
        bclr.b  #7,0xdaa000             // Disable Gayle interrupts (IDE)

        // Clear CIAA and CIAB
        move.b  #0,0xbfee01             // CIAA control register A
        move.b  #0,0xbfef01             // CIAA control register B
        move.b  #0,0xbfde00             // CIAB control register A
        move.b  #0,0xbfdf00             // CIAB control register B 

        // clear CIA interrupts (ICR)
        move.b  #0b01111111,0xbfed01    // CIAA
        move.b  #0b01111111,0xbfdd00    // CIAB

        // clear prescalers
        move.b  #0x00,0xbfe401 // CIAA
        move.b  #0x00,0xbfe501
        move.b  #0x00,0xbfe601
        move.b  #0x00,0xbfe701

        move.b  #0x00,0xbfd400 // CIAB
        move.b  #0x00,0xbfd500
        move.b  #0x00,0xbfd600
        move.b  #0x0d,0xbfd700

        jmp     (a6)                    // Return

/******************************************************************************/
/* memconf                                                                    */
/* Freely inspired from Markus Wandel's Exec disassembly                      */
/* https://wandel.ca/homepage/execdis/exec_disassembly.txt                     */
/******************************************************************************/

        .globl  amiga_memconf
amiga_memconf:
        lea     0,a0                    // Start looking at location 0.
        lea     0x800000,a1             // Don't look past 8 megabytes.

checkmem:
        // On entry, A0 is the lower bound of the area to check, and A1 is
        // the high bound.  Memory is checked in 4K blocks.

        moveq   #0,d1
        move.l  d1,(a0)                 // Write a zero to the first location.
        move.l  a0,a2                   // Save the first location.
        move.l  #0xf2d4b698,d0          // Use this as a signature value.

        // Main loop:  We enter here to check each 4K block.

checkmore:
        lea     0x1000(a0),a0           // Increment current location by 4K.
        cmp.l   a0,a1                   // See if upper bound reached.
        jls     exitcheck               // If so, exit from the loop.
        move.l  d0,(a0)                 // Write the signature into memory.

        // Longword 0 of the block being checked was initially cleared to
        // zero.  If it is now no longer zero, we have "wrapped around",
        // i.e. due to incomplete address decoding, we have written the
        // signature value at the beginning of the block.  When this
        // occurs, we have reached the end of memory, even though the
        // signature value would read back correctly.

        tst.l   (a2)                    // Check location 0.
        jne     exitcheck               // Exit if signature appears there.
        cmp.l   (a0),d0                 // See if signature can be read back.
        jeq     checkmore               // If successful, go check more memory.

exitcheck:
        move.l  a0,d5                   // this is phystop - the end of RAM
        moveq   #0,d6                   // fake memctrl

        jmp     (a6)                    // Return

/******************************************************************************/
/* Interrupt 2: Keyboard                                                      */
/******************************************************************************/

        .globl  _amiga_init_keyboard_interrupt
_amiga_init_keyboard_interrupt:
        move.l  #_amiga_int_2,0x68.w
        move.l  #_amiga_int_6,0x78.w

        move.w  #0x89,0xbfed01          // CIAA Enable keyboard and Timer A IRQ
   
        move.b  #0b00000001,0xbfdf00    // CIAB Start Timer B (200Hz timer)
        move.b  #0b10000010,0xbfdd00    // CIAB Enable Timer B IRQ

        move.w  #0xe008,0xdff09a        // INTENA INTEN + CIAA + CIAB
        rts

/* Interrupt 2 handler */

        .globl  _amiga_int_2
_amiga_int_2:
        movem.l d0-d1/a0-a1,-(sp)
        move.w #0x2700,sr

        move.w  0xdff01e,d0             // INTREQR
        btst    #3,d0                   // PORTS interrupt?
        jbeq    int2_end

        lea     0xbfee01,a0
        move.b  -0x100(a0),d0           // CIAA?
        jbpl    int2_end                // bit 7 not set; no CIA interrupt

ciaa_ta:lsr.w   #1,d0
        jbcc    ciaa_sp                 // bit 0 not set means no Timer A interrupt

        clr.b   (a0)                    // SPMODE = 0, end keyboard acknowledge

ciaa_sp:lsr.w   #3,d0
        jbcc    int2_end                // bit 3 not set means no serial interrupt

        move.b  -0x200(a0),d0           // read character
        move.b  #0x59,(a0)              // start ack (SPMODE out) and Timer A (one shot)

        not.b   d0
        ror.b   #1,d0
        move.b  d0,d1
        andi.b  #0x80,d0
        andi.w  #0x007f,d1
        lea     _scancode_atari_from_amiga,a0
        or.b    0(a0,d1.w),d0

        move.l  _kbdvecs-4,a0           // kbdvec
        jsr     (a0)

        move.w  #0x0008,0xdff09c        // Ack PORTS
        move.w  #0x0008,0xdff09c        // 2CCK delay (propagate IPL from Paula to CPU)

int2_end:
        movem.l (sp)+,d0-d1/a0-a1
        rte

/******************************************************************************/
/* Interrupt 5: RS-232 interrupt                                              */
/******************************************************************************/

/* Interrupt 5 handler */

        .globl  _amiga_int_5
_amiga_int_5:
        movem.l d0-d1/a0-a1,-(sp)

        move.w  0xdff01e,d0             // INTREQR
        btst    #11,d0                  // RBF interrupt?
        jeq     amiga_int_5_end

        jbsr    _amiga_rs232_rbf_interrupt

        move.w  #0x0800,0xdff09c        // INTREQ = interrupt processed.
        move.w  #0x0800,0xdff09c        // twice for compatibility

amiga_int_5_end:
        movem.l (sp)+,d0-d1/a0-a1
        rte

/******************************************************************************/
/* Interrupt 6: 200 Hz timer                                                  */
/******************************************************************************/

/* Interrupt 6 handler */

        .globl  _amiga_int_6
_amiga_int_6:
        movem.l d0-d1/a0-a1,-(sp)

        move.w  0xdff01e,d0             // INTREQR
        btst    #13,d0                  // EXTER interrupt?
        jeq     amiga_int_6_end

        move.b  0xbfdd00,d0             // CIAB?
        jbpl    amiga_int_6_end

        lsr.w   #2,d0
        jbcc    amiga_int_6_end         // bit 1 not set means no Timer B interrupt

        jbsr    _amiga_mouse_poll
        jbsr    _amiga_joystick_poll

        jsr     _amiga_int_ciab_timer_b
  
        move.w  #0x2000,0xdff09c        // INTREQ = interrupt processed.
        move.w  #0x2000,0xdff09c        // twice for compatibility

amiga_int_6_end:
        movem.l (sp)+,d0-d1/a0-a1
        rte

/* CIA A Timer B interrupt (for 200 Hz system timer) */

_amiga_int_ciab_timer_b:
// Call vector_5ms. The return address is already on the stack.
// As it will return with RTE, we must setup a proper stack frame
        tst.w   _longframe              // CPU > 68000, long stack frame?
        jeq     nolong

        subq.l  #2,sp                   // Make room for additional word
        move.l  2(sp),(sp)              // Move return address
        clr.w   4(sp)                   // Format/Vector word
nolong:
        move.w  sr,-(sp)                // Push SR

        .extern _vector_5ms
        move.l  _vector_5ms,a0
        jmp     (a0)                    // Call the 200 Hz system timer

/******************************************************************************/
/* VBL interrupt                                                              */
/******************************************************************************/

        .extern _copper_list

        .globl  _amiga_vbl
_amiga_vbl:
        movem.l d0-d1/a0-a1,-(sp)

        move.w  0xdff01e,d0             // INTREQR
        btst    #5,d0                   // VBL interrupt?
        jeq     amivblend

        move.l  _amiga_screenbase,d0    // Video Base address

        tst.w   0xdff004                // VPOSR: Test bit 15 = LOF bit
        jmi     setcopper               // LOF = 1, odd field: start at line 1

        moveq   #0,d1
        move.w  _amiga_screen_width_in_bytes,d1
        add.l   d1,d0                   // LOF = 0, even field: start at line 2

setcopper:
        move.l  _copper_list,a0
        move.w  d0,10(a0)               // BPL1PTL
        swap    d0
        move.w  d0,6(a0)                // BPL1PTH

        jbsr    _int_vbl                // Call the standard VBL handler

        move.w  #0x0020,0xdff09c        // INTREQ: Clear VBL interrupt
        move.w  #0x0020,0xdff09c        // Twice for compatibility

amivblend:
        movem.l (sp)+,d0-d1/a0-a1
        rte

#endif /* MACHINE_AMIGA */
