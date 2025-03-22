/*
 * v4sa2.S - V4SA specific assembler functions
 *
 * Copyright (C) 2024 The EmuTOS development team
 *
 * Authors:
 *  PEP   Peter Persson
 *
 * This file is distributed under the GPL, version 2 or at your
 * option any later version.  See doc/license.txt for details.
 */

#include "asmdefs.h"

#ifdef MACHINE_V4SA

#ifdef FEJK
        .equ    ikbd_acia_stat, 0x00000700
        .equ    ikbd_acia_data, 0x00000702
        .equ    midi_acia_stat, 0x00000704
        .equ    midi_acia_data, 0x00000706
#else
        .equ    ikbd_acia_stat, 0x00fffc00
        .equ    ikbd_acia_data, 0x00fffc02
        .equ    midi_acia_stat, 0x00fffc04
        .equ    midi_acia_data, 0x00fffc06
#endif

/******************************************************************************/
/* Startup                                                                    */
/******************************************************************************/

        .globl  v4sa_startup

#ifdef FEJK
never_called:
#else
v4sa_startup:
#endif
        reset

        move.l  #0x020000,D0
delay:  subq.l  #1,D0                   // Delay loop.
        jgt     delay

        // some code to make this run on Hatari,
        clr.w   0xdff3fc                // VREG_BOARD (read only)
        tst.w   0xdff3fc
        jeq     not_v4

        move.l  0x00e00000,0x0.w        // Support reset routines
        move.l  0x00e00004,0x4.w        // ... by initialising initial PC/SP

not_v4: move.w #0xaaaa,0xdff3fe         // Enable Atari-like features

        // Set up port A on the first CIA (8520-A).

        move.b  #3,0xbfe201             // Set low two bits for output.
        move.b  #2,0xbfe001             // Set boot ROM off, power light dim.

        // Disable interrupts and DMA.

        lea     0xdff000,a0             // Base address of custom chip area.
        move.w  #0x7fff,d0
        move.w  d0,0x9a(a0)             // Disable Amiga interrupts (INTENA)
        move.w  d0,0x29a(a0)            // Disable SAGA interrupts (INTENA2)
        move.w  d0,0x9c(a0)             // Clear pending Amiga interrupts (INTREQ)
        move.w  d0,0x29c(a0)            // Clear pending SAGA interrupts (INTREQ2)
        move.w  d0,0x96(a0)             // Disable Amiga DMA (DMACON.
        move.w  d0,0x296(a0)            // Disable SAGA DMA (DMACON2).       
        bclr.b  #7,0xdaa000             // Disable Gayle interrupts (IDE)
        move.w  #0x0000,0x1f4(a0)       // Clear SAGA VIDEO
        move.w  #0x0000,0x1fc(a0)       // FMODE: reset to default

        bclr.b  #7,0xdaa000             // Disable Gayle interrupts (IDE)
        move.w  #0x8200,0x96(a0)        // DMA Master bit enable

        // Clear CIAs
        move.b  #0,0xbfee01             // CIAA control register A
        move.b  #0,0xbfef01             // CIAA control register B
        move.b  #0,0xbfde00             // CIAB control register A
        move.b  #0,0xbfdf00             // CIAB control register B 

        // clear CIA interrupts
        move.b  #0b01111111,0xbfed01    // CIAA interrupt control register: Clear all interrupts
        move.b  #0b01111111,0xbfdd00    // CIAB interrupt control register: Clear all interrupts

        // clear prescalers
        move.b  #0x00,0xbfe401 // CIAA
        move.b  #0x00,0xbfe501
        move.b  #0x00,0xbfe601
        move.b  #0x00,0xbfe701

        move.b  #0x00,0xbfd400 // CIAB
        move.b  #0x00,0xbfd500
        move.b  #0x00,0xbfd600
        move.b  #0x0d,0xbfd700

#if 0
        move.b #0x02,ikbd_acia_stat     // No IRQ, TX/RX empty
        move.b #0x00,ikbd_acia_data
        move.b #0x02,midi_acia_stat     // No IRQ, TX/RX empty
        move.b #0x00,midi_acia_data
#else
        move.b #0x83,midi_acia_stat
        move.b #0x00,midi_acia_data
#endif
        move.w #0x0000,0xdd1020         // IDE speed = slow

        jmp     (a6)                    // Return


/******************************************************************************/
/* memconf                                                                    */
/******************************************************************************/

        .globl  v4sa_memconf
v4sa_memconf:
        move.l  #0xB00000,d5            // max. phystop - 11.0MB (0.5MB reserved for video.)
        moveq   #0,d6                   // fake memctrl
        jmp     (a6)                    // Return

/******************************************************************************/
/* Interrupt Emulation                                                        */
/******************************************************************************/

        .macro  atari_irq vector, return
                clr.w   -(sp)
                pea \return(pc)
                move.w sr,-(sp)
                jmp ([\vector])
        .endm

/******************************************************************************/
/* ACIA Emulation                                                             */
/******************************************************************************/
        .globl _acia_ikbd_rx

_acia_ikbd_rx:
        move.l  4(sp),d0
acia_ikbd_rx:
        move.b  #0x02,midi_acia_stat    // No IRQ, TX/RX empty
        move.b  #0x83,ikbd_acia_stat    // IRQ, TX empty, RX full
        move.b  d0,ikbd_acia_data

        move.b  #0x40,d0                // ACIA input
        and.b   0xfffffa09.w,d0         // int. enable B
        and.b   0xfffffa15.w,d0         // int. mask B
        jbeq    2$

        atari_irq 0x118.w, 1$

1$:     move.b #0x02,ikbd_acia_stat     // No IRQ, TX/RX empty
2$:     move.b #0x02,midi_acia_stat     // No IRQ, TX/RX empty
        rts

/******************************************************************************/
/* Audio interrupt                                                            */
/******************************************************************************/

        .globl  _v4sa_int_aud
        .extern _amiga_extra_sound
        .extern _play_int_ch15

_v4sa_int_aud:
        movem.l d0-d1/a0-a1,-(sp)
        move.w #0x2700,sr

        move.w  0xdff21e,d0               // INTREQR2
        btst    #11,d0                    // AUD15?
        jbeq    int4_end

aud15:  
        //jsr     _v4sa_extra_sound        // Call extra sound handler
        //tst.w   _play_int_ch15            // do interrupt ?
        jbeq    aud15_done

        //atari_irq _sound_int_call, aud15_done // 0x134?

aud15_done:
        //lea     0xdff29c,a0
        //move.w  #0x1000,(a0)            // Ack AUD15(INTREQ2)
        //move.w  #0x1000,(a0)            // 2CCK delay (propagate IPL from Paula to CPU)

int4_end:
        movem.l (sp)+,d0-d1/a0-a1
        rte

/******************************************************************************/
/* Interrupt 1: TBE, DISKBLK, SOFTINT                                         */
/******************************************************************************/

        .globl  _v4sa_int_1
_v4sa_int_1:
        move.l d0,-(sp)

        move.w  0xdff01e,d0             // INTREQR
        and.w   #0x0007,d0              // TBE, DISKBLK or SOFTINT?
        jbeq    int1_end

        move.w  d0,0xdff09c             // Ack interrupts (INTREQ)
        move.w  d0,0xdff09c             // 2CCK delay (propagate IPL from Paula to CPU)

int1_end:
        move.l (sp)+,d0
        rte

/******************************************************************************/
/* Interrupt 2: PORTS (IDE interrupt, but NOT CIAA                            */
/******************************************************************************/

        .globl  _v4sa_int_2
_v4sa_int_2:
        move.l d0,-(sp)

        move.w  0xdff01e,d0             // INTREQR
        and.w   #0x0008,d0              // PORTS?
        jbeq    int2_end

        move.w  d0,0xdff09c             // Ack interrupts (INTREQ)
        move.w  d0,0xdff09c             // 2CCK delay (propagate IPL from Paula to CPU)

int2_end:
        move.l (sp)+,d0
        rte

/******************************************************************************/
/* Interrupt 3: COPER, VERTB, BLIT (VERTB not generated anymore)              */
/******************************************************************************/

        .globl  _v4sa_int_3

_v4sa_int_3:
        move.l d0,-(sp)

        move.w  0xdff01e,d0             // INTREQR
        and.w   #0x0070,d0              // COPER, VERTB or BLIT?
        jbeq    int3_end

        move.w  d0,0xdff09c             // Ack interrupts (INTREQ)
        move.w  d0,0xdff09c             // 2CCK delay (propagate IPL from Paula to CPU)

int3_end:
        move.l (sp)+,d0
        rte

/******************************************************************************/
/* Interrupt 4: Atari-like VBL only                                           */
/******************************************************************************/
        .extern _IKBD_SendAutoKeyboardCommands
#ifdef FEJK
        .globl  _v4sa_int_4
_v4sa_int_4:
        movem.l d0-d1/a0-a1,-(sp)

        move.b _v4sa_mouseb,d0
        lsl.b  #7,d0
        or.b   joy1_temp,d0
        move.b d0,_v4sa_joy1

        jsr _IKBD_SendAutoKeyboardCommands
        movem.l (sp)+,d0-d1/a0-a1

is_v4:  jmp ([0x70.w]) // jump through Atari VBL vector
#endif
/******************************************************************************/
/* Interrupt 5: RFB, DSKSYNC                                                  */
/******************************************************************************/

        .globl  _v4sa_int_5

_v4sa_int_5:
        move.l d0,-(sp)

        move.w  0xdff01e,d0             // INTREQR
        and.w   #0x1800,d0              // RFB, DSKSYNC?
        jbeq    int5_end

        move.w  d0,0xdff09c             // Ack interrupts (INTREQ)
        move.w  d0,0xdff09c             // 2CCK delay (propagate IPL from Paula to CPU)

int5_end:
        move.l (sp)+,d0
        rte

/******************************************************************************/
/* Interrupt 6: EXTER (CIAA and CIAB)                                         */
/******************************************************************************/

db9conv:
        dc.b 0x00, 0x02, 0x0a, 0x08, 0x01, 0x00, 0x08, 0x09, 0x05, 0x04, 0x00, 0x01, 0x04, 0x06, 0x02, 0x00

        .extern _v4sa_scancodes
        .globl  _v4sa_int_6
_v4sa_int_6:
        movem.l d0-d1/a0,-(sp)
        move.w #0x2700,sr

        move.w  0xdff01e,d0             // INTREQR
        btst    #13,d0                  // EXTER interrupt?
        jbeq    exter_skip

        move.b  0xbfdd00,d0             // CIAB?
        jbpl    kbd

        btst    #0,d0
        jbeq    kbd                     // bit 0 not set means no Timer A interrupt
#if 1
joypad: move.w   0xdff00c,d1            // ---- --Yy ---- --Xx
        move.w   d1,d0
        lsr.w    #1,d0                  // ---- ---Y y--- ---X
        and.w    #0x0101,d0
        eor.w    d0,d1

        lsl.b    #6,d1                  // ---- --lu rd00 0000
        lsl.w    #6,d1                  // lurd 0000 0000 0000

        move.w   0xdff220,d0            // lurd -sb8 7654 321c
        lsl.l    #6,d0                  // ...6 5432 1c00 0000
        move.b   0xbfe001,d0            // bit 7 = fire (inverted)
        not.b    d0                     // ...6 5432 ffff ffff
        lsr.l    #7,d0                  // ---- --sb 8765 432f
        and.w    #0x03ff,d0             // 0000 00sb 8765 432f
        or.w     d1,d0                  // lurd 00sb 8765 432f
        move.w   d0,_v4sa_joyb1

#endif
mouse:  move.b   mousex_old,d0
        move.w   0xdff00a,d1
        move.b   d1,mousex_old
        sub.b    d1,d0
        ext.w    d0
        sub.w    d0,_v4sa_mousex

        move.b   mousey_old,d0
        lsr.w    #8,d1
        move.b   d1,mousey_old
        sub.b    d1,d0
        ext.w    d0
        sub.w    d0,_v4sa_mousey

        move.b   mousew_old,d0
        move.w   0xdff212,d1 // low byte = wheel, high = buttons
        move.b   d1,mousew_old
        sub.b    d1,d0
        ext.w    d0
        sub.w    d0,_v4sa_mousew

        moveq.l  #0,d0
        btst.l   #8,d1       // extra mouse button 1
        seq.b    d0          // 0000 0000 ZZZZ ZZZZ
        lsl.w    #1,d0
        btst.l   #9,d1       // extra mouse button 2
        seq.b    d0          // 0000 000Z XXXX XXXX
        lsl.w    #1,d0

        move.w   0xdff016,d1
        move.b   0xbfe001,d1

        btst.l   #8,d1       // middle mouse button
        seq.b    d0          // 0000 00ZX MMMM MMMM
        lsl.w    #1,d0
        btst.l   #6,d1       // left mouse button
        seq.b    d0          // 0000 0ZXM LLLL LLLL
        lsl.w    #1,d0
        btst.l   #10,d1      // right mouse button
        seq.b    d0          // 0000 ZXML RRRR RRRR
        lsr.w    #7,d0
        move.b   d0,_v4sa_mouseb

        jsr _IKBD_SendAutoKeyboardCommands

kbd:    move.b  0xbfed01,d0             // CIAA?
        jbpl    kbd_done

        btst    #3,d0
        jbeq    kbd_done                // bit 3 not set means no serial interrupt

        move.b  0xbfec01,d0             // read character
        bset    #6,0xbfee01             // start ack
        ror.b   #1,D0
        not.b   D0
        moveq   #50,D1
wack:   tst.b   0xbfe001
        dbra    D1,wack
        bclr    #6,0xbfee01

        move.b  d0,d1
        andi.b  #0x80,d0
        andi.w  #0x007f,d1
        lea     _v4sa_scancodes,a0
        or.b    0(a0,d1.w),d0

        move.l d0,-(sp)               // is this necessary
        jsr _IKBD_PressSTKey
        move.l (sp)+,d0

kbd_done:

        move.w  #0x2000,0xdff09c                 // Ack EXTER (INTREQ)
        move.w  #0x2000,0xdff09c                 // 2CCK delay (propagate IPL from Paula to CPU)

exter_skip:
        movem.l (sp)+,d0-d1/a0
        rte

#ifdef FEJK
        .globl _acia_init
_acia_init:
        move.b #0x03,0xfc00.w // master reset
        move.b #0x96,0xfc00.w // configure and enable interrupts
        rts

v4sa_startup:
        jmp     (a6)

        .globl _acia_int
_acia_int:
        tst.l acia_0x118
        bne.s ok

        move.l #ikbd,acia_0x118

ok:     jmp ([acia_0x118])

ikbd:
        movem.l d0/d1/a0,-(sp)

        | Check if source is IKBD or MIDI
        btst.b  #0,0xfc00.w
        beqs    ikbd_done

        moveb   0xfc02:w,d0

        | Joystick packet ?
        
        cmpb    #0xff,d0
        beqs    ikbd_yes_joystick

        | Mouse packet ?

        cmpb    #0xf8,d0
        bmis    ikbd_no_mouse
        cmpb    #0xfc,d0
        bpls    ikbd_no_mouse

        | Mouse packet, byte #1

ikbd_yes_mouse:
        andb    #3,d0
        moveb   d0,_v4sa_mouseb

        move.l  #ikbd_mousex,acia_0x118
        bras    ikbd_endit_stack

        | Joystick packet, byte #1

ikbd_yes_joystick:
        move.l  #ikbd_joystick,acia_0x118
        bras    ikbd_endit_stack

        | Keyboard press/release

        .extern _IKBD_PressSTKey
ikbd_no_mouse:
        movem.l a0-a1/d0-d1,-(sp)
        jsr _IKBD_PressSTKey
        movem.l (sp)+,a0-a1/d0-d1

        | End of interrupt

ikbd_endit_stack:
        bclr.b #6,0xfa11.w

ikbd_done:
        movem.l (sp)+,d0/d1/a0
        rte


        | Mouse packet, byte #2

ikbd_mousex:
        movem.l d0/d1/a0,-(sp)

        | Check if source is IKBD or MIDI
        btst.b  #0,0xfc00.w
        beqs    ikbd_done

        moveb   0xfc02.w,d0
        //neg.b   d0
        add.w   d0,_v4sa_mousex

        move.l  #ikbd_mousey,acia_0x118
        bras    ikbd_endit_stack

        | Mouse packet, byte #3

ikbd_mousey:
        movem.l d0/d1/a0,-(sp)

        | Check if source is IKBD or MIDI
        btst.b  #0,0xfc00.w
        beqs    ikbd_done

        moveb   0xfc02:w,d0
        add.w   d0,_v4sa_mousey

        move.l #ikbd,acia_0x118
        bra     ikbd_endit_stack

        | Joystick packet, byte #2

ikbd_joystick:
        movem.l d0/d1/a0,-(sp)

        | Check if source is IKBD or MIDI
        btst.b #0,0xfc00.w
        beq     ikbd_done

        move.b  0xfc02.w,joy1_temp

        move.l #ikbd,acia_0x118
        bra     ikbd_endit_stack

        .bss
        .even

acia_0x118:     .ds.l   1
joy1_temp:      .ds.b   1
        .ds.b 1
#endif // FEJK

        .bss
        .even
        .globl _v4sa_mousey
        .globl _v4sa_mousex
        .globl _v4sa_mouseb
        .globl _v4sa_mousew
        .globl _v4sa_joyb0
        .globl _v4sa_joyb1
        .globl _v4sa_joy0
        .globl _v4sa_joy1

mousew_old:     .ds.b   1
mousey_old:     .ds.b   1
mousex_old:     .ds.b   1

_v4sa_mousey:   .ds.w   1
_v4sa_mousex:   .ds.w   1
_v4sa_mouseb:   .ds.b   1
_v4sa_mousew:   .ds.b   1
_v4sa_joyb0:    .ds.w   1
_v4sa_joyb1:    .ds.w   1
_v4sa_joy0:     .ds.b   1 // Shared between mouse/joystick.
_v4sa_joy1:     .ds.b   1 // Joystick only.

#endif /* MACHINE_V4SA*/
