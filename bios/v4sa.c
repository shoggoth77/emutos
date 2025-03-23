#include "emutos.h"
#include "../bdos/bdosstub.h"
#include "videl.h"
#include "v4sa.h"
#include "biosdefs.h"
#include "tosvars.h"
#include "screen.h"
#include "biosext.h"
#include "string.h"
#if defined(MACHINE_V4SA)
#include "v4sa_ikbd.c"
#include "v4sa_video.c"

# define VPROXY_VECTORS ((volatile PFVOID*)0x1C0)
# define VPROXY_NUM_VEC ((0x380 - (ULONG)VPROXY_VECTORS)>>2)

#define CODE(vec_num) { 0x4ef0, 0x01e1, (vec_num) << 2 } /* jmp [(vec_num<<2.w)] */
#define CODE_BLOCK16(vec_base) \
CODE((vec_base) + 0 ), CODE((vec_base) + 1 ), CODE((vec_base) + 2 ), CODE((vec_base) + 3 ), \
CODE((vec_base) + 4 ), CODE((vec_base) + 5 ), CODE((vec_base) + 6 ), CODE((vec_base) + 7 ), \
CODE((vec_base) + 8 ), CODE((vec_base) + 9 ), CODE((vec_base) + 10), CODE((vec_base) + 11), \
CODE((vec_base) + 12), CODE((vec_base) + 13), CODE((vec_base) + 14), CODE((vec_base) + 15)

static const UWORD proxy_handlers[256][3] =
{
    CODE_BLOCK16(16 * 0 ), CODE_BLOCK16(16 * 1 ), CODE_BLOCK16(16 * 2 ), CODE_BLOCK16(16 * 3 ),
    CODE_BLOCK16(16 * 4 ), CODE_BLOCK16(16 * 5 ), CODE_BLOCK16(16 * 6 ), CODE_BLOCK16(16 * 7 ),
    CODE_BLOCK16(16 * 8 ), CODE_BLOCK16(16 * 9 ), CODE_BLOCK16(16 * 10), CODE_BLOCK16(16 * 11),
    CODE_BLOCK16(16 * 12), CODE_BLOCK16(16 * 13), CODE_BLOCK16(16 * 14), CODE_BLOCK16(16 * 15),
};

void v4sa_ikbd_writeb(UBYTE b)
{
    IKBD_Process_RDR(b);
}

void v4sa_machine_init(void)
{
	int i;

    for(i = 0; i < VPROXY_NUM_VEC; i++)
        VPROXY_VECTORS[i] = (void*)&proxy_handlers[i];

    volatile PFVOID* autovector = &VPROXY_VECTORS[0x60>>2];

    autovector[1] = v4sa_int_1; /* TT VME (TBE, DISKBLK, SOFTINT) */
    autovector[2] = v4sa_int_2; /* HBL (PORTS) */
    autovector[3] = v4sa_int_3; /* TT VME (COPER, BLIT) */
    // vector 4 not set because it goes straight to Atari VBL
    //autovector[4] = v4sa_int_4; /* VBL (1:1)*/
    autovector[5] = v4sa_int_5; /* - (RFB, DSKSYNC) */
    autovector[6] = v4sa_int_6; /* MFP (EXTER, CIAA and CIAB) */

    *(volatile UBYTE *)ST_SHIFTER = ST_LOW; /* boot mode */

    __asm__ volatile
    (
        "move.l %0,d0\n\t"
        "dc.w 0x4e7b, 0x0801\n" /* movec d0,VBR */
    : /* outputs */
    : "g"((void*)VPROXY_VECTORS) /* inputs  */
    : __CLOBBER_RETURN("d0")  /* clobbered regs */
    );

    IKBD_Init();
    IKBD_Reset(true);

    CIAAICR = 0x89;  /* Enable CIAA interrupts (now EXTER) */

    // initialise a 200Hz interrupts to poll mouse/joystick
    CIABTALO = 0xdb; /* 0x0ddb == 3547 => ~200Hz */
    CIABTAHI = 0x0d;

    CIABCRA  = 0x11; /* Start timer */
    CIABICR  = 0x81; /* Enable CIA Timer A interrupt generation */
    
    INTENA   = SETBITS | INTEN | EXTER; /* Enable interrupts from CIAA *AND* CIAB */
}

const char *v4sa_machine_name(void)
{
    static char name[40];

    if(!COREREV)
        return "V4SA (Core \xF3""10000)";

    sprintf(name, "V4SA (Core %d)", COREREV);

    return name;
}

#endif /* MACHINE_V4SA */
