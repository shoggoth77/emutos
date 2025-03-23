#include "emutos.h"
#include "videl.h"
#include "v4sa.h"
#include "biosdefs.h"
#include "tosvars.h"
#include "screen.h"
#include "videl.h"
#include "has.h"
#include "biosext.h"

#if defined(MACHINE_V4SA)

#include "videl.h"

#define ARRAYSIZE(a) (sizeof(a)/sizeof(a[0]))

/*
 * Maximum display sizes
 */
#define SAGA_VIDEO_MAXHV          0x4000
#define SAGA_VIDEO_MAXVV          0x4000

/* SAGA sprite */
#define SAGA_VIDEO_SPRITEX      *(volatile UWORD *)0xDFF1D0
#define SAGA_VIDEO_SPRITEY      *(volatile UWORD *)0xDFF1D2

/* SAGA video mode */
#define SAGA_VIDEO_FORMAT_OFF    0x0 /* video out */
#define SAGA_VIDEO_FORMAT_CLUT8  0x1 /* 8bit chuncky */
#define SAGA_VIDEO_FORMAT_RGB16  0x2 /* R5G6B5 */
#define SAGA_VIDEO_FORMAT_RGB15  0x3 /* X1R5G5B5 */
#define SAGA_VIDEO_FORMAT_RGB24  0x4 /* R8G8B8 */
#define SAGA_VIDEO_FORMAT_RGB32  0x5 /* X8R8G8B8 */
#define SAGA_VIDEO_FORMAT_YUV422 0x6 /* Y4U2V2 */
#define SAGA_VIDEO_FORMAT_IRGB16 0x7 /* I1R5G5B5 */
#define SAGA_VIDEO_FORMAT_STHIGH 0x8 /* Atari STHIGH format (1 plane)*/
#define SAGA_VIDEO_FORMAT_STMID  0x9 /* Atari STMID format (2 plane) */
#define SAGA_VIDEO_FORMAT_STLOW  0xA /* Atari STLOW format (4 plane) */ 
#define SAGA_VIDEO_FORMAT_TTLOW  0xB /* Atari TTLOW format (8 plane) */ 

#define SAGA_VIDEO_PALV4SA      *(volatile ULONG*)0xdff388 /* only 1 register format : IRGB where I is index on 8 bits */
#define SAGA_SET_VIDEO_MODE     *(volatile UWORD*)0xdff1f4 /* set video mode */
#define SAGA_GET_VIDEO_MODE     *(volatile UWORD*)0xdfe1f4 /* get video mode */
#define SAGA_SET_VIDEO_MODULO   *(volatile UWORD*)0xdff1e6 /* set video modulo */
#define SAGA_GET_VIDEO_MODULO   *(volatile UWORD*)0xdfe1e6 /* get video modulo */
#define SAGA_SET_VIDEO_PHYSBASE *(volatile ULONG*)0xdff1ec /* write chunky plane ptr */
#define SAGA_GET_VIDEO_PHYSBASE *(volatile ULONG*)0xdfe1ec /* read chunky plane ptr */
#define SAGA_SET_CONTROL_REG    *(volatile UWORD*)0xdff3ec
#define SAGA_GET_CONTROL_REG    *(volatile UWORD*)0xdfe3ec

#define SAGA_ENABLE 0x4000
#define MODECODE_TTMED  (VIDEL_VGA | VIDEL_COMPAT | VIDEL_80COL | VIDEL_4BPP)
#define MODECODE_TTLOW  (VIDEL_VGA | VIDEL_COMPAT | VIDEL_8BPP)
#define MODECODE_TTHIGH (0x1200 | SAGA_VIDEO_FORMAT_STHIGH | SAGA_ENABLE)
#define MODECODE_STLOW  (VIDEL_VGA | VIDEL_COMPAT | VIDEL_4BPP  | VIDEL_VERTICAL)
#define MODECODE_STMED  (VIDEL_VGA | VIDEL_COMPAT | VIDEL_80COL | VIDEL_2BPP | VIDEL_VERTICAL)
#define MODECODE_STHIGH (VIDEL_VGA | VIDEL_COMPAT | VIDEL_80COL | VIDEL_1BPP)

#define NO_CLUT    0
#define HARD_CLUT  1
#define SOFT_CLUT  2

#define INTERLEAVE_PLANES  0
#define STANDARD_PLANES    1
#define PACKEDPIX_PLANES   2

#define STANDARD_BITS  1
#define FALCON_BITS    2
#define INTEL_BITS     8

#define FORCEOCSEN (1U << 6)
#define PALZOOMEN  (1U << 5)
#define SYNCDIS    (1U << 4)
#define CHIP2MEN   (1U << 3)
#define ZOOMEN     (1U << 2)
#define AGAEN      (1U << 1)
#define SCANLINEEN (1U << 0)

#define VREG_BOARD          *(volatile UWORD*)0xdff3fc
#define VREG_BOARD_Unknown  0x00
#define VREG_BOARD_V600     0x01
#define VREG_BOARD_V500     0x02
#define VREG_BOARD_V4       0x03
#define VREG_BOARD_V666     0x04
#define VREG_BOARD_V4SA     0x05
#define VREG_BOARD_V1200    0x06
#define VREG_BOARD_V4000    0x07
#define VREG_BOARD_VCD32    0x08
#define VREG_BOARD_Future   0x09

typedef union {
    ULONG l;
    UBYTE b[4];
} RGB;

static const struct
{
    UWORD width;
    UWORD height;

} saga_res[] = 
{
    { 320,  200 }, // 0x00 - Illegal mode
    { 320,  200 }, // 0x01
    { 320,  240 }, // 0x02
    { 320,  256 }, // 0x03

    { 640,  400 }, // 0x04
    { 640,  480 }, // 0x05
    { 640,  512 }, // 0x06
    { 960,  540 }, // 0x07

    { 480,  270 }, // 0x08
    { 304,  224 }, // 0x09
    { 1280, 720 }, // 0x0A
    { 640,  360 }, // 0x0B

    { 800,  600 }, // 0x0C
    { 1024, 768 }, // 0x0D
    { 720,  576 }, // 0x0E
    { 848,  480 }, // 0x0F

    { 640,  200 }, // 0x10
    { 1920,1080 }, // 0x11
    { 1280,1024 }, // 0x12
    { 1280, 800 }, // 0x13

    { 1440, 900 }, // 0x14
};

static const struct
{
    UBYTE bpp;
    UBYTE format;
    UBYTE flags;
    UBYTE clut;
    ULONG rbits;
    ULONG gbits;
    ULONG bbits;
    ULONG abits; /* alpha   */
    ULONG lbits; /* genlock */
    ULONG ubits; /* unused  */
}
saga_fmt_info[] =
{
    { .bpp = 1,  .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = HARD_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x0000FF00UL, .bbits = 0x000000FFUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0xFF000000 }, // 0x00 (illegal)    
    { .bpp = 8,  .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = HARD_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x0000FF00UL, .bbits = 0x000000FFUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0xFF000000 }, // 0x01 CLUT8  
    { .bpp = 16, .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = SOFT_CLUT, .rbits =     0xf800UL, .gbits =     0x07e0UL, .bbits =     0x001fUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0x0000     }, // 0x02 RGB16  
    { .bpp = 16, .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = SOFT_CLUT, .rbits =     0x7c00UL, .gbits =     0x03e0UL, .bbits =     0x001fUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0x8000     }, // 0x03 RGB15  
    { .bpp = 24, .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = SOFT_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x00FF0000UL, .bbits = 0x000000FFUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0x00000000 }, // 0x04 RGB24  
    { .bpp = 32, .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = SOFT_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x00FF0000UL, .bbits = 0x000000FFUL, .abits = 0xFF000000UL, .lbits = 0, .ubits = 0x00000000 }, // 0x05 RGB32  
    { .bpp = 16, .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = SOFT_CLUT, .rbits = 0x00000000UL, .gbits = 0x00000000UL, .bbits = 0x00000000UL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0x00000000 }, // 0x06 YUV422 
    { .bpp = 16, .format = PACKEDPIX_PLANES,  .flags = STANDARD_BITS, .clut = SOFT_CLUT, .rbits =     0x7c00UL, .gbits =     0x03e0UL, .bbits =     0x001fUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0x0000     }, // 0x07 IRGB16  
    { .bpp = 1,  .format = INTERLEAVE_PLANES, .flags = STANDARD_BITS, .clut = HARD_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x0000FF00UL, .bbits = 0x000000FFUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0xFF000000 }, // 0x08 STHIGH 
    { .bpp = 2,  .format = INTERLEAVE_PLANES, .flags = STANDARD_BITS, .clut = HARD_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x0000FF00UL, .bbits = 0x000000FFUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0xFF000000 }, // 0x09 STMID  
    { .bpp = 4,  .format = INTERLEAVE_PLANES, .flags = STANDARD_BITS, .clut = HARD_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x0000FF00UL, .bbits = 0x000000FFUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0xFF000000 }, // 0x0A STLOW  
    { .bpp = 8,  .format = INTERLEAVE_PLANES, .flags = STANDARD_BITS, .clut = HARD_CLUT, .rbits = 0x00FF0000UL, .gbits = 0x0000FF00UL, .bbits = 0x000000FFUL, .abits = 0x00000000UL, .lbits = 0, .ubits = 0xFF000000 }, // 0x0A TTLOW  
};

static const UWORD modecode_saga_fmt[8] =
{
    SAGA_VIDEO_FORMAT_STHIGH,
    SAGA_VIDEO_FORMAT_STMID,
    SAGA_VIDEO_FORMAT_STLOW,  
    SAGA_VIDEO_FORMAT_TTLOW,
    SAGA_VIDEO_FORMAT_RGB16,
    SAGA_VIDEO_FORMAT_RGB32,
    SAGA_VIDEO_FORMAT_RGB24,
    SAGA_VIDEO_FORMAT_CLUT8,
};

union sreg
{
    UBYTE b[2];
    WORD w;
};

static void calc_modecode_info(WORD mode, UWORD *planes, UWORD *hz_rez, UWORD *vt_rez)
{
    UWORD sindex, width, height;

    if(mode & SAGA_ENABLE)
    {
        union sreg s;

        s.w = mode & ~SAGA_ENABLE;

        if((!s.b[0]) || (s.b[0] >= ARRAYSIZE(saga_res)))
            s.b[0] = 1;

        if( (s.b[1] == SAGA_VIDEO_FORMAT_OFF) ||
            (s.b[1] >= ARRAYSIZE(saga_fmt_info)) )
            s.b[1] = 1;

        width  = saga_res[s.b[0]].width;
        height = saga_res[s.b[0]].height;
        sindex = s.b[1];
    }
    else
    {
        if(mode & VIDEL_COMPAT)
        {
            switch(mode & VIDEL_BPPMASK)
            {
                default:
                    mode = MODECODE_STHIGH; /* Fallthrough */

                case VIDEL_1BPP: width = 640; height = 400; break;
                case VIDEL_2BPP: width = 640; height = 200; break;

                case VIDEL_4BPP:
                {
                    if(mode & VIDEL_80COL)
                    {
                        width = 640;
                        height = 480;
                    }
                    else
                    {
                        width = 320;
                        height = 200;
                    }
                }
                    break;

                case VIDEL_8BPP: width = 320; height = 480; break;
            }
        }
        else
        {
            width = 640;

            if(mode & VIDEL_VGA)
            {
                height = 480;
            }
            else
            {
                height = 400;
                mode ^= VIDEL_VERTICAL;
            }

            if(!(mode & VIDEL_80COL))
                width >>= 1;

            if(mode & VIDEL_VERTICAL)
                height >>= 1;
        }

        sindex = modecode_saga_fmt[mode & VIDEL_BPPMASK];
    }

    if(planes)
       *planes = saga_fmt_info[sindex].bpp;;

    if(hz_rez)
       *hz_rez = width;

    if(vt_rez)
       *vt_rez = height;
}

WORD vsetmode(WORD mode)
{
    WORD prev_modecode;

    if (mode == -1) 
        return current_video_mode;

    prev_modecode = current_video_mode;
    current_video_mode  = mode;

    SAGA_SET_VIDEO_MODULO = 0;
    SAGA_SET_CONTROL_REG = CLRBITS | SCANLINEEN;

    *(volatile UBYTE *)STE_LINE_OFFSET = 0;

    if(mode & SAGA_ENABLE)
    {
        union sreg s;

        s.w = mode & ~SAGA_ENABLE;

        SAGA_SET_VIDEO_MODE = s.w;
        sshiftmod = (s.b[1] == SAGA_VIDEO_FORMAT_STHIGH) ? ST_HIGH : FALCON_REZ;

        SAGA_SET_CONTROL_REG = CLRBITS | SCANLINEEN;
        return prev_modecode;
    }

    if(mode & VIDEL_COMPAT)
    {
        switch(mode & VIDEL_BPPMASK)
        {
            default:
            case VIDEL_1BPP:
                (*(volatile UBYTE*)ST_SHIFTER) = ST_HIGH;
                sshiftmod = ST_HIGH;

                SAGA_SET_CONTROL_REG = CLRBITS | SCANLINEEN;
                break;

            case VIDEL_2BPP:
                (*(volatile UBYTE*)ST_SHIFTER) = ST_MEDIUM;
                sshiftmod = ST_MEDIUM;

                SAGA_SET_CONTROL_REG = SETBITS | SCANLINEEN;
                break;

            case VIDEL_4BPP:
                if(mode & VIDEL_80COL)
                {
                    SAGA_SET_VIDEO_MODE = (0x0500 | SAGA_VIDEO_FORMAT_STLOW);
                    sshiftmod = FALCON_REZ;

                    SAGA_SET_CONTROL_REG = CLRBITS | SCANLINEEN;
                    break;
                }

                (*(volatile UBYTE*)ST_SHIFTER) = ST_LOW;
                sshiftmod = ST_LOW;

                SAGA_SET_CONTROL_REG = SETBITS | SCANLINEEN;;
                break;

            case VIDEL_8BPP:
                SAGA_SET_VIDEO_MODE = 0x0300 | SAGA_VIDEO_FORMAT_TTLOW;
                sshiftmod = FALCON_REZ;

                SAGA_SET_CONTROL_REG = CLRBITS | SCANLINEEN;
                break;
        }

        return prev_modecode;
    }

    static const UWORD res[] =
    {
        /* RGB                     */
        0x0100, /* 320 x 200       */
        0x0300, /* 320 x 256 (400) */
        0x1000, /* 640 x 200       */
        0x0400, /* 640 x 400       */

        /* VGA                     */
        0x0300, /* 320 x 256 (480) */
        0x0200, /* 320 x 240       */
        0x0500, /* 640 x 480       */
        0x0B00, /* 640 x 360 (240) */
    };

    UWORD i = 0;

    i += ( mode & VIDEL_VGA )      ? 4 : 0;
    i += ( mode & VIDEL_80COL )    ? 2 : 0;
    i += ( mode & VIDEL_VERTICAL ) ? 1 : 0;

    SAGA_SET_VIDEO_MODE  = res[i] | modecode_saga_fmt[mode & VIDEL_BPPMASK];
    sshiftmod = ((mode & VIDEL_BPPMASK) == VIDEL_1BPP) ? ST_HIGH : FALCON_REZ;

    if(mode & VIDEL_VGA)
        SAGA_SET_CONTROL_REG = CLRBITS | SCANLINEEN;
    else 
    {
        if(mode & VIDEL_VERTICAL)
            SAGA_SET_CONTROL_REG = CLRBITS | SCANLINEEN;
        else
            SAGA_SET_CONTROL_REG = SETBITS | SCANLINEEN;
    }

    if(HIBYTE(VREG_BOARD) != VREG_BOARD_V4SA)
    {
        switch(mode & VIDEL_BPPMASK)
        {
            case VIDEL_1BPP:
                *(volatile UBYTE *)ST_SHIFTER = ST_HIGH;
                break;

            case VIDEL_2BPP:
                *(volatile UBYTE *)ST_SHIFTER = ST_MEDIUM;
                
                if(!(mode & VIDEL_VERTICAL))
                    *(volatile UBYTE *)STE_LINE_OFFSET = 80;
                break;

            default:
            case VIDEL_4BPP:
                *(volatile UBYTE *)ST_SHIFTER = ST_LOW;
                
                if(!(mode & VIDEL_VERTICAL))
                    *(volatile UBYTE *)STE_LINE_OFFSET = 80;

                break;
        }
    }

    return prev_modecode;  
}

WORD vmontype(void)
{
    return MON_VGA;
}

UWORD get_videl_bpp(void)
{
    UWORD bpp;
    calc_modecode_info(current_video_mode, &bpp, NULL, NULL);

    return bpp;
}

LONG vgetsize(WORD mode)  
{
    UWORD bpp;
    UWORD hz_rez;
    UWORD vt_rez;

    calc_modecode_info(mode, &bpp, &hz_rez, &vt_rez);

    return ((ULONG)bpp * (ULONG)hz_rez * (ULONG)vt_rez) / 8;
}

WORD vfixmode(WORD mode)
{
    if(mode & SAGA_ENABLE)
    {
        union sreg s;

        s.w = mode & ~SAGA_ENABLE;

        if((!s.b[0]) || (s.b[0] >= ARRAYSIZE(saga_res)))
            s.b[0] = 1;

        if( (s.b[1] == SAGA_VIDEO_FORMAT_OFF) ||
            (s.b[1] >= ARRAYSIZE(saga_fmt_info)) )
            s.b[1] = 1;

        return s.w | SAGA_ENABLE;
    }

    if(mode & VIDEL_COMPAT)
    {
        switch(mode & VIDEL_BPPMASK)
        {
            default:
            case VIDEL_1BPP: mode = MODECODE_STHIGH; break;
            case VIDEL_2BPP: mode = MODECODE_STMED; break;
            case VIDEL_4BPP: mode = (mode & VIDEL_80COL) ? MODECODE_TTMED : MODECODE_STLOW; break;
            case VIDEL_8BPP: mode = MODECODE_TTLOW; break;
        }

        return mode;
    }

    if(!(mode & VIDEL_VGA))
        mode ^= VIDEL_VERTICAL | VIDEL_VGA;

    mode &= VIDEL_VALID & ~(VIDEL_OVERSCAN | VIDEL_PAL);
    
    return mode;
}

void videl_get_current_mode_info(UWORD *planes, UWORD *hz_rez, UWORD *vt_rez)
{
    calc_modecode_info(current_video_mode, planes, hz_rez, vt_rez);
}

const VMODE_ENTRY *lookup_videl_mode(WORD mode)
{
    return NULL;
}

void v4sa_setphys(const UBYTE *addr)
{
    if(addr < (const UBYTE*)0x01000000UL)
    {
        *(volatile UBYTE *) VIDEOBASE_ADDR_HI = ((ULONG) addr) >> 16;
        *(volatile UBYTE *) VIDEOBASE_ADDR_MID = ((ULONG) addr) >> 8;
        *(volatile UBYTE *) VIDEOBASE_ADDR_LOW = ((ULONG) addr);
    }

    SAGA_SET_VIDEO_PHYSBASE = (ULONG)addr;


    /* Disable hardware cursor (not the right place for this) */
//    SAGA_VIDEO_SPRITEX = SAGA_VIDEO_MAXHV - 1;
  //  SAGA_VIDEO_SPRITEY = SAGA_VIDEO_MAXVV - 1;
}

const UBYTE *v4sa_physbase(void)
{
    return (UBYTE *)SAGA_GET_VIDEO_PHYSBASE;
}

#endif /* MACHINE_V4SA */
