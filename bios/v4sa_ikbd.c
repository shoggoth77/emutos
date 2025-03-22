/*
  Hatari - ikbd.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  The keyboard processor(6301) handles any joystick/mouse/keyboard task
  and sends bytes to the ACIA(6850).
  The IKBD has a small ROM which is used to process various commands send
  by the main CPU to the THE IKBD.
  Due to lack of real HD6301 emulation, those commands are handled by
  functionally equivalent code that tries to be as close as possible
  to a real HD6301.

  For program using their own HD6301 code, we also use some custom
  handlers to emulate the expected result.
*/


#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include "v4sa_ikbd.h"

extern volatile int16_t  v4sa_mousex;
extern volatile int16_t  v4sa_mousey;
extern volatile uint8_t  v4sa_mouseb;
extern volatile int8_t   v4sa_mousew;
extern volatile uint16_t v4sa_joyb0;
extern volatile uint16_t v4sa_joyb1;

extern void acia_ikbd_rx(uint32_t d);

uint8_t v4sa_scancodes[128];

#define ATARIJOY_BITMASK_UP    0x01
#define ATARIJOY_BITMASK_DOWN  0x02
#define ATARIJOY_BITMASK_LEFT  0x04
#define ATARIJOY_BITMASK_RIGHT 0x08
#define ATARIJOY_BITMASK_FIRE  0x80

enum
{
	JOYID_JOYSTICK0,
	JOYID_JOYSTICK1
};

#define ACIA_CYCLES    7200         /* Cycles (Multiple of 4) between sent to ACIA from keyboard along serial line - 500Hz/64, (approx' 6920-7200cycles from test program) */

#define ABS_X_ONRESET    0          /* Initial XY for absolute mouse position after RESET command */
#define ABS_Y_ONRESET    0
#define ABS_MAX_X_ONRESET  320      /* Initial absolute mouse limits after RESET command */
#define ABS_MAY_Y_ONRESET  200      /* These values are never actually used as user MUST call 'IKBD_Cmd_AbsMouseMode' before ever using them */

#define ABS_PREVBUTTONS  (0x02|0x8) /* Don't report any buttons up on first call to 'IKBD_Cmd_ReadAbsMousePos' */

#define	IKBD_ROM_VERSION	0xF1	/* On reset, the IKBD will return either 0xF0 or 0xF1, depending on the IKBD's ROM */
					/* version. Only very early ST returned 0xF0, so we use 0xF1 which is the most common case.*/
					/* Beside, some programs explicitly wait for 0xF1 after a reset (Dragonnels demo) */

typedef struct
{
	struct
	{
		uint8_t scancode[16];
		uint8_t joystick[16];
	} map;

	struct
	{
		volatile uint16_t* source;
		uint16_t state;
	} input;

	uint8_t joystick;
} joypad_t;

static joypad_t joypad[4];

/* Keyboard state */
static KEYBOARD Keyboard;

/* Keyboard processor */
static KEYBOARD_PROCESSOR KeyboardProcessor;   /* Keyboard processor details */

static bool bMouseDisabled, bJoystickDisabled;
static bool bDuringResetCriticalTime, bBothMouseAndJoy;
static bool bMouseEnabledDuringReset;
static uint16_t ikbd_reset_counter;

/*
  HD6301 processor by Hitachi

  References :
   - HD6301V1, HD63A01V1, HD63B01V1 CMOS MCU datasheet by Hitachi

  The HD6301 is connected to the ACIA through TX and RX pins.
  Serial transfers are made with 8 bit word, 1 stop bit, no parity and 7812.5 baud

  The IKBD's ROM is using 2 buffers to handle input/output on the serial line
  in an asynchronous way, by using the SCI's interrupt at address $FEE2. This means
  the IKBD can execute a new command as soon as the current one is completed, as it is
  the interrupt function that will handle sending bytes to the ACIA.

  Input buffer : 8 bytes, located at $CD-$D4 in the IKBD's RAM.
	New bytes received in RDR are added to this buffer, until we have
	enough bytes to obtain a valid command (with its potential parameters)
	If the buffer already contains 8 bytes, new bytes are ignored (lost).
	This buffer is emptied if a valid command was processed or if the first
	byte in the buffer is not a valid command.

  Output buffer : 20 bytes as a ring buffer, located at $D9-$ED in the IKBD's RAM.
	When the IKBD automatically reports events or when a command returns some bytes,
	those 'n' bytes are added to the ring buffer.
	If the ring buffer doesn't have enough space to store 'n' new bytes, the 'n' bytes
	are ignored (lost).
	Each time a byte is correctly sent in TDR, a new byte is processed, until the ring
	buffer becomes empty.


  Special behaviours during the IKBD reset :
    If the following commands are received during the reset of the IKBD,
    the IKBD will go in a special mode and report both mouse and joystick at the same time :
	0x08 0x14		relative mouse on , joysticks auto
	0x08 0x0b 0x14		relative mouse on , mouse threshold , joysticks auto (eg Barbarian 1 by Psygnosis)
	0x12 0x14		disable mouse , joysticks auto (eg Hammerfist)
	0x12 0x1a		disable mouse , disable joysticks

    In that case mouse and joystick buttons will be reported in a "mouse report" packet
    and joystick actions (except buttons) will be reported in a "joystick report" packet.

*/


static void IKBD_RunKeyboardCommand(uint8_t aciabyte);


/* List of possible keyboard commands, others are seen as NOPs by keyboard processor */
static void IKBD_Cmd_Reset(void);
static void IKBD_Cmd_MouseAction(void);
static void IKBD_Cmd_RelMouseMode(void);
static void IKBD_Cmd_AbsMouseMode(void);
static void IKBD_Cmd_MouseCursorKeycodes(void);
static void IKBD_Cmd_SetMouseThreshold(void);
static void IKBD_Cmd_SetMouseScale(void);
static void IKBD_Cmd_ReadAbsMousePos(void);
static void IKBD_Cmd_SetInternalMousePos(void);
static void IKBD_Cmd_SetYAxisDown(void);
static void IKBD_Cmd_SetYAxisUp(void);
static void IKBD_Cmd_StartKeyboardTransfer(void);
static void IKBD_Cmd_TurnMouseOff(void);
static void IKBD_Cmd_StopKeyboardTransfer(void);
static void IKBD_Cmd_ReturnJoystickAuto(void);
static void IKBD_Cmd_StopJoystick(void);
static void IKBD_Cmd_ReturnJoystick(void);
static void IKBD_Cmd_SetJoystickMonitoring(void);
static void IKBD_Cmd_SetJoystickFireDuration(void);
static void IKBD_Cmd_SetCursorForJoystick(void);
static void IKBD_Cmd_DisableJoysticks(void);
static void IKBD_Cmd_SetClock(void);
static void IKBD_Cmd_ReadClock(void);
static void IKBD_Cmd_LoadMemory(void);
static void IKBD_Cmd_ReadMemory(void);
static void IKBD_Cmd_Execute(void);
static void IKBD_Cmd_ReportMouseAction(void);
static void IKBD_Cmd_ReportMouseMode(void);
static void IKBD_Cmd_ReportMouseThreshold(void);
static void IKBD_Cmd_ReportMouseScale(void);
static void IKBD_Cmd_ReportMouseVertical(void);
static void IKBD_Cmd_ReportMouseAvailability(void);
static void IKBD_Cmd_ReportJoystickMode(void);
static void IKBD_Cmd_ReportJoystickAvailability(void);

/* Keyboard Command */
static const struct {
  uint8_t Command;
  uint8_t NumParameters;
  void (*pCallFunction)(void);
} KeyboardCommands[] =
{
	/* Known messages, counts include command byte */
	{ 0x80,2,  IKBD_Cmd_Reset },
	{ 0x07,2,  IKBD_Cmd_MouseAction },
	{ 0x08,1,  IKBD_Cmd_RelMouseMode },
	{ 0x09,5,  IKBD_Cmd_AbsMouseMode },
	{ 0x0A,3,  IKBD_Cmd_MouseCursorKeycodes },
	{ 0x0B,3,  IKBD_Cmd_SetMouseThreshold },
	{ 0x0C,3,  IKBD_Cmd_SetMouseScale },
	{ 0x0D,1,  IKBD_Cmd_ReadAbsMousePos },
	{ 0x0E,6,  IKBD_Cmd_SetInternalMousePos },
	{ 0x0F,1,  IKBD_Cmd_SetYAxisDown },
	{ 0x10,1,  IKBD_Cmd_SetYAxisUp },
	{ 0x11,1,  IKBD_Cmd_StartKeyboardTransfer },
	{ 0x12,1,  IKBD_Cmd_TurnMouseOff },
	{ 0x13,1,  IKBD_Cmd_StopKeyboardTransfer },
	{ 0x14,1,  IKBD_Cmd_ReturnJoystickAuto },
	{ 0x15,1,  IKBD_Cmd_StopJoystick },
	{ 0x16,1,  IKBD_Cmd_ReturnJoystick },
	{ 0x17,2,  IKBD_Cmd_SetJoystickMonitoring },
	{ 0x18,1,  IKBD_Cmd_SetJoystickFireDuration },
	{ 0x19,7,  IKBD_Cmd_SetCursorForJoystick },
	{ 0x1A,1,  IKBD_Cmd_DisableJoysticks },
	{ 0x1B,7,  IKBD_Cmd_SetClock },
	{ 0x1C,1,  IKBD_Cmd_ReadClock },
	{ 0x20,4,  IKBD_Cmd_LoadMemory },
	{ 0x21,3,  IKBD_Cmd_ReadMemory },
	{ 0x22,3,  IKBD_Cmd_Execute },

	/* Report message (top bit set) */
	{ 0x87,1,  IKBD_Cmd_ReportMouseAction },
	{ 0x88,1,  IKBD_Cmd_ReportMouseMode },
	{ 0x89,1,  IKBD_Cmd_ReportMouseMode },
	{ 0x8A,1,  IKBD_Cmd_ReportMouseMode },
	{ 0x8B,1,  IKBD_Cmd_ReportMouseThreshold },
	{ 0x8C,1,  IKBD_Cmd_ReportMouseScale },
	{ 0x8F,1,  IKBD_Cmd_ReportMouseVertical },
	{ 0x90,1,  IKBD_Cmd_ReportMouseVertical },
	{ 0x92,1,  IKBD_Cmd_ReportMouseAvailability },
	{ 0x94,1,  IKBD_Cmd_ReportJoystickMode },
	{ 0x95,1,  IKBD_Cmd_ReportJoystickMode },
	{ 0x99,1,  IKBD_Cmd_ReportJoystickMode },
	{ 0x9A,1,  IKBD_Cmd_ReportJoystickAvailability },

	{ 0xFF,0,  NULL }  /* Term */
};

typedef struct {
	/* Date/Time is stored in the IKBD using 6 bytes in BCD format */
	/* Clock is cleared on cold reset, but keeps its values on warm reset */
	/* Original RAM location :  $82=year $83=month $84=day $85=hour $86=minute $87=second */
	uint8_t		Clock[ 6 ];
	int64_t		Clock_micro;				/* Incremented every VBL to update Clock[] every second */

} IKBD_STRUCT;


static IKBD_STRUCT	IKBD;
static IKBD_STRUCT	*pIKBD = &IKBD;

static void	IKBD_Boot_ROM ( bool ClearAllRAM );

#define IKBD_OutputBuffer_CheckFreeCount(n) true

static bool	IKBD_BCD_Check ( uint8_t val );
static uint8_t	IKBD_BCD_Adjust ( uint8_t val );

#define CRC32_POLY	0x04c11db7
static void	crc32_reset ( uint32_t *crc );
static void	crc32_add_byte ( uint32_t *crc , uint8_t c );

/*-----------------------------------------------------------------------*/
/* Belows part is used to emulate the behaviour of custom 6301 programs	*/
/* sent to the IKBD's RAM.						*/
/*-----------------------------------------------------------------------*/

static void IKBD_LoadMemoryByte ( uint8_t aciabyte );
static void IKBD_CustomCodeHandler_CommonBoot ( uint8_t aciabyte );
static void IKBD_CustomCodeHandler_FroggiesMenu_Read ( void );
static void IKBD_CustomCodeHandler_FroggiesMenu_Write ( uint8_t aciabyte );
static void IKBD_CustomCodeHandler_Transbeauce2Menu_Read ( void );
static void IKBD_CustomCodeHandler_Transbeauce2Menu_Write ( uint8_t aciabyte );
static void IKBD_CustomCodeHandler_DragonnelsMenu_Read ( void );
static void IKBD_CustomCodeHandler_DragonnelsMenu_Write ( uint8_t aciabyte );
static void IKBD_CustomCodeHandler_ChaosAD_Read ( void );
static void IKBD_CustomCodeHandler_ChaosAD_Write ( uint8_t aciabyte );
static void IKBD_CustomCodeHandler_AudioSculpture_Color_Read ( void );
static void IKBD_CustomCodeHandler_AudioSculpture_Mono_Read ( void );
static void IKBD_CustomCodeHandler_AudioSculpture_Read ( bool ColorMode );
static void IKBD_CustomCodeHandler_AudioSculpture_Write ( uint8_t aciabyte );

static int	MemoryLoadNbBytesAddr = 0;		/* base address for bytes to send with the command 0x20 */
static int	MemoryLoadNbBytesTotal = 0;		/* total number of bytes to send with the command 0x20 */
static int	MemoryLoadNbBytesLeft = 0;		/* number of bytes that remain to be sent  */
static uint32_t	MemoryLoadCrc = 0xffffffff;		/* CRC of the bytes sent to the IKBD */
static int	MemoryExeNbBytes = 0;			/* current number of bytes sent to the IKBD when IKBD_ExeMode is true */

static void	(*pIKBD_CustomCodeHandler_Read) ( void );
static void	(*pIKBD_CustomCodeHandler_Write) ( uint8_t );
static bool	IKBD_ExeMode = false;

static uint8_t	ScanCodeState[ 128 ];			/* state of each key : 0=released 1=pressed */

/* This array contains all known custom 6301 programs, with their CRC */
static const struct
{
	uint32_t		LoadMemCrc;			/* CRC of the bytes sent using the command 0x20 */
	void		(*ExeBootHandler) ( uint8_t );	/* function handling write to $fffc02 during the 'boot' mode */
	int		MainProgNbBytes;		/* number of bytes of the main 6301 program */
	uint32_t		MainProgCrc;			/* CRC of the main 6301 program */
	void		(*ExeMainHandler_Read) ( void );/* function handling read to $fffc02 in the main 6301 program */
	void		(*ExeMainHandler_Write) ( uint8_t ); /* function handling write to $fffc02 in the main 6301 program */
	const char	*Name;
}
CustomCodeDefinitions[] =
{
	{
		0x2efb11b1 ,
		IKBD_CustomCodeHandler_CommonBoot ,
		167,
		0xe7110b6d ,
		IKBD_CustomCodeHandler_FroggiesMenu_Read ,
		IKBD_CustomCodeHandler_FroggiesMenu_Write ,
		"Froggies Over The Fence Main Menu"
	} ,
	{
		0xadb6b503 ,
		IKBD_CustomCodeHandler_CommonBoot ,
		165,
		0x5617c33c ,
		IKBD_CustomCodeHandler_Transbeauce2Menu_Read ,
		IKBD_CustomCodeHandler_Transbeauce2Menu_Write ,
		"Transbeauce 2 Main Menu"
	} ,
	{
		0x33c23cdf ,
		IKBD_CustomCodeHandler_CommonBoot ,
		83 ,
		0xdf3e5a88 ,
		IKBD_CustomCodeHandler_DragonnelsMenu_Read ,
		IKBD_CustomCodeHandler_DragonnelsMenu_Write ,
		"Dragonnels Main Menu"
	},
	{
		0x9ad7fcdf ,
		IKBD_CustomCodeHandler_CommonBoot ,
		109 ,
		0xa11d8be5 ,
		IKBD_CustomCodeHandler_ChaosAD_Read ,
		IKBD_CustomCodeHandler_ChaosAD_Write ,
		"Chaos A.D."
	},
	{
		0xbc0c206d,
		IKBD_CustomCodeHandler_CommonBoot ,
		91 ,
		0x119b26ed ,
		IKBD_CustomCodeHandler_AudioSculpture_Color_Read ,
		IKBD_CustomCodeHandler_AudioSculpture_Write ,
		"Audio Sculpture Color"
	},
	{
		0xbc0c206d ,
		IKBD_CustomCodeHandler_CommonBoot ,
		91 ,
		0x63b5f4df ,
		IKBD_CustomCodeHandler_AudioSculpture_Mono_Read ,
		IKBD_CustomCodeHandler_AudioSculpture_Write ,
		"Audio Sculpture Mono"
	}
};

static const struct
{
	int start;
	int end;
	uint8_t* mapaddr;

} mem_map[] =
{
	{ 0xb000, 0xb010, (uint8_t*)&joypad[0].map.scancode, },
	{ 0xb010, 0xb020, (uint8_t*)&joypad[0].map.joystick, },
	{ 0xb100, 0xb110, (uint8_t*)&joypad[1].map.scancode, },
	{ 0xb110, 0xb120, (uint8_t*)&joypad[1].map.joystick, },
	{ 0xb200, 0xb210, (uint8_t*)&joypad[2].map.scancode, },
	{ 0xb210, 0xb220, (uint8_t*)&joypad[2].map.joystick, },
	{ 0xb300, 0xb310, (uint8_t*)&joypad[3].map.scancode, },
	{ 0xb310, 0xb320, (uint8_t*)&joypad[3].map.joystick, },
	{ 0xa000, 0xa080, v4sa_scancodes },
};

/*--------------------------------------------------------------*/
/* Reset the crc32 value. This should be done before calling	*/
/* crc32_add_byte().						*/
/*--------------------------------------------------------------*/

static void	crc32_reset ( uint32_t *crc )
{
	*crc = 0xffffffff;
}


/*--------------------------------------------------------------*/
/* Update the current value of crc with a new byte.		*/
/* Call crc32_reset() first to init the crc value.		*/
/*--------------------------------------------------------------*/

static void	crc32_add_byte ( uint32_t *crc , uint8_t c )
{
	int	bit;
    
	for ( bit=0 ; bit<8; bit++ )
	{
		if ( ( c & 0x80 ) ^ ( *crc & 0x80000000 ) )
			*crc = ( *crc << 1 ) ^ CRC32_POLY;

		else
			*crc = *crc << 1;

            c <<= 1;
        }
}

/*-----------------------------------------------------------------------*/
/**
 * Init the IKBD processor.
 * Connect the IKBD RX/TX callback functions to the ACIA.
 * This is called only once, when the emulator starts.
 */
void	IKBD_Init ( void )
{
	static const uint8_t default_scancodes[128] =
	{
		0x5b, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x29, 0x00, 0x70,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
		0x18, 0x19, 0x1a, 0x1b, 0x00, 0x6d, 0x6e, 0x6f,
		0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
		0x26, 0x27, 0x28, 0x2b, 0x00, 0x6a, 0x6b, 0x6c,
		0x60, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32,
		0x33, 0x34, 0x35, 0x00, 0x71, 0x67, 0x68, 0x69,
		0x39, 0x0e, 0x0f, 0x72, 0x1c, 0x01, 0x53, 0x00,
		0x00, 0x00, 0x4a, 0x62, 0x48, 0x50, 0x4d, 0x4b,
		0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42,
		0x43, 0x44, 0x63, 0x64, 0x65, 0x66, 0x4e, 0x62,
		0x2a, 0x36, 0x3a, 0x1d, 0x38, 0x4C, 0x56, 0x57,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61,
		0x47, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x59, 0x5A, 0x5C, 0x5D, 0x37, 0x00,
	};

	static const joypad_t defaults[2] = 
	{
		{
			.map = {
				.scancode = {
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
				},

				.joystick = {
					ATARIJOY_BITMASK_FIRE,
					0x00,
					0x00,
					0x00,
					0x00,0x00,0x00,0x00,
					0x00,0x00,0x00,0x00,
					ATARIJOY_BITMASK_DOWN,
					ATARIJOY_BITMASK_RIGHT,
					ATARIJOY_BITMASK_UP,
					ATARIJOY_BITMASK_LEFT,
				},
			},
			.input = {
				.source = &v4sa_joyb0,
				.state = 0,
			},

			.joystick = 0,
		},
		{
			.map = {
				.scancode = {
					0, 0, 0, 0,
					20,21,22,23,
					24,25,30,31,
					0, 0, 0, 0,
				},

				.joystick = {
					ATARIJOY_BITMASK_FIRE,
					ATARIJOY_BITMASK_UP,
					ATARIJOY_BITMASK_FIRE | ATARIJOY_BITMASK_UP,
					ATARIJOY_BITMASK_FIRE | ATARIJOY_BITMASK_DOWN,
					0x00,0x00,0x00,0x00,
					0x00,0x00,0x00,0x00,
					ATARIJOY_BITMASK_DOWN,
					ATARIJOY_BITMASK_RIGHT,
					ATARIJOY_BITMASK_UP,
					ATARIJOY_BITMASK_LEFT,
				},
			},
			.input = {
				.source = &v4sa_joyb1,
				.state = 0,
			},

			.joystick = 0,
		},
	};

	memcpy(v4sa_scancodes, default_scancodes, sizeof(v4sa_scancodes));

	joypad[0] = defaults[0];
	joypad[1] = defaults[1];
	joypad[2] = defaults[0];
	joypad[3] = defaults[1];
}


/*-----------------------------------------------------------------------*/
/**
 * Reset the IKBD processor
 */

/* This function is called after a hardware reset of the IKBD.
 * Cold reset is when the computer is turned off/on.
 * Warm reset is when the reset button is pressed or the 68000
 * RESET instruction is used.
 * We clear the serial interface and we execute the function
 * that emulates booting the ROM at 0xF000.
 */
void	IKBD_Reset ( bool bCold )
{
	/* On cold reset, clear the whole RAM (including clock data) */
	/* On warm reset, the clock data should be kept */
	if ( bCold )
		IKBD_Boot_ROM ( true );
	else
		IKBD_Boot_ROM ( false );
}



/* This function emulates the boot code stored in the ROM at address $F000.
 * This boot code is called either after a hardware reset, or when the
 * reset command ($80 $01) is received.
 * Depending on the conditions, we should clear the clock data or not (the
 * real IKBD will test+clear RAM either in range $80-$FF or in range $89-$FF)
 */
static void	IKBD_Boot_ROM ( bool ClearAllRAM )
{
	int	i;

	/* Clear clock data when the 128 bytes of RAM are cleared */
	if ( ClearAllRAM )
	{
		/* Clear clock data on cold reset */
		for ( i=0 ; i<6 ; i++ )
			pIKBD->Clock[ i ] = 0;
		pIKBD->Clock_micro = 0;
	}

// pIKBD->Clock[ 0 ] = 0x99;
// pIKBD->Clock[ 1 ] = 0x12;
// pIKBD->Clock[ 2 ] = 0x31;
// pIKBD->Clock[ 3 ] = 0x23;
// pIKBD->Clock[ 4 ] = 0x59;
// pIKBD->Clock[ 5 ] = 0x57;

	/* Set default reporting mode for mouse/joysticks */
	KeyboardProcessor.MouseMode = AUTOMODE_MOUSEREL;
	KeyboardProcessor.JoystickMode = AUTOMODE_JOYSTICK;

	KeyboardProcessor.Abs.X = ABS_X_ONRESET;
	KeyboardProcessor.Abs.Y = ABS_Y_ONRESET;
	KeyboardProcessor.Abs.MaxX = ABS_MAX_X_ONRESET;
	KeyboardProcessor.Abs.MaxY = ABS_MAY_Y_ONRESET;
	KeyboardProcessor.Abs.PrevReadAbsMouseButtons = ABS_PREVBUTTONS;

	KeyboardProcessor.Mouse.DeltaX = KeyboardProcessor.Mouse.DeltaY = 0;
	KeyboardProcessor.Mouse.XScale = KeyboardProcessor.Mouse.YScale = 0;
	KeyboardProcessor.Mouse.XThreshold = KeyboardProcessor.Mouse.YThreshold = 1;
	KeyboardProcessor.Mouse.KeyCodeDeltaX = KeyboardProcessor.Mouse.KeyCodeDeltaY = 1;
	KeyboardProcessor.Mouse.YAxis = 1;          /* Y origin at top */
	KeyboardProcessor.Mouse.Action = 0;

	KeyboardProcessor.Joy.PrevJoyData[0] = KeyboardProcessor.Joy.PrevJoyData[1] = 0;

	for ( i=0 ; i<128 ; i++ )
		ScanCodeState[ i ] = 0;				/* key is released */


	/* Reset our keyboard states and clear key state table */
	Keyboard.nBytesInInputBuffer = 0;
	Keyboard.PauseOutput = false;

	Keyboard.bLButtonDown = BUTTON_NULL;
	Keyboard.bRButtonDown = BUTTON_NULL;
	Keyboard.bOldLButtonDown = Keyboard.bOldRButtonDown = BUTTON_NULL;

	/* Store bool for when disable mouse or joystick */
	bMouseDisabled = bJoystickDisabled = false;
	/* do emulate hardware 'quirk' where if disable both with 'x' time
	 * of a RESET command they are ignored! */

	ikbd_reset_counter = 40; /* ikbd emulation cycles from ROM reset -> running (<300ms according to docs) */
	bDuringResetCriticalTime = true;
	bBothMouseAndJoy = false;
	bMouseEnabledDuringReset = false;


	/* Remove any custom handlers used to emulate code loaded to the 6301's RAM */
	if ( ( MemoryLoadNbBytesLeft != 0 ) || ( IKBD_ExeMode == true ) )
	{
		MemoryLoadNbBytesLeft = 0;
		pIKBD_CustomCodeHandler_Read = NULL;
		pIKBD_CustomCodeHandler_Write = NULL;
		IKBD_ExeMode = false;
	}

	/* Add auto-update function to the queue */
	/* We add it only if it was not active, else this can lead to unresponsive keyboard/input */
	/* when RESET instruction is called in a loop in less than 150000 cycles */
	Keyboard.AutoSendCycles = 150000;				/* approx every VBL */
}



/*-----------------------------------------------------------------------*/
/**
 * Handle the byte that was received in the RDR from the ACIA.
 * Depending on the IKBD's emulation mode, we either pass it to the standard
 * ROM's emulation layer, or we pass it to the custom handlers.
 */
void	IKBD_Process_RDR ( uint8_t RDR )
{
	/* If IKBD is executing custom code, send the byte to the function handling this code */
	if ( IKBD_ExeMode && pIKBD_CustomCodeHandler_Write )
	{
		(*pIKBD_CustomCodeHandler_Write) ( RDR );
		return;
	}

	if ( MemoryLoadNbBytesLeft == 0 )				/* No pending MemoryLoad command */
		IKBD_RunKeyboardCommand ( RDR );			/* Check for known commands */

	else								/* MemoryLoad command is not finished yet */
		IKBD_LoadMemoryByte ( RDR );				/* Process bytes sent to the IKBD's RAM */
}



/************************************************************************/
/* End of the Serial Communication Interface				*/
/************************************************************************/


/**
 * Check that the value is a correctly encoded BCD number
 */
static bool	IKBD_BCD_Check ( uint8_t val )
{
	if ( ( ( val & 0x0f ) > 0x09 )
	  || ( ( val & 0xf0 ) > 0x90 ) )
		return false;

	return true;
}


/**
 * After adding an integer number to a BCD number, the result is no more
 * in BCD format. This function adjusts the value to be a valid BCD number again.
 * In the HD6301, this is done using the 'DAA' instruction (Decimal Adjust)
 * to "propagate" values 10-15 to the next 4 bits and keep each nibble
 * in the 0-9 range.
 */

static uint8_t	IKBD_BCD_Adjust ( uint8_t val )
{
	if ( ( val & 0x0f ) > 0x09 )	/* low nibble no more in BCD */
		val += 0x06;		/* clear bit 4 and add 1 to high nibble */
	if ( ( val & 0xf0 ) > 0x90 )	/* high nibble no more in BCD */
		val += 0x60;		/* propagate carry (but bits>7 will be lost) */

	return val;
}



/**
 * Update the IKBD's internal clock.
 *
 * This function is called on every VBL and we add the number of microseconds
 * per VBL. When we reach 1000000 microseconds (1 sec), we update the Clock[]
 * array by incrementing the 'second' byte.
 *
 * This code uses the same logic as the ROM version in the IKBD,
 * don't try to optimise/rewrite it in a different way, as the TOS
 * expects data to be handled this way.
 * This works directly with BCD numbers and propagates the increment
 * to the next byte each time the current byte reaches its maximum
 * value.
 *  - when SetClock is used, the IKBD doesn't check the range of each byte,
 *    just that it's BCD encoded. So it's possible to set month/day/... to
 *    invalid values beyond the maximum allowed. These values will not correctly
 *    propagate to the next byte until they reach 0x99 and start again at 0x00.
 *  - check leap year for the number of days in february if ( year & 3 == 0 )
 *  - there's no explicit max for year : if year is 99 and increments,
 *    next year will be 00 (due to the BCD overflow)
 *    (used in the game 'Captain Blood' which sets clock to "99 12 31 00 00 00"
 *    and ends the game when clock reaches "00 01 01 00 00 00")
 */
void	IKBD_UpdateClockOnVBL ( void )
{

}




/*-----------------------------------------------------------------------*/
/**
 * Calculate out 'delta' that mouse has moved by each frame, and add this to our internal keyboard position
 */
static void IKBD_UpdateInternalMousePosition(void)
{

	KeyboardProcessor.Mouse.DeltaX = v4sa_mousex;
	KeyboardProcessor.Mouse.DeltaY = v4sa_mousey;
	v4sa_mousex = 0;
	v4sa_mousey = 0;

	/* Update internal mouse coords - Y axis moves according to YAxis setting(up/down) */
	/* Limit to Max X/Y(inclusive) */
	if ( KeyboardProcessor.Mouse.XScale > 1 )
		KeyboardProcessor.Abs.X += KeyboardProcessor.Mouse.DeltaX * KeyboardProcessor.Mouse.XScale;
	else
		KeyboardProcessor.Abs.X += KeyboardProcessor.Mouse.DeltaX;
	if (KeyboardProcessor.Abs.X < 0)
		KeyboardProcessor.Abs.X = 0;
	if (KeyboardProcessor.Abs.X > KeyboardProcessor.Abs.MaxX)
		KeyboardProcessor.Abs.X = KeyboardProcessor.Abs.MaxX;

	if ( KeyboardProcessor.Mouse.YScale > 1 )
		KeyboardProcessor.Abs.Y += KeyboardProcessor.Mouse.DeltaY*KeyboardProcessor.Mouse.YAxis * KeyboardProcessor.Mouse.YScale;
	else
		KeyboardProcessor.Abs.Y += KeyboardProcessor.Mouse.DeltaY*KeyboardProcessor.Mouse.YAxis;
	if (KeyboardProcessor.Abs.Y < 0)
		KeyboardProcessor.Abs.Y = 0;
	if (KeyboardProcessor.Abs.Y > KeyboardProcessor.Abs.MaxY)
		KeyboardProcessor.Abs.Y = KeyboardProcessor.Abs.MaxY;

}


/*-----------------------------------------------------------------------*/
/**
 * Convert button to bool value
 */
static bool IKBD_ButtonBool(int Button)
{
	/* Button pressed? */
	if (Button)
		return true;
	return false;
}


/*-----------------------------------------------------------------------*/
/**
 * Return true if buttons match, use this as buttons are a mask and not boolean
 */
static bool IKBD_ButtonsEqual(int Button1,int Button2)
{
	/* Return bool compare */
	return (IKBD_ButtonBool(Button1) == IKBD_ButtonBool(Button2));
}


/*-----------------------------------------------------------------------*/
/**
 * According to if the mouse is enabled or not the joystick 1 fire
 * button/right mouse button will become the same button. That means
 * pressing one will also press the other and vice-versa.
 * If both mouse and joystick are enabled, report it as a mouse button
 * (needed by the game Big Run for example).
 */
static void IKBD_DuplicateMouseFireButtons(void)
{
	/* If mouse is off then joystick fire button goes to joystick */
	if (KeyboardProcessor.MouseMode == AUTOMODE_OFF)
	{
		/* If pressed right mouse button, should go to joystick 1 */
		if (Keyboard.bRButtonDown&BUTTON_MOUSE)
			KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] |= ATARIJOY_BITMASK_FIRE;
		/* And left mouse button, should go to joystick 0 */
		if (Keyboard.bLButtonDown&BUTTON_MOUSE)
			KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK0] |= ATARIJOY_BITMASK_FIRE;
	}
	/* If mouse is on, joystick 1 fire button goes to the mouse instead */
	else
	{
#if 1
		/* Not technically correct, but should not do any harm */

		int pressed = (KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1]&ATARIJOY_BITMASK_FIRE) || (Keyboard.bRButtonDown&BUTTON_MOUSE);

		if(pressed)
		{
			KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] |= ATARIJOY_BITMASK_FIRE;
			Keyboard.bRButtonDown |= BUTTON_JOYSTICK;
		}
		else
		{
			KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] &= ~ATARIJOY_BITMASK_FIRE;
			Keyboard.bRButtonDown &= ~BUTTON_JOYSTICK;			
		}

#else
		/* Is fire button pressed? */
		if (KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1]&ATARIJOY_BITMASK_FIRE)
		{
			KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] &= ~ATARIJOY_BITMASK_FIRE;  /* Clear fire button bit */
			Keyboard.bRButtonDown |= BUTTON_JOYSTICK;  /* Mimic right mouse button */
		}
		else
			Keyboard.bRButtonDown &= ~BUTTON_JOYSTICK;
#endif
	}
}


/*-----------------------------------------------------------------------*/
/**
 * Send 'relative' mouse position
 * In case DeltaX or DeltaY are more than 127 units, we send the position
 * using several packets (with a while loop)
 */
static void IKBD_SendRelMousePacket(void)
{
	int8_t ByteRelX,ByteRelY;
	uint8_t Header;

	while ( true )
	{
		ByteRelX = (int8_t)KeyboardProcessor.Mouse.DeltaX;
		ByteRelY = (int8_t)KeyboardProcessor.Mouse.DeltaY;

		if ( ( ( ByteRelX < 0 ) && ( ByteRelX <= -KeyboardProcessor.Mouse.XThreshold ) )
		  || ( ( ByteRelX > 0 ) && ( ByteRelX >= KeyboardProcessor.Mouse.XThreshold ) )
		  || ( ( ByteRelY < 0 ) && ( ByteRelY <= -KeyboardProcessor.Mouse.YThreshold ) )
		  || ( ( ByteRelY > 0 ) && ( ByteRelY >= KeyboardProcessor.Mouse.YThreshold ) )
		  || ( !IKBD_ButtonsEqual(Keyboard.bOldLButtonDown,Keyboard.bLButtonDown ) )
		  || ( !IKBD_ButtonsEqual(Keyboard.bOldRButtonDown,Keyboard.bRButtonDown ) ) )
		{
			Header = 0xf8;
			if (Keyboard.bLButtonDown)
				Header |= 0x02;
			if (Keyboard.bRButtonDown)
				Header |= 0x01;

			if ( IKBD_OutputBuffer_CheckFreeCount ( 3 ) )
			{
				acia_ikbd_rx (Header);
				acia_ikbd_rx (ByteRelX);
				acia_ikbd_rx (ByteRelY*KeyboardProcessor.Mouse.YAxis);
			}

			KeyboardProcessor.Mouse.DeltaX -= ByteRelX;
			KeyboardProcessor.Mouse.DeltaY -= ByteRelY;

			/* Store buttons for next time around */
			Keyboard.bOldLButtonDown = Keyboard.bLButtonDown;
			Keyboard.bOldRButtonDown = Keyboard.bRButtonDown;
		}
		else
			break;					/* exit the while loop */
	}

}


/**
 * Get joystick data
 */
static void IKBD_GetJoystickData(void)
{
	int i, j;

	for(i = 0; i < 2; i++)
	{
		joypad_t* jp = &joypad[i];

		uint16_t state = *jp->input.source;
		uint16_t changed = jp->input.state ^ state;

		jp->input.state = state;
		jp->joystick = 0;

		for(j = 0; changed || state; j++, changed >>= 1, state >>= 1)
		{
			if(state & 1)
				jp->joystick |= jp->map.joystick[j];

			if(!(changed & 1))
				continue;

			uint8_t scan = jp->map.scancode[j];

			if(!scan)
				continue;

			if(!(state & 1))
				scan |= 0x80;

			acia_ikbd_rx(scan);
		}
	}

	/* Joystick 1 */
	KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] = joypad[1].joystick;

	/* If mouse is on, joystick 0 is not connected */
	if (KeyboardProcessor.MouseMode==AUTOMODE_OFF
	        || (bBothMouseAndJoy && KeyboardProcessor.MouseMode==AUTOMODE_MOUSEREL))
		KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK0] = joypad[0].joystick;
	else
		KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK0] = 0x00;
}


/*-----------------------------------------------------------------------*/
/**
 * Send 'joysticks' bit masks
 */
static void IKBD_SendAutoJoysticks(void)
{
	uint8_t JoyData;

	/* Did joystick 0/mouse change? */
	JoyData = KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK0];
	if (JoyData!=KeyboardProcessor.Joy.PrevJoyData[JOYID_JOYSTICK0])
	{
		if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
		{
			acia_ikbd_rx (0xFE);			/* Joystick 0 / Mouse */
			acia_ikbd_rx (JoyData);
		}
		KeyboardProcessor.Joy.PrevJoyData[JOYID_JOYSTICK0] = JoyData;
	}

	/* Did joystick 1(default) change? */
	JoyData = KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1];
	if (JoyData!=KeyboardProcessor.Joy.PrevJoyData[JOYID_JOYSTICK1])
	{
		if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
		{
			acia_ikbd_rx (0xFF);			/* Joystick 1 */
			acia_ikbd_rx (JoyData);
		}
		KeyboardProcessor.Joy.PrevJoyData[JOYID_JOYSTICK1] = JoyData;
	}
}

/*-----------------------------------------------------------------------*/
/**
 * Send 'joysticks' bit masks when in monitoring mode
 *	%000000xy	; where y is JOYSTICK1 Fire button
 *			; and x is JOYSTICK0 Fire button
 *	%nnnnmmmm	; where m is JOYSTICK1 state
 *			; and n is JOYSTICK0 state
 */
static void IKBD_SendAutoJoysticksMonitoring(void)
{
	uint8_t Byte1;
	uint8_t Byte2;

	Byte1 = ( ( KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK0] & ATARIJOY_BITMASK_FIRE ) >> 6 )
		| ( ( KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] & ATARIJOY_BITMASK_FIRE ) >> 7 );

	Byte2 = ( ( KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK0] & 0x0f ) << 4 )
		| ( KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] & 0x0f );

	acia_ikbd_rx (Byte1);
	acia_ikbd_rx (Byte2);
}

/*-----------------------------------------------------------------------*/
/**
 * Send packets which are generated from the mouse action settings
 * If relative mode is on, still generate these packets
 */
static void IKBD_SendOnMouseAction(void)
{
	bool bReportPosition = false;

	/* Report buttons as keys? Do in relative/absolute mode */
	if (KeyboardProcessor.Mouse.Action&0x4)
	{
		if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
		{
			/* Left button? */
			if ( (IKBD_ButtonBool(Keyboard.bLButtonDown) && (!IKBD_ButtonBool(Keyboard.bOldLButtonDown))) )
				acia_ikbd_rx (0x74);		/* Left */
			else if ( (IKBD_ButtonBool(Keyboard.bOldLButtonDown) && (!IKBD_ButtonBool(Keyboard.bLButtonDown))) )
				acia_ikbd_rx (0x74|0x80);
			/* Right button? */
			if ( (IKBD_ButtonBool(Keyboard.bRButtonDown) && (!IKBD_ButtonBool(Keyboard.bOldRButtonDown))) )
				acia_ikbd_rx (0x75);		/* Right */
			else if ( (IKBD_ButtonBool(Keyboard.bOldRButtonDown) && (!IKBD_ButtonBool(Keyboard.bRButtonDown))) )
				acia_ikbd_rx (0x75|0x80);
		}
		/* Ignore bottom two bits, so return now */
		return;
	}

	/* Check MouseAction - report position on press/release */
	/* MUST do this before update relative positions as buttons get reset */
	if (KeyboardProcessor.Mouse.Action&0x3)
	{
		/* Check for 'press'? */
		if (KeyboardProcessor.Mouse.Action&0x1)
		{
			/* Did 'press' mouse buttons? */
			if ( (IKBD_ButtonBool(Keyboard.bLButtonDown) && (!IKBD_ButtonBool(Keyboard.bOldLButtonDown))) )
			{
				bReportPosition = true;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons &= ~0x04;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons |= 0x02;
			}
			if ( (IKBD_ButtonBool(Keyboard.bRButtonDown) && (!IKBD_ButtonBool(Keyboard.bOldRButtonDown))) )
			{
				bReportPosition = true;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons &= ~0x01;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons |= 0x08;
			}
		}
		/* Check for 'release'? */
		if (KeyboardProcessor.Mouse.Action&0x2)
		{
			/* Did 'release' mouse buttons? */
			if ( (IKBD_ButtonBool(Keyboard.bOldLButtonDown) && (!IKBD_ButtonBool(Keyboard.bLButtonDown))) )
			{
				bReportPosition = true;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons &= ~0x08;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons |= 0x01;
			}
			if ( (IKBD_ButtonBool(Keyboard.bOldRButtonDown) && (!IKBD_ButtonBool(Keyboard.bRButtonDown))) )
			{
				bReportPosition = true;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons &= ~0x02;
				KeyboardProcessor.Abs.PrevReadAbsMouseButtons |= 0x04;
			}
		}

		/* Do need to report? */
		if (bReportPosition)
		{
			/* Only report if mouse in absolute mode */
			if (KeyboardProcessor.MouseMode==AUTOMODE_MOUSEABS)
				IKBD_Cmd_ReadAbsMousePos();
		}
	}
}


/*-----------------------------------------------------------------------*/
/**
 * Send mouse movements as cursor keys
 */
static void IKBD_SendCursorMousePacket(void)
{
	int i=0;

	/* Run each 'Delta' as cursor presses */
	/* Limit to '10' loops as host mouse cursor might have a VERY poor quality. */
	/* Eg, a single mouse movement on and ST gives delta's of '1', mostly, */
	/* but host mouse might go as high as 20+! */

	while ( (i<10) && ((KeyboardProcessor.Mouse.DeltaX!=0) || (KeyboardProcessor.Mouse.DeltaY!=0)
	                   || (!IKBD_ButtonsEqual(Keyboard.bOldLButtonDown,Keyboard.bLButtonDown)) || (!IKBD_ButtonsEqual(Keyboard.bOldRButtonDown,Keyboard.bRButtonDown))) )
	{
		if ( KeyboardProcessor.Mouse.DeltaX != 0 )
		{
			/* Left? */
			if (KeyboardProcessor.Mouse.DeltaX <= -KeyboardProcessor.Mouse.KeyCodeDeltaX)
			{
				if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
				{
					acia_ikbd_rx (75);		/* Left cursor */
					acia_ikbd_rx (75|0x80);
				}
				KeyboardProcessor.Mouse.DeltaX += KeyboardProcessor.Mouse.KeyCodeDeltaX;
			}
			/* Right? */
			if (KeyboardProcessor.Mouse.DeltaX >= KeyboardProcessor.Mouse.KeyCodeDeltaX)
			{
				if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
				{
					acia_ikbd_rx (77);		/* Right cursor */
					acia_ikbd_rx (77|0x80);
				}
				KeyboardProcessor.Mouse.DeltaX -= KeyboardProcessor.Mouse.KeyCodeDeltaX;
			}
		}

		if ( KeyboardProcessor.Mouse.DeltaY != 0 )
		{
			/* Up? */
			if (KeyboardProcessor.Mouse.DeltaY <= -KeyboardProcessor.Mouse.KeyCodeDeltaY)
			{
				if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
				{
					acia_ikbd_rx (72);		/* Up cursor */
					acia_ikbd_rx (72|0x80);
				}
				KeyboardProcessor.Mouse.DeltaY += KeyboardProcessor.Mouse.KeyCodeDeltaY;
			}
			/* Down? */
			if (KeyboardProcessor.Mouse.DeltaY >= KeyboardProcessor.Mouse.KeyCodeDeltaY)
			{
				if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
				{
					acia_ikbd_rx (80);		/* Down cursor */
					acia_ikbd_rx (80|0x80);
				}
				KeyboardProcessor.Mouse.DeltaY -= KeyboardProcessor.Mouse.KeyCodeDeltaY;
			}
		}

		if ( IKBD_OutputBuffer_CheckFreeCount ( 2 ) )
		{
			/* Left button? */
			if ( (IKBD_ButtonBool(Keyboard.bLButtonDown) && (!IKBD_ButtonBool(Keyboard.bOldLButtonDown))) )
				acia_ikbd_rx (0x74);		/* Left */
			else if ( (IKBD_ButtonBool(Keyboard.bOldLButtonDown) && (!IKBD_ButtonBool(Keyboard.bLButtonDown))) )
				acia_ikbd_rx (0x74|0x80);
			/* Right button? */
			if ( (IKBD_ButtonBool(Keyboard.bRButtonDown) && (!IKBD_ButtonBool(Keyboard.bOldRButtonDown))) )
				acia_ikbd_rx (0x75);		/* Right */
			else if ( (IKBD_ButtonBool(Keyboard.bOldRButtonDown) && (!IKBD_ButtonBool(Keyboard.bRButtonDown))) )
				acia_ikbd_rx (0x75|0x80);
		}
		Keyboard.bOldLButtonDown = Keyboard.bLButtonDown;
		Keyboard.bOldRButtonDown = Keyboard.bRButtonDown;

		/* Count, so exit if try too many times! */
		i++;
	}
}


/*-----------------------------------------------------------------------*/
/**
 * Return packets from keyboard for auto, rel mouse, joystick etc...
 */
void IKBD_SendAutoKeyboardCommands(void)
{
	static uint8_t mouseb_old;

	if(ikbd_reset_counter)
	{
		ikbd_reset_counter--;

		if(!ikbd_reset_counter)
		{
			/* Reset timer is over */
			bDuringResetCriticalTime = false;
			bMouseEnabledDuringReset = false;

			/* Return $F1 when IKBD's boot is complete */
			acia_ikbd_rx(IKBD_ROM_VERSION);
		}

		return;
	}

	Keyboard.bLButtonDown = (v4sa_mouseb & 2) ? BUTTON_MOUSE : 0;
	Keyboard.bRButtonDown = (v4sa_mouseb & 1) ? BUTTON_MOUSE : 0;

	/* Read joysticks for this frame */
	IKBD_GetJoystickData();

	/* Handle Joystick/Mouse fire buttons */
	IKBD_DuplicateMouseFireButtons();

	/* Send any packets which are to be reported by mouse action */
	IKBD_SendOnMouseAction();

	/* Update internal mouse absolute position by find 'delta' of mouse movement */
	IKBD_UpdateInternalMousePosition();

	/* If IKBD is monitoring only joysticks, don't report other events */
	if ( KeyboardProcessor.JoystickMode == AUTOMODE_JOYSTICK_MONITORING )
	{
		IKBD_SendAutoJoysticksMonitoring();
		return;
	}

	/* Send automatic joystick packets */
	if (KeyboardProcessor.JoystickMode==AUTOMODE_JOYSTICK)
		IKBD_SendAutoJoysticks();
	/* Send automatic relative mouse positions(absolute are not send automatically) */
	if (KeyboardProcessor.MouseMode==AUTOMODE_MOUSEREL)
		IKBD_SendRelMousePacket();
	/* Send cursor key directions for movements */
	else if (KeyboardProcessor.MouseMode==AUTOMODE_MOUSECURSOR)
		IKBD_SendCursorMousePacket();

	/* Store buttons for next time around */
	Keyboard.bOldLButtonDown = Keyboard.bLButtonDown;
	Keyboard.bOldRButtonDown = Keyboard.bRButtonDown;

	while(v4sa_mousew > 0)
	{
#if 1
		IKBD_PressSTKey(0xf6); 
		IKBD_PressSTKey(0x05); 
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x00);  
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x00);
		IKBD_PressSTKey(0x59); 
#else
	  IKBD_PressSTKey(0x59); /* Press */
#endif
	  v4sa_mousew--;
	}

	while(v4sa_mousew < 0)
	{
#if 1
		IKBD_PressSTKey(0xf6); 
		IKBD_PressSTKey(0x05); 
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x00); 
		IKBD_PressSTKey(0x5a);
#else
	  IKBD_PressSTKey(0x5a); /* Press */
#endif
	  v4sa_mousew++;
	}

	uint8_t diff_mouseb = v4sa_mouseb ^ mouseb_old;

	if(diff_mouseb & 0x04)
		IKBD_PressSTKey(0x37 | ((v4sa_mouseb & 0x04) ? 0x00 : 0x80));

	if(diff_mouseb & 0x08)
		IKBD_PressSTKey(0x5e | ((v4sa_mouseb & 0x08) ? 0x00 : 0x80));

	if(diff_mouseb & 0x10)
		IKBD_PressSTKey(0x5f | ((v4sa_mouseb & 0x10) ? 0x00 : 0x80));

  mouseb_old = v4sa_mouseb;

	/* If we're executing a custom IKBD program, call it to process the key/mouse/joystick event */
	if ( IKBD_ExeMode && pIKBD_CustomCodeHandler_Read )
		(*pIKBD_CustomCodeHandler_Read) ();
}


/*-----------------------------------------------------------------------*/
/**
 * When press/release key under host OS, execute this function.
 */
void IKBD_PressSTKey(uint32_t ScanCode)
{
	/* If IKBD is monitoring only joysticks, don't report key */
	if ( KeyboardProcessor.JoystickMode == AUTOMODE_JOYSTICK_MONITORING )
		return;

	ScanCodeState[ScanCode & 0x7f] = ScanCode & 0x80 ? 0 : 1;
	acia_ikbd_rx (ScanCode);

	/* If we're executing a custom IKBD program, call it to process the key event */
	if ( IKBD_ExeMode && pIKBD_CustomCodeHandler_Read )
		(*pIKBD_CustomCodeHandler_Read) ();
}


/*-----------------------------------------------------------------------*/
/**
 * Check if a key is pressed in the ScanCodeState array
 * Return the scancode >= 0 for the first key we find, else return -1
 * if no key is pressed
 */
static int IKBD_CheckPressedKey(void)
{
	unsigned int	i;

	for (i=0 ; i<sizeof(ScanCodeState) ; i++ )
		if ( ScanCodeState[ i ] )
			return i;

	return -1;
}


/*-----------------------------------------------------------------------*/
/**
 * On ST if disable Mouse AND Joystick with a set time of a RESET command they are
 * actually turned back on! (A number of games do this so can get mouse and joystick
 * packets at the same time)
 */
static void IKBD_CheckResetDisableBug(void)
{
	/* Have disabled BOTH mouse and joystick? */
	if (bMouseDisabled && bJoystickDisabled)
	{
		/* And in critical time? */
		if (bDuringResetCriticalTime)
		{
			/* Emulate relative mouse and joystick reports being turned back on */
			KeyboardProcessor.MouseMode = AUTOMODE_MOUSEREL;
			KeyboardProcessor.JoystickMode = AUTOMODE_JOYSTICK;
			bBothMouseAndJoy = true;
		}
	}
}



/*-----------------------------------------------------------------------*/
/**
 * When a byte is received by the IKBD, it is added to a small 8 byte buffer.
 * - If the first byte is a valid command, we wait for additional bytes if needed
 *   and then we execute the command's handler.
 * - If the first byte is not a valid command or after a successful command, we
 *   empty the input buffer (extra bytes, if any, are lost)
 * - If the input buffer is full when a new byte is received, the new byte is lost.
 * - In case the first byte read is not a valid command then IKBD does nothing
 *   (it doesn't return any byte to indicate the command was not recognized)
 */
static void IKBD_RunKeyboardCommand(uint8_t aciabyte)
{
	int i=0;

	/* Write into our keyboard input buffer if it's not full yet */
	if ( Keyboard.nBytesInInputBuffer < SIZE_KEYBOARDINPUT_BUFFER )
		Keyboard.InputBuffer[Keyboard.nBytesInInputBuffer++] = aciabyte;

	/* Now check bytes to see if we have a valid/in-valid command string set */
	while (KeyboardCommands[i].Command!=0xff)
	{
		/* Found command? */
		if (KeyboardCommands[i].Command==Keyboard.InputBuffer[0])
		{
			/* If the command is complete (with its possible parameters) we can execute it */
			/* Else, we wait for the next bytes until the command is complete */
			if (KeyboardCommands[i].NumParameters==Keyboard.nBytesInInputBuffer)
			{
				/* Any new valid command will unpause the output (if command 0x13 was used) */
				Keyboard.PauseOutput = false;

				KeyboardCommands[i].pCallFunction();
				Keyboard.nBytesInInputBuffer = 0;	/* Clear input buffer after processing a command */
			}

			return;
		}

		i++;
	}

	/* Command not known, reset buffer(IKBD assumes a NOP) */
	Keyboard.nBytesInInputBuffer = 0;
}




/************************************************************************/
/* List of keyboard commands handled by the standard IKBD's ROM.	*/
/* Each IKBD's command is emulated to get the same result as if we were	*/
/* running a full HD6301 emulation.					*/
/************************************************************************/


/*-----------------------------------------------------------------------*/
/**
 * RESET
 *
 * 0x80
 * 0x01
 *
 * Performs self test and checks for stuck (closed) keys, if OK returns
 * IKBD_ROM_VERSION (0xF1). Otherwise returns break codes for keys (not emulated).
 */
static void IKBD_Cmd_Reset(void)
{
	/* Check that 0x01 was received after 0x80 */
	if (Keyboard.InputBuffer[1] == 0x01)
	{
		IKBD_Boot_ROM ( false );
	}
	/* else if not 0x80,0x01 just ignore */
}


/*-----------------------------------------------------------------------*/
/**
 * SET MOUSE BUTTON ACTION
 *
 * 0x07
 * %00000mss  ; mouse button action
 *       ;  (m is presumed =1 when in MOUSE KEYCODE mode)
 *       ; mss=0xy, mouse button press or release causes mouse
 *       ;  position report
 *       ;  where y=1, mouse key press causes absolute report
 *       ;  and x=1, mouse key release causes absolute report
 *       ; mss=100, mouse buttons act like keys
 */
static void IKBD_Cmd_MouseAction(void)
{
	KeyboardProcessor.Mouse.Action = Keyboard.InputBuffer[1];
	KeyboardProcessor.Abs.PrevReadAbsMouseButtons = ABS_PREVBUTTONS;
}


/*-----------------------------------------------------------------------*/
/**
 * SET RELATIVE MOUSE POSITION REPORTING
 *
 * 0x08
 */
static void IKBD_Cmd_RelMouseMode(void)
{
	KeyboardProcessor.MouseMode = AUTOMODE_MOUSEREL;

	/* Some games (like Barbarian by Psygnosis) enable both, mouse and
	 * joystick directly after a reset. This causes the IKBD to send both
	 * type of packets. To emulate this feature, we've got to remember
	 * that the mouse has been enabled during reset. */
	if (bDuringResetCriticalTime)
		bMouseEnabledDuringReset = true;
}


/*-----------------------------------------------------------------------*/
/**
 * SET ABSOLUTE MOUSE POSITIONING
 *
 * 0x09
 * XMSB      ;X maximum (in scaled mouse clicks)
 * XLSB
 * YMSB      ;Y maximum (in scaled mouse clicks)
 * YLSB
 */
static void IKBD_Cmd_AbsMouseMode(void)
{
	/* These maximums are 'inclusive' */
	KeyboardProcessor.MouseMode = AUTOMODE_MOUSEABS;
	KeyboardProcessor.Abs.MaxX = (((unsigned int)Keyboard.InputBuffer[1])<<8) | (unsigned int)Keyboard.InputBuffer[2];
	KeyboardProcessor.Abs.MaxY = (((unsigned int)Keyboard.InputBuffer[3])<<8) | (unsigned int)Keyboard.InputBuffer[4];
}


/*-----------------------------------------------------------------------*/
/**
 * SET MOUSE KEYCODE MODE
 *
 * 0x0A
 * deltax      ; distance in X clicks to return (LEFT) or (RIGHT)
 * deltay      ; distance in Y clicks to return (UP) or (DOWN)
 */
static void IKBD_Cmd_MouseCursorKeycodes(void)
{
	KeyboardProcessor.MouseMode = AUTOMODE_MOUSECURSOR;
	KeyboardProcessor.Mouse.KeyCodeDeltaX = Keyboard.InputBuffer[1];
	KeyboardProcessor.Mouse.KeyCodeDeltaY = Keyboard.InputBuffer[2];
}


/*-----------------------------------------------------------------------*/
/**
 * SET MOUSE THRESHOLD
 *
 * 0x0B
 * X      ; x threshold in mouse ticks (positive integers)
 * Y      ; y threshold in mouse ticks (positive integers)
 */
static void IKBD_Cmd_SetMouseThreshold(void)
{
	KeyboardProcessor.Mouse.XThreshold = (unsigned int)Keyboard.InputBuffer[1];
	KeyboardProcessor.Mouse.YThreshold = (unsigned int)Keyboard.InputBuffer[2];
}


/*-----------------------------------------------------------------------*/
/**
 * SET MOUSE SCALE
 *
 * 0x0C
 * X      ; horizontal mouse ticks per internal X
 * Y      ; vertical mouse ticks per internal Y
 */
static void IKBD_Cmd_SetMouseScale(void)
{
	KeyboardProcessor.Mouse.XScale = (unsigned int)Keyboard.InputBuffer[1];
	KeyboardProcessor.Mouse.YScale = (unsigned int)Keyboard.InputBuffer[2];
}


/*-----------------------------------------------------------------------*/
/**
 * INTERROGATE MOUSE POSITION
 *
 * 0x0D
 *   Returns:  0xF7  ; absolute mouse position header
 *     BUTTONS
 *       0000dcba
 *       where a is right button down since last interrogation
 *       b is right button up since last
 *       c is left button down since last
 *       d is left button up since last
 *     XMSB      ; X coordinate
 *     XLSB
 *     YMSB      ; Y coordinate
 *     YLSB
 */
static void IKBD_Cmd_ReadAbsMousePos(void)
{
	uint8_t Buttons,PrevButtons;

	/* Test buttons */
	Buttons = 0;
	/* Set buttons to show if up/down */
	if (Keyboard.bRButtonDown)
		Buttons |= 0x01;
	else
		Buttons |= 0x02;
	if (Keyboard.bLButtonDown)
		Buttons |= 0x04;
	else
		Buttons |= 0x08;
	/* Mask off it didn't send last time */
	PrevButtons = KeyboardProcessor.Abs.PrevReadAbsMouseButtons;
	KeyboardProcessor.Abs.PrevReadAbsMouseButtons = Buttons;
	Buttons &= ~PrevButtons;

	/* And send packet */
	if ( IKBD_OutputBuffer_CheckFreeCount ( 6 ) )
	{
		acia_ikbd_rx (0xf7); /* Delay 18000-ACIA_CYCLES */
		acia_ikbd_rx (Buttons);
		acia_ikbd_rx ((unsigned int)KeyboardProcessor.Abs.X>>8);
		acia_ikbd_rx ((unsigned int)KeyboardProcessor.Abs.X&0xff);
		acia_ikbd_rx ((unsigned int)KeyboardProcessor.Abs.Y>>8);
		acia_ikbd_rx ((unsigned int)KeyboardProcessor.Abs.Y&0xff);
	}
}


/*-----------------------------------------------------------------------*/
/**
 * LOAD MOUSE POSITION
 *
 * 0x0E
 * 0x00      ; filler
 * XMSB      ; X coordinate
 * XLSB      ; (in scaled coordinate system)
 * YMSB      ; Y coordinate
 * YLSB
 */
static void IKBD_Cmd_SetInternalMousePos(void)
{
	/* Setting these do not clip internal position(this happens on next update) */
	KeyboardProcessor.Abs.X = (((unsigned int)Keyboard.InputBuffer[2])<<8) | (unsigned int)Keyboard.InputBuffer[3];
	KeyboardProcessor.Abs.Y = (((unsigned int)Keyboard.InputBuffer[4])<<8) | (unsigned int)Keyboard.InputBuffer[5];
}


/*-----------------------------------------------------------------------*/
/**
 * SET Y=0 AT BOTTOM
 *
 * 0x0F
 */
static void IKBD_Cmd_SetYAxisDown(void)
{
	KeyboardProcessor.Mouse.YAxis = -1;
}


/*-----------------------------------------------------------------------*/
/**
 * SET Y=0 AT TOP
 *
 * 0x10
 */
static void IKBD_Cmd_SetYAxisUp(void)
{
	KeyboardProcessor.Mouse.YAxis = 1;
}


/*-----------------------------------------------------------------------*/
/**
 * RESUME
 *
 * Any command received by the IKBD will also resume the output if it was
 * paused by command 0x13, so this command is redundant.
 *
 * 0x11
 */
static void IKBD_Cmd_StartKeyboardTransfer(void)
{
	Keyboard.PauseOutput = false;
}


/*-----------------------------------------------------------------------*/
/**
 * DISABLE MOUSE
 *
 * 0x12
 */
static void IKBD_Cmd_TurnMouseOff(void)
{
	KeyboardProcessor.MouseMode = AUTOMODE_OFF;
	bMouseDisabled = true;

	IKBD_CheckResetDisableBug();
}


/*-----------------------------------------------------------------------*/
/**
 * PAUSE OUTPUT
 *
 * 0x13
 */
static void IKBD_Cmd_StopKeyboardTransfer(void)
{
	if (bDuringResetCriticalTime)
	{
		/* Required for the loader of 'Just Bugging' by ACF */
		return;
	}

	Keyboard.PauseOutput = true;
}


/*-----------------------------------------------------------------------*/
/**
 * SET JOYSTICK EVENT REPORTING
 *
 * 0x14
 */
static void IKBD_Cmd_ReturnJoystickAuto(void)
{
	KeyboardProcessor.JoystickMode = AUTOMODE_JOYSTICK;
	KeyboardProcessor.MouseMode = AUTOMODE_OFF;

	/* If mouse was also enabled within time of a reset (0x08 command) it isn't disabled now!
	 * (used by the game Barbarian 1 by Psygnosis for example) */
	if ( bDuringResetCriticalTime && bMouseEnabledDuringReset )
	{
		KeyboardProcessor.MouseMode = AUTOMODE_MOUSEREL;
		bBothMouseAndJoy = true;
	}
	/* If mouse was disabled during the reset (0x12 command) it is enabled again
	 * (used by the game Hammerfist for example) */
	else if ( bDuringResetCriticalTime && bMouseDisabled )
	{
		KeyboardProcessor.MouseMode = AUTOMODE_MOUSEREL;
		bBothMouseAndJoy = true;
	}

	/* This command resets the internally previously stored joystick states */
	KeyboardProcessor.Joy.PrevJoyData[JOYID_JOYSTICK0] = KeyboardProcessor.Joy.PrevJoyData[JOYID_JOYSTICK1] = 0;

	/* This is a hack for the STE Utopos (=> v1.50) and Falcon Double Bubble
	 * 2000 games. They expect the joystick data to be sent within a certain
	 * amount of time after this command, without checking the ACIA control
	 * register first.
	 */
	IKBD_GetJoystickData();
	IKBD_SendAutoJoysticks();
}


/*-----------------------------------------------------------------------*/
/**
 * SET JOYSTICK INTERROGATION MODE
 *
 * 0x15
 */
static void IKBD_Cmd_StopJoystick(void)
{
	KeyboardProcessor.JoystickMode = AUTOMODE_OFF;
}


/*-----------------------------------------------------------------------*/
/**
 * JOYSTICK INTERROGATE
 *
 * 0x16
 */
static void IKBD_Cmd_ReturnJoystick(void)
{

	if ( IKBD_OutputBuffer_CheckFreeCount ( 3 ) )
	{
		acia_ikbd_rx (0xFD); /* IKBD_Delay_Random ( 7500 , 10000 ) */
		acia_ikbd_rx (KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK0]);
		acia_ikbd_rx (KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1]);
	}
}


/*-----------------------------------------------------------------------*/
/**
 * SET JOYSTICK MONITORING
 *
 * 0x17
 * rate      ; time between samples in hundredths of a second
 *   Returns: (in packets of two as long as in mode)
 *     %000000xy  where y is JOYSTICK1 Fire button
 *         and x is JOYSTICK0 Fire button
 *     %nnnnmmmm  where m is JOYSTICK1 state
 *         and n is JOYSTICK0 state
 *
 * TODO : we use a fixed 8 MHz clock to convert rate in 1/100th of sec into cycles.
 * This should be replaced by using MachineClocks.CPU_Freq.
 */
static void IKBD_Cmd_SetJoystickMonitoring(void)
{
	int	Rate;
	int	Cycles;

	Rate = (unsigned int)Keyboard.InputBuffer[1];

	KeyboardProcessor.JoystickMode = AUTOMODE_JOYSTICK_MONITORING;
	KeyboardProcessor.MouseMode = AUTOMODE_OFF;

	if ( Rate == 0 )
		Rate = 1;

	Cycles = 8021247 * Rate / 100;
	Keyboard.AutoSendCycles = Cycles;
}


/*-----------------------------------------------------------------------*/
/**
 * SET FIRE BUTTON MONITORING
 *
 * 0x18
 *   Returns: (as long as in mode)
 *     %bbbbbbbb  ; state of the JOYSTICK1 fire button packed
 *           ; 8 bits per byte, the first sample if the MSB
 */
static void IKBD_Cmd_SetJoystickFireDuration(void)
{
	/* IKBD_Cmd_SetJoystickFireDuration (not implemented) */
}


/*-----------------------------------------------------------------------*/
/**
 * SET JOYSTICK KEYCODE MODE
 *
 * 0x19
 * RX        ; length of time (in tenths of seconds) until
 *         ; horizontal velocity breakpoint is reached
 * RY        ; length of time (in tenths of seconds) until
 *         ; vertical velocity breakpoint is reached
 * TX        ; length (in tenths of seconds) of joystick closure
 *         ; until horizontal cursor key is generated before RX
 *         ; has elapsed
 * TY        ; length (in tenths of seconds) of joystick closure
 *         ; until vertical cursor key is generated before RY
 *         ; has elapsed
 * VX        ; length (in tenths of seconds) of joystick closure
 *         ; until horizontal cursor keystrokes are generated after RX
 *         ; has elapsed
 * VY        ; length (in tenths of seconds) of joystick closure
 *         ; until vertical cursor keystrokes are generated after RY
 *         ; has elapsed
 */
static void IKBD_Cmd_SetCursorForJoystick(void)
{
	/* IKBD_Cmd_SetCursorForJoystick (not implemented) */
}


/*-----------------------------------------------------------------------*/
/**
 * DISABLE JOYSTICKS
 *
 * 0x1A
 */
static void IKBD_Cmd_DisableJoysticks(void)
{
	KeyboardProcessor.JoystickMode = AUTOMODE_OFF;
	bJoystickDisabled = true;

	/* IKBD_Cmd_DisableJoysticks */

	IKBD_CheckResetDisableBug();
}


/*-----------------------------------------------------------------------*/
/**
 * TIME-OF-DAY CLOCK SET
 *
 * 0x1B
 * YY        ; year (2 least significant digits)
 * MM        ; month
 * DD        ; day
 * hh        ; hour
 * mm        ; minute
 * ss        ; second
 *
 * All bytes are stored in BCD format. If a byte is not in BCD, we ignore it
 * but we process the rest of the bytes.
 * Note that the IKBD doesn't check that month/day/hour/second/minute are in
 * their correct range, just that they're BCD encoded (so you can store 0x30 in hour
 * for example, see IKBD_UpdateClockOnVBL())
 */
static void IKBD_Cmd_SetClock(void)
{
	int	i;
	uint8_t	val;

	for ( i=1 ; i<=6 ; i++ )
	{
		val = Keyboard.InputBuffer[ i ];
		if ( IKBD_BCD_Check ( val ) )			/* Check if valid BCD, else ignore */
			pIKBD->Clock[ i-1 ] = val;		/* Store new value */
	}
}


/*-----------------------------------------------------------------------*/
/**
 * INTERROGATE TIME-OF-DAY CLOCK
 *
 * 0x1C
 *   Returns:
 *     0xFC  ; time-of-day event header
 *     YY    ; year (2 least significant digits)
 *     MM    ; month
 *     DD    ; day
 *     hh    ; hour
 *     mm    ; minute
 *     ss    ; second
 *
 * All bytes are stored/returned in BCD format.
 * Date/Time is updated in IKBD_UpdateClockOnVBL()
 */
static void IKBD_Cmd_ReadClock(void)
{
	int	i;

	/* Return packet header */
	if ( IKBD_OutputBuffer_CheckFreeCount ( 7 ) )
	{
		acia_ikbd_rx (0xFC); /* IKBD_Delay_Random ( 7000 , 7500 ) */

		/* Return the 6 clock bytes */
		for ( i=0 ; i<6 ; i++ )
			acia_ikbd_rx ( pIKBD->Clock[ i ] );
	}
}

static uint8_t* resolve_address(int addr)
{
	int i;

	for(i = 0; i < (sizeof(mem_map) / sizeof(mem_map[0])); i++)
	{
		if(addr < mem_map[i].start)
			continue;

		if(addr > mem_map[i].end)
			continue;

		return &mem_map[i].mapaddr[addr - mem_map[i].start];
	}
	return NULL;
}

/*-----------------------------------------------------------------------*/
/**
 * MEMORY LOAD
 *
 * 0x20
 * ADRMSB      ; address in controller
 * ADRLSB      ; memory to be loaded
 * NUM        ; number of bytes (0-128)
 * { data }
 */
static void IKBD_Cmd_LoadMemory(void)
{
	MemoryLoadNbBytesAddr = ((int)Keyboard.InputBuffer[1] << 8) | Keyboard.InputBuffer[2];

	MemoryLoadNbBytesTotal = Keyboard.InputBuffer[3];
	MemoryLoadNbBytesLeft = MemoryLoadNbBytesTotal;
	crc32_reset ( &MemoryLoadCrc );
}


/*-----------------------------------------------------------------------*/
/**
 * MEMORY READ
 *
 * 0x21
 * ADRMSB        ; address in controller
 * ADRLSB        ; memory to be read
 *   Returns:
 *     0xF6    ; status header
 *     0x20    ; memory access
 *     { data }  ; 6 data bytes starting at ADR
 *
 * NOTE : This function requires to handle the IKBD's RAM, which is only
 * possible when emulating a real HD6301 CPU. For now, we only return
 * the correct header and 6 empty bytes.
 */
static void IKBD_Cmd_ReadMemory(void)
{
	int	i;

	/* Return packet header */
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx ( 0xF6 );
		acia_ikbd_rx ( 0x20 );

		int addr = ((int)Keyboard.InputBuffer[1] << 8) | Keyboard.InputBuffer[2];
		for ( i=0 ; i<6 ; i++ )
		{
			uint8_t* src = resolve_address (addr++);
			acia_ikbd_rx ( src ? *src++ : 0x00);
		}
	}
}


/*-----------------------------------------------------------------------*/
/**
 * CONTROLLER EXECUTE
 *
 * 0x22
 * ADRMSB      ; address of subroutine in
 * ADRLSB      ; controller memory to be called
 */
static void IKBD_Cmd_Execute(void)
{
	if ( pIKBD_CustomCodeHandler_Write )
	{
		IKBD_ExeMode = true;	/* turn 6301's custom mode ON */
	}
	else
	{
		/* unknown code uploaded to ikbd RAM */
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT MOUSE BUTTON ACTION
 *
 * 0x87
 */
static void IKBD_Cmd_ReportMouseAction(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ); */
		acia_ikbd_rx (7);
		acia_ikbd_rx (KeyboardProcessor.Mouse.Action);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT MOUSE MODE
 *
 * 0x88 or 0x89 or 0x8A
 */
static void IKBD_Cmd_ReportMouseMode(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ); */
		switch (KeyboardProcessor.MouseMode)
		{
		case AUTOMODE_MOUSEREL:
			acia_ikbd_rx (8);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			break;
		case AUTOMODE_MOUSEABS:
			acia_ikbd_rx (9);
			acia_ikbd_rx (KeyboardProcessor.Abs.MaxX >> 8);
			acia_ikbd_rx (KeyboardProcessor.Abs.MaxX);
			acia_ikbd_rx (KeyboardProcessor.Abs.MaxY >> 8);
			acia_ikbd_rx (KeyboardProcessor.Abs.MaxY);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			break;
		case AUTOMODE_MOUSECURSOR:
			acia_ikbd_rx (10);
			acia_ikbd_rx (KeyboardProcessor.Mouse.KeyCodeDeltaX);
			acia_ikbd_rx (KeyboardProcessor.Mouse.KeyCodeDeltaY);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			break;
		}
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT MOUSE THRESHOLD
 *
 * 0x8B
 */
static void IKBD_Cmd_ReportMouseThreshold(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ) */
		acia_ikbd_rx (0x0B);
		acia_ikbd_rx (KeyboardProcessor.Mouse.XThreshold);
		acia_ikbd_rx (KeyboardProcessor.Mouse.YThreshold);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT MOUSE SCALE
 *
 * 0x8C
 */
static void IKBD_Cmd_ReportMouseScale(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ) */
		acia_ikbd_rx (0x0C);
		acia_ikbd_rx (KeyboardProcessor.Mouse.XScale);
		acia_ikbd_rx (KeyboardProcessor.Mouse.YScale);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT MOUSE VERTICAL COORDINATES
 *
 * 0x8F and 0x90
 */
static void IKBD_Cmd_ReportMouseVertical(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ) */
		if (KeyboardProcessor.Mouse.YAxis == -1)
			acia_ikbd_rx (0x0F);
		else
			acia_ikbd_rx (0x10);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT MOUSE AVAILABILITY
 *
 * 0x92
 */
static void IKBD_Cmd_ReportMouseAvailability(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ) */
		if (KeyboardProcessor.MouseMode == AUTOMODE_OFF)
			acia_ikbd_rx (0x12);
		else
			acia_ikbd_rx (0x00);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT JOYSTICK MODE
 *
 * 0x94 or 0x95 or 0x99
 */
static void IKBD_Cmd_ReportJoystickMode(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ) */
		switch (KeyboardProcessor.JoystickMode)
		{
		case AUTOMODE_JOYSTICK:
			acia_ikbd_rx (0x14);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			break;
		default:    /* TODO: Joystick keycodes mode not supported yet! */
			acia_ikbd_rx (0x15);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			acia_ikbd_rx (0);
			break;
		}
	}
}


/*-----------------------------------------------------------------------*/
/**
 * REPORT JOYSTICK AVAILABILITY
 *
 * 0x9A
 */
static void IKBD_Cmd_ReportJoystickAvailability(void)
{
	if ( IKBD_OutputBuffer_CheckFreeCount ( 8 ) )
	{
		acia_ikbd_rx (0xF6); /* IKBD_Delay_Random ( 7000 , 7500 ) */
		if (KeyboardProcessor.JoystickMode == AUTOMODE_OFF)
			acia_ikbd_rx (0x1A);
		else
			acia_ikbd_rx (0x00);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
		acia_ikbd_rx (0);
	}
}




/************************************************************************/
/* End of the IKBD's commands emulation.				*/
/************************************************************************/




/*************************************************************************/
/**
 * Below part is for emulating custom 6301 program sent to the IKBD's RAM
 * Specific read/write functions for each demo/game should be added here,
 * after being defined in the CustomCodeDefinitions[] array.
 *
 * The 6301 has 256 bytes of RAM, but only 128 bytes are available to
 * put a program (from $80 to $ff).
 *
 * Executing a program in the 6301 is a 2 steps process :
 *	1) a very small program is sent to the RAM using the 0x20 command.
 *	   This is often loaded at address $b0.
 * 	   This program will stop interruptions in the 6301 and will accept
 *	   a second small program that will relocate itself to $80.
 *	2) the relocated program at address $80 will accept a third (main)
 *	   program and will execute it once reception is complete.
 *
 * Writes during step 1 are handled with the ExeBootHandler matching the
 * LoadMemory CRC.
 * ExeBootHandler will compute a 2nd CRC for the writes corresponding to
 * the 2nd and 3rd programs sent to the 6301's RAM.
 *
 * If a match is found for this 2nd CRC, we will override default IKBD's behaviour
 * for reading/writing to $fffc02 with ExeMainHandler_Read / ExeMainHandler_Write
 * (once the Execute command 0x22 is received).
 *
 * When using custom program (ExeMode==true), we must ignore all keyboard/mouse/joystick
 * events sent to acia_ikbd_rx . Only our functions can add bytes
 * to the keyboard buffer.
 *
 * To exit 6301's execution mode, we can use the 68000 'reset' instruction.
 * Some 6301's programs also handle a write to $fffc02 as an exit signal.
 */


/*-----------------------------------------------------------------------*/
/**
 * Handle writes to $fffc02 when loading bytes in the IKBD's RAM.
 * We compute a CRC of the bytes that are sent until MemoryLoadNbBytesLeft
 * reaches 0.
 * When all bytes are loaded, we look for a matching CRC ; if found, we
 * use the ExeBootHandler defined for this CRC to process the next writes
 * that will occur in $fffc02.
 * LoadMemory is often used to load a small boot code into the 6301's RAM.
 * This small program will be executed later using the command 0x22.
 */

static void IKBD_LoadMemoryByte ( uint8_t aciabyte )
{
	unsigned int i;

	crc32_add_byte ( &MemoryLoadCrc , aciabyte );

	uint8_t* src = resolve_address (MemoryLoadNbBytesAddr++);

	if(src)
		*src = aciabyte;

	MemoryLoadNbBytesLeft--;
	if ( MemoryLoadNbBytesLeft == 0 )				/* all bytes were received */
	{
		/* Search for a match amongst the known custom routines */
		for ( i = 0 ; i < sizeof ( CustomCodeDefinitions ) / sizeof ( CustomCodeDefinitions[0] ) ; i++ )
			if ( CustomCodeDefinitions[ i ].LoadMemCrc == MemoryLoadCrc )
				break;

		if ( i < sizeof ( CustomCodeDefinitions ) / sizeof ( CustomCodeDefinitions[0] ) )	/* found */
		{
			crc32_reset ( &MemoryLoadCrc );
			MemoryExeNbBytes = 0;
			pIKBD_CustomCodeHandler_Read = NULL;
			pIKBD_CustomCodeHandler_Write = CustomCodeDefinitions[ i ].ExeBootHandler;
		}

		else							/* unknown code uploaded to IKBD's RAM */
		{
			pIKBD_CustomCodeHandler_Read = NULL;
			pIKBD_CustomCodeHandler_Write = NULL;
		}
	}
}



/*-----------------------------------------------------------------------*/
/**
 * Handle writes to $fffc02 when executing custom code in the IKBD's RAM.
 * This is used to send the small IKBD program that will handle
 * keyboard/mouse/joystick input.
 * We compute a CRC of the bytes that are sent until we found a match
 * with a known custom IKBD program.
 */

static void IKBD_CustomCodeHandler_CommonBoot ( uint8_t aciabyte )
{
	unsigned int i;

	crc32_add_byte ( &MemoryLoadCrc , aciabyte );
	MemoryExeNbBytes++;

	/* Search for a match amongst the known custom routines */
	for ( i = 0 ; i < sizeof ( CustomCodeDefinitions ) / sizeof ( CustomCodeDefinitions[0] ) ; i++ )
		if ( ( CustomCodeDefinitions[ i ].MainProgNbBytes == MemoryExeNbBytes )
			&& ( CustomCodeDefinitions[ i ].MainProgCrc == MemoryLoadCrc ) )
			break;

	if ( i < sizeof ( CustomCodeDefinitions ) / sizeof ( CustomCodeDefinitions[0] ) )	/* found */
	{
		pIKBD_CustomCodeHandler_Read = CustomCodeDefinitions[ i ].ExeMainHandler_Read;
		pIKBD_CustomCodeHandler_Write = CustomCodeDefinitions[ i ].ExeMainHandler_Write;
	}

	/* If not found, we keep on accumulating bytes until we find a matching crc */
}



/*----------------------------------------------------------------------*/
/* Froggies Over The Fence menu.					*/
/* Returns 'n' bytes with the mouse position, keyboard can be used too.	*/
/* Writing a <0 byte to $fffc02 will cause the 6301 to exit custom exe	*/
/* mode (jmp $f000).							*/
/* When writing byte 'n' >0 to $fffc02, the 6301 will return the content*/
/* of RAM $7f+n to $7f+1.						*/
/* $80/$81 contains deltaY/deltaX + left mouse button in bit 7, $82	*/
/* contains LMB in bit 7 and $83 contains a fixed value 0xfc.		*/
/* On each VBL, the demo will ask for 1 byte, then for 4 bytes ; only	*/
/* the last 2 bytes ($81/$80) will be used, $83/$82 are ignored.	*/
/* IKBD's $81 will be stored in $600 (CPU RAM), and $80 in $601.	*/
/*									*/
/* TODO : an extra delay of 7000 cycles is necessary to have $81 and $80*/
/* received after the overrun condition was cleared at the 68000 level.	*/
/* Does it mean some timings are wrong with acia/ikbd ?			*/
/*----------------------------------------------------------------------*/

static void IKBD_CustomCodeHandler_FroggiesMenu_Read ( void )
{
	/* Ignore read */
}

static void IKBD_CustomCodeHandler_FroggiesMenu_Write ( uint8_t aciabyte )
{
	uint8_t		res80 = 0;
	uint8_t		res81 = 0;
	uint8_t		res82 = 0;
	uint8_t		res83 = 0xfc;					/* fixed value, not used */

	/* When writing a <0 byte to $fffc02, Froggies ikbd's program will terminate itself */
	/* and leave Execution mode (jmp $f000) */
	if ( aciabyte & 0x80 )
	{
		IKBD_Boot_ROM ( false );
		return;
	}

	if ( KeyboardProcessor.Mouse.DeltaY < 0 )	res80 = 0x7a;	/* mouse up */
	if ( KeyboardProcessor.Mouse.DeltaY > 0 )	res80 = 0x06;	/* mouse down */
	if ( KeyboardProcessor.Mouse.DeltaX < 0 )	res81 = 0x7a;	/* mouse left */
	if ( KeyboardProcessor.Mouse.DeltaX > 0 )	res81 = 0x06;	/* mouse right */
	if ( Keyboard.bLButtonDown & BUTTON_MOUSE )	res82 |= 0x80;	/* left mouse button */

	if ( ScanCodeState[ 0x48 ] )			res80 |= 0x7a;	/* up */
	if ( ScanCodeState[ 0x50 ] )			res80 |= 0x06;	/* down */
	if ( ScanCodeState[ 0x4b ] )			res81 |= 0x7a;	/* left */
	if ( ScanCodeState[ 0x4d ] )			res81 |= 0x06;	/* right */
	if ( ScanCodeState[ 0x70 ] )			res82 |= 0x80;	/* keypad 0 */

	res80 |= res82;							/* bit 7 is left mouse button */
	res81 |= res82;

//	res80 = 0x10 ; res81 = 0x11 ; res82 = 0x12 ; res83 = 0x13 ;	/* force some discernible values to debug */
	
	if ( aciabyte == 1 )						/* Send 1 byte */
		acia_ikbd_rx ( res80 );			/* $80 in IKBD's RAM */

	else if ( aciabyte == 4 )					/* Send 4 bytes */
	{
		acia_ikbd_rx ( res83 );			/* $83 in IKBD's RAM */  /* Delay 7000 */			
		acia_ikbd_rx ( res82 );			/* $82 in IKBD's RAM */
		acia_ikbd_rx ( res81 );			/* $81 in IKBD's RAM */
		acia_ikbd_rx ( res80 );			/* $80 in IKBD's RAM */
	}
}



/*----------------------------------------------------------------------*/
/* Transbeauce II menu.							*/
/* Returns 1 byte with the joystick position, keyboard can be used too.	*/
/*----------------------------------------------------------------------*/

static void IKBD_CustomCodeHandler_Transbeauce2Menu_Read ( void )
{
	uint8_t		res = 0;

	/* keyboard emulation */
	if ( ScanCodeState[ 0x48 ] )	res |= 0x01;		/* up */
	if ( ScanCodeState[ 0x50 ] )	res |= 0x02;		/* down */
	if ( ScanCodeState[ 0x4b ] )	res |= 0x04;		/* left */
	if ( ScanCodeState[ 0x4d ] )	res |= 0x08;		/* right */
	if ( ScanCodeState[ 0x62 ] )	res |= 0x40;		/* help */
	if ( ScanCodeState[ 0x39 ] )	res |= 0x80;		/* space */

	/* joystick emulation (bit mapping is same as cursor above, with bit 7 = fire button */
	res |= ( KeyboardProcessor.Joy.JoyData[JOYID_JOYSTICK1] & 0x8f ) ;			/* keep bits 0-3 and 7 */

	acia_ikbd_rx ( res );
}

static void IKBD_CustomCodeHandler_Transbeauce2Menu_Write ( uint8_t aciabyte )
{
  /* Ignore write */
}



/*----------------------------------------------------------------------*/
/* Dragonnels demo menu.						*/
/* When any byte is written in $fffc02, returns one byte with the	*/
/* Y position of the mouse and the state of the left button.		*/
/*----------------------------------------------------------------------*/

static void IKBD_CustomCodeHandler_DragonnelsMenu_Read ( void )
{
	/* Ignore read */
}

static void IKBD_CustomCodeHandler_DragonnelsMenu_Write ( uint8_t aciabyte )
{
	uint8_t		res = 0;

	if ( KeyboardProcessor.Mouse.DeltaY < 0 )	res = 0xfc;	/* mouse up */
	if ( KeyboardProcessor.Mouse.DeltaY > 0 )	res = 0x04;	/* mouse down */

	if ( Keyboard.bLButtonDown & BUTTON_MOUSE )	res = 0x80;	/* left mouse button */

	acia_ikbd_rx ( res );
}



/*----------------------------------------------------------------------*/
/* Chaos A.D. protection's decoder					*/
/* This custom program reads bytes, decode them and send back the result*/
/* to the 68000.							*/
/* The program first returns $fe to indicate it's ready to receive the	*/
/* encoded bytes.							*/
/* The program then receives the 8 bytes used to decode the data and	*/
/* store them in $f0 - $f7 (KeyBuffer is already initialized, so we	*/
/* ignore those 8 bytes).						*/
/* Then for any received byte a XOR is made with one of the byte in the	*/
/* 8 bytes buffer, by incrementing an index in this buffer.		*/
/* The decoded byte is written to addr $13 (TDR) to be received by ACIA	*/
/*----------------------------------------------------------------------*/

static void IKBD_CustomCodeHandler_ChaosAD_Read ( void )
{
	static bool	FirstCall = true;

	if ( FirstCall == true )
		acia_ikbd_rx ( 0xfe );

	FirstCall = false;
}

static void IKBD_CustomCodeHandler_ChaosAD_Write ( uint8_t aciabyte )
{
	static int	IgnoreNb = 8;
	uint8_t		KeyBuffer[] = { 0xca , 0x0a , 0xbc , 0x00 , 0xde , 0xde , 0xfe , 0xca };
	static int	Index = 0;
	static int	Count = 0;

	/* We ignore the first 8 bytes we received (they're already in KeyBuffer) */
	if ( IgnoreNb > 0 )
	{
		IgnoreNb--;
		return;
	}

	if ( Count <= 6080 )						/* there're 6081 bytes to decode */
	{
		Count++;
		
		aciabyte ^= KeyBuffer[ Index ];
		Index++;
		Index &= 0x07;

		acia_ikbd_rx ( aciabyte );
	}

	else
	{
		/* When all bytes were decoded if 0x08 is written to $fffc02 */
		/* the program will terminate itself and leave Execution mode */
		if ( aciabyte == 0x08 )
			IKBD_Boot_ROM ( false );
	}
}

/*----------------------------------------------------------------------*/
/* Audio Sculpture decryption support					*/
/* The main executable is decrypted with a key extracted from a   	*/
/* previously uploaded program in the 6301. When the magic value 0x42 	*/
/* is sent to fffc02 it will output the two bytes 0x4b and 0x13		*/
/* and exit the custom handler again					*/
/* [NP] The custom program has 2 parts :				*/
/*  - 1st part is used during the intro and wait for key 'space' in	*/
/*    color mode or any key in mono mode (but intro screen in mono	*/
/*    exits automatically without testing a key !)			*/
/*  - 2nd part wait to receive $42 from the ACIA, then send $4b and $13	*/
/*----------------------------------------------------------------------*/

static bool ASmagic = false;

static void IKBD_CustomCodeHandler_AudioSculpture_Color_Read ( void )
{
	IKBD_CustomCodeHandler_AudioSculpture_Read ( true );
}

static void IKBD_CustomCodeHandler_AudioSculpture_Mono_Read ( void )
{
	IKBD_CustomCodeHandler_AudioSculpture_Read ( false );
}

static void IKBD_CustomCodeHandler_AudioSculpture_Read ( bool ColorMode )
{
	uint8_t		res = 0;
	static int	ReadCount = 0;

	if ( ASmagic )
	{
		ReadCount++;
		if ( ReadCount == 2 )			/* We're done reading out the 2 bytes, exit the custom handler */
		{
			IKBD_Boot_ROM ( false );
			ASmagic = false;
			ReadCount = 0;
		}
	}

	else if ( ( ( ColorMode == false ) && ( IKBD_CheckPressedKey() >= 0 ) )		/* wait for any key in mono mode */
		|| ScanCodeState[ 0x39 ] )		/* wait for 'space' key in color mode */
	{
		res = 0x39;				/* send scancode for 'space' */
		acia_ikbd_rx ( res );
	}
}

static void IKBD_CustomCodeHandler_AudioSculpture_Write ( uint8_t aciabyte )
{
	uint8_t		Magic = 0x42;
	uint8_t		Key[] = { 0x4b , 0x13 };

	if ( aciabyte == Magic )
	{
		ASmagic = true;
		acia_ikbd_rx ( Key[0] );
		acia_ikbd_rx ( Key[1] );
	}
}
