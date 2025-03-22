/*
  Hatari - ikbd.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.
*/

#ifndef HATARI_IKBD_H
#define HATARI_IKBD_H

/* Keyboard processor details */

typedef struct {
  int X,Y;                        /* Position of mouse */
  int MaxX,MaxY;                  /* Max limits of mouse */
  uint8_t PrevReadAbsMouseButtons;  /* Previous button mask for 'IKBD_Cmd_ReadAbsMousePos' */
} ABS_MOUSE;

typedef struct {
  int dx, dy;                     /* Mouse delta to be added */
  int DeltaX,DeltaY;              /* Final XY mouse position delta */
  int XScale,YScale;              /* Scale of mouse */
  int XThreshold,YThreshold;      /* Threshold */
  uint8_t KeyCodeDeltaX,KeyCodeDeltaY;    /* Delta X,Y for mouse keycode mode */
  int YAxis;                      /* Y-Axis direction */
  uint8_t Action;                   /* Bit 0-Report abs position of press, Bit 1-Report abs on release */
} MOUSE;

typedef struct {
  uint8_t JoyData[2];               /* Joystick details */
  uint8_t PrevJoyData[2];           /* Previous joystick details, used to check for 'IKBD_SendAutoJoysticks' */
} JOY;

typedef struct {
  ABS_MOUSE  Abs;
  MOUSE    Mouse;
  JOY      Joy;
  int MouseMode;                  /* AUTOMODE_xxxx */
  int JoystickMode;               /* AUTOMODE_xxxx */
} KEYBOARD_PROCESSOR;

/* Keyboard state */
#define SIZE_KEYBOARDINPUT_BUFFER 8
typedef struct {

  bool PauseOutput;				/* If true, don't send bytes anymore (see command 0x13) */

  uint8_t InputBuffer[SIZE_KEYBOARDINPUT_BUFFER];	/* Buffer for data send from CPU to keyboard processor (commands) */
  int nBytesInInputBuffer;			/* Number of command bytes in above buffer */

  int bLButtonDown,bRButtonDown;                /* Mouse states in emulation system, BUTTON_xxxx */
  int bOldLButtonDown,bOldRButtonDown;

  int32_t AutoSendCycles;				/* Number of cpu cycles to call INTERRUPT_IKBD_AUTOSEND */
} KEYBOARD;

/* Button states, a bit mask so can mimic joystick/right mouse button duplication */
#define BUTTON_NULL      0x00     /* Button states, so can OR together mouse/joystick buttons */
#define BUTTON_MOUSE     0x01
#define BUTTON_JOYSTICK  0x02

/* Mouse/Joystick modes */
#define AUTOMODE_OFF			0
#define AUTOMODE_MOUSEREL		1
#define AUTOMODE_MOUSEABS		2
#define AUTOMODE_MOUSECURSOR		3
#define AUTOMODE_JOYSTICK		4
#define AUTOMODE_JOYSTICK_MONITORING	5

extern void IKBD_Init ( void );
extern void IKBD_Reset(bool bCold);
extern void IKBD_UpdateClockOnVBL ( void );
extern void IKBD_SendAutoKeyboardCommands(void);
extern void IKBD_PressSTKey(uint32_t ScanCode);
extern void IKBD_Process_RDR ( uint8_t RDR );

//extern void IKBD_Info(FILE *fp, uint32_t dummy);

#endif  /* HATARI_IKBD_H */