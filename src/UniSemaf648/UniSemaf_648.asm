;;======================================================================;;
;;			DCC ACCESORY DECODER				;;
;;======================================================================;;
;;									;;
;; Program:         UNISEMAF -- DCC semaphore decoder			;;
;; Code:            Paco Cañada	-- http://www.fut.es/~fmco		;;
;; Platform:        Microchip PIC16F628/PIC16F648A, 8 Mhz		;;
;; Date:            05.05.2007						;;
;; First release:   11.05.2007						;;
;; LastDate:        06.08.2008						;;
;;									;;
;;======================================================================;;
;
; This program is distributed as is but WITHOUT ANY WARRANTY
; I hope you enjoy!!
;
; Revisions:
; 05.05.2007	Start of writting code (PIC12F629)
; 08.05.2007	All aspects works
; 10.05.2007	Flashing effects
; 06.08.2008	PIC16F628/16F648A version. 8 lights, 32 aspects (24 aspects with PIC16F628)


; ----- Definitions

#define		__VERNUM	D'1'
#define		__VERDAY	0x06
#define		__VERMONTH	0x08
#define		__VERYEAR	0x08


;#define		__PIC628	1		; select processor. Uncomment this line for 16F628, comment it for 16F648A
#define		__PUSHBUTTON	1		; Uncomment this line for enable programming by pushbutton status
;		^Yes	^No		; 



		errorlevel -302
		errorlevel -306

	ifdef	__PIC628


                list    p=16F628,r=hex

	        INCLUDE "P16F628.INC"

                __FUSES _BODEN_ON & _CP_OFF & _PWRTE_ON & _WDT_OFF & _LVP_OFF & _MCLRE_OFF  & _HS_OSC

#define		RAMINI0		0x020		; 80 bytes
#define		RAMINI1		0x0A0		; 80 bytes
#define		RAMINI2		0x120		; 48 bytes
#define		RAMINT		0x070		; 16 bytes

	else

                list    p=16F648A,r=hex



	        INCLUDE "P16F648A.INC"

                __FUSES _BODEN_ON & _CP_OFF & _PWRTE_ON & _WDT_OFF & _LVP_OFF & _MCLRE_OFF  & _HS_OSC

#define		RAMINI0		0x020		; 80 bytes
#define		RAMINI1		0x0A0		; 80 bytes
#define		RAMINI2		0x120		; 80 bytes
#define		RAMINT		0x070		; 16 bytes

	endif



; --- Macros

#define		DNOP		goto	$+1


; --- Constant values 

FXTAL		equ	D'8000000'


	ifdef	__PUSHBUTTON

RB_TRIS		equ	0xA0			; RB7,RB5: input

	else

RB_TRIS		equ	0x80			; RB7: input

	endif

RA_TRIS         equ     0x10			; RA4: input
RA_INI          equ     0x00			; all zero
RB_INI          equ     0x00			; all zero
OPTION_INI	equ	0x88			; Option register: no pull-up, no prescaler, wdt 1:1

INTC_INI	equ	0x88			; GIE, RBIE enable, PEIE disable
PIE1_INI	equ	0x00			; no interrupts

#define		OUT1A	0			; Semaphore 1. Port A
#define		OUT1B	1			; 
#define		OUT2A	2			; Semaphore 2
#define		OUT2B	3			; 
#define		OUT3A	4			; Semaphore 3. Port B
#define		OUT3B	5			; 
#define		OUT4A	6			; Semaphore 4
#define		OUT4B	7			;
#define		SWITCH	PORTB,5			; switch
#define		ACKOUT	PORTB,6			; acknowledge
#define		DCCIN	PORTB,7			; DCC input pin



; --- EEPROM Section

#define		EE_INI		0x00

E_CV513		equ	EE_INI+0x00		; CV513	Primary Adress low
E_CV515		equ	EE_INI+0x01		; CV515	Max.light 0..15
E_CV516		equ	EE_INI+0x02		; CV516	
E_CV517		equ	EE_INI+0x03		; CV517	
E_CV518		equ	EE_INI+0x04		; CV518	
E_CV7		equ	EE_INI+0x05		; Manufacturer Version
E_CV8		equ	EE_INI+0x06		; Manufacturer ID
E_CV521		equ	EE_INI+0x07		; CV521	Primary Adress high
E_CV541		equ	EE_INI+0x08		; config
E_CV545		equ	EE_INI+0x09		; CV545	Slope
E_CV546		equ	EE_INI+0x0A		; CV546	Flashing rate

EE_GRP1A	equ	EE_INI+0x0B		; saved groups
EE_GRP1B	equ	EE_INI+0x0C
EE_GRP2A	equ	EE_INI+0x0D
EE_GRP2B	equ	EE_INI+0x0E
EE_GRP3A	equ	EE_INI+0x0F
EE_GRP3B	equ	EE_INI+0x10
EE_GRP4A	equ	EE_INI+0x11
EE_GRP4B	equ	EE_INI+0x12

EE_OUT1A	equ	EE_INI+0x13		; saved outputs
EE_OUT1B	equ	EE_INI+0x14
EE_OUT2A	equ	EE_INI+0x15
EE_OUT2B	equ	EE_INI+0x16
EE_OUT3A	equ	EE_INI+0x17
EE_OUT3B	equ	EE_INI+0x18
EE_OUT4A	equ	EE_INI+0x19
EE_OUT4B	equ	EE_INI+0x1A


E_CV547		equ	EE_INI+0x20		; CV547	Aspect table

E_CV579		equ	EE_INI+0x40		; aspect 9



; ----- Variables

; --- Internal RAM Section

; --- Top on all banks

INT_W		equ	RAMINT+0x00		; interrupt context registers
INT_STAT	equ	RAMINT+0x01

SHIFT0		equ	RAMINT+0x02
DATA1		equ	RAMINT+0x03		; interrupt shift register
DATA2		equ	RAMINT+0x04
DATA3		equ	RAMINT+0x05
DATA4		equ	RAMINT+0x06

PREAMBLE	equ	RAMINT+0x07
DCCSTATE	equ	RAMINT+0x08
DCCBYTE		equ	RAMINT+0x09

PAGEREG		equ	RAMINT+0x0A		; Page register
FLAGS		equ	RAMINT+0x0B

EEDATA0		equ	RAMINT+0x0C		; EEPROM shadow variables
EEADR0		equ	RAMINT+0x0D

; --- Bank 0

CV513		equ	RAMINI0+0x00		; Primary Adress low byte
CV515		equ	RAMINI0+0x01		; max. bright
CV516		equ	RAMINI0+0x02		; 
CV517		equ	RAMINI0+0x03		; 
CV518		equ	RAMINI0+0x04		; 
CV519		equ	RAMINI0+0x05		; 
CV520		equ	RAMINI0+0x06		; 
CV521		equ	RAMINI0+0x07		; Primary Adress high byte
CV541		equ	RAMINI0+0x08		;
CV545		equ	RAMINI0+0x09		; slope
CV546		equ	RAMINI0+0x0A		; Flasing rate

CVENABLE	equ	RAMINI0+0x0C		; Current aspect definition
CVLIGHT		equ	RAMINI0+0x0D
CVFLASH		equ	RAMINI0+0x0E
CVFLASHAB	equ	RAMINI0+0x0F

LIGHT1A		equ	RAMINI0+0x10		; Light status flags
LIGHT1B		equ	RAMINI0+0x11
LIGHT2A		equ	RAMINI0+0x12
LIGHT2B		equ	RAMINI0+0x13
LIGHT3A		equ	RAMINI0+0x14
LIGHT3B		equ	RAMINI0+0x15
LIGHT4A		equ	RAMINI0+0x16
LIGHT4B		equ	RAMINI0+0x17

#define		BRIGHT	0
#define		FLASH	1
#define		FLSH_AB	2

GROUP1A		equ	RAMINI0+0x18		; light groups
GROUP1B		equ	RAMINI0+0x19
GROUP2A		equ	RAMINI0+0x1A
GROUP2B		equ	RAMINI0+0x1B
GROUP3A		equ	RAMINI0+0x1C
GROUP3B		equ	RAMINI0+0x1D
GROUP4A		equ	RAMINI0+0x1E
GROUP4B		equ	RAMINI0+0x1F

PWM1A		equ	RAMINI0+0x20		; current bright
PWM1B		equ	RAMINI0+0x21
PWM2A		equ	RAMINI0+0x22
PWM2B		equ	RAMINI0+0x23
PWM3A		equ	RAMINI0+0x24
PWM3B		equ	RAMINI0+0x25
PWM4A		equ	RAMINI0+0x26
PWM4B		equ	RAMINI0+0x27

SPEED1A		equ	RAMINI0+0x28		; final bright
SPEED1B		equ	RAMINI0+0x29
SPEED2A		equ	RAMINI0+0x2A
SPEED2B		equ	RAMINI0+0x2B
SPEED3A		equ	RAMINI0+0x2C
SPEED3B		equ	RAMINI0+0x2D
SPEED4A		equ	RAMINI0+0x2E
SPEED4B		equ	RAMINI0+0x2F

ACC_COUNT	equ	RAMINI0+0x30		; slope timer
ASPECT		equ	RAMINI0+0x31		; current aspect
FLASH_CNT	equ	RAMINI0+0x32

OUTPUT		equ	RAMINI0+0x34		; output buffer
DIR_UP		equ	RAMINI0+0x35		; bright / fade
LIGHT_UP	equ	RAMINI0+0x36		; light up pending
TIMER		equ	RAMINI0+0x37		; timer 1:16

TEMP		equ	RAMINI0+0x38
COUNT		equ	RAMINI0+0x39
DEBOUNCE	equ	RAMINI0+0x3A
DBC_CNT		equ	RAMINI0+0x3B
MSB		equ	RAMINI0+0x3C		; reset variables
LSB		equ	RAMINI0+0x3D
R0		equ	RAMINI0+0x3F


; --- Flags
						; FLAGS
#define		NEW_PACKET	FLAGS,0		; New packet received
#define		NOCV		FLAGS,1		; No CV finded
#define		RDONLY		FLAGS,2		; CV read only
#define		DCC4BYTE	FLAGS,3		; DCC command 4 bytes
#define		KEY_PROG	FLAGS,4		; Key pressed for programming
#define		FLASHAB		FLAGS,5		; Flash phase
#define		PROG_2X		FLAGS,6		; 2x prog
#define		RESET_FLG	FLAGS,7		; reset packet

KEYPRG_MSK	equ	0x10
FLASHAB_MSK	equ	0x20



; --------------- Program Section --------------------------------------


		org	0x000

PowerUp:
		clrf	STATUS			; Bank 0 default
		clrf	INTCON			; Disable all interrupts
		clrf	PCLATH			; tables on page 0
		goto	INIT

; ----------------------------------------------------------------------

		org	0x004

Interrupt:
		movwf	INT_W			; save context registers		;1
		swapf	STATUS,w							;2
		movwf	INT_STAT							;3
		clrf	STATUS			; interrupt uses bank 0			;4

Int_DCC:
		btfss	DCCIN								;5
		goto	Int_Low_Half							;6,7

Int_High_Half:
		movf	DCCSTATE,w							; 8
		addwf	PCL,f								; 9

		goto	Preamble							; 10,11
		goto	WaitLow
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadBit
		goto	ReadLastBit
		goto	EndByte1
		goto	EndByte2
		goto	EndByte3
		goto	EndByte4

Int_Low_Half:
		movlw	d'256' - d'154'		; 77us: between 64us (one) and 90us (zero);8
		movwf	TMR0								;9
		bcf	INTCON,T0IF		; clear overflow flag for counting	;10
;		movf	PORTB,w
		bcf	INTCON,RBIF							;11
		swapf	INT_STAT,w		; restore context registers		;13
		movwf	STATUS								;14
		swapf	INT_W,f								;15
		swapf	INT_W,w								;16
		retfie									;17,18

EndHighHalf:
;		movf	PORTB,w
		bcf	INTCON,RBIF							;21
EndInt:
		swapf	INT_STAT,w		; restore context registers		;22
		movwf	STATUS								;23
		swapf	INT_W,f								;24
		swapf	INT_W,w								;25
		retfie									;26,27


Preamble:
		btfss	NEW_PACKET		; wait until last decoded		;12
		incf	PREAMBLE,f		;					;13
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;14
		clrf	PREAMBLE		;					;15
		movlw	0xF6			; 10 preamble bits?			;16
		addwf	PREAMBLE,w		;					;17
		btfsc	STATUS,C		;					;18
		incf	DCCSTATE,f		; yes, next state			;19
		goto	EndHighHalf		;					;20,21
		

WaitLow:
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;12
		incf	DCCSTATE,f		; then state				;13
		clrf	DCCBYTE			;					;14
		clrf	PREAMBLE		;					;15
		clrf	DATA4			;					;16
		goto	EndHighHalf		;					;17,18


ReadBit:
		bsf	STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf	STATUS,C							;14
		rlf	SHIFT0,f		; receiver shift register		;15
		incf	DCCSTATE,f		;					;16
		goto	EndHighHalf		;					;17,18
			
ReadLastBit:
		bsf	STATUS,C							;12
		btfsc	INTCON,T0IF		; if timer 0 overflows then is a DCC zero;13
		bcf	STATUS,C							;14
		rlf	SHIFT0,f		; receiver shift register		;15
		incf	DCCBYTE,w							;16
		addwf	DCCSTATE,f							;17
		goto	EndHighHalf		;					;18,19

EndByte1:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA1			;					;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20

EndByte2:
		movlw	0x00			;					;12
		btfsc	INTCON,T0IF		; End bit=1, invalid packet		;13
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA2			;					;17
		incf	DCCBYTE,f		;					;18
		goto	EndHighHalf		;					;19,20


EndByte3:
		btfss	INTCON,T0IF		; End bit=1, end of packet		;12
		goto	EndByte3x		;					;13,14
		movlw	0x02			;					;14
		movwf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA3			;					;17
		incf	DCCBYTE,f		;					;18
		bsf	DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21
EndByte3x:
		clrf	DCCSTATE		;					;15
		movf	SHIFT0,w		;					;16
		movwf	DATA3			;					;17
		bsf	NEW_PACKET		;					;18
		bcf	DCC4BYTE		;					;19
		goto	EndHighHalf		;					;20,21

EndByte4:
		clrf	DCCSTATE		;					;12
		btfsc	INTCON,T0IF		; End bit=1, end of packet		;13
		goto	EndInt			; End bit=0, invalid packet		;14,15
		movf	SHIFT0,w		;					;15
		movwf	DATA4			;					;16
		bsf	NEW_PACKET		;					;17
		goto	EndHighHalf		;					;18,19


; ----------------------------------------------------------------------

BitPos:
		clrf	PCLATH
		addwf	PCL,f

		retlw	0x01
		retlw	0x02
		retlw	0x04
		retlw	0x08
		retlw	0x10
		retlw	0x20
		retlw	0x40
		retlw	0x80


; ----------------------------------------------------------------------

INIT:
		clrf	PORTA
		clrf	PORTB
		movlw	0x07
		movwf	CMCON			; set GP2:0 to digital I/O

		bsf	STATUS,RP0		; bank 1
		movlw	RB_TRIS
		movwf   TRISB
		movlw   RA_TRIS         	; Set port A I/O configuration
		movwf   TRISA
		clrf	VRCON			; voltage reference off
		movlw	OPTION_INI		; Option register: no pull-up, no prescaler, wdt 1:1
		movwf	OPTION_REG
		movlw	PIE1_INI
		movwf	PIE1
		bcf	STATUS,RP0		; bank 0
		clrf	PIR1
		movlw	0x11			; Timer 1 on, 1:2
		movwf	T1CON

		movlw	0x20			; clear RAM
		movwf	FSR
ClearRAM:
		clrf	INDF
		incf	FSR,f
		movlw	0x80
		xorwf	FSR,w
		btfss	STATUS,Z
		goto	ClearRAM

		movlw	INTC_INI
		movwf	INTCON			; enable GP2 external interrupt

		clrf	PAGEREG			; page register default

		call	LoadCV			; load CV values


		call	LoadOutputs		; load saved outputs
		movlw	0xFF
		movwf	ASPECT

		movlw	d'2'			; init slope counter
		movwf	ACC_COUNT
		movwf	FLASH_CNT

; ----------------------------------------------------------------------

MainLoop:
		btfsc	NEW_PACKET		; new packet?
		call	Decode			; yes, decode

	ifdef	__PUSHBUTTON

		decfsz	DBC_CNT,f		; debounce time
		goto	Loop
		bsf	DBC_CNT,2
		bcf	STATUS,C		; check key
		btfsc	SWITCH
		bsf	STATUS,C
		rlf	DEBOUNCE,w
		movwf	DEBOUNCE
		xorlw	0xC0
		movlw	KEYPRG_MSK
		btfsc	STATUS,Z
		xorwf	FLAGS,f

	else

		bcf	KEY_PROG

	endif

Loop:
		btfsc	KEY_PROG
		goto	DoFlashProg

		btfsc	RESET_FLG		; no output on reset
		goto	MainLoop


		btfss	PIR1,TMR1IF		; flash phase
		goto	Slope
		bcf	PIR1,TMR1IF
		decfsz	FLASH_CNT,f
		goto	Slope
		movf	CV546,w
		movwf	FLASH_CNT
		movlw	FLASHAB_MSK
		xorwf	FLAGS,f

Flash1A:
		btfss	LIGHT1A,FLASH		; flashing sequence
		goto	Flash1B
		clrf	SPEED1A
		bcf	DIR_UP,OUT1A
		bcf	LIGHT_UP,OUT1A
		btfss	FLASHAB
		goto	Flash1A_B
		btfsc	LIGHT1A,FLSH_AB
		bsf	LIGHT_UP,OUT1A
Flash1A_B:
		btfsc	FLASHAB
		goto	Flash1B
		btfss	LIGHT1A,FLSH_AB
		bsf	LIGHT_UP,OUT1A

Flash1B:
		btfss	LIGHT1B,FLASH
		goto	Flash2A
		clrf	SPEED1B
		bcf	DIR_UP,OUT1B
		bcf	LIGHT_UP,OUT1B
		btfss	FLASHAB
		goto	Flash1B_B
		btfsc	LIGHT1B,FLSH_AB
		bsf	LIGHT_UP,OUT1B
Flash1B_B:
		btfsc	FLASHAB
		goto	Flash2A
		btfss	LIGHT1B,FLSH_AB
		bsf	LIGHT_UP,OUT1B

Flash2A:
		btfss	LIGHT2A,FLASH
		goto	Flash2B
		clrf	SPEED2A
		bcf	DIR_UP,OUT2A
		bcf	LIGHT_UP,OUT2A
		btfss	FLASHAB
		goto	Flash2A_B
		btfsc	LIGHT2A,FLSH_AB
		bsf	LIGHT_UP,OUT2A
Flash2A_B:
		btfsc	FLASHAB
		goto	Flash2B
		btfss	LIGHT2A,FLSH_AB
		bsf	LIGHT_UP,OUT2A

Flash2B:
		btfss	LIGHT2B,FLASH
		goto	Flash3A
		clrf	SPEED2B
		bcf	DIR_UP,OUT2B
		bcf	LIGHT_UP,OUT2B
		btfss	FLASHAB
		goto	Flash2B_B
		btfsc	LIGHT2B,FLSH_AB
		bsf	LIGHT_UP,OUT2B
Flash2B_B:
		btfsc	FLASHAB
		goto	Flash3A
		btfss	LIGHT2B,FLSH_AB
		bsf	LIGHT_UP,OUT2B

Flash3A:
		btfss	LIGHT3A,FLASH		; flashing sequence
		goto	Flash3B
		clrf	SPEED3A
		bcf	DIR_UP,OUT3A
		bcf	LIGHT_UP,OUT3A
		btfss	FLASHAB
		goto	Flash3A_B
		btfsc	LIGHT3A,FLSH_AB
		bsf	LIGHT_UP,OUT3A
Flash3A_B:
		btfsc	FLASHAB
		goto	Flash3B
		btfss	LIGHT3A,FLSH_AB
		bsf	LIGHT_UP,OUT3A

Flash3B:
		btfss	LIGHT3B,FLASH
		goto	Flash4A
		clrf	SPEED3B
		bcf	DIR_UP,OUT3B
		bcf	LIGHT_UP,OUT3B
		btfss	FLASHAB
		goto	Flash3B_B
		btfsc	LIGHT3B,FLSH_AB
		bsf	LIGHT_UP,OUT3B
Flash3B_B:
		btfsc	FLASHAB
		goto	Flash4A
		btfss	LIGHT3B,FLSH_AB
		bsf	LIGHT_UP,OUT3B

Flash4A:
		btfss	LIGHT4A,FLASH
		goto	Flash4B
		clrf	SPEED4A
		bcf	DIR_UP,OUT4A
		bcf	LIGHT_UP,OUT4A
		btfss	FLASHAB
		goto	Flash4A_B
		btfsc	LIGHT4A,FLSH_AB
		bsf	LIGHT_UP,OUT4A
Flash4A_B:
		btfsc	FLASHAB
		goto	Flash4B
		btfss	LIGHT4A,FLSH_AB
		bsf	LIGHT_UP,OUT4A

Flash4B:
		btfss	LIGHT4B,FLASH
		goto	Slope
		clrf	SPEED4B
		bcf	DIR_UP,OUT4B
		bcf	LIGHT_UP,OUT4B
		btfss	FLASHAB
		goto	Flash4B_B
		btfsc	LIGHT4B,FLSH_AB
		bsf	LIGHT_UP,OUT4B
Flash4B_B:
		btfsc	FLASHAB
		goto	Slope
		btfss	LIGHT4B,FLSH_AB
		bsf	LIGHT_UP,OUT4B

Slope:
		decfsz	ACC_COUNT,f		; slope
		goto	SpeedOn

		movf	CV545,w			; slope CV
		movwf	ACC_COUNT

		
		movlw	0x00			; flashing outputs
		btfsc	LIGHT1A,FLASH
		iorlw	(1<<OUT1A)
		btfsc	LIGHT1B,FLASH
		iorlw	(1<<OUT1B)
		btfsc	LIGHT2A,FLASH
		iorlw	(1<<OUT2A)
		btfsc	LIGHT2B,FLASH
		iorlw	(1<<OUT2B)
		btfsc	LIGHT3A,FLASH
		iorlw	(1<<OUT3A)
		btfsc	LIGHT3B,FLASH
		iorlw	(1<<OUT3B)
		btfsc	LIGHT4A,FLASH
		iorlw	(1<<OUT4A)
		btfsc	LIGHT4B,FLASH
		iorlw	(1<<OUT4B)
		movwf	CVFLASH

DoLight1A:
		movf	GROUP1A,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP1A,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoLight1B		; no

		movf	PWM1A,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT1A
		goto	DoLight1B
		bcf	LIGHT_UP,OUT1A
		bsf	DIR_UP,OUT1A
		movf	CV515,w
		movwf	SPEED1A
DoLight1B:		
		movf	GROUP1B,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP1B,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoLight2A		; no

		movf	PWM1B,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT1B
		goto	DoLight2A
		bcf	LIGHT_UP,OUT1B
		bsf	DIR_UP,OUT1B
		movf	CV515,w
		movwf	SPEED1B
DoLight2A:
		movf	GROUP2A,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP2A,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoLight2B		; no

		movf	PWM2A,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT2A
		goto	DoLight2B
		bcf	LIGHT_UP,OUT2A
		bsf	DIR_UP,OUT2A
		movf	CV516,w
		movwf	SPEED2A
DoLight2B:		
		movf	GROUP2B,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP2B,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoLight3A		; no

		movf	PWM2B,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT2B
		goto	DoLight3A
		bcf	LIGHT_UP,OUT2B
		bsf	DIR_UP,OUT2B
		movf	CV516,w
		movwf	SPEED2B
DoLight3A:
		movf	GROUP3A,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP3A,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoLight3B		; no

		movf	PWM3A,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT3A
		goto	DoLight3B
		bcf	LIGHT_UP,OUT3A
		bsf	DIR_UP,OUT3A
		movf	CV517,w
		movwf	SPEED3A
DoLight3B:		
		movf	GROUP3B,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP3B,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoLight4A		; no

		movf	PWM3B,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT3B
		goto	DoLight4A
		bcf	LIGHT_UP,OUT3B
		bsf	DIR_UP,OUT3B
		movf	CV517,w
		movwf	SPEED3B
DoLight4A:
		movf	GROUP4A,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP4A,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoLight4B		; no

		movf	PWM4A,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT4A
		goto	DoLight4B
		bcf	LIGHT_UP,OUT4A
		bsf	DIR_UP,OUT4A
		movf	CV518,w
		movwf	SPEED4A
DoLight4B:		
		movf	GROUP4B,w		; wait all off
		andwf	CVFLASH,w
		btfsc	STATUS,Z
		movf	GROUP4B,w
		movwf	TEMP
		movlw	0x00			; current status of lights
		btfsc	TEMP,OUT1A
		iorwf	PWM1A,w
		btfsc	TEMP,OUT1B
		iorwf	PWM1B,w
		btfsc	TEMP,OUT2A
		iorwf	PWM2A,w
		btfsc	TEMP,OUT2B
		iorwf	PWM2B,w
		btfsc	TEMP,OUT3A
		iorwf	PWM3A,w
		btfsc	TEMP,OUT3B
		iorwf	PWM3B,w
		btfsc	TEMP,OUT4A
		iorwf	PWM4A,w
		btfsc	TEMP,OUT4B
		iorwf	PWM4B,w
		btfss	STATUS,Z		; all off?
		goto	DoPWM1A			; no

		movf	PWM4B,w
		btfsc	STATUS,Z
		btfss	LIGHT_UP,OUT4B
		goto	DoPWM1A
		bcf	LIGHT_UP,OUT4B
		bsf	DIR_UP,OUT4B
		movf	CV518,w
		movwf	SPEED4B

DoPWM1A:
		movf	PWM1A,w			; calc PWM acc/dec
		xorwf	SPEED1A,w
		btfsc	STATUS,Z		; arrived to final bright?
		goto	DoPWM1B			; yes
		movlw	0x01			; no, light / fade
		btfss	DIR_UP,OUT1A
		movlw	0xFF
		addwf	PWM1A,f
DoPWM1B:
		movf	PWM1B,w
		xorwf	SPEED1B,w
		btfsc	STATUS,Z
		goto	DoPWM2A
		movlw	0x01
		btfss	DIR_UP,OUT1B
		movlw	0xFF
		addwf	PWM1B,f
DoPWM2A:
		movf	PWM2A,w
		xorwf	SPEED2A,w
		btfsc	STATUS,Z
		goto	DoPWM2B
		movlw	0x01
		btfss	DIR_UP,OUT2A
		movlw	0xFF
		addwf	PWM2A,f
DoPWM2B:
		movf	PWM2B,w
		xorwf	SPEED2B,w
		btfsc	STATUS,Z
		goto	DoPWM3A
		movlw	0x01
		btfss	DIR_UP,OUT2B
		movlw	0xFF
		addwf	PWM2B,f
DoPWM3A:
		movf	PWM3A,w			; calc PWM acc/dec
		xorwf	SPEED3A,w
		btfsc	STATUS,Z		; arrived to final bright?
		goto	DoPWM3B			; yes
		movlw	0x01			; no, light / fade
		btfss	DIR_UP,OUT3A
		movlw	0xFF
		addwf	PWM3A,f
DoPWM3B:
		movf	PWM3B,w
		xorwf	SPEED3B,w
		btfsc	STATUS,Z
		goto	DoPWM4A
		movlw	0x01
		btfss	DIR_UP,OUT3B
		movlw	0xFF
		addwf	PWM3B,f
DoPWM4A:
		movf	PWM4A,w
		xorwf	SPEED4A,w
		btfsc	STATUS,Z
		goto	DoPWM4B
		movlw	0x01
		btfss	DIR_UP,OUT4A
		movlw	0xFF
		addwf	PWM4A,f
DoPWM4B:
		movf	PWM4B,w
		xorwf	SPEED4B,w
		btfsc	STATUS,Z
		goto	MainLoop
		movlw	0x01
		btfss	DIR_UP,OUT4B
		movlw	0xFF
		addwf	PWM4B,f
		goto	MainLoop


SpeedOn:
		clrf	OUTPUT			; do PWM of outputs

;		rrf	TMR1H,w			; 'XXXXXXAB''CDEFGHIJ'
;		rrf	TMR1L,w
;		bcf	STATUS,C
;		rrf	TIMER,w
;		btfsc	TMR1H,1
;		iorlw	0x80
;		iorlw	0x0F
;		movwf	TIMER


		rrf	TMR1H,w			; 'XXXXXXAB''CDEFGHIJ'
		rrf	TMR1L,w
;		movwf	TIMER			; 'BCDEFGHI'
;		bcf	STATUS,C
;		rrf	TIMER,w			; '0BCDEFGH'
;		btfsc	TMR1H,1
;		iorlw	0x8F
;		movwf	TIMER			; 'ABCDEFGH'

;		movf	TMR1L,w
		iorlw	0x0F
		movwf	TIMER

		addwf	PWM1A,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT1A
		movf	TIMER,w
		addwf	PWM1B,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT1B
		movf	TIMER,w
		addwf	PWM2A,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT2A
		movf	TIMER,w
		addwf	PWM2B,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT2B
		movf	TIMER,w
		addwf	PWM3A,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT3A
		movf	TIMER,w
		addwf	PWM3B,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT3B
		movf	TIMER,w
		addwf	PWM4A,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT4A
		movf	TIMER,w
		addwf	PWM4B,w
		btfsc	STATUS,C
		bsf	OUTPUT,OUT4B

		movf	OUTPUT,w
		andlw	0x0F
		movwf	PORTA			; output
		swapf	OUTPUT,w
		andlw	0x0F
		movwf	PORTB
		goto	MainLoop


DoFlashProg:
		btfss	PIR1,TMR1IF
		goto	MainLoop

		bcf	PIR1,TMR1IF

		movlw	(1<<OUT1A)		; default
		btfsc	PORTA,OUT1A		; rotate lights
		movlw	(1<<OUT1B)
		btfsc	PORTA,OUT1B
		movlw	(1<<OUT2A)
		btfsc	PORTA,OUT2A
		movlw	(1<<OUT2B)
		btfsc	PORTA,OUT2B
		movlw	(1<<OUT1A)
		movwf	PORTA

		movlw	(1<<OUT1A)		; default
		btfsc	PORTB,OUT1A		; rotate lights
		movlw	(1<<OUT1B)
		btfsc	PORTB,OUT1B
		movlw	(1<<OUT2A)
		btfsc	PORTB,OUT2A
		movlw	(1<<OUT2B)
		btfsc	PORTB,OUT2B
		movlw	(1<<OUT1A)
		movwf	PORTB

		goto	MainLoop


; ----------------------------------------------------------------------

LoadAspect:
		andlw	0x1F			; new aspect?
		movwf	EEADR0
		xorwf	ASPECT,w
		btfsc	STATUS,Z
		goto	ExitFunction		; no
		movf	EEADR0,w
		movwf	ASPECT
		bcf	STATUS,C		; multiply by 4
		rlf	EEADR0,f
		bcf	STATUS,C
		rlf	EEADR0,w
		addlw	E_CV547			; first aspect CV
		movwf	EEADR0
		call	EE_Read
		movwf	CVENABLE
		btfsc	CVENABLE,OUT1A		; group
		movwf	GROUP1A
		btfsc	CVENABLE,OUT1B
		movwf	GROUP1B
		btfsc	CVENABLE,OUT2A
		movwf	GROUP2A
		btfsc	CVENABLE,OUT2B
		movwf	GROUP2B
		btfsc	CVENABLE,OUT3A
		movwf	GROUP3A
		btfsc	CVENABLE,OUT3B
		movwf	GROUP3B
		btfsc	CVENABLE,OUT4A
		movwf	GROUP4A
		btfsc	CVENABLE,OUT4B
		movwf	GROUP4B

		incf	EEADR0,w
		call	EE_Read
		movwf	CVLIGHT
		movf	EEADR0,w
		addlw	0x02
		call	EE_Read
		movwf	CVFLASH
		movf	EEADR0,w
		addlw	0x03
		call	EE_Read
		movwf	CVFLASHAB

Light1A:
		btfss	CVENABLE,OUT1A		; apply to this light?
		goto	Light1B			; yes
		clrf	LIGHT1A
		btfsc	CVLIGHT,OUT1A
		bsf	LIGHT1A,BRIGHT
		btfsc	CVFLASH,OUT1A
		bsf	LIGHT1A,FLASH
		btfsc	CVFLASHAB,OUT1A
		bsf	LIGHT1A,FLSH_AB
		clrf	SPEED1A			; turn off
		bcf	DIR_UP,OUT1A
		bcf	LIGHT_UP,OUT1A
		btfsc	CVLIGHT,OUT1A
		bsf	LIGHT_UP,OUT1A		; next light this

Light1B:
		btfss	CVENABLE,OUT1B		; apply to this light?
		goto	Light2A			; yes
		clrf	LIGHT1B
		btfsc	CVLIGHT,OUT1B
		bsf	LIGHT1B,BRIGHT
		btfsc	CVFLASH,OUT1B
		bsf	LIGHT1B,FLASH
		btfsc	CVFLASHAB,OUT1B
		bsf	LIGHT1B,FLSH_AB
		clrf	SPEED1B			; turn off
		bcf	DIR_UP,OUT1B
		bcf	LIGHT_UP,OUT1B
		btfsc	CVLIGHT,OUT1B
		bsf	LIGHT_UP,OUT1B		; next light this

Light2A:
		btfss	CVENABLE,OUT2A		; apply to this light?
		goto	Light2B			; yes
		clrf	LIGHT2A
		btfsc	CVLIGHT,OUT2A
		bsf	LIGHT2A,BRIGHT
		btfsc	CVFLASH,OUT2A
		bsf	LIGHT2A,FLASH
		btfsc	CVFLASHAB,OUT2A
		bsf	LIGHT2A,FLSH_AB
		clrf	SPEED2A			; turn off
		bcf	DIR_UP,OUT2A
		bcf	LIGHT_UP,OUT2A
		btfsc	CVLIGHT,OUT2A
		bsf	LIGHT_UP,OUT2A		; next light this

Light2B:
		btfss	CVENABLE,OUT2B		; apply to this light?
		goto	Light3A			; yes
		clrf	LIGHT2B
		btfsc	CVLIGHT,OUT2B
		bsf	LIGHT2B,BRIGHT
		btfsc	CVFLASH,OUT2B
		bsf	LIGHT2B,FLASH
		btfsc	CVFLASHAB,OUT2B
		bsf	LIGHT2B,FLSH_AB
		clrf	SPEED2B			; turn off
		bcf	DIR_UP,OUT2B
		bcf	LIGHT_UP,OUT2B
		btfsc	CVLIGHT,OUT2B
		bsf	LIGHT_UP,OUT2B		; next light this

Light3A:
		btfss	CVENABLE,OUT3A		; apply to this light?
		goto	Light3B			; yes
		clrf	LIGHT3A
		btfsc	CVLIGHT,OUT3A
		bsf	LIGHT3A,BRIGHT
		btfsc	CVFLASH,OUT3A
		bsf	LIGHT3A,FLASH
		btfsc	CVFLASHAB,OUT3A
		bsf	LIGHT3A,FLSH_AB
		clrf	SPEED3A			; turn off
		bcf	DIR_UP,OUT3A
		bcf	LIGHT_UP,OUT3A
		btfsc	CVLIGHT,OUT3A
		bsf	LIGHT_UP,OUT3A		; next light this

Light3B:
		btfss	CVENABLE,OUT3B		; apply to this light?
		goto	Light4A			; yes
		clrf	LIGHT3B
		btfsc	CVLIGHT,OUT3B
		bsf	LIGHT3B,BRIGHT
		btfsc	CVFLASH,OUT3B
		bsf	LIGHT3B,FLASH
		btfsc	CVFLASHAB,OUT3B
		bsf	LIGHT3B,FLSH_AB
		clrf	SPEED3B			; turn off
		bcf	DIR_UP,OUT3B
		bcf	LIGHT_UP,OUT3B
		btfsc	CVLIGHT,OUT3B
		bsf	LIGHT_UP,OUT3B		; next light this

Light4A:
		btfss	CVENABLE,OUT4A		; apply to this light?
		goto	Light4B			; yes
		clrf	LIGHT4A
		btfsc	CVLIGHT,OUT4A
		bsf	LIGHT4A,BRIGHT
		btfsc	CVFLASH,OUT4A
		bsf	LIGHT4A,FLASH
		btfsc	CVFLASHAB,OUT4A
		bsf	LIGHT4A,FLSH_AB
		clrf	SPEED4A			; turn off
		bcf	DIR_UP,OUT4A
		bcf	LIGHT_UP,OUT4A
		btfsc	CVLIGHT,OUT4A
		bsf	LIGHT_UP,OUT4A		; next light this

Light4B:
		btfss	CVENABLE,OUT4B		; apply to this light?
		goto	ExitFunction		; yes
		clrf	LIGHT4B
		btfsc	CVLIGHT,OUT4B
		bsf	LIGHT4B,BRIGHT
		btfsc	CVFLASH,OUT4B
		bsf	LIGHT4B,FLASH
		btfsc	CVFLASHAB,OUT4B
		bsf	LIGHT4B,FLSH_AB
		clrf	SPEED4B			; turn off
		bcf	DIR_UP,OUT4B
		bcf	LIGHT_UP,OUT4B
		btfsc	CVLIGHT,OUT4B
		bsf	LIGHT_UP,OUT4B		; next light this


ExitFunction:
		bcf	RESET_FLG
		bsf	INTCON,GIE		; enable interrupts

;		return

		movf	CVENABLE,w		; save groups
		movwf	EEDATA0
		movlw	EE_GRP1A
		btfsc	CVENABLE,OUT1A
		call	SetParm
		movlw	EE_GRP1B
		btfsc	CVENABLE,OUT1B
		call	SetParm
		movlw	EE_GRP2A
		btfsc	CVENABLE,OUT2A
		call	SetParm
		movlw	EE_GRP2B
		btfsc	CVENABLE,OUT2B
		call	SetParm
		movlw	EE_GRP3A
		btfsc	CVENABLE,OUT3A
		call	SetParm
		movlw	EE_GRP3B
		btfsc	CVENABLE,OUT3B
		call	SetParm
		movlw	EE_GRP4A
		btfsc	CVENABLE,OUT4A
		call	SetParm
		movlw	EE_GRP4B
		btfsc	CVENABLE,OUT4B
		call	SetParm

		movf	LIGHT1A,w		; save output state
		movwf	EEDATA0
		movlw	EE_OUT1A
		call	SetParm
		movf	LIGHT1B,w
		movwf	EEDATA0
		movlw	EE_OUT1B
		call	SetParm
		movf	LIGHT2A,w
		movwf	EEDATA0
		movlw	EE_OUT2A
		call	SetParm
		movf	LIGHT2B,w
		movwf	EEDATA0
		movlw	EE_OUT2B
		call	SetParm
		movf	LIGHT3A,w
		movwf	EEDATA0
		movlw	EE_OUT3A
		call	SetParm
		movf	LIGHT3B,w
		movwf	EEDATA0
		movlw	EE_OUT3B
		call	SetParm
		movf	LIGHT4A,w
		movwf	EEDATA0
		movlw	EE_OUT4A
		call	SetParm
		movf	LIGHT4B,w
		movwf	EEDATA0
		movlw	EE_OUT4B
		goto	SetParm


; -----------------------------------------------------------------------------------

Decode:
		bcf	INTCON,GIE		; disable interrupts for more speed
		bcf	NEW_PACKET		; prepare for next packet

		movf	DATA1,w			; exclusive or check
		xorwf	DATA2,w
		xorwf	DATA3,w
		xorwf	DATA4,w

		btfss	STATUS,Z		; valid packet?
		goto	ExitDecode		; no, return

		movf	DATA1,w			; address = '00000000' ?
		btfsc	STATUS,Z
		goto	Broadcast
;		movf	DATA1,w
		andlw	0xC0
		xorlw	0x80			;'10xxxxxx'? accessory operation
		btfss	STATUS,Z
		goto	ChkProg

Accessory:
		btfsc	KEY_PROG		; programming mode?
		goto	AccessProg		; yes

;		bsf	ACKOUT			; ******
;		movlw	0x0F			;
;		movwf	PORTA
;		movwf	PORTB
;		goto	$


		btfsc	DATA2,7			;'1AAA1xxx'
		btfss	DATA2,3
		goto	ExitDecode
						; *****
		movf	CV513,w			; prepare registers
		andlw	0x3F
		movwf	LSB
		btfss	CV521,4
		bsf	LSB,6
		btfss	CV521,5
		bsf	LSB,7

		movf	DATA1,w
		andlw	0x3F
		movwf	MSB
		btfss	DATA2,4
		bsf	MSB,6
		btfss	DATA2,5
		bsf	MSB,7

		clrf	TEMP			; Aspects 1..8
		movf	LSB,w
		xorwf	MSB,w
		btfsc	STATUS,Z
		goto	AccChange

		incf	LSB,f			; Aspects 9..16
		movlw	0x08
		movwf	TEMP
		movf	LSB,w
		xorwf	MSB,w
		btfsc	STATUS,Z
		goto	AccChange

		incf	LSB,f			; Aspects 17..24
		movlw	0x10
		movwf	TEMP
		movf	LSB,w
		xorwf	MSB,w
		btfsc	STATUS,Z
		goto	AccChange

	ifdef	__PIC628

		goto	ExitDecode

	else

		incf	LSB,f			; Aspects 25..32
		movlw	0x18
		movwf	TEMP
		movf	LSB,w
		xorwf	MSB,w
		btfsc	STATUS,Z
		goto	AccChange
		goto	ExitDecode
	endif


AccChange:
		movf	DATA2,w			; '1xxx1aaa'
		andlw	0x07
		addwf	TEMP,w
		goto	LoadAspect

ChkProg:
		movf	DATA1,w
		andlw	0xF0
		xorlw	0x70			; '0111xxxx'?
		btfsc	STATUS,Z
		goto	CheckSM			; yes, may be service mode

		incfsz	DATA1,w			;'11111111' idle packet
		goto	ExitDecode
		bsf	INTCON,GIE		; yes, don't clear reset flag
		return
		
; ----------------------------------------------------------------------

Broadcast:
		movf	DATA2,w			; reset packet?
		btfss	STATUS,Z
		goto	ExitDecode
		bcf	PROG_2X
		bsf	RESET_FLG
						; reset decoder
		clrf	PORTA
		clrf	PORTB

		bsf	INTCON,GIE		; enable interrupts
		return


ExitDecode:
		bcf	RESET_FLG
		bsf	INTCON,GIE		; enable interrupts
		return

		
; -----------------------------------------------------------------------------------

;************* SM Mode *******************************************
; Service Mode

CheckSM:
		btfss	RESET_FLG		; check for SM, reset packet has to come first
		goto	ServModeError
		btfss	PROG_2X
		goto	SetSM_Flag

		btfsc	DCC4BYTE		; 3 or 4 byte packet?
		goto	ChkProg4

; 0111CRRR DDDDDDDD EEEEEEEE

		bcf	PROG_2X
		movf	DATA2,w				; save data
		movwf	EEDATA0

		movf	DATA1,w				; 3 byte programming
		andlw	b'11110111'
		xorlw	b'01110101'			; Reg6
		btfsc	STATUS,Z		
		goto	REG6
		xorlw	(b'01110101')^(b'01110100')	; Reg5
		btfsc	STATUS,Z		
		goto	REG5
		xorlw	(b'01110100')^(b'01110110')	; reg7
		btfsc	STATUS,Z
		goto	REG7	
		xorlw	(b'01110110')^(b'01110111')	; reg8
		btfsc	STATUS,Z
		goto	REG8	

		movf	DATA1,w
		andlw	0x03
		addwf	PAGEREG,w
		call	FindCV
		btfsc	NOCV
		goto	ExitProg
ProgReg:
		btfss	DATA1,3
		goto	EEVERI
		goto	EEPROG

REG5:
		movlw	E_CV541			; CV29 configuration
		goto	ProgReg

REG6:
		btfss	DATA1,3			; read or write
		goto	REG6RD
		decf	DATA2,f			; Page register
		rlf	DATA2,f
		rlf	DATA2,w
		andlw	b'11111100'		; page 1 and 129 are the same. CV1 & CV513
		movwf	PAGEREG
		goto	ExitProg
REG6RD:
		decf	DATA2,f			; read page register
		rlf	DATA2,f
		rlf	DATA2,w
		andlw	b'11111100'		; page 1 and 129 are the same. CV1 & CV513
		xorwf	PAGEREG,w
		goto	EEVERIP

REG7:
		movlw	E_CV7			; only read
		btfss	DATA1,3
		goto	EEVERI
		goto	ExitProg

REG8:
		movlw	E_CV8			; only read
		btfss	DATA1,3
		goto	EEVERI
		goto	CheckResetCV		; if CV8 = 33 reset CV

	
EEPROG:
		btfsc	RDONLY
		goto	CheckCV8
		call	SetParm			; program EEPROM
		call	AckPulse		; do ACK
		bcf	PROG_2X
		bcf	NEW_PACKET
		call	LoadCV
		goto	ExitProg


EEVERI:
		call	EE_Read			; check data
		xorwf	DATA2,w
EEVERIP:
		btfss	STATUS,Z
		goto	ExitProg
DoAck:
		call	AckPulse		; equal, do ACK
		bcf	PROG_2X
;		bcf	NEW_PACKET
		goto	ExitProg


SetSM_Flag:
		bsf	PROG_2X
		goto	ExitProg

ServModeError:
		bcf	RESET_FLG
		bcf	PROG_2X
		goto	ExitProg

CheckCV8:
		xorlw	E_CV8			; CV8?
		btfss	STATUS,Z
		goto	ExitProg
CheckResetCV:
		movlw	d'33'			; CV8 = 33 -> reset CV
		xorwf	DATA2,w
		btfss	STATUS,Z
		goto	ExitProg
CheckResetCVP:
		call	ResetCV			; program CV defaults
		call	AckPulse		; do ACK
		bcf	PROG_2X
		bcf	NEW_PACKET
		call	LoadCV
		call	LoadOutputs
		goto	ExitProg

ExitProg:
		bsf	INTCON,GIE		; enable interrupts
		return

; -----------------------------------------------------------------------------------

AckPulse:
		movlw	b'00001111'		; all lights on
		movwf	PORTA
		movlw	b'01001111'		; all lights on, Ack out
		movwf	PORTB
		movlw	d'6'			; 6ms pulse
		movwf	TEMP
		movlw	0x00
AckNext:
		DNOP				;1,2
		DNOP				;3,4
		addlw	0xFF			;1
		btfss	STATUS,Z		;2
		goto	AckNext			;3,4
		decfsz	TEMP,f
		goto	AckNext
		clrf	PORTA			; all light off	
		clrf	PORTB
		return

; -----------------------------------------------------------------------------------

; 0111CCAA AAAAAAAA DDDDDDDD EEEEEEEE

ChkProg4:
		bcf	PROG_2X
;		bcf	RESET_FLG
		movf	DATA3,w			; save data
		movwf	EEDATA0

		btfsc	DATA1,0			; CV513.. or CV1..
		goto	ExitProg

		movf	DATA2,w			; x0AAAAAAAA
		call	FindCV
		btfsc	NOCV
		goto	ExitProg
ProgDirect:
		btfsc	DATA1,3	
		goto	RomNxt
		btfss	DATA1,2
		goto	ExitProg		;00 not defined
RomNxt:
		btfss	DATA1,2
		goto	BitMan			;10 Bit Manipulation
		btfss	DATA1,3
		goto	EEVERI4			;01 Verify byte
WriteDirect:					;11 Write byte
		btfsc	RDONLY
		goto	CheckCV8D
		call	SetParm
		call	AckPulse
		bcf	PROG_2X
		bcf	NEW_PACKET
		call	LoadCV
		goto	ExitProg

EEVERI4:
		call	EE_Read			; check data
		xorwf	DATA3,w
		goto	EEVERIP

CheckCV8D:
		movlw	d'33'			; CV8 = 33 -> reset CV
		xorwf	DATA3,w
		btfss	STATUS,Z
		goto	ExitProg
		goto	CheckResetCVP


; 0111CCAA AAAAAAAA 111KDBBB EEEEEEEE

BitMan:
		call	EE_Read
		movwf	EEDATA0
		movlw	EEDATA0
		movwf	FSR
		movlw	b'00000111'
		andwf	DATA3,w
		call	BitPos
		btfss	DATA3,4			; K
		goto	Vbit			; K=0,verify bit
		btfsc	DATA3,3
		iorwf	INDF,f			; D=1,set bit
		xorlw	0xFF
		btfss	DATA3,3
		andwf	INDF,f			; D=0,clear bit
		movf	DATA2,w
		call	FindCV
		goto	ProgDirect		; write complete byte

Vbit:
		andwf	INDF,w
		btfsc	STATUS,Z
		goto	BitClear
BitSet:
		btfsc	DATA3,3			;D=0
		goto	DoAck			;D=1, ack
		goto	ExitProg
BitClear:
		btfss	DATA3,3			;D=1
		goto	DoAck			;D=0, ack
		goto	ExitProg

; -----------------------------------------------------------------------------------

LoadCV:
		movlw	CV513			; first CV to read
		movwf	FSR
		movlw	E_CV513
		movwf	EEADR0
LoadNext:
		movf	EEADR0,w
		call	EE_Read
		movwf	INDF
		movlw	CV546			; last CV to read
		xorwf	FSR,w
		btfsc	STATUS,Z
		goto	LoadSet
		incf	FSR,f
		incf	EEADR0,f
		goto	LoadNext
LoadSet:
		swapf	CV521,f
		comf	CV521,w			; top address is complemented
		andlw	0x70
		movwf	CV521			; now CV521='0AAA0000'

		movlw	0xF0			; set max. bright
		iorwf	CV515,f
		swapf	CV515,f
		iorwf	CV516,f
		swapf	CV516,f
		iorwf	CV517,f
		swapf	CV517,f
		iorwf	CV518,f
		swapf	CV518,f

		return


LoadOutputs:
		clrf	DIR_UP			; read saved outputs
		movlw	EE_OUT1A
		call	EE_Read
		movwf	LIGHT1A
		btfsc	LIGHT1A,BRIGHT
		bsf	DIR_UP,OUT1A
		movf	CV515,w
		btfss	DIR_UP,OUT1A
		clrw
		movwf	SPEED1A

		movlw	EE_OUT1B
		call	EE_Read
		movwf	LIGHT1B
		btfsc	LIGHT1B,BRIGHT
		bsf	DIR_UP,OUT1B
		movf	CV515,w
		btfss	DIR_UP,OUT1B
		clrw
		movwf	SPEED1B

		movlw	EE_OUT2A
		call	EE_Read
		movwf	LIGHT2A
		btfsc	LIGHT2A,BRIGHT
		bsf	DIR_UP,OUT2A
		movf	CV516,w
		btfss	DIR_UP,OUT2A
		clrw
		movwf	SPEED2A

		movlw	EE_OUT2B
		call	EE_Read
		movwf	LIGHT2B
		btfsc	LIGHT2B,BRIGHT
		bsf	DIR_UP,OUT2B
		movf	CV516,w
		btfss	DIR_UP,OUT2B
		clrw
		movwf	SPEED2B


		movlw	EE_OUT3A
		call	EE_Read
		movwf	LIGHT3A
		btfsc	LIGHT3A,BRIGHT
		bsf	DIR_UP,OUT3A
		movf	CV517,w
		btfss	DIR_UP,OUT3A
		clrw
		movwf	SPEED3A

		movlw	EE_OUT3B
		call	EE_Read
		movwf	LIGHT3B
		btfsc	LIGHT3B,BRIGHT
		bsf	DIR_UP,OUT3B
		movf	CV517,w
		btfss	DIR_UP,OUT3B
		clrw
		movwf	SPEED3B

		movlw	EE_OUT4A
		call	EE_Read
		movwf	LIGHT4A
		btfsc	LIGHT4A,BRIGHT
		bsf	DIR_UP,OUT4A
		movf	CV518,w
		btfss	DIR_UP,OUT4A
		clrw
		movwf	SPEED4A

		movlw	EE_OUT4B
		call	EE_Read
		movwf	LIGHT4B
		btfsc	LIGHT4B,BRIGHT
		bsf	DIR_UP,OUT4B
		movf	CV518,w
		btfss	DIR_UP,OUT4B
		clrw
		movwf	SPEED4B

		movlw	EE_GRP1A		; load groups
		call	EE_Read
		movwf	GROUP1A
		movlw	EE_GRP1B
		call	EE_Read
		movwf	GROUP1B
		movlw	EE_GRP2A
		call	EE_Read
		movwf	GROUP2A
		movlw	EE_GRP2B
		call	EE_Read
		movwf	GROUP2B
		movlw	EE_GRP3A
		call	EE_Read
		movwf	GROUP3A
		movlw	EE_GRP3B
		call	EE_Read
		movwf	GROUP3B
		movlw	EE_GRP4A
		call	EE_Read
		movwf	GROUP4A
		movlw	EE_GRP4B
		call	EE_Read
		movwf	GROUP4B
		clrf	LIGHT_UP
		return
		

; -----------------------------------------------------------------------------------

FindCV:
		bcf	NOCV
		bcf	RDONLY

		xorlw	0x00			; CV513
		btfsc	STATUS,Z
		retlw	E_CV513
		xorlw	(0x00 ^ 0x08)		; CV521
		btfsc	STATUS,Z
		retlw	E_CV521
		xorlw	(0x08 ^ 0x1C)		; CV541
		btfsc	STATUS,Z
		retlw	E_CV541

		xorlw	(0x1C ^ 0x02)		; CV515
		btfsc	STATUS,Z
		retlw	E_CV515
		xorlw	(0x02 ^ 0x03)		; CV516
		btfsc	STATUS,Z
		retlw	E_CV516
		xorlw	(0x03 ^ 0x04)		; CV517
		btfsc	STATUS,Z
		retlw	E_CV517
		xorlw	(0x04 ^ 0x05)		; CV518
		btfsc	STATUS,Z
		retlw	E_CV518

		xorlw	(0x05 ^ 0x20)		; CV545
		btfsc	STATUS,Z
		retlw	E_CV545
		xorlw	(0x20 ^ 0x21)		; CV546
		btfsc	STATUS,Z
		retlw	E_CV546

		movwf	TEMP
		xorlw	0x21

		addlw	0xDE			; CV547...
		btfss	STATUS,C
		goto	FindCVNoTable

	ifdef	__PIC628
		addlw	0xA0			; ...CV642 -> 0x22...0x81
		btfsc	STATUS,C
		goto	FindCVNoTable
		addlw	0x60
		addlw	E_CV547
	else
		addlw	0x80			; ...CV674 -> 0x22...0xA1
		btfsc	STATUS,C
		goto	FindCVNoTable
		addlw	0x80
		addlw	E_CV547
	endif

		return

FindCVNoTable:
		movf	TEMP,w

		bsf	RDONLY
		xorlw	(0x21 ^ 0x06)		; CV519
		btfsc	STATUS,Z
		retlw	E_CV7
		xorlw	(0x06 ^ 0x07)		; CV520
		btfsc	STATUS,Z
		retlw	E_CV8

		bsf	NOCV			; CV not finded
		retlw	0x7F			; return last location

;---------------------------------------------------------------------------

ResetCV:
		movlw	HIGH (CVCopy)
		movwf	MSB
		movlw	LOW  (CVCopy)
		movwf	LSB
		movlw	E_CV513
		movwf	R0
		movlw	d'27'			; CV513...CV546, groups and aspect
		movwf	COUNT
ResetCVLoop:
		call	GetChar
		movwf	EEDATA0
		movf	R0,w
		call	SetParm
		incf	LSB,f			; next character
		btfsc	STATUS,Z
		incf	MSB,f
		incf	R0,f
		decfsz	COUNT,f
		goto	ResetCVLoop


		movlw	E_CV547
		movwf	R0
		movlw	d'32'			; CV547...CV578
		movwf	COUNT
ResetCV1Loop:
		call	GetChar
		movwf	EEDATA0
		movf	R0,w
		call	SetParm
		incf	LSB,f			; next character
		btfsc	STATUS,Z
		incf	MSB,f
		incf	R0,f
		decfsz	COUNT,f
		goto	ResetCV1Loop

		clrf	EEDATA0			; clear from aspect 9 (CV579)
		movlw	E_CV579
		movwf	R0
	ifdef	__PIC628
		movlw	d'64'			; to aspect 24 for PIC16F628
	else
		movlw	d'96'			; to aspect 32 for PIC16F648A
	endif
		movwf	COUNT
ResetCV9Loop:
		movf	R0,w
		call	SetParm
		incf	R0,f
		decfsz	COUNT,f
		goto	ResetCV9Loop
		clrf	PCLATH
		return

GetChar:
		movf	MSB,w			; goto MSB,LSB
		movwf	PCLATH
		movf	LSB,w
		movwf	PCL



; ----------------------------------------------------------------------


AccessProg:
		btfss	DATA2,7			; '10AAAAAA'1AAAxPxx'? accesory operation
		goto	ExitDecode
		clrf	PORTA
		clrf	PORTB

		movf	DATA1,w			; get 6 low bits
		andlw	0x3F
		movwf	CV513
		movwf	EEDATA0
		movlw	E_CV513
		call	SetParm

		swapf	DATA2,w			; get 3 high bits
		andlw	0x07
		movwf	CV521
		swapf	CV521,f
		xorlw	0x07			; complement bits
		movwf	EEDATA0
		movlw	E_CV521
		call	SetParm

		bcf	KEY_PROG
		goto	Accessory


;---------------------------------------------------------------------------

EE_Read:
		bsf	STATUS,RP0		; w=ADR
		movwf	EEADR
		bsf	EECON1,RD
		movf	EEDATA,w
		bcf	STATUS,RP0
		return

SetParm:
		call	EE_Read			; w=ADR, EEDATA0=data. Write only changes
		xorwf	EEDATA0,w
		btfsc	STATUS,Z
		return
EE_Write:		
		movf	EEDATA0,w
		bsf	STATUS,RP0
		movwf	EEDATA
		bsf	EECON1,WREN
		bcf	INTCON,GIE
		movlw	0x55
		movwf	EECON2
		movlw	0xAA
		movwf	EECON2
		bsf	EECON1,WR
		bsf	INTCON,GIE
		bcf	EECON1,WREN
EEWrite0:
		btfsc	EECON1,WR
		goto	EEWrite0
		bcf	STATUS,RP0
		return


; ----- Default CV values

CVCopy:
		retlw	0x01			; CV513..CV546
		retlw	0x0F
		retlw	0x0F
		retlw	0x0F
		retlw	0x0F
		retlw	0x0A
		retlw	0x0D
		retlw	0x00
		retlw	0x80
		retlw	0x1E
		retlw	0x0B

		retlw	0x03			; default groups
		retlw	0x03
		retlw	0x0C
		retlw	0x0C
		retlw	0x30
		retlw	0x30
		retlw	0xC0
		retlw	0xC0

		retlw	0x01			; default output aspect
		retlw	0x00
		retlw	0x01
		retlw	0x00
		retlw	0x01
		retlw	0x00
		retlw	0x01
		retlw	0x00

		retlw	0x03			; aspect 1
		retlw	0x01
		retlw	0x00
		retlw	0x00
		retlw	0x03			; aspect 2
		retlw	0x02
		retlw	0x00
		retlw	0x00
		retlw	0x0C			; aspect 3
		retlw	0x04
		retlw	0x00
		retlw	0x00
		retlw	0x0C			; aspect 4
		retlw	0x08
		retlw	0x00
		retlw	0x00
		retlw	0x30			; aspect 5
		retlw	0x10
		retlw	0x00
		retlw	0x00
		retlw	0x30			; aspect 6
		retlw	0x20
		retlw	0x00
		retlw	0x00
		retlw	0xC0			; aspect 7
		retlw	0x40
		retlw	0x00
		retlw	0x00
		retlw	0xC0			; aspect 8
		retlw	0x80
		retlw	0x00
		retlw	0x00



; ----- EEPROM default values


		org	0x2100

		dw	0x01			; CV513	Adress low
		dw	0x0F			; CV515 Max. bright
		dw	0x0F			; CV516
		dw	0x0F			; CV517
		dw	0x0F			; CV518
		dw	0x0A			; CV519 Manufacturer Version
		dw	0x0D			; CV520	Manufacturer ID
		dw	0x00			; CV521 Address high
		dw	0x80			; CV541 Config
		dw	0x1E			; CV545 Slope
		dw	0x0B			; CV546 Flash rate

		dw	0x03			; default groups
		dw	0x03
		dw	0x0C
		dw	0x0C
		dw	0x30
		dw	0x30
		dw	0xC0
		dw	0xC0

		dw	0x01			; default output aspect
		dw	0x00
		dw	0x01
		dw	0x00
		dw	0x01
		dw	0x00
		dw	0x01
		dw	0x00

		org	0x2120


		dw	0x03			; CV547	Enable aspect 1		BARV
		dw	0x01			; CV548 Light aspect 1
		dw	0x00			; CV549 Flash aspect 1
		dw	0x00			; CV550	Flash_AB aspect 1

		dw	0x03			; CV551	Enable aspect 2
		dw	0x02			; CV552 Light aspect 2
		dw	0x00			; CV553 Flash aspect 2
		dw	0x00			; CV554	Flash_AB aspect 2

		dw	0x0C			; CV555	Enable aspect 3
		dw	0x04			; CV556 Light aspect 3
		dw	0x00			; CV557 Flash aspect 3
		dw	0x00			; CV558	Flash_AB aspect 3

		dw	0x0C			; CV559	Enable aspect 4
		dw	0x08			; CV560 Light aspect 4
		dw	0x00			; CV561 Flash aspect 4
		dw	0x00			; CV562	Flash_AB aspect 4

		dw	0x30			; CV563	Enable aspect 5
		dw	0x10			; CV564 Light aspect 5
		dw	0x00			; CV565 Flash aspect 5
		dw	0x00			; CV566	Flash_AB aspect 5

		dw	0x30			; CV567	Enable aspect 6
		dw	0x20			; CV568 Light aspect 6
		dw	0x00			; CV569 Flash aspect 6
		dw	0x00			; CV570	Flash_AB aspect 6

		dw	0xC0			; CV571	Enable aspect 7
		dw	0x40			; CV572 Light aspect 7
		dw	0x00			; CV573 Flash aspect 7
		dw	0x00			; CV574	Flash_AB aspect 7

		dw	0xC0			; CV575	Enable aspect 8
		dw	0x80			; CV576 Light aspect 8
		dw	0x00			; CV577 Flash aspect 8
		dw	0x00			; CV578	Flash_AB aspect 8



	ifdef	__PIC628

		fill	0x00,d'64'		; CV579..CV674	Aspect 9 to 24

	else

		fill	0x00,d'96'		; CV579..CV674	Aspect 9 to 32


		org	0x21C8

		dt	"UniSemaf"
		dt	"-648 v."
		dt	__VERNUM  +0x30
		dt	"F.Cañada"
		dt	(__VERDAY   >> 4)  +0x30
		dt	(__VERDAY   & 0x0F)+0x30,"/"
		dt	(__VERMONTH >> 4)  +0x30
		dt	(__VERMONTH & 0x0F)+0x30,"/"
		dt	(__VERYEAR  >> 4)  +0x30
		dt	(__VERYEAR  & 0x0F)+0x30


	endif



	end

