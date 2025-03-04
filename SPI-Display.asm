;-------------------------------------------------------------------------------
; MSP430 Assembler Code Template for use with TI Code Composer Studio
;
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file

;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.
;-------------------------------------------------------------------------------
            .text                           ; Assemble into program memory.
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.

;-------------------------------------------------------------------------------
RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer

.text                           ; program start
            .global _main		    ; define entry point

_main:

			NOP
			mov.w   #WDTPW+WDTHOLD,&WDTCTL  ; stop watchdog timer
				;Set the I/O port that controls the RED LED to an output.
				;  To do this set bit 0 in the P1DIR register.  Leave all
				;  all other I/O lines as inputs
			;mov.b #0x01, P1DIR
			;mov.b #0x00, P1OUT
			call  #SetIOPorts
			call  #InitilizeDisplaySPI
			call  #InitilizeDisplay

;looptemp:
			;mov.b	#0x1F,R10
			;call	#SendSPI
			;jmp	looptemp

			call  #Displaynumbers

			;bic   #0X4100,&UCSCTL6
			call	#HalfSecondDelay
			;bic.b   #0X70,&UCSCTL1
			;MOV	  #0X11FF,&UCSCTL2
			;call	#HalfSecondDelay


Mainloop:
			bis.b #0x01, P1OUT
			;call #Displaynumbers
			jmp     Mainloop; jump to the Mainloop label
			nop

InitilizeDisplay:
			bic.b	#01000000b,P1OUT		;reset the display
			call	#HalfSecondDelay		;only need 20us but I have a half second delay.
			bis.b	#01000000b,P1OUT

			mov.a	#DisplayInitilizationData,R4

InitDisplayMore:
			bic.b	#00001000b,P2OUT
			mov		#0x1f,R10
			call	#SendSPI
			mov.b	@R4+,R5
			cmp.b	#0xff,R5
			jz		InitDisplayDone
			mov.b	R5,R10
			and.b	#0x0f,R10
			call	#SendSPI
			rra.b   R5
			rra.b   R5
			rra.b   R5
			rra.b   R5
			mov.b	R5,R10
			and.b	#0x0f,R10
			call	#SendSPI
			bis.b	#00001000b,P2OUT
			jmp		InitDisplayMore
InitDisplayDone:
			bis.b	#00001000b,P2OUT
			ret
DisplayInitilizationData:
			.byte 0x3A,0x01f,0x09,0x06,0x1E,0x39,0x1B,0x6E,0x56,0x7A,0x38,0x01,0x0F,0x39,0x56,0x70,0x38,0xff

Displaynumbers:

			mov.a	#DisplayNumberData,R4
			bic.b	#00001000b,P2OUT
DisplaynumbersMore:
			mov.b	@R4+,R10
			cmp.b	#0xff,R10
			jz		DisplaynumbersDone
			call	#SendSPI
			jmp		DisplaynumbersMore
DisplaynumbersDone:
			bis.b	#00001000b,P2OUT
			ret
DisplayNumberData:

			.byte 0x5f,0x01,0x03,0x02,0x03,0xff			; send number 12 to the LCD display
														; display name on the LCD display
			;.byte 0x5F,0x06,0x05,0x09,0x06,0x0E,0x06,0x03,0x06,0x05,0x06,0x0E,0x06,0x04,0x07,0xFF

HalfSecondDelay:
			mov		#0x1,R14
OuterTimercountLoop:
			mov		#0xffff,R15
TimercountLoop:
			nop
			sub		#1,R15
			jnz		TimercountLoop
			sub		#1,R14
			jnz		OuterTimercountLoop
			ret
SetIOPorts:
	;	IO ports need to be defined as Inputs and outputs based on the development board
	;
	; 	PORT 1		bit 	direction 		function
	;				0		output			LED on Developmet board
	;				1		input (pullup)	Switch 2 on development board
	;				2		input			right distance sensor
	;				3		input			left distance sensor
	;				4		output			left pwm
	;				5		output			right pwm
	;				6		output 			Display reset
	;				7		input			EzEET_uart
		mov.b	#00000011b,P1OUT		;set up all default output values and pullups
		mov.b	#00000010b,P1REN		;enable pull up/dn resistors
		mov.b	#01110001b,P1DIR		;set all pin directions per definition

	; 	PORT 2		bit 	direction 		function
	;				0		output			Distance Right Trigger
	;				1		input (pullup)	Switch 1 on development board
	;				2		output			Direction of right motor
	;				3		output			Chip Select for display
	;				4		input			front distance sensor
	;				5		output			front distance sensor trigger
	;				6		output			left distance sensor trigger
	;				7		output			direction of left motor
		mov.b	#00001010b,P2OUT		;set up all default output values and pullups
		mov.b	#00000010b,P2REN		;enable pull up/dn resistors
		mov.b	#11101101b,P2DIR		;set all pin directions per definition

	; 	PORT 3		bit 	direction 		function
	;				0		output			MOSI (also LED 1)
	;				1		input 			MISO (also LED 2)
	;				2		output			SCLK (also LED 3)
	;				3		output			UART Tx (also LED 4)
	;				4		input			UART Rx (also LED 5)
	;				5		output			LED 6
	;				6		output			LED 7
	;				7		output			LED 8
		mov.b	#00000110b,P3OUT		;set up all default output values and pullups
		mov.b	#00000000b,P3REN		;enable pull up/dn resistors
		mov.b	#11101101b,P3DIR		;set all pin directions per definition
		mov.b	#00000111b,P3SEL
	; 	PORT 4		bit 	direction 		function
	; 	PORT 5		bit 	direction 		function
	; 	PORT 6		bit 	direction 		function
	; 	PORT 7		bit 	direction 		function
		ret

InitilizeDisplaySPI:
		bis.b #UCSWRST, &UCB0CTL1 		; set WRST bit to enable modifying timer registers
		bis.b #11000000b, &UCB0CTL1		; set CTL1 register to set clock and data settings
		mov.b #01001001b, &UCB0CTL0		; may need to change b value 10001001
		bis.b #00001010b, &UCB0BR0		; set baud rate to divide by 10 for 100 khz bit clock
		bic.b #UCSWRST, &UCB0CTL1 		; clear RST bit to save timer register changes
		ret

SendSPI:
		mov.b R10, &UCB0TXBUF

IfBusy:
		bit.b #00000001b, &UCB0STAT		; check if busy flag is set
		jnz IfBusy						; continue checking if busy flag is indeed set
		mov.b &UCB0RXBUF, R13			; if busy bit not set, read recieve buffer
		ret

;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack

;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".reset"                ; MSP430 RESET Vector
            .short  RESET

