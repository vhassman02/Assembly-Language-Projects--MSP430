;-------------------------------------------------------------------------------
; 									FINAL CODE v2.0
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
; SendSpace macro: Defines the code to send a space to the display via SPI
SendSpace		.macro
				mov.b #0x20,R6
				call #SendASCII
				.endm
TwoSecondDelay	.macro
				call #HalfSecondDelay
				call #HalfSecondDelay
				call #HalfSecondDelay
				call #HalfSecondDelay
				.endm
TenMicroSecondDelay .macro
				nop
				nop
				nop
				nop
				nop
				nop
				nop
				nop
				nop
				nop
				.endm
;-------------------------------------------------------------------------------

RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer

			.text ; program start
			.global _main ; define entry point
;-------------------------------------------------------------------------------
; Define RAM space
			.bss AccelerationX, 2 		;ADC12MEM2
			.bss AccelerationY, 2 		;ADC12MEM1
			.bss AccelerationZ, 2 		;ADC12MEM0
			.bss RAM1, 2 				;spare 2-byte RAM space
			.bss RAM2, 2				;spare 2-byte RAM space
			.bss RAM3, 2				;spare 2-byte RAM space
			.bss RAMX1, 2				;start of Xacc-values from adc
			.bss RAMX2, 2
			.bss RAMX3, 2
			.bss RAMX4, 2
			.bss RAMX5, 2
			.bss RAMX6, 2
			.bss RAMX7, 2
			.bss RAMX8, 2
			.bss RAMXavg, 4
;-------------------------------------------------------------------------------
_main:
	call #SetIOPorts
	call #ADCSetup
	call #InitilizeDisplaySPI
	call #InitilizeDisplay
	call #InitializeDistanceSensors
	call #TimerBPWMSetup
	call #StandbyLoop
	call #CheckTilt

Mainloop:
	call #GetDistances
	;call #PWMControl ; Must be before DisplayDistance
	;call #DisplayDistance
	;call #HalfSecondDelay
	;call #QuarterSecondDelay
	nop
	jmp Mainloop
	nop

;------------------------------------------------------------------------------------
;									STANDBY SUBROUTINE
;------------------------------------------------------------------------------------
StandbyLoop:
	bit.b #BIT1, &P2IN
	jnz StandbyLoop
	ret

;------------------------------------------------------------------------------------
;									MOVEMENT SUBROUTINES
;------------------------------------------------------------------------------------
CheckTilt:
	call #GetADC
	bit.w #0x8000, R14					; check x-orientation
	jz TurnRight
	jnz TurnLeft
	ret

TurnLeft:
	mov.w #0, &TB0CCR5
	mov.w #0xFF, &TB0CCR6
	TwoSecondDelay
	mov.w #0x0, &TB0CCR6

	ret

TurnRight:
	mov.w #0, &TB0CCR6
	mov.w #0xFF, &TB0CCR5
	TwoSecondDelay
	mov.w #0, &TB0CCR5

	ret

CheckTilt2:
	call #GetADC
	bit.w #0x8000, R14				; check x-orientation
	jz TurnRight2
	jnz TurnLeft2
	ret

TurnLeft2:

	mov.w #0, &TB0CCR5
	mov.w #0xFF, &TB0CCR6
	call #HalfSecondDelay

	ret

TurnRight2:
	mov.w #0, &TB0CCR6
	mov.w #0xFF, &TB0CCR5
	call #HalfSecondDelay

	ret

GetADC:
	mov.w ADC12MEM2, RAMX1     			;Move ADC values into RAM variables
	mov.w ADC12MEM2, RAMX2
	mov.w ADC12MEM2, RAMX3
	mov.w ADC12MEM2, RAMX4
	mov.w ADC12MEM2, RAMX5
	mov.w ADC12MEM2, RAMX6
	mov.w ADC12MEM2, RAMX7
	mov.w ADC12MEM2, RAMX8

	add RAMX1, RAMXavg        			;Add all of the values together
	add RAMX2, RAMXavg
	add RAMX3, RAMXavg
	add RAMX4, RAMXavg
	add RAMX5, RAMXavg
	add RAMX6, RAMXavg
	add RAMX7, RAMXavg
	add RAMX8, RAMXavg

	rra RAMXavg		    				;Divide by 8 to average
	rra RAMXavg
	rra RAMXavg
	mov RAMXavg, R14 	    			;Move value into R14 so that we can see it
	ret

Reverse:
	bis.b #10000000b, P2OUT					; change bits 2.2 to 0 , 2.7 to 1 so car will reverse
	bic.b #00000100b, P2OUT
	mov.b #0x66, &TB0CCR5					; set speed to half
	mov.b #0x66, &TB0CCR6					; set speed to half
	TwoSecondDelay							; reverse for two seconds
	mov.b #0x0, &TB0CCR5					; set speed to off
	mov.b #0x0, &TB0CCR6					; set speed to off
	bic.b #10000000b, P2OUT					; set direction bits back to forward after obstacle is avoided
	bis.b #00000100b, P2OUT
	call #CheckTilt2							; turn away from obstacle by checking tilt

	ret

	;move setting bits back, need to have a loop in main that checks if we need to call reverse

;------------------------------------------------------------------------------------
;									SETUP SUBROUTINES
;------------------------------------------------------------------------------------
SetIOPorts:
	;IO ports need to be defined as inputs and outputs based on the development board
	;PORT 1
	;        bit  direction 	 function
	; 		 0	  output		 LED on Development board
	; 		 1 	  input 		 (pullup)  Switch 2 on development board
	;		 2 	  input  	 	 right distance sensor
	; 		 3 	  input 		 left distance sensor
	;		 4 	  output 	 	 left pwm
	; 		 5 	  output		 right pwm
	;		 6	  output		 Display reset
	;		 7	  input		 	 EzEET_uart
	mov.b #00000011b,P1OUT ;set up all default output values and pullups
	mov.b #00000010b,P1REN ;enable pull up/dn resistors
	mov.b #01110001b,P1DIR ;set all pin directions per definition
	mov.b #10000100b,P1SEL
	;PORT 2
	;		 bit  direction 	 function
	;		 0	  output		 Distance Right Trigger
	;		 1	  input 		 (pullup)  Switch 1 on development board
	;		 2	  output		 Direction of right motor
	;		 3 	  output		 Chip Select for display
	;		 4 	  input		 	 front distance sensor
	;		 5	  output		 front distance sensor trigger
	;		 6	  output		 left distance sensor trigger
	;		 7	  output		 direction of left motor
	mov.b #00001110b,P2OUT ;set up all default output values and pullups
	mov.b #00000010b,P2REN ;enable pull up/dn resistors
	mov.b #11101101b,P2DIR ;set all pin directions per definition ;add reason for deleted section
	;PORT 3
	;		 bit  direction	     function
	;		 0 	  output		 MOSI (also LED 1)
	;		 1 	  input 		 MISO (also LED 2)
	;		 2 	  output		 SCLK (also LED 3)
	;		 3 	  output		 UART Tx (also LED 4)
	;		 4	  input		 	 UART Rx (also LED 5)
	;		 5	  output		 LED 6
	;		 6	  output 	 	 LED 7
	;		 7	  output		 LED 8
	mov.b #00000110b,P3OUT ;set up all default output values and pullups
	mov.b #00000000b,P3REN ;enable pull up/dn resistors
	mov.b #11101101b,P3DIR ;set all pin directions per definition
	mov.b #01100111b,P3SEL ;select SPI for bits 0-2, enable bit 5-6 for timer control of LEDs

	bis.b #1100001b, &P2DIR
	;setup input of P1.2 & P1.3 (dis R and dis L)
	bis.b #1100b, &P1SEL
	bic.b #1100b, &P1DIR
	bis.b #1100b, &P1REN
	bis.b #1100b, &P1OUT
	;setup input of P2.4 (dis F)
	bis.b #10000b, &P2SEL
	bic.b #10000b, &P2DIR
	bis.b #10000b, &P2REN
	bis.b #10000b, &P2OUT
	ret

ADCSetup:
	mov.w #0000111110010000b, &ADC12CTL0	; setup control register 0
	mov.w #0000001011100110b, &ADC12CTL1	; setup control register 1
	mov.w #0000000100101100b, &ADC12CTL2	; setup control register 2
	mov.b #00000000b, &ADC12MCTL0			; setup memory control register 0
	mov.b #00000001b, &ADC12MCTL1			; setup memory control register 1
	mov.b #00000010b, &ADC12MCTL2			; setup memory control register 2
	mov.b #10001010b, &ADC12MCTL3			; setup memory control register 3
	bis.b #BIT1, &ADC12CTL0
	bis.b #BIT0, &ADC12CTL0
	ret

; setup timer B for PWM control
TimerBPWMSetup:
	bis.w #0000001011000000b , &TB0CTL
	bis.w #0000000000000111b, &TB0EX0
	bis.w #0x0004, &TB0CTL
	bis.w #0000101011100000b, &TB0CCTL5	; using reset/set mode
	bis.w #0000101011100000b, &TB0CCTL6	; using reset/set mode
	;bis.w #0000101001000000b, &TB0CCTL5 	; using set/reset mode
	;bis.w #0000101001000000b, &TB0CCTL6		; using set/reset mode
	mov.w #0x0032, &TB0CCR0
	bis.w #0000001000010000b, &TB0CTL
	ret

;------------------------------------------------------------------------------------
;								INITIALIZATION SUBROUTINES
;------------------------------------------------------------------------------------
InitializeDistanceSensors:
;initialize control registers
	mov.w #1000100000b, &TA0CTL ;distance R and L ports
	mov.w #1000100000b, &TA2CTL ;distance F port
	;bits 9-8: clock select: 	 10 = smclock (1us)
	;bits 7-6: divider: 	 	 11 = divider of 8 (increments every 8 clock cycles)
	;bits 5-4: mode control: 	 10 = continuous mode (counts up to FFFF)
	;bit 3:    reserved:		 DC
	;bit 2:    clear: 		 	 0 = not cleared
	;bit 1:    interrupt enable: 0 = interrupt disabled
	;bit 0:    interrupt flag: 	 0 = no interrupt pending

;initialize capture registers
	mov.w #1100100100010000b, &TA0CCTL1 ;R distance port
	mov.w #1100100100010000b, &TA0CCTL2 ;L distance port
	mov.w #1100100100010000b, &TA2CCTL1 ;F distance port
	;bits 15-14: capture mode: 				 11 = capture on rising and falling edge
	;bits 13-12: capture input: 			 10 = CCISxA (Px.x)
	;bit 11: 	 synchronous capture source: 1 = synchronous capture
	;bit 10: 	 synchronized capture input: DC (bit is an output)
	;bit 9: 	 reserved: 			   		 DC
	;bit 8: 	 capture/compare mode: 		 1 = capture mode
	;bit 7-5: 	 output mode: 		  		 DC (000 = output updates with out bit)
	;bit 4: 	 interrupt enable: 	   		 1 = interrupt enabled
	;bit 3: 	 capture input: 	   		 DC (bit is an output)
	;bit 2: 	 output: 			   		 DC (0 = output starts low)
	;bit 1: 	 capture overflow: 	   		 0 = no capture overflow occured
	;bit 0: 	 interrupt flag: 	   		 0 = no interrupt pending
	ret

InitilizeDisplaySPI:
												;go through all registers and set 1 by 1 every bit
	bis.b #UCSWRST,&UCB0CTL1					;allow for control registers to be modified
	mov.b #11000001b,&UCB0CTL1 					;set the control register with the clock source (SMCLK is used)
	mov.b #01001001b,&UCB0CTL0 					;set the control register with clcok phase and polarity both set to 1,
												;LSB first, 8-bit data, master mode, 3 pin SPI, and synchronous mode
	mov.b #00001010b,&UCB0BR0  					;set baud rate with BRCLK = 1 MHz, Bit Clock = 100 kHz
	bic.b #UCSWRST,&UCB0CTL1					;clear the bit allowing control registers to be modified
	ret

SendSPI:
	bit.b #1b,&UCB0STAT 						;check if busy
	jnz SendSPI									;if busy jump back to send loop
	mov.b R6, &UCB0TXBUF 						;move data to transmit buffer

SendSPIMore:
	bit.b #1b,&UCB0STAT 						;check if busy
	jnz SendSPIMore								;loop if busy
	ret

InitilizeDisplay:
	bic.b #01000000b,P1OUT 						;reset the display
	call #HalfSecondDelay 						;only need 20us but I have a half second delay.
	bis.b #01000000b,P1OUT						;turn on display
	mov.a #DisplayInitilizationData,R4 			;load initialization data into R4

InitDisplayMore:
	bic.b #00001000b,P2OUT 						;chip select low
	mov.b #0x1f,R6								;0001 1111 -> R10 (start signal 0->7)
	call #SendSPI 								;send data
	mov.b @R4+,R5 								;move to next initialization data
	cmp.b #0xff,R5 								;set initialization data to R5
	jz	 InitDisplayDone						;have we incremented all initialization values?
	mov.b R5,R6									;move data to R10
	and.b #0x0f,R6								;take lower byte of data
	call #SendSPI								;send data with it
	rra.b R5									;rotate to higher bye
	rra.b R5
	rra.b R5
	rra.b R5
	mov.b R5,R6
	and.b #0x0f,R6								;take higher byte of data
	call #SendSPI								;send data with it
	bis.b #00001000b,P2OUT						;chip select high
	jmp InitDisplayMore							;reset

InitDisplayDone:
	bis.b #00001000b,P2OUT						;chip select high
	ret

DisplayInitilizationData:
	.byte 0x3A,0x01f,0x09,0x06,0x1E,0x39,0x1B,0x6E,0x56,0x7A,0x38,0x01,0x0F,0x39,0x56,0x70,0x38,0xff

;------------------------------------------------------------------------------------
;							PWM  AND DUTY CYCLE SUBROUTINES
;------------------------------------------------------------------------------------
PWMControl:





	ret

ConvertDutyR:
	;convert uptime in R12 to address of ASCII distance in R4
		;R10(input): uptime of echo pulse
		;R4(output): address of ASCII distance
	mov.a #DisplayDutyNumbers,R4				;load lookup table into R4
	mov.w R10,MPY								; use the multiplier to adjust
	mov.w #5,OP2
	mov.w RES0,R10
	add.w R10,R4								;set index based on R12
	ret

ConvertDutyL:
	;convert uptime in R12 to address of ASCII distance in R4
		;R10(input): uptime of echo pulse
		;R4(output): address of ASCII distance
	mov.a #DisplayDutyNumbers,R4				;load lookup table into R4
	mov.w R11,MPY
	mov.w #5,OP2
	mov.w RES0,R11
	add.w R11,R4								;set index based on R12
	ret

;------------------------------------------------------------------------------------
;								DISTANCE SENSOR SUBROUTINES
;------------------------------------------------------------------------------------
GetDistances:

Floop:
	mov.w #0xFFFF, R15
	call #FrontTrigger
	mov.w #0, &TA2R 		; reset clock
F_rising:
	bit.b #0x2, &TA2IV 		; check and clear capture flag
	jz F_rising 			; loop if no capture
	mov.w &TA2CCR1, R9 		; move rising edge time into R9
F_falling:
	cmp R15, &TA2R  		; re-run function if no return pulse
	jn Floop
	bit.b #0x2, &TA2IV 		; check and clear capture flag
	jz F_falling 			; loop if no capture
	mov.w &TA2CCR1, R12 	; move falling edge time into R12
	sub.w R9, R12 			; move falling edge time minus rising edge time (total uptime) into R12

	cmp #0x83F, R12				; compare front sensor capture value to a minimum number
	jl Reverse						; if sensor value is less than 0x500F, reverse


Rloop:
	mov.w #0xFFFF, R15
	call #RightTrigger
	mov.w #0, &TA0R 		; reset clock
R_rising:
	bit.b #0x2, &TA0IV 		; check and clear capture flag
	jz R_rising 			; loop if no capture
	mov.w &TA0CCR1, R9 		; move rising edge time into R9
R_falling:
	cmp R15, &TA0R 			; re-run function if no return pulse
	jn Rloop
	bit.b #0x2, &TA0IV 		; check and clear capture flag
	jz R_falling 			; loop if no capture
	mov.w &TA0CCR1, R10 	; move falling edge time into R10
	sub.w R9, R10 			; move falling edge time minus rising edge time (total uptime) into R10

	mov.a #DutyCycleTable, R7
	rra R10
	rra R10
	rra.w R10
	rra.w R10
	rra.w R10
	rra.w R10
	;rra.w R10
	rla.w R10
	rla.w R10
	add.a R10, R7
	mov.b @R7, &TB0CCR5


Lloop:
	mov.w #0xFFFF, R15
	call #LeftTrigger
	mov.w #0, &TA0R 		; reset clock
L_rising:
	bit.b #0x4, &TA0IV 		; check and clear capture flag
	jz L_rising 			; loop if no capture
	mov.w &TA0CCR2, R9 		; move rising edge time into R11
L_falling:
	cmp R15, &TA0R 			; re-run function if no return pulse
	jn Lloop
	bit.b #0x4, &TA0IV 		; check and clear capture flag
	jz L_falling 			; loop if no capture
	mov.w &TA0CCR2, R11 	; move falling edge time into R10
	sub.w R9, R11 			; move falling edge time minus rising edge time (total uptime) into R11

	mov.a #DutyCycleTable, R7
	rra R11
	rra R11
	rra.w R11
	rra.w R11
	rra.w R11
	rra.w R11
	;rra.w R11
	rla.w R11
	rla.w R11
	add.a R11, R7
	mov.b @R7, &TB0CCR6

	ret

RightTrigger:
	bis.b #00000001b, &P2OUT ;P2.0 Trigger R before delay
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	bic.b #00000001b, &P2OUT ; P2.0 Trigger R after delay
	ret

LeftTrigger:
	bis.b #01000000b, &P2OUT ; P2.6 Trigger L before delay
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	bic.b #01000000b, &P2OUT ; P2.6 Trigger L after delay
	ret

FrontTrigger:
	bis.b #00100000b, &P2OUT ; P2.5 Trigger F before delay
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	bic.b #00100000b, &P2OUT ; P2.5 Trigger F before delay
	ret

ConvertDistance:
	;convert uptime in R12 to address of ASCII distance in R4
		;R12(input): uptime of echo pulse
		;R4(output): address of ASCII distance
	rra.w R12									; divide uptime by 64 for index of table
	rra.w R12
	rra.w R12
	rra.w R12
	rra.w R12
	rra.w R12
	rla.w R12									; multiply by 4 for index of .byte (4 bytes per data)
	rla.w R12
	mov.a #ConvertDistanceData,R4				; load lookup table into R4
	add.w R12,R4								; set index based on R12
	ret

;------------------------------------------------------------------------------------
;									ADC SUBROUTINES
;------------------------------------------------------------------------------------
StoreAccelerationData:
	;bit.b #1b,ADC12CTL1
	;jnz Accelerometer
	bis.b #00000001b,ADC12CTL0
	mov.w &ADC12MEM0, AccelerationZ
	mov.w &ADC12MEM1, AccelerationY
	mov.w &ADC12MEM2, AccelerationX
	ret
	; When MSB of ADC12MEM2 (AccelerationX) = 1, car tilts left looking back->front
	; When MSB of ADC12MEM2 = 0, car is flat or tilts right looking back->front
	; bit.w 0x100, ADC12MEM2 will be used to determine initial tilt of car on ramp
	; jz or jnz accordingly based on bit result

;------------------------------------------------------------------------------------
;									DISPLAY SUBROUTINES
;------------------------------------------------------------------------------------
DisplayDistance:
	;sends echo pulse uptime (R12) to display in the form "0.00m"
		;R12(input): echo pulse uptime (distance)
		;R4(dummy): distance address
		;R6(dummy): SPI sending
		;P2OUT(perm): SPI chip select
	call #ClearDisplay							; clear display
	bic.b #00001000b,P2OUT						; set CS low
	mov.b #0x5f,R6								; send initialization
	call #SendSPI
	call #ConvertDistance						; load distance address into R4
	mov.b @R4, R6								; send meter value to display
	call #SendASCII
	add.a #1, R4
	mov.b @R4, R6								; send decimal point
	call #SendASCII
	add.a #1, R4
	mov.b @R4, R6								;send decimeter value
	call #SendASCII
	add.a #1, R4
	mov.b @R4, R6								;send centimeter value
	call #SendASCII
	mov.b #0x6D,R6								; send m
	call #SendASCII
	mov.b #0x65,R6								; send e
	call #SendASCII
	mov.b #0x74,R6								; send t
	call #SendASCII
	mov.b #0x65,R6								; send e
	call #SendASCII
	mov.b #0x72,R6								; send r
	call #SendASCII
	mov.b #0x73,R6								; send s
	call #SendASCII
	SendSpace									; send 10 spaces for new line
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	;SendNewLine2
	call #ConvertDutyL							; apply duty cycle % lookup table to send PWM signal for L side
	mov.b @R4, R6
	call #SendASCII								; send percentage digits and % symbol to display
	add.a #1, R4
	mov.b @R4, R6
	call #SendASCII
	add.a #1, R4
	mov.b @R4, R6
	call #SendASCII
	add.a #1, R4
	mov.b @R4, R6
	call #SendASCII
	SendSpace									; send 2 spaces
	SendSpace
	call #ConvertDutyR							; apply duty cycle % lookup table to send PWM signal for L side
	mov.b @R4, R6
	call #SendASCII								; send percentage digits and % symbol to display
	add.a #1, R4
	mov.b @R4, R6
	call #SendASCII
	add.a #1, R4
	mov.b @R4, R6
	call #SendASCII
	add.a #1, R4
	mov.b @R4, R6
	call #SendASCII
	SendSpace									; send 10 spaces for new line
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	SendSpace
	mov.b #0x41, R6
	call #SendASCII								; send A
	mov.b #0x63, R6
	call #SendASCII								; send c
	mov.b #0x63, R6
	call #SendASCII								; send c
	mov.b #0x58, R6
	call #SendASCII
	mov.b #0x3A, R6
	call #SendASCII								; send :
	; IDEA 1: display a number to represent AccelerationX (ADCMEM2):
	; AND ADC12MEM2 w/1111000000000000b to remove all but upper 4 bits
		;mov.w ADC12MEM2, RAM1
		;and.w #0xF000, RAM1 ; to remove all but upper 4 bits
	; there will be a maximum of 15 values in a lookup table
	; display the value using mov.b @R4, R6 + #SendASCII
		;mov.a #ADCLookup, R4						; store ADC lookup in R4
		;add.w RAM1,R4								;
		;mov.w @R4, R6								;
		;call #SendASCII								;

	; IDEA 2: Display <- or -> on the display based on bit of highest bit in ADC12MEM2
	; bit test on MSB of ADC12MEM2
	; display <- when MSB = 1
	; display -> when MSB = 0
	bit.w #0x8000, ADC12MEM2					; check x-orientation
	jz DisplayTiltL
	jnz DisplayTiltR

	bit.w #0x8000, ADC12MEM1					; check y-orientation
	jz DisplayTiltF
	jnz DisplayTiltB

	mov.b #0xff,R6								; send end of transmission
	call #SendSPI
	bis.b #00001000b,P2OUT						; set chip select high
	ret

DisplayTiltL:
	mov.b #0x7E, R6
	call #SendASCII
	SendSpace
	ret

DisplayTiltR:
	mov.b #0x7F, R6
	call #SendASCII
	SendSpace
	ret

DisplayTiltF:
	mov.b #0x46, R6
	call #SendASCII
	SendSpace
	ret

DisplayTiltB:
	mov.b #0x42, R6
	call #SendASCII
	SendSpace
	ret

ClearDisplay:
	;clears the display
		;R6(dummy): SPI sending
		;P2OUT(perm): SPI chip select
	bic.b #00001000b,P2OUT 						;chip select low
	mov.b #0x1f,R6								;send initialization
	call #SendSPI
	mov.b #0x0A, R6								;send ()
	call #SendSPI
	mov.b #0x03, R6
	call #SendSPI
	bis.b #00001000b,P2OUT						;chip select high

	bic.b #00001000b,P2OUT 						;chip select low
	mov.b #0x1f,R6								;send initialization
	call #SendSPI
	mov.b #0x0F, R6								;send ()
	call #SendSPI
	mov.b #0x01, R6
	call #SendSPI
	bis.b #00001000b,P2OUT						;chip select high

	bic.b #00001000b,P2OUT 						;chip select low
	mov.b #0x1f,R6								;send initialization
	call #SendSPI
	mov.b #0x01, R6								;send clear display signal
	call #SendSPI
	mov.b #0x00, R6
	call #SendSPI
	bis.b #00001000b,P2OUT						;chip select high
	ret

SendASCII:
	;sends R6 (byte) as an ASCII character to the display
		;R6(input): ASCII character
		;R8(dummy): R6 storage
	mov.b R6, R8								;store input
	and.b #0xf, R6								;send lower nibble to display
	call #SendSPI
	mov.b R8, R6								;recover input
	rra.b R6									;send upper nibble to display
	rra.b R6
	rra.b R6
	rra.b R6
	and.b #0xf, R6
	call #SendSPI
	ret

;------------------------------------------------------------------------------------
;							DELAY SUBROUTINES AND LOOKUP TABLES
;------------------------------------------------------------------------------------
ConvertDistanceData:
	;table of distances in ASCII corresponding to uptime of echo pulse /64
	.byte "0.00","0.01","0.02","0.03","0.04","0.05","0.06","0.07","0.08","0.09","0.10","0.11","0.12","0.13","0.15","0.16","0.17","0.18","0.19","0.20","0.21","0.22","0.23","0.24","0.25","0.26","0.27","0.28","0.29","0.30","0.31","0.32","0.33","0.34","0.35","0.36","0.37","0.38","0.39","0.41","0.42","0.43","0.44","0.45","0.46","0.47","0.48","0.49","0.50","0.51","0.52","0.53","0.54","0.55","0.56","0.57","0.58","0.59","0.60","0.61","0.62","0.63","0.64","0.65","0.66","0.68","0.69","0.70","0.71","0.72","0.73","0.74","0.75","0.76","0.77","0.78","0.79","0.80","0.81","0.82","0.83","0.84","0.85","0.86","0.87","0.88","0.89","0.90","0.91","0.92","0.93","0.94","0.96","0.97","0.98","0.99","1.00","1.01","1.02","1.03","1.04","1.05","1.06","1.07","1.08","1.09","1.10","1.11","1.12","1.13","1.14","1.15","1.16","1.17","1.18","1.19","1.20","1.21","1.23","1.24","1.25","1.26","1.27","1.28","1.29","1.30","1.31","1.32","1.33","1.34","1.35","1.36","1.37","1.38","1.39","1.40","1.41","1.42","1.43","1.44","1.45","1.46","1.47","1.48","1.50","1.51","1.52","1.53","1.54","1.55","1.56","1.57","1.58","1.59","1.60","1.61","1.62","1.63","1.64","1.65","1.66","1.67","1.68","1.69","1.70","1.71","1.72","1.73","1.74","1.75","1.77","1.78","1.79","1.80","1.81","1.82","1.83","1.84","1.85","1.86","1.87","1.88","1.89","1.90","1.91","1.92","1.93","1.94","1.95","1.96","1.97","1.98","1.99","2.00","2.01","2.02","2.04","2.05","2.06","2.07","2.08","2.09","2.10","2.11","2.12","2.13","2.14","2.15","2.16","2.17","2.18","2.19","2.20","2.21","2.22","2.23","2.24","2.25","2.26","2.27","2.28","2.29","2.31","2.32","2.33","2.34","2.35","2.36","2.37","2.38","2.39","2.40","2.41","2.42","2.43","2.44","2.45","2.46","2.47","2.48","2.49","2.50","2.51","2.52","2.53","2.54","2.55","2.56","2.58","2.59","2.60","2.61","2.62","2.63","2.64","2.65","2.66","2.67","2.68","2.69","2.70","2.71","2.72","2.73","2.74","2.75","2.76","2.77","2.78","2.79","2.80","2.81","2.82","2.83","2.85","2.86","2.87","2.88","2.89","2.90","2.91","2.92","2.93","2.94","2.95","2.96","2.97","2.98","2.99","3.00","3.01","3.02","3.03","3.04","3.05","3.06","3.07","3.08","3.09","3.10","3.12","3.13","3.14","3.15","3.16","3.17","3.18","3.19","3.20","3.21","3.22","3.23","3.24","3.25","3.26","3.27","3.28","3.29","3.30","3.31","3.32","3.33","3.34","3.35","3.36","3.37","3.39","3.40","3.41","3.42","3.43","3.44","3.45","3.46","3.47","3.48","3.49","3.50","3.51","3.52","3.53","3.54","3.55","3.56","3.57","3.58","3.59","3.60","3.61","3.62","3.63","3.64","3.66","3.67","3.68","3.69","3.70","3.71","3.72","3.73","3.74","3.75","3.76","3.77","3.78","3.79","3.80","3.81","3.82","3.83","3.84","3.85","3.86","3.87","3.88","3.89","3.90","3.91","3.92","3.94","3.95","3.96","3.97","3.98","3.99","4.00","4.01","4.02","4.03","4.04","4.05","4.06","4.07","4.08","4.09","4.10","4.11","4.12","4.13","4.14","4.15","4.16","4.17","4.18","4.19","4.21","4.22","4.23","4.24","4.25","4.26","4.27","4.28","4.29","4.30","4.31","4.32","4.33","4.34","4.35","4.36","4.37","4.38","4.39","4.40","4.41","4.42","4.43","4.44","4.45","4.46","4.48","4.49","4.50"

QuarterSecondDelay:
	mov.b #0x5FFC, R13					; value for 1/4 second delay
QuarterSecondDelay2:
	sub.b #0x01, R13
	cmp.b #0x01, R13
	jnz QuarterSecondDelay2
	mov.w #0x5FFC, R13
	ret

HalfSecondDelay:
	;delays processsing for half a second
		;R15(dummy): loop incrementor
	mov.w #0, R15
HalfSecondDelayLoop:
		NOP ;10 NOPs (10)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (20)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (30)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (40)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (50)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (60)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (70)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (80)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (90)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP ;10 NOPs (100) (100us)
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		add.w #1b, R15						;increment R15
		cmp #0x1388, R15					;have we done 5000 loops of 100us delay (0.5s)
		jne HalfSecondDelayLoop				;repeat if not
		mov.w #0,R15						;set R15 to 0
		ret									;exit if have

DutyCycleTable:
		.byte 0
		.byte 0
		.byte 0
		.byte 0
		.byte 0
		.byte 0
		.byte 0
		.byte 0
		.byte 1
		.byte 1
		.byte 1
		.byte 1
		.byte 1
		.byte 1
		.byte 1
		.byte 1
		.byte 2
		.byte 2
		.byte 2
		.byte 2
		.byte 2
		.byte 2
		.byte 2
		.byte 3
		.byte 3
		.byte 3
		.byte 3
		.byte 3
		.byte 3
		.byte 3
		.byte 3
		.byte 4
		.byte 4
		.byte 4
		.byte 4
		.byte 4
		.byte 4
		.byte 4
		.byte 5
		.byte 5
		.byte 5
		.byte 5
		.byte 5
		.byte 5
		.byte 5
		.byte 5
		.byte 6
		.byte 6
		.byte 6
		.byte 6
		.byte 6
		.byte 6
		.byte 6
		.byte 6
		.byte 7
		.byte 7
		.byte 7
		.byte 7
		.byte 7
		.byte 7
		.byte 7
		.byte 8
		.byte 8
		.byte 8
		.byte 8
		.byte 8
		.byte 8
		.byte 8
		.byte 8
		.byte 9
		.byte 9
		.byte 9
		.byte 9
		.byte 9
		.byte 9
		.byte 9
		.byte 10
		.byte 10
		.byte 10
		.byte 10
		.byte 10
		.byte 10
		.byte 10
		.byte 10
		.byte 11
		.byte 11
		.byte 11
		.byte 11
		.byte 11
		.byte 11
		.byte 11
		.byte 12
		.byte 12
		.byte 12
		.byte 12
		.byte 12
		.byte 12
		.byte 12
		.byte 12
		.byte 13
		.byte 13
		.byte 13
		.byte 13
		.byte 13
		.byte 13
		.byte 13
		.byte 13
		.byte 14
		.byte 14
		.byte 14
		.byte 14
		.byte 14
		.byte 14
		.byte 14
		.byte 15
		.byte 15
		.byte 15
		.byte 15
		.byte 15
		.byte 15
		.byte 15
		.byte 15
		.byte 16
		.byte 16
		.byte 16
		.byte 16
		.byte 16
		.byte 16
		.byte 16
		.byte 17
		.byte 17
		.byte 17
		.byte 17
		.byte 17
		.byte 17
		.byte 17
		.byte 17
		.byte 18
		.byte 18
		.byte 18
		.byte 18
		.byte 18
		.byte 18
		.byte 18
		.byte 19
		.byte 19
		.byte 19
		.byte 19
		.byte 19
		.byte 19
		.byte 19
		.byte 19
		.byte 20
		.byte 20
		.byte 20
		.byte 20
		.byte 20
		.byte 20
		.byte 20
		.byte 20
		.byte 21
		.byte 21
		.byte 21
		.byte 21
		.byte 21
		.byte 21
		.byte 21
		.byte 22
		.byte 22
		.byte 22
		.byte 22
		.byte 22
		.byte 22
		.byte 22
		.byte 22
		.byte 23
		.byte 23
		.byte 23
		.byte 23
		.byte 23
		.byte 23
		.byte 23
		.byte 24
		.byte 24
		.byte 24
		.byte 24
		.byte 24
		.byte 24
		.byte 24
		.byte 24
		.byte 25
		.byte 25
		.byte 25
		.byte 25
		.byte 25
		.byte 25
		.byte 25
		.byte 26
		.byte 26
		.byte 26
		.byte 26
		.byte 26
		.byte 26
		.byte 26
		.byte 26
		.byte 27
		.byte 27
		.byte 27
		.byte 27
		.byte 27
		.byte 27
		.byte 27
		.byte 27
		.byte 28
		.byte 28
		.byte 28
		.byte 28
		.byte 28
		.byte 28
		.byte 28
		.byte 29
		.byte 29
		.byte 29
		.byte 29
		.byte 29
		.byte 29
		.byte 29
		.byte 29
		.byte 30
		.byte 30
		.byte 30
		.byte 30
		.byte 30
		.byte 30
		.byte 30
		.byte 31
		.byte 31
		.byte 31
		.byte 31
		.byte 31
		.byte 31
		.byte 31
		.byte 31
		.byte 32
		.byte 32
		.byte 32
		.byte 32
		.byte 32
		.byte 32
		.byte 32
		.byte 33
		.byte 33
		.byte 33
		.byte 33
		.byte 33
		.byte 33
		.byte 33
		.byte 33
		.byte 34
		.byte 34
		.byte 34
		.byte 34
		.byte 34
		.byte 34
		.byte 34
		.byte 34
		.byte 35
		.byte 35
		.byte 35
		.byte 35
		.byte 35
		.byte 35
		.byte 35
		.byte 36
		.byte 36
		.byte 36
		.byte 36
		.byte 36
		.byte 36
		.byte 36
		.byte 36
		.byte 37
		.byte 37
		.byte 37
		.byte 37
		.byte 37
		.byte 37
		.byte 37
		.byte 38
		.byte 38
		.byte 38
		.byte 38
		.byte 38
		.byte 38
		.byte 38
		.byte 38
		.byte 39
		.byte 39
		.byte 39
		.byte 39
		.byte 39
		.byte 39
		.byte 39
		.byte 39
		.byte 40
		.byte 40
		.byte 40
		.byte 40
		.byte 40
		.byte 40
		.byte 40
		.byte 41
		.byte 41
		.byte 41
		.byte 41
		.byte 41
		.byte 41
		.byte 41
		.byte 41
		.byte 42
		.byte 42
		.byte 42
		.byte 42
		.byte 42
		.byte 42
		.byte 42
		.byte 43
		.byte 43
		.byte 43
		.byte 43
		.byte 43
		.byte 43
		.byte 43
		.byte 43
		.byte 44
		.byte 44
		.byte 44
		.byte 44
		.byte 44
		.byte 44
		.byte 44
		.byte 45
		.byte 45
		.byte 45
		.byte 45
		.byte 45
		.byte 45
		.byte 45
		.byte 45
		.byte 46
		.byte 46
		.byte 46
		.byte 46
		.byte 46
		.byte 46
		.byte 46
		.byte 46
		.byte 47
		.byte 47
		.byte 47
		.byte 47
		.byte 47
		.byte 47
		.byte 47
		.byte 48
		.byte 48
		.byte 48
		.byte 48
		.byte 48
		.byte 48
		.byte 48
		.byte 48
		.byte 49
		.byte 49
		.byte 49
		.byte 49
		.byte 49
		.byte 49
		.byte 49
		.byte 50
		.byte 50
		.byte 50
		.byte 50
		.byte 50
		.byte 50
		.byte 50
		.byte 50
		.byte 51
		.byte 51
		.byte 51
		.byte 51
		.byte 51
		.byte 51
		.byte 51
		.byte 52
		.byte 52
		.byte 52
		.byte 52
		.byte 52
		.byte 52
		.byte 52
		.byte 52
		.byte 53
		.byte 53
		.byte 53
		.byte 53
		.byte 53
		.byte 53
		.byte 53
		.byte 53
		.byte 54
		.byte 54
		.byte 54
		.byte 54
		.byte 54
		.byte 54
		.byte 54
		.byte 55
		.byte 55
		.byte 55
		.byte 55
		.byte 55
		.byte 55
		.byte 55
		.byte 55
		.byte 56
		.byte 56
		.byte 56
		.byte 56
		.byte 56
		.byte 56
		.byte 56
		.byte 57
		.byte 57
		.byte 57
		.byte 57
		.byte 57
		.byte 57
		.byte 57
		.byte 57
		.byte 58
		.byte 58
		.byte 58
		.byte 58
		.byte 58
		.byte 58
		.byte 58
		.byte 59
		.byte 59
		.byte 59
		.byte 59
		.byte 59
		.byte 59
		.byte 59
		.byte 59
		.byte 60
		.byte 60
		.byte 60
		.byte 60
		.byte 60
		.byte 60
		.byte 60
		.byte 60
		.byte 61
		.byte 61
		.byte 61
		.byte 61
		.byte 61
		.byte 61
		.byte 61
		.byte 62
		.byte 62
		.byte 62
		.byte 62
		.byte 62
		.byte 62
		.byte 62
		.byte 62
		.byte 63
		.byte 63
		.byte 63
		.byte 63
		.byte 63
		.byte 63
		.byte 63
		.byte 64
		.byte 64
		.byte 64
		.byte 64
		.byte 64
		.byte 64
		.byte 64
		.byte 64
		.byte 65
		.byte 65
		.byte 65
		.byte 65
		.byte 65
		.byte 65
		.byte 65
		.byte 66

RandomPseudo:
		.byte 0xFF

DisplayDutyNumbers:
		.byte "000%",	 "000%",	 "000%",	 "000%",	 "000%",	 "000%",	 "000%",	 "000%",	 "001%",	 "001%",	 "001%",	 "001%",	 "001%",	 "001%",	 "001%",	 "001%",	 "003%",	 "003%",	 "003%",	 "003%",	 "003%",	 "003%",	 "003%",	 "004%",	 "004%",	 "004%",	 "004%",	 "004%",	 "004%",	 "004%",	 "004%",	 "006%",	 "006%",	 "006%",	 "006%",	 "006%",	 "006%",	 "006%",	 "007%",	 "007%",	 "007%",	 "007%",	 "007%",	 "007%",	 "007%",	 "007%",	 "009%",	 "009%",	 "009%",	 "009%",	 "009%",	 "009%",	 "009%",	 "009%",	 "010%",	 "010%",	 "010%",	 "010%",	 "010%",	 "010%",	 "010%",	 "012%",	 "012%",	 "012%",	 "012%",	 "012%",	 "012%",	 "012%",	 "012%",	 "013%",	 "013%",	 "013%",	 "013%",	 "013%",	 "013%",	 "013%",	 "015%",	 "015%",	 "015%",	 "015%",	 "015%",	 "015%",	 "015%",	 "015%",	 "016%",	 "016%",	 "016%",	 "016%",	 "016%",	 "016%",	 "016%",	 "018%",	 "018%",	 "018%",	 "018%",	 "018%",	 "018%",	 "018%",	 "018%",	 "019%",	 "019%",	 "019%",	 "019%",	 "019%",	 "019%",	 "019%",	 "019%",	 "021%",	 "021%",	 "021%",	 "021%",	 "021%",	 "021%",	 "021%",	 "022%",	 "022%",	 "022%",	 "022%",	 "022%",	 "022%",	 "022%",	 "022%",	 "024%",	 "024%",	 "024%",	 "024%",	 "024%",	 "024%",	 "024%",	 "025%",	 "025%",	 "025%",	 "025%",	 "025%",	 "025%",	 "025%",	 "025%",	 "027%",	 "027%",	 "027%",	 "027%",	 "027%",	 "027%",	 "027%",	 "028%",	 "028%",	 "028%",	 "028%",	 "028%",	 "028%",	 "028%",	 "028%",	 "030%",	 "030%",	 "030%",	 "030%",	 "030%",	 "030%",	 "030%",	 "030%",	 "031%",	 "031%",	 "031%",	 "031%",	 "031%",	 "031%",	 "031%",	 "033%",	 "033%",	 "033%",	 "033%",	 "033%",	 "033%",	 "033%",	 "033%",	 "034%",	 "034%",	 "034%",	 "034%",	 "034%",	 "034%",	 "034%",	 "036%",	 "036%",	 "036%",	 "036%",	 "036%",	 "036%",	 "036%",	 "036%",	 "038%",	 "038%",	 "038%",	 "038%",	 "038%",	 "038%",	 "038%",	 "039%",	 "039%",	 "039%",	 "039%",	 "039%",	 "039%",	 "039%",	 "039%",	 "041%",	 "041%",	 "041%",	 "041%",	 "041%",	 "041%",	 "041%",	 "041%",	 "042%",	 "042%",	 "042%",	 "042%",	 "042%",	 "042%",	 "042%",	 "044%",	 "044%",	 "044%",	 "044%",	 "044%",	 "044%",	 "044%",	 "044%",	 "045%",	 "045%",	 "045%",	 "045%",	 "045%",	 "045%",	 "045%",	 "047%",	 "047%",	 "047%",	 "047%",	 "047%",	 "047%",	 "047%",	 "047%",	 "048%",	 "048%",	 "048%",	 "048%",	 "048%",	 "048%",	 "048%",	 "050%",	 "050%",	 "050%",	 "050%",	 "050%",	 "050%",	 "050%",	 "050%",	 "051%",	 "051%",	 "051%",	 "051%",	 "051%",	 "051%",	 "051%",	 "051%",	 "053%",	 "053%",	 "053%",	 "053%",	 "053%",	 "053%",	 "053%",	 "054%",	 "054%",	 "054%",	 "054%",	 "054%",	 "054%",	 "054%",	 "054%",	 "056%",	 "056%",	 "056%",	 "056%",	 "056%",	 "056%",	 "056%",	 "057%",	 "057%",	 "057%",	 "057%",	 "057%",	 "057%",	 "057%",	 "057%",	 "059%",	 "059%",	 "059%",	 "059%",	 "059%",	 "059%",	 "059%",	 "059%",	 "060%",	 "060%",	 "060%",	 "060%",	 "060%",	 "060%",	 "060%",	 "062%",	 "062%",	 "062%",	 "062%",	 "062%",	 "062%",	 "062%",	 "062%",	 "063%",	 "063%",	 "063%",	 "063%",	 "063%",	 "063%",	 "063%",	 "065%",	 "065%",	 "065%",	 "065%",	 "065%",	 "065%",	 "065%",	 "065%",	 "066%",	 "066%",	 "066%",	 "066%",	 "066%",	 "066%",	 "066%",	 "068%",	 "068%",	 "068%",	 "068%",	 "068%",	 "068%",	 "068%",	 "068%",	 "069%",	 "069%",	 "069%",	 "069%",	 "069%",	 "069%",	 "069%",	 "069%",	 "071%",	 "071%",	 "071%",	 "071%",	 "071%",	 "071%",	 "071%",	 "072%",	 "072%",	 "072%",	 "072%",	 "072%",	 "072%",	 "072%",	 "072%",	 "074%",	 "074%",	 "074%",	 "074%",	 "074%",	 "074%",	 "074%",	 "076%",	 "076%",	 "076%",	 "076%",	 "076%",	 "076%",	 "076%",	 "076%",	 "077%",	 "077%",	 "077%",	 "077%",	 "077%",	 "077%",	 "077%",	 "079%",	 "079%",	 "079%",	 "079%",	 "079%",	 "079%",	 "079%",	 "079%",	 "080%",	 "080%",	 "080%",	 "080%",	 "080%",	 "080%",	 "080%",	 "080%",	 "082%",	 "082%",	 "082%",	 "082%",	 "082%",	 "082%",	 "082%",	 "083%",	 "083%",	 "083%",	 "083%",	 "083%",	 "083%",	 "083%",	 "083%",	 "085%",	 "085%",	 "085%",	 "085%",	 "085%",	 "085%",	 "085%",	 "086%",	 "086%",	 "086%",	 "086%",	 "086%",	 "086%",	 "086%",	 "086%",	 "088%",	 "088%",	 "088%",	 "088%",	 "088%",	 "088%",	 "088%",	 "089%",	 "089%",	 "089%",	 "089%",	 "089%",	 "089%",	 "089%",	 "089%",	 "091%",	 "091%",	 "091%",	 "091%",	 "091%",	 "091%",	 "091%",	 "091%",	 "092%",	 "092%",	 "092%",	 "092%",	 "092%",	 "092%",	 "092%",	 "094%",	 "094%",	 "094%",	 "094%",	 "094%",	 "094%",	 "094%",	 "094%",	 "095%",	 "095%",	 "095%",	 "095%",	 "095%",	 "095%",	 "095%",	 "097%",	 "097%",	 "097%",	 "097%",	 "097%",	 "097%",	 "097%",	 "097%",	 "098%",	 "098%",	 "098%",	 "098%",	 "098%",	 "098%",	 "098%",	 "100%"

ADCLookup:
		.byte 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50

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
