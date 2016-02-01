;**********************************************************************
;                                                                     *
;    Filename:	   Super_centering.asm                                           *
;    Date:           09/05/2012                                                 *
;    File Version:    Build_alpha                                                *
;                                                                     *
;    Author:       Sameer Sunani                                                   *
;    Company:      Echelon V                                                   *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files Required: P16F877A.INC                                     *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes: Test to check the wall detection possibilities.                                           *
;**********************************************************************



	list		p=16f877A	; list directive to define processor
	#include	<p16f877A.inc>	; processor specific variable definitions
	
	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _RC_OSC & _WRT_OFF & _LVP_ON & _CPD_OFF
	
;################### VARIABLE DEFINITIONS ######################################
w_temp		EQU	0x7D		; variable used for context saving 
status_temp	EQU	0x7E		; variable used for context saving
pclath_temp	EQU	0x7F		; variable used for context saving			
RP0    		equ  		0x05 					;PAGE BIT FOR STATUS REGISTER
Z      		equ  		0x02					;ZERO BIT

STATUS 		equ  		0x03 					;STATUS REGISTER
INTCON 		equ  		0x0B 					;INTERRUPT REGISTER
TRISB  		equ  		0x86 					;I/O REGISTER FOR PORTB
PORTB  		equ  		0x06 					;PORT REGISTER FOR OUTPUT OR INPUT
PORTA		equ			0x05

TRISA		equ			0x85
Z			equ			0x02
GO			equ			0x02

RP0    	 	equ 		5
TOIF    	equ 		2
TOIE    	equ 		5
GIE     	equ 		7
RB0			equ			0
RB1			equ 		1
RB2			equ			2
RB3			equ			3
RC1			equ			1
LSB			equ			0
RA7			equ			7
RA6			equ			6

	CBLOCK		0x20
			inNUM	
			asc100
			asc10
			asc1
			thisDIG
			count
			count1
			count2
			COUNTA
			COUNTB
			COUNT20
			COUNTA5
			COUNTB5
			countd
			move
			PORTD_TEMP
			TURN_COUNT
			LEFT_COUNT
			FORWARD_COUNT
			OUTER_FORWARD_COUNT
			left_motor_control1
			right_motor_control1
			left_motor_control2
			right_motor_control2
			ENDC


;**********************************************************************
	ORG     0x000             ; processor reset vector

  	goto    main_start              ; go to beginning of program

	ORG     0x004             ; interrupt vector location
	goto	isr_start

;---------------------------------------------------------------------------
;					MACROS
;-----------------------------------------------------------------------------

forward_move	MACRO	cnt1,cnt2
				movlw	cnt1
				movwf	FORWARD_COUNT

				movlw	cnt2
				movwf	OUTER_FORWARD_COUNT
			
				bcf		PORTC,RC3
				bsf		PORTC,RC4

				movlw	b'00001001'
				movwf	left_motor_control1
				movwf	right_motor_control1

				movlw	b'00001000'
				movwf	left_motor_control2
				movwf	right_motor_control2
			
				endm

turn_setter		MACRO	cnt1
		
				movlw	cnt1
				movwf	TURN_COUNT	
				
				movlw	b'00001001'
				movwf	left_motor_control1
				movwf	right_motor_control1
			
				movlw	b'00001000'
				movwf	left_motor_control2
				movwf	right_motor_control2
			
				endm
				
lft_crct_setter	MACRO	cnt1
				movlw	cnt1
				movwf	TURN_COUNT

				movlw	b'00001001'
				movwf	left_motor_control1	;make sure that only left motor turns
			
				movlw	b'00001000'
				movwf	left_motor_control2
				
				clrf	right_motor_control1	;right motor doesnt move
				clrf	right_motor_control2
				endm
	
	
rt_crct_setter	MACRO	cnt1
				movlw	cnt1
				movwf	TURN_COUNT
				

				
				movlw	b'00001001'	;clears the pin, so you have a proper sine wave
				movwf	right_motor_control1
				
				movlw	b'00001000'
				movwf	right_motor_control2	;only right motor will turn
			
				clrf	left_motor_control1		;left motor wont move now.
				clrf	left_motor_control2

				endm
;================================================================================
;				INITIALIZERS
;===============================================================================

;*************************************************************************************
;	ANALOG TO DIGITAL PROCEDURE
;*********************************************************************************
;INIT A/D MODULE
;CONFIGURE PIC I/O LINES
;2. SELECT PARTS TO BE USED BY SETTING THE PCFGX BITS IN ADCON1 SELECT ADFM AS WELL
;3. SLEECT ANALOG CHANNELS A/D CONVERSION CLOCK AND ENABLE A/D MODULE
;4. WAIT ACQUISITION TIME
;5. INITIATE ONVERSION BY SETTING GO/DONE BIT IN ADCON0
;6. WAIT FOR CONVERSION TO COMPLETE
;7. READ AND STORE DIGITAL VALUE

Init_A2D
;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	BANKSEL	TRISA
	MOVLW	b'00000001'	;RA0 AS INPUT
	MOVWF	TRISA


;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	MOVLW	b'00001110'
	MOVWF	ADCON1		;ADFM=0 HENCE LEFT JUSTIFIED
						;0011-RA0 AS ANALOG AND RA3 IS VREF+ REST ARE DIGITAL
;SET ADCON0 THE MAIN ADC CONVERTOR ROUTINE
	BANKSEL ADCON0
	MOVLW	b'01000000'	;FOSC/8
						;CHANNELS AS 000=RA0
						;ADON=1 HENCE TURN ADC MODULE ON
	MOVWF	ADCON0
						
	BCF		INTCON,ADIE	
	BSF		ADCON0,ADON
	CALL	DELAY_20uS

	RETURN
	
;*************************************************************************************
;	READ A/D LINE
;*********************************************************************************
;READ VALUE AND CONVERT TO DIGITAL, MAIN ADC ROUTINE FOR CONVERSION

Read_A2D
	CALL	DELAY_20uS
	BANKSEL	ADCON0
	BSF		ADCON0,GO	;SET GO BIT TO START ADC CONVERSION...AUTOMATICALLY GETS CLEARED WHEN CONVERSION IS COMLPLETE

convWAIT
	BTFSC	ADCON0,GO
	GOTO	convWAIT

;READ MSBS
	MOVF	ADRESH,0
	BANKSEL	PORTB
	RETURN
;*************************************************************************************
;	BINARY TO ASCII VALUE DECIMAL CONVERSION
;************************************************************************************
 ;ON ENTRY:
;	W HAS BINARY VALUE IN RANGE 0 TO 255
;ON EXIT:
;	OUTPUT VARIABLE asc100, asc10 AND asc1 HAVE THREE DECIMAL UNITS
;VALUE 100 IS SUBTRACTED FROM SOURCE OPERAND UNTIL 0 REMAINS
;NUMBER OF SUBTRACTIONS IS THE DECIMAL HUNDREDS RESULT. 100 IS ADDED TO COMPENSATE FOR LAST SUBTRACTION
;10 IS SUBTRACTED IN SAME MANNER
; VARIABLES
;inNUM	-STORAGE SPACE FOR OPERAND
;asc100
;asc10
;asc1
;thisDIG
bin_2ASC

	MOVWF	inNUM			;COPY OF SOURCE VALUE
	CLRF	asc100
	CLRF	asc10
	CLRF	asc1
	CLRF	thisDIG

sub100
	MOVLW	d'100'
	SUBWF	inNUM,1
	BTFSC	STATUS,C		;DID SUBTRACT OVERFLOW
	GOTO	bump100			;NO COUNT OVERFLOW
	GOTO	end100

bump100
	INCF	thisDIG,1
	GOTO	sub100
;STORE 100TH DIGIT

end100
	MOVF	thisDIG,0		;DIGIT COUNTER
	ADDLW	0x30			;CONVERT TO ASCII	
	MOVWF	asc100			;STORE THE VALUE

;CALCULATE THE 10S POSITION
	CLRF	thisDIG
;adjust value
	MOVLW	d'100'
	ADDWF	inNUM,1

sub10
	MOVLW	d'10'
	SUBWF	inNUM,1			;SUBTRACT 10
	BTFSC	STATUS,C		;CHECK FOR OVERFLOW
	GOTO	bump10
	GOTO	end10

bump10
	INCF	thisDIG,1		;INCREMENT DIGIT COUNTER
	GOTO	sub10

end10
	MOVLW	d'10'
	ADDWF	inNUM,1			;ADJUST FOR LAST SUBTRACTION
	MOVF	thisDIG,0
	ADDLW	0x30			;CONVERT TO ASCII
	MOVWF	asc10			;STORE THE DIGITAL VALUE
;CALCULATE AND STORE UNITS DIGIT
	MOVF	inNUM,0			;STORE UNITS VALUE
	ADDLW	0x30
	MOVWF	asc1
	RETURN
;************************
; DELAY FOR ABOUT 40uS
;************************

DELAY_40uS
	MOVLW	d'8'
	MOVWF	COUNTA

	NOP
	NOP
	NOP
			
LOOP_40		
	NOP
	DECFSZ	COUNTA,1
	GOTO	LOOP_40
	RETURN

;************************
; DELAY FOR ABOUT 20uS
;************************
DELAY_20uS
	MOVLW	d'5'
	MOVWF	COUNT20
LOOP_20
	DECFSZ	COUNT20,1
	GOTO	LOOP_20
	RETURN	


;##############################***DELAYS***#########################################
;############################### WAIT SUBROUTINE = 1 SEC #########################################

DELAY_1S		
			movlw 		d'24'					;SET OUTERMOST LOOP TO 24
  
 			movwf 		count 					;MOVE STATEMENTS=2 INSTRUCTION CYCLES
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
  			nop									;11 NOPS=11 INSTRUCTION CYCLES
  
loop_ext1 	movlw 		d'254'					;BEGINNING OF OUTERMOST LOOP
  			movwf		count1	 				;2 INSTRUCTION CYCLES
  			nop
  			nop
  			nop
  			nop
  			nop
  			nop									;6 NOPS=6 INSTRUCTION CYCLES
   
loop_ext 	movlw 		d'16'  					;BEGINNING OF MIDDLE LOOP
  			movwf 		count2					;2 INSTRUCTION CYCLES
   
loop 		nop									;BEGINNING OF INNERMOST LOOP
  			nop
  			nop
  			nop
  			nop
  			nop
  			nop									;7 NOPS=7INSTRUCTION CYCLES
   
  			decfsz 		count2,1				;INNERMOST LOOP (10 CYCLES)-WILL REPEAT COUNT2(16) TIMES
  			goto 		loop					;((16x10)-1)x254x24=969,264 CYCLES
   
  			decfsz 		count1,1				;MIDDLE  LOOP (5 CYCLES)-WILL REPEAT COUNT1(254) TIMES
  			goto 		loop_ext				;(5x254-1)x24=30,456
   
  			decfsz 		count,1					;OUTER LOOP	(11 CYCLES)-WILL REPEAT COUNT(24) TIMES
  			goto 		loop_ext1				;11x24-1=263
   
  			return								;2 INSTRUCTION CYCLES
					

;***************************************************************************************************
; DELAY FOR 5msec; Delay equation : [50*([199*10-1]+10)-1]+2+2+2 = 10000cycles = 5msec (at Fosc=8MHz)
;***************************************************************************************************


delay_50ms

	MOVLW	D'35'
	MOVWF	COUNTA
					
OUTER_5
	MOVLW	D'199'
	MOVWF	COUNTB
	
	NOP
	NOP
	NOP
	NOP
	NOP
	
		
INNER_5
	NOP
	NOP
	NOP
	NOP
	NOP

	DECFSZ	COUNTB,1
	GOTO	INNER_5

	DECFSZ	COUNTA,1
	GOTO	OUTER_5

	RETURN	
; #####################################################
; #########  set-up interrupts and ports  #############
; #####################################################
; set up port B pins to inputs

setup	
	banksel	TRISB
	clrf	TRISC		; set all portc pins to outputs for motor PULSES
	clrf	TRISB
	movlw	0xFF
	movwf	TRISD		; set the higher nibble for sensor pulses 
						; the lower nibble as inputs
	banksel	INTCON

; initialisation of interrupts



    movlw   b'10000000'  	; enable interrupts globally and disable PIE
    movwf   INTCON
	return


; ################################################
; ######### 	     set-up timer 	##########
; ################################################


timer_setup
	bcf		INTCON,TOIF	;clear timer overflow flag
						;always do this to reset the timer
	bsf		STATUS,RP0	;set page to bank1
	movlw	b'11010100'	
	movwf	OPTION_REG	;set timer to be clocked by fosc/4
						;set prescalar to 2:1
						;1XXXXXXX=pull-ups disabled
						;x1xxxxxx=interrupt on rising edge
						;xx0xxxxx=timer source clock internal
						;xxxx0xxx=prescalar assigned to timer
						;xxxxx100=32:1
	bcf		STATUS,RP0	;set page 0
	movlw	d'113'		;255-113=142 cycles between time-outs approx
						;therefor the period of the waveform becomes
						;142*32=4545uS or 4.545msmS which is 220Hz
	movwf	TMR0		;initialise timer value
	bsf		INTCON,TOIE	;enable timer overflow interrupt

;###########	INCLUSIVE TIMER1 SETUP
	movlw	b'00110100'
	movwf	T1CON		;00XX XXXX=we dont care what values they hold
						;XX11 XXXX=Timer1 Prescaler	11=1:8(ideal)
						;XXXX 0XXX=oscillator is disabled(internal RC)
						;XXXX X1XX=Do not Synchronize clock external input
						;XXXX XX0X=Timer1 Clock Source Select Bit=>Use internal Clock
						;XXXX XXX0=Timer1 ON BIT
	bsf		T1CON,0		;TURN timer1 on
	bcf		INTCON,TOIE	;for now, turn timer0 off

	return	

	
;################	MAIN PROGRAM ####################################
main_start

	clrf	STATUS
	clrf	PORTB
	clrf	PORTC
	clrf	PORTA
	call	setup
	movlw	d'11'
	movwf	LEFT_COUNT
	call	timer_setup
	call	Init_A2D

read_distance
	;part of the code that gives the forward movement priority
;the code first checks the distance sensor to see if the mouse is able to move.
;Two cases under the distance sensor: 1. Mouse can move, 2. The mouse cannot move.
;If the mouse can move: it will check if both walls are present or not.
	
	call	Read_A2D	;now the w register has the ADRESH value
	call	bin_2ASC	;converts the analog value to an ascii value

	decfsz	LEFT_COUNT,1
	goto	distance
	goto	left_check

distance
	

	movlw	'0'
	xorwf	asc100,0
	btfss	STATUS,Z
	goto	dont_move
	
	movlw	'9'			;dont move if this value matches
	xorwf	asc10,0
	BTFSC	STATUS,Z
	goto	dont_move
	
	movlw	'8'			;dont move if this value matches
	xorwf	asc10,0
	BTFSC	STATUS,Z
	goto	dont_move

	movlw	'7'			;dont move if this value matches
	xorwf	asc10,0
	BTFSC	STATUS,Z
	goto	dont_move

	movlw	'6'			;dont move if this matches
	xorwf	asc10,0
	BTFSC	STATUS,Z
	goto 	dont_move

	movlw	'5'			;dont move if this matches
	xorwf	asc10,0
	BTFSC	STATUS,Z
	goto 	wall_detected

;----------------------------------------------------------------------------

forward	

	forward_move	d'32',d'1'	


	bsf		move,0
	
	
	goto	test
;----------------------------------------------------------------------------
dont_move	

	bcf		move,0

	goto	sensor_check	;see if mouse is able to move again
;-------------------------------------------------------------------------------
wall_detected
	forward_move	d'170',d'1'

	bsf		move,0

	
	goto	test_center

;----------------------------------------------------------------------------------
test
	btfss	move,0
	goto	read_distance
	goto	move_forward

move_forward

	bsf		move,0
	bsf		INTCON,TOIE

	bcf		PORTC,RC3
	bsf		PORTC,RC4
moving
	btfsc	move,0
	goto	moving
	bcf		INTCON,TOIE
	decfsz	FORWARD_COUNT,1
	goto	move_forward
	goto	check_centering_sensors
;---------------------------------------------------------------------------------------
test_center
	btfss	move,0
	goto	read_distance
	goto	move_forward_center_reset
 
move_forward_center_reset
	movlw	d'170'
	movwf	FORWARD_COUNT

move_forward_center
	bsf		move,0
	bsf		INTCON,TOIE
	bcf		PORTC,RC3
	bsf		PORTC,RC4

moving_center
	btfsc	move,0
	goto	moving_center
	bcf		INTCON,TOIE
	decfsz	FORWARD_COUNT,1
	goto	move_forward_center
	goto	wall_detect_centering
resume_forward	
	decfsz	OUTER_FORWARD_COUNT,1
	goto	move_forward_center_reset
	goto	sensor_check

;----------------------------------------------------------------------------------
;			SENSOR CHECKS
;ZERO SIGNIFIES PRESENCE OF WALL
;ONE SIGNIFIES ABSENCE OF WALL
;-----------------------------------------------------------------------------------
sensor_check

	movlw	d'11'
	movwf	LEFT_COUNT

	movf	PORTD,0
	andlw	b'00001001'
	movwf	PORTD_TEMP

	
	movlw	b'00001000'	;left wall is absent AND RIGHT WALL IS PRESENT therefore turn left
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	left_turn
	

	movlw	b'00000001'	;Right wall is absent AND LEFT WALL IS ABSENT therefore turn right
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	right_turn


	movlw	b'00000000'	;left turn
	xorwf	PORTD_TEMP,0
	btfsc	STATUS,Z
	goto	left_turn

	movlw	b'00001001'	;make left turn
	xorwf	PORTD_TEMP,0
	btfsc	STATUS,Z
	goto	left_turn

	goto	sensor_check ;keep checking sensors


;---------------------------------------------

left_check
	movlw	d'11'
	movwf	LEFT_COUNT

;	CALL	DELAY_1S
	movf	PORTD,0
	andlw	b'00001001'
	movwf	PORTD_TEMP

	movlw	b'00001000'	;left wall is absent AND RIGHT WALL IS PRESENT therefore turn left
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	left_turn

	movlw	b'00001001'
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	left_turn	

	goto	read_distance


;----------------------------------------------------------------------------
;				TURNS
;------------------------------------------------------------------------

left_turn
	turn_setter 	d'153'

	bsf		PORTC,RC3
	bsf		PORTC,RC4
	
	call	DELAY_1S

	goto	turn_set
	
right_turn
	turn_setter		d'153'

	bcf		PORTC,RC3
	bcf		PORTC,RC4

	call	DELAY_1S

	goto	turn_set


turn_set
	bsf		move,0
	bsf		INTCON,TOIE	;enable timer overflow interrupt
turning
	btfsc	move,0
	goto 	turning
	bcf 	INTCON,TOIE
	decfsz	TURN_COUNT,1
	goto	turn_set
	goto	read_distance 



;----------------------------------------------------------------------------------
;				CENTERING
;-------------------------------------------------------------------------------------

check_centering_sensors
;############### Checking for possible collisions###################

	movf	PORTD,0
	andlw	b'00000110'
	movwf	PORTD_TEMP


	movlw	b'00000100'	;right wall is present, correct
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	left_collision_correct
	

	movlw	b'00000010'	;left wall is present, correct
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	right_collision_correct

;;############## check for right wing #############################

	movf	PORTD,0
	andlw	b'00110000'
	movwf	PORTD_TEMP

	movlw	b'00100000'
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	right_wall_centering

	movlw	b'00010000'
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	left_wall_centering

;################## check for left wing ########################

	movf	PORTD,0
	andlw	b'11000000'
	movwf	PORTD_TEMP

	movlw	b'10000000'
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	right_wall_centering

	movlw	b'01000000'
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	left_wall_centering


	goto	read_distance	;if everything is centered then keep moving forward
;-----------------------------------------------------------------

left_wall_centering

	lft_crct_setter	d'2'
	bcf		PORTC,RC3
	bsf		PORTC,RC4	

	goto	turn_set
	
right_wall_centering
	
	rt_crct_setter	d'2'

	bcf		PORTC,RC3
	bsf		PORTC,RC4
	
	goto	turn_set

left_collision_correct
	lft_crct_setter d'6'
	bcf		PORTC,RC3
	bsf		PORTC,RC4
	
	goto	turn_set

right_collision_correct
	rt_crct_setter d'6'
	
	bcf		PORTC,RC3
	bsf		PORTC,RC4
	
	goto	turn_set
	

;----------------------------------------------------------------------
;###################################################################
wall_detect_centering
	movf	PORTD,0
	andlw	b'00000110'
	movwf	PORTD_TEMP

	movlw	b'00000100'
	xorwf	PORTD_TEMP,0
	BTFSC	STATUS,Z
	goto	left_collision_WD
	
	movlw	b'00000010'
	xorwf	PORTD_TEMP,0
	btfsc	STATUS,Z
	goto	right_collision_WD
;

;;############## check for right wing #############################
;
;	movf	PORTD,0
;	andlw	b'00110000'
;	movwf	PORTD_TEMP
;
;	movlw	b'00100000'
;	xorwf	PORTD_TEMP,0
;	BTFSC	STATUS,Z
;	goto	left_wall_centering_WD
;
;	movlw	b'00010000'
;	xorwf	PORTD_TEMP,0
;	BTFSC	STATUS,Z
;	goto	right_wall_centering_WD
;
;;################## check for left wing ########################
;
;	movf	PORTD,0
;	andlw	b'11000000'
;	movwf	PORTD_TEMP
;
;	movlw	b'10000000'
;	xorwf	PORTD_TEMP,0
;	BTFSC	STATUS,Z
;	goto	left_wall_centering_WD
;
;	movlw	b'01000000'
;	xorwf	PORTD_TEMP,0
;	BTFSC	STATUS,Z
;	goto	right_wall_centering_WD


	goto	resume_forward	;goes back to where it came back from
;-------------------------------------------------------------------------
left_collision_WD
	lft_crct_setter d'7'

	;we set the direction pins to both have forward movement
	bcf		PORTC,RC3
	bsf		PORTC,RC4

	goto	turn_set_WD

right_collision_WD

	rt_crct_setter	d'7'

	bcf		PORTC,RC3
	bsf		PORTC,RC4
	goto	turn_set_WD

left_wall_centering_WD

	lft_crct_setter	d'3'
	bcf		PORTC,RC3
	bsf		PORTC,RC4	

	goto	turn_set_WD
	
right_wall_centering_WD
	
	rt_crct_setter	d'3'

	bcf		PORTC,RC3
	bsf		PORTC,RC4
	
	goto	turn_set_WD
;---------------------------------------------------------------------------
turn_set_WD
	bsf		move,0
	bsf		INTCON,TOIE

turning_wd
	btfsc	move,0
	goto	turning_wd
	bcf		INTCON,TOIE
	decfsz	TURN_COUNT,1
	goto	turn_set_WD
	goto	resume_forward
	
	
; #########################################################
; #############  		interrupt service routine 	 ###############
; #########################################################
isr_start
;context saving save
	movwf   w_temp            ; save off current W register contents
	movf	STATUS,w          ; move status register into W register
	movwf	status_temp       ; save off contents of STATUS register
	movf	PCLATH,w	  ; move pclath register into w register
	movwf	pclath_temp	  ; save off contents of PCLATH register

;actual ISR routine
	call 	timer_setup

	bsf		PORTC,RC1
	bsf		PORTC,RC2

	clrf	TMR1L
	clrf	TMR1H

	movlw	b'00011100'		;this makes 284*8=2272(half period)
	movwf	CCPR2L		;move this value into both registers for CCP module
	movwf	CCPR1L
	movlw	b'00000001'
	movwf	CCPR2H		;set the higher registers
	movwf	CCPR1H

	movf	right_motor_control1,0
	movwf	CCP2CON
	movf	left_motor_control1,0
	movwf	CCP1CON

	clrf	TMR1L
	clrf	TMR1H
	movlw	b'00011100'	;this makes 284*8=2272(half period)
	movwf	CCPR2L		;move this value into both registers for CCP module
	movwf	CCPR1L
	movlw	b'00000001'
	movwf	CCPR2H		;set the higher registers
	movwf	CCPR1H

	movf	right_motor_control2,0
	movwf	CCP2CON
	movf	left_motor_control2,0
	movwf	CCP1CON

	bcf		move,0
;end of ISR Routine
;context Saving restore

	movf	pclath_temp,w	  ; retrieve copy of PCLATH register
	movwf	PCLATH		  ; restore pre-isr PCLATH register contents
	movf    status_temp,w     ; retrieve copy of STATUS register
	movwf	STATUS            ; restore pre-isr STATUS register contents
	swapf   w_temp,f
	swapf   w_temp,w          ; restore pre-isr W register contents

	retfie
    end