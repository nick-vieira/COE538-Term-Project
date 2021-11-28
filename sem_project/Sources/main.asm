;*****************************************************************
;* This stationery serves as the framework for a                 *
;* user application (single file, absolute assembly application) *
;* For a more comprehensive program that                         *
;* demonstrates the more advanced functionality of this          *
;* processor, please see the demonstration applications          *
;* located in the examples subdirectory of the                   *
;* Freescale CodeWarrior for the HC12 Program directory          *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point
          
; Include derivative-specific definitions 
		        INCLUDE 'derivative.inc'
		       
; equates section

LCD_DAT     EQU PORTB ; LCD data port
LCD_CNTR    EQU PTJ
LCD_E       EQU $80   ;LCD E-signal pin
LCD_RS      EQU $40   ;LCD RS-signal pin

FWD_INT     EQU 69 ; 3 second delay (at 23Hz)
REV_INT     EQU 69 ; 3 second delay (at 23Hz)
FWD_TRN_INT EQU 46 ; 2 second delay (at 23Hz)
REV_TRN_INT EQU 46 ; 2 second delay (at 23Hz)

START       EQU 0
FWD         EQU 1
REV         EQU 2
ALL_STP     EQU 3
FWD_TRN     EQU 4
REV_TRN     EQU 5

; variable section

            ORG $3000 ; Where our TOF counter register lives
            
TOF_COUNTER dc.b 0 ; The timer, incremented at 23Hz
CRNT_STATE  dc.b 3 ; Current state register
T_FWD       ds.b 1 ; FWD time
T_REV       ds.b 1 ; REV time
T_FWD_TRN   ds.b 1 ; FWD_TURN time
T_REV_TRN   ds.b 1 ; REV_TURN time
TEN_THOUS   ds.b 1 ; 10,000 digit
THOUSANDS   ds.b 1 ; 1,000 digit
HUNDREDS    ds.b 1 ; 100 digit
TENS        ds.b 1 ; 10 digit
UNITS       ds.b 1 ; 1 digit
NO_BLANK    ds.b 1 ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE   RMB 10   ; Extra space for decimal point and string terminator

;ATD variables

ATDDIEN:     RMB 8
ATDDR0L      RMB 8
ATDSTAT0:    RMB 1
ATDDR4       RMB 4
ATDCTL2      RMB 4
ATDCTL3      RMB 4
ATDCTL4      RMB 4
ATDCTL5      RMB 4

;code section
            ORG $3000 ; Where our TOF counter register lives
Entry:
_Startup:

            LDS $4000 ;Stack pointer initialization
            JSR initPORTS  ;
            JSR initAD   ;AD converter
            JSR initLCD ;LCD initlization
            JSR clrLCD  ;Clear LCD and home cursor
            JSR initTCNT  ;Timer overflow counter initialization
            CLI   ;Enable interrupts
            
            LDX #msg1 ;Display msg1
            JSR putsLCD
            
            LDAA #$8A   ;Move LCD cursor to end of msg1
            JSR cmd2LCD
            LDX #msg2   ;Display msg2
            JSR putsLCD
            
            LDAA #$C0   ;Move LCD cursor to end of msg2
            JSR cmd2LCD
            LDX #msg3   ;Display msg3
            JSR putsLCD
            
            LDAA #$C7   ;Move LCD cursor to end of msg3
            JSR cmd2LCD
            LDX #msg4   ;Display msg4
            JSR putsLCD
                       

;************************************************************
;* Motor Control                                            *
;************************************************************
            BSET DDRA,%00000011
            BSET DDRT,%00110000
            JSR  STARFWD
            JSR  PORTFWD
            JSR  STARON
            JSR  PORTON
            JSR  STARREV
            JSR  PORTREV
            JSR  STAROFF
            JSR  PORTOFF
            BRA  *
            
STARON      LDAA PTT
            ORAA #%00100000
            STAA PTT
            RTS
            
STAROFF     LDAA PTT
            ANDA #%11011111
            STAA PTT
            RTS
            
PORTON:     LDAA PTT
            ORAA #%00010000
            STAA PTT
            RTS
            
PORTOFF:    LDAA PTT
            ANDA #%11101111
            STAA PTT
            RTS
            
STARFWD:    LDAA PORTA
            ANDA #%11111101
            STAA PORTA
            RTS
            
STARREV:    LDAA PORTA
            ORAA #%00000010
            STAA PORTA
            RTS

PORTFWD     LDAA PORTA
            ANDA #%11111110
            STAA PORTA
            RTS
            
PORTREV     LDAA PORTA
            ORAA #%00000001
            STAA PTH
            RTS

;data section

msg1        dc.b "State ",0
msg2        dc.b "Readings ",0
msg3:       dc.b "Battery Voltage ",0
msg4        dc.b "Bumper status ",0
tab         dc.b "START ",0
            dc.b "FWD ",0
            dc.b "REV ",0
            dc.b "ALL_STP",0
            dc.b "FWD_TRN",0
            dc.b "REV_TRN",0

;subroutine section

DISPATCHER  CMPA #START ; If it’s the START state 
            BNE NOT_START 
            JSR START_ST ; then call START_ST routine       
            BRA DISP_EXIT ; and exit                        
;                                                           
NOT_START   CMPA #FWD ;Else if it's the FORWARD state
            BNE NO_FWD
            JSR FWD_ST ; then call the FORWARD routine
            JMP DISP_EXIT ; and exit
            
NOT_FORWARD   CMPA #REV ;Else if it's the REVERSE state
              BNE NOT_REV
              JSR REV_ST ; then call the REVERSE routine
              JMP DISP_EXIT ; and exit
            
NOT_REV       CMPA #LT_TRN ;Else if it's the LT_TRN state
              BNE NOT_LT_TRN
              JSR LT_TRN_ST ; then call the LT_TRN routine
              JMP DISP_EXIT ; and exit
            
NOT_LT_TRN  CMPA #RT_TRN ;Else if it's the RT_TRN state
            BNE NOT_RT_TRN
            JSR RT_TRN_ST ; then call the RT_TRN routine
            JMP DISP_EXIT ; and exit
            
NOT_RT_TRN  CMPA #REV_TRN  ;Else if it's the REV_TRN state
            BNE NOT_REV_TRN
            JSR REV_TRN_ST ; then call the REVERSE TURN routine
            JMP DISP_EXIT ; and exit
                
NOT_BK_TRK   CMPA #SBY                     
             BNE NOT_SBY  
             JSR SBY_ST   
             JMP DISP_EXITl               

NOT_SBY     NOP       
DISP_EXIT   RTS ; Exit from the state dispatcher ----------

;*******************************************************************

START_ST    BRCLR PORTAD0, $04, NO_FWD ;If FWD_BUMP
            JSR INIT_FWD ; initialize the FWD state
            MOVB #FWD, CRNT_STATE  ; Go into the FWD state
            
NO_FWD      NOP ; Else
START_EXIT  RTS ; return to the MAIN routine

;*******************************************************************

FWD_ST      BRSET PORTAD0, $04, NO_FWD_BUMP ; if FWD_BUMP then
            JSR INIT_REV   ; initialize the REVRSE routine
            MOVB #REV, CRNT_STATE  ;set the state to REVERSE
            JMP FWD_EXIT  ; return
            
NO_FWD_BUMP BRSET PORTAD0,$08,NO_REV_BUMP ; If REAR_BUMP, then we should stop
            JSR INIT_REV_BUMP ; so initialize the ALL_STOP state
            MOVB #REV_ST, CRNT_STATE ; and change state to ALL_STOP
            JMP FWD_EXIT ; and return
            
NO_REV_BUMP  LDAA TOF_COUNTER ; If Tc>Tfwd then
             CMPA T_FWD ; the robot should make a turn
             BNE NO_LT_TRN ; so
             JSR INIT_LT_TRN ; initialize the FORWARD_TURN state
             MOVB #LT_TRN,CRNT_STATE ; and go to that state
             JMP FWD_EXIT
             
LT_TURN       LDAA  NEXT_D   ; Push direction for the previous
              PSHA      ; Intersection to the stack pointer
              LDAA  SEC_PTH_INT ; Then store direction taken to NEXT_D
              STAA  NEXT_D 
              JSR   INIT_LT_TRN  ; The robot should make a LEFT turn
              MOVB  #LT_TRN,CRNT_STATE ; Initialize the LT_TRN state             

FWD_EXIT    RTS ; return to the MAIN routine

;*******************************************************************

REV_ST      LDAA TOF_COUNTER ; If Tc>Trev then
            CMPA T_REV ; the robot should make a FWD turn
            BNE NO_REV_TRN ; so
            JSR INIT_REV_TRN ; initialize the REV_TRN state
            MOVB #REV_TRN,CRNT_STATE ; set state to REV_TRN
            BRA REV_EXIT ; and return
NO_REV_TRN  NOP ; Else
REV_EXIT    RTS ; return to the MAIN routine

;*******************************************************************

INIT_FWD      BCLR PORTA,%00000011 ; Set FWD direction for both motors
              BSET PTT,%00110000 ; Turn on the drive motors
              LDAA TOF_COUNTER ; Mark the fwd time Tfwd
              ADDA #FWD_INT
              STAA T_FWD
              RTS

;*******************************************************************

INIT_REV      BSET PORTA,%00000011 ; Set REV direction for both motors
              BSET PTT,%00110000 ; Turn on the drive motors
              LDAA TOF_COUNTER ; Mark the fwd time Tfwd
              ADDA #REV_INT
              STAA T_REV
              RTS

;*******************************************************************

INIT_SBY     BCLR  PTT,%00110000  ; Turn off the drive motors
             RTS

; utility subroutines
;*******************************************************************

initLCD     BSET DDRB,%11111111 ; configure pins PS7,PS6,PS5,PS4 for output
            BSET DDRJ,%11000000 ; configure pins PE7,PE4 for output
            LDY #2000 ; wait for LCD to be ready
            JSR del_50us ; -"-
            LDAA #$28 ; set 4-bit data, 2-line display
            JSR cmd2LCD ; -"-
            LDAA #$0C ; display on, cursor off, blinking off
            JSR cmd2LCD ; -"-
            LDAA #$06 ; move cursor right after entering a character
            JSR cmd2LCD ; -"-
            RTS

;*******************************************************************

clrLCD      LDAA #$01 ; clear cursor and return to home position
            JSR cmd2LCD ; -"-
            LDY #40 ; wait until "clear cursor" command is complete
            JSR del_50us ; -"-
            RTS

;*******************************************************************

del_50us:   PSHX ;2 E-clk
eloop:      LDX #30 ;2 E-clk -
iloop:      PSHA    ;2 E-clk |
            PULA    ;3 E-clk |
            PSHA    ;2 E-clk | 50us
            PULA    ;3 E-clk |
            PSHA    ;2 E-clk |
            PULA    ;3 E-clk |
            PSHA    ;2 E-clk |
            PULA    ;3 E-clk |
            PSHA    ;2 E-clk |
            PULA    ;3 E-clk |
            PSHA    ;2 E-clk |
            PULA    ;3 E-clk |
            NOP     ;1 E-clk |
            NOP     ;1 E-clk |
            DBNE X,iloop ;3 E-clk -
            DBNE Y,eloop ;3 E-clk
            PULX ;3 E-clk
            RTS ;5 E-clk

;*******************************************************************

cmd2LCD:    BCLR LCD_CNTR,LCD_RS ; select the LCD Instruction Register (IR)
            JSR dataMov ; send data to IR
            RTS

;*******************************************************************

putsLCD     LDAA 1, X+ ; get one character from the string
            BEQ donePS ; reach NULL character?
            JSR putcLCD
            BRA putsLCD
            
donePS      RTS

;*******************************************************************

putcLCD     BSET LCD_CNTR,LCD_RS ; select the LCD Data register (DR)
            JSR dataMov ; send data to DR
            RTS

;*******************************************************************

dataMov     BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
            STAA LCD_DAT ; send the upper 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
            LSLA ; match the lower 4 bits with the LCD data pins
            LSLA ; -"-
            LSLA ; -"-
            LSLA ; -"-
            BSET LCD_CNTR,LCD_E ; pull the LCD E signal high
            STAA LCD_DAT ; send the lower 4 bits of data to LCD
            BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
            LDY #1 ; adding this delay will complete the internal
            JSR del_50us ; operation for most instructions
            RTS
            
;*******************************************************************

initAD      MOVB #$C0,ATDCTL2 ;power up AD, select fast flag clear
            JSR del_50us ;wait for 50 us
            MOVB #$00,ATDCTL3 ;8 conversions in a sequence
            MOVB #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
            BSET ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
            RTS

;*******************************************************************

int2BCD     XGDX      ; Save the binary number into .X
            LDAA #0   ;lear the BCD_BUFFER
            STAA TEN_THOUS
            STAA THOUSANDS
            STAA HUNDREDS
            STAA TENS
            STAA UNITS
            STAA BCD_SPARE
            STAA BCD_SPARE+1
            
            CPX #0 ;Check for a zero input
            BEQ CON_EXIT ; and if so, exit

            XGDX         ;Not zero, get the binary number back to .D as dividend
            LDX #10      ; Setup 10 (Decimal!) as the divisor
            IDIV         ;Divide: Quotient is now in .X, remainder in .D
            STAB UNITS    ;Store remainder
            CPX #0       ;If quotient is zero,
            BEQ CON_EXIT   ;then exit

            XGDX  ;swap first quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB TENS
            CPX #0
            BEQ CON_EXIT

            XGDX ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB HUNDREDS
            CPX #0
            BEQ CON_EXIT

            XGDX ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB THOUSANDS
            CPX #0
            BEQ CON_EXIT

            XGDX    ;Swap quotient back into .D
            LDX #10 ;and setup for another divide by 10
            IDIV
            STAB TEN_THOUS

CON_EXIT    RTS ;We’re done the conversion

;*******************************************************************

BCD2ASC     LDAA  #0  ; Initialize the blanking flag
            STAA NO_BLANK

  C_TTHOU:     LDAA TEN_THOUS ;Check the ’ten_thousands’ digit
               ORAA NO_BLANK
               BNE NOT_BLANK1

  ISBLANK1:    LDAA #' '             ; It's blank
               STAA TEN_THOUS ;so store a space
               BRA  C_THOU ;and check the ’thousands’ digit

  NOT_BLANK1:  LDAA TEN_THOUS ;Get the ’ten_thousands’ digit
               ORAA #$30 ;Convert to ascii
               STAA TEN_THOUS
               LDAA #$1 ;Signal that we have seen a ’non-blank’ digit
               STAA NO_BLANK

  C_THOU:      LDAA THOUSANDS ;Check the thousands digit for blankness
               ORAA NO_BLANK  ;If it’s blank and ’no-blank’ is still zero
               BNE  NOT_BLANK2

  ISBLANK2:    LDAA  #' '  ; Thousands digit is blank
               STAA THOUSANDS ;so store a space
               BRA  C_HUNS ;and check the hundreds digit

  NOT_BLANK2:  LDAA THOUSANDS ;(similar to ’ten_thousands’ case)
               ORAA #$30
               STAA THOUSANDS
               LDAA #$1
               STAA NO_BLANK

  C_HUNS:      LDAA HUNDREDS ;Check the hundreds digit for blankness
               ORAA NO_BLANK ;If it’s blank and ’no-blank’ is still zero
               BNE NOT_BLANK3

  ISBLANK3:    LDAA  #' '  ; Hundreds digit is blank
               STAA HUNDREDS ;so store a space
               BRA C_TENS ;and check the tens digit

  NOT_BLANK3:  LDAA HUNDREDS ;(similar to ’ten_thousands’ case)
               ORAA #$30
               STAA HUNDREDS
               LDAA #$1
               STAA NO_BLANK

  C_TENS:      LDAA TENS ;Check the tens digit for blankness
               ORAA NO_BLANK ;If it’s blank and ’no-blank’ is still zero
               BNE NOT_BLANK4  ;

  ISBLANK4:    LDAA  #' '  ; Tens digit is blank
               STAA TENS ;so store a space
               BRA C_UNITS ;and check the units digit

  NOT_BLANK4:  LDAA TENS ;(similar to ’ten_thousands’ case)
               ORAA #$30
               STAA TENS

  C_UNITS:     LDAA UNITS ;No blank check necessary, convert to ascii.
               ORAA #$30
               STAA UNITS

            RTS ;We’re done

;*******************************************************************

ENABLE_TOF  LDAA #%10000000
            STAA TSCR1 ; Enable TCNT
            STAA TFLG2 ; Clear TOF
            LDAA #%10000100 ; Enable TOI and select prescale factor equal to 16
            STAA TSCR2
            RTS

;*******************************************************************

TOF_ISR     INC TOF_COUNTER
            LDAA #%10000000; Clear
            STAA TFLG2 ; TOF
            RTI
            
;*******************************************************************
;* Update Display (Battery Voltage + Current State) *
;*******************************************************************

UPDT_DISPL  MOVB #$90,ATDCTL5 ; R-just., uns., sing. conv., mult., ch=0, start
            BRCLR ATDSTAT0,$80,* ; Wait until the conver. seq. is complete
            LDAA ATDDR0L ; Load the ch0 result - battery volt - into A
            
            ; Display the battery voltage
            
			      MOVB #$90,ATDCTL5 ; R-just., uns., sing. conv., mult., ch=0, start
            BRCLR ATDSTAT0,$80,* ; Wait until the conver. seq. is complete
            LDAA ATDDR0L ; Load the ch0 result - battery volt - into A
            LDAB #39 ;AccB = 39
            MUL ;AccD = 1st result x 39
            ADDD #600 ;AccD = 1st result x 39 + 600
            JSR int2BCD
            JSR BCD2ASC
            LDAA #$8F ;move LCD cursor to the 1st row, end of msg1
            JSR cmd2LCD ;"
            LDAA TEN_THOUS ;output the TEN_THOUS ASCII character
            JSR putcLCD ;"
            LDAA THOUSANDS
            JSR putcLCD
            LDAA #$2E
            JSR putcLCD
            LDAA HUNDREDS
            JSR putcLCD ; Display the battery voltage
            
            LDAA #$C6 ; Move LCD cursor to the 2nd row, end of msg2
            JSR cmd2LCD ;
            LDAB CRNT_STATE ; Display current state
            LSLB ; "
            LSLB ; "
            LSLB ; "
            LDX #tab ; "
            ABX ; "
            JSR putsLCD ; "
            RTS

;*******************************************************************
;* Interrupt Vectors *
;*******************************************************************
            ORG $FFFE
            DC.W Entry ; Reset Vector
            
            ORG $FFDE
            DC.W TOF_ISR ; Timer Overflow Interrupt Vector
