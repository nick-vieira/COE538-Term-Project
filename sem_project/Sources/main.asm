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

;LCD equates section

CLEAR_HOME    EQU $01  ;Clear display and home cursor
INTERFACE     EQU $38  ;8-bit interface, two line display
CURSOR_OFF    EQU $0C  ;Display on, cursor off
SHIFT_OFF     EQU $06  ;Address increments without character shifts
LCD_SEC_LINE  EQU 64   ;Starting addr, of 2nd line of LCD
            
;LCD addresses

LCD_CNTR  EQU PTJ   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT   EQU PORTB ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_RS    EQU $40   ; LCD RS-signal pin
LCD_E     EQU $80   ; LCD E-signal pin
            
; Other codes

NULL  EQU 00  ; The string ’null terminator’
CR    EQU $0D ; ’Carriage Return’ character
SPACE EQU ' ' ; The ’space’ character		       

; equates section

FWD_INT     EQU 69 ; 3 second delay (at 23Hz)
REV_INT     EQU 69 ; 3 second delay (at 23Hz)
FWD_TRN_INT EQU 46 ; 2 second delay (at 23Hz)
REV_TRN_INT EQU 46 ; 2 second delay (at 23Hz)

START       EQU 0
FWD         EQU 1
REV         EQU 2
LT_TRN      EQU 3
RT_TRN	    EQU	4
REV_TRN     EQU 5
BK_TRK	    EQU 6
ALL_STP     EQU 7

; variable section
            ORG $3800 ; Where our TOF counter register lives
	    
IRS_CNT1    DC.W 0 ; initialize first interrupt routine at address $0000
IRS_CNT2    DC.W 0 ; initialize second interrupt routine at address $0000
	    
; Storage Registers

SENSOR_LINE FCB $01 ; Storage for guider sensor readings
SENSOR_BOW  FCB $23 ; Initialized to test values
SENSOR_PORT FCB $45
SENSOR_MID  FCB $67
SENSOR_STBD FCB $89
SENSOR_NUM  RMB 1   ; The currently selected sensor
            
TOP_LINE   RMB 20 ; Top line of display
           FCB NULL ; terminated by null
BOT_LINE   RMB 20 ; Bottom line of display
           FCB NULL ; terminated by null
CLEAR_LINE FCC ' '
           FCB NULL ; terminated by null
TEMP       RMB 1 ; Temporary location


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
            ORG $4000 
Entry:
_Startup:

            LDS $4000 ; Stack pointer initialization
            CLI	; Enable interrupts
            JSR initPORTS 
            JSR openADC ; ATD initialization
            JSR initLCD ; LCD initlization
            JSR clrLCD  ; Clear LCD and home cursor
            JSR initTCNT  ; Timer overflow counter initialization
            
            BSET DDRA,%00000011  ; STAR_DIR, PORT_DIR                        
            BSET DDRT,%00110000  ; STAR_SPEED, PORT_SPEED                                                                                   
            JSR initAD   ; Initialize ATD converter
	      
            LDX #msg1 ; Display msg1
            JSR putsLCD
          
            LDAA #$C0   ;Move LCD cursor to end of msg1
            JSR cmd2LCD
            LDX #msg2   ;Display msg2
            JSR putsLCD
            
            JSR ENABLE_TOF  ; jump to TOF initialization
            
MAIN        JSR G_LEDS_ON   ; Enable guider LEDs
            JSR READ_SENSORS  ; Read guider snesors
            JSR G_LEDS_OFF  ; Disable guider LEDs
            LDY #2000   ; set 300 ms delay for eebot initialization
            JSR del_50us   ; jump to delay routine         
     
;data section

msg1        dc.b "Battery Voltage ",0
msg2        dc.b "State ",0
tab         dc.b "START ",0
            dc.b "FWD ",0
            dc.b "REV ",0
            dc.b "ALL_STP",0
            dc.b "FWD_TRN",0
            dc.b "REV_TRN",0
	    dc.b "LT_TRN", 0
	    dc.b "RT_TRN", 0

;***************************************************************************;
;			Subroutine Section				    ;
;***************************************************************************;

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
            
NOT_REV      CMPA #LT_TRN ;Else if it's the LT_TRN state
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
	    
NOT_REV_TRN  CMPA #BK_TRK  
             BNE NOT_BK_TRK 
             JMP BK_TRK_ST
	           JMP DISP_EXIT ; and exit	    
                
NOT_BK_TRK   CMPA #ALL_STP
	           BNE NOT_ALL_STP
	           JSR ALL_STP_ST
	           JMP DISP_EXIT1
	           JMP DISP_EXIT ; and exit
	     
NOT_ALL_STP  CMPA #SBY                     
             BNE NOT_SBY  
             JSR SBY_ST   
             JMP DISP_EXITl ; and exit               

NOT_SBY     NOP       
DISP_EXIT   RTS ; Exit from the state dispatcher ----------

;*******************************************************************

START_ST    BRCLR PORTAD0, $04, NO_FWD ;If FWD_BUMP
            JSR INIT_FWD ; initialize the FWD state
            MOVB #FWD, CRNT_STATE  ; Go into the FWD state
            
START_EXIT  RTS ; return to the MAIN routine

;*******************************************************************

FWD_ST      BRSET PORTAD0, $04, NO_FWD_BUMP ; if FWD_BUMP then
            JSR INIT_REV   ; initialize the REVRSE routine
            MOVB #REV, CRNT_STATE  ;set the state to REVERSE
            JMP FWD_EXIT  ; return
            
NO_FWD_BUMP BRSET PORTAD0,$08,NO_REV_BUMP ; If REAR_BUMP, then we should stop
            JSR INIT_ALL_STP ; so initialize the ALL_STOP state
            MOVB #ALL_STP_ST, CRNT_STATE ; and change state to ALL_STOP
            JMP FWD_EXIT ; and return
            
NO_REV_BUMP  LDAA SENSOR_BOW
	     ADDA VAR_BOW
	     CMPA BASE_BOW
	     BPL NO_ALIGN
	     
	     LDAA SENSOR_MID
	     ADDA VAR_MID
	     CMPA BASE_MID
	     BPL NO_ALIGN
             
	     LDAA SENSOR_LINE
	     ADDA VAR_LINE
	     CMPA BASE_LINE
	     BMI GO_RIGHT_ALIGN
	     
NO_ALIGN     LDAA SENSOR_PORT
	     ADDA VAR_PORT
	     CMPA BASE_PORT
	     BPL PART_LEFT_TURN
	     BMI NO_PART_DET
	     
NO_PORT_DET  LDAA SENSOR_BOW
	     ADDA VAR_BOW
	     CMPA BASE_BOW
	     BPL EXIT
	     BMI NO_BOW_DET
	     
NO_BOW_DET   LDAA SENSOR_STBD
	     ADDA VAR_STBD
	     CMPA BASE_STBD
	     BPL EXIT
	     BMI NO_BOW_DET
	     
LT_TURN       LDAA  NEXT_D   ; Push direction for the previous
              PSHA      ; Intersection to the stack pointer
              LDAA  SEC_PTH_INT ; Then store direction taken to NEXT_D
              STAA  NEXT_D 
              JSR   INIT_LT_TRN  ; The robot should make a LEFT turn
              MOVB  #LT_TRN,CRNT_STATE ; Initialize the LT_TRN state
              JMP FWD_EXIT             

RT_TURN       LDAA  NEXT_D   ; Push direction for the previous
              PSHA      ; Intersection to the stack pointer
              LDAA  SEC_PTH_INT ; Then store direction taken to NEXT_D
              STAA  NEXT_D 
              JSR   INIT_RT_TRN  ; The robot should make a RIGHT turn
              MOVB  #RT_TRN,CRNT_STATE ; Initialize the RT_TRN state
              JMP FWD_EXIT
	      
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

INIT_ALL_STP    BCLR  PTT,%00110000  ; Turn off the drive motors
                RTS
                                                     
;***************************************************************************;
;			   Motor control				    ;
;***************************************************************************;

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

;***************************************************************************;
;			  Initialize the ADC				    ;
;***************************************************************************;

openADC     MOVB #$80,ATDCTL2 ; Turn on ADC (ATDCTL2 @ $0082)
            LDY #1            ; Wait for 50 us for ADC to be ready
            JSR del_50us ; - " -
            MOVB #$20,ATDCTL3 ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
            MOVB #$97,ATDCTL4 ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
            RTS
            
; Clear LCD Buffer

CLR_LCD_BUF LDX #CLEAR_LINE
            LDY #TOP_LINE
            JSR STRCPY
CLB_SECOND  LDX #CLEAR_LINE
            LDY #BOT_LINE
            JSR STRCPY
CLB_EXIT    RTS

; String Copy

STRCPY      PSHX            ; Protect the registers used
            PSHY
            PSHA
STRCPY_LOOP LDAA 0,X        ; Get a source character
            STAA 0,Y        ; Copy it to the destination
            BEQ STRCPY_EXIT ; If it was the null, then exit
            INX             ; Else increment the pointers
            INY
            BRA STRCPY_LOOP ; and do it again
STRCPY_EXIT PULA            ; Restore the registers
            PULY
            PULX
            RTS             

; Guider LEDs ON

G_LEDS_ON   BSET PORTA,%00100000 ; Set bit 5
            RTS
            
; Guider LEDs OFF

G_LEDS_OFF  BCLR PORTA,%00100000 ; Clear bit 5
            RTS
 
;***************************************************************************;
;			  Read Sensors					    ;
;***************************************************************************;

READ_SENSORS  CLR SENSOR_NUM       ; Select sensor number 0
              LDX #SENSOR_LINE     ; Point at the start of the sensor array
              
RS_MAIN_LOOP  LDAA SENSOR_NUM      ; Select the correct sensor input
              JSR SELECT_SENSOR    ; on the hardware
              LDY #400             ; 20 ms delay to allow the
              JSR del_50us         ; sensor to stabilize
              LDAA #%10000001      ; Start A/D conversion on AN1
              STAA ATDCTL5
              BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
              LDAA ATDDR0L         ; A/D conversion is complete in ATDDR0L
              STAA 0,X             ; so copy it to the sensor register
              CPX #SENSOR_STBD     ; If this is the last reading
              BEQ RS_EXIT          ; Then exit
              INC SENSOR_NUM       ; Else, increment the sensor number
              INX                  ; and the pointer into the sensor array
              BRA RS_MAIN_LOOP     ; and do it again
              
RS_EXIT       RTS   

;***************************************************************************;
;			  Select Sensors				    ;
;***************************************************************************;

SELECT_SENSOR PSHA            ; Save the sensor number for the moment
              LDAA PORTA      ; Clear the sensor selection bits to zeros
              ANDA #%11100011 ;
              STAA TEMP       ; and save it into TEMP
              PULA            ; Get the sensor number
              ASLA            ; Shift the selection number left, twice
              ASLA 
              ANDA #%00011100 ; Clear irrelevant bit positions
              ORAA TEMP       ; OR it into the sensor bit positions
              STAA PORTA      ; Update the hardware
              RTS         
 
;***************************************************************************;
;			  Display Sensors Readings			    ;
;***************************************************************************;

DP_FRONT_SENSOR EQU TOP_LINE+3
DP_PORT_SENSOR  EQU BOT_LINE+0
DP_MID_SENSOR   EQU BOT_LINE+3
DP_STBD_SENSOR  EQU BOT_LINE+6
DP_LINE_SENSOR  EQU BOT_LINE+9
DISPLAY_SENSORS LDAA SENSOR_BOW      ; Get the FRONT sensor value

                JSR BIN2ASC          ; Convert to ascii string in D
                LDX #DP_FRONT_SENSOR ; Point to the LCD buffer position
                STD 0,X              ; and write the 2 ascii digits there
                LDAA SENSOR_PORT     ; Repeat for the PORT value
                JSR BIN2ASC
                LDX #DP_PORT_SENSOR
                STD 0,X
                LDAA SENSOR_MID      ; Repeat for the MID value
                JSR BIN2ASC
                LDX #DP_MID_SENSOR
                STD 0,X
                LDAA SENSOR_STBD     ; Repeat for the STARBOARD value
                JSR BIN2ASC
                LDX #DP_STBD_SENSOR
                STD 0,X
                LDAA SENSOR_LINE     ; Repeat for the LINE value
                JSR BIN2ASC
                LDX #DP_LINE_SENSOR
                STD 0,X
                LDAA #CLEAR_HOME     ; Clear the display and home the cursor
                JSR cmd2LCD          ; "
                LDY #40              ; Wait 2 ms until "clear display" command is complete
                JSR del_50us
                LDX #TOP_LINE        ; Now copy the buffer top line to the LCD
                JSR putsLCD
                LDAA #LCD_SEC_LINE   ; Position the LCD cursor on the second line
                JSR LCD_POS_CRSR
                LDX #BOT_LINE        ; Copy the buffer bottom line to the LCD
                JSR putsLCD
                RTS

;***************************************************************************;
;			 Binary to ASCII				    ;
;***************************************************************************;
                
HEX_TABLE       FCC '0123456789ABCDEF' ; Table for converting values

BIN2ASC         PSHA ; Save a copy of the input number on the stack
                TAB ; and copy it into ACCB
                ANDB #%00001111 ; Strip off the upper nibble of ACCB
                CLRA ; D now contains 000n where n is the LSnibble
                ADDD #HEX_TABLE ; Set up for indexed load
                XGDX
                LDAA 0,X ; Get the LSnibble character
                PULB ; Retrieve the input number into ACCB
                PSHA ; and push the LSnibble character in its place
                RORB ; Move the upper nibble of the input number
                RORB ; into the lower nibble position.
                RORB
                RORB
                ANDB #%00001111 ; Strip off the upper nibble
                CLRA ; D now contains 000n where n is the MSnibble
                ADDD #HEX_TABLE ; Set up for indexed load
                XGDX
                LDAA 0,X ; Get the MSnibble character into ACCA
                PULB ; Retrieve the LSnibble character into ACCB
                RTS
                
;***************************************************************************;
;			   utility subroutine				    ;
;***************************************************************************;
; Initialize the LCD

openLCD         LDY #2000 ; Wait 100 ms for LCD to be ready
                JSR del_50us ; "
                LDAA #INTERFACE ; Set 8-bit data, 2-line display, 5x8 font
                JSR cmd2LCD ; "
                LDAA #CURSOR_OFF ; Display on, cursor off, blinking off
                JSR cmd2LCD ; "
                LDAA #SHIFT_OFF ; Move cursor right (address increments, no char. shift)
                JSR cmd2LCD ; "
                LDAA #CLEAR_HOME ; Clear the display and home the cursor
                JSR cmd2LCD ; "
                LDY #40 ; Wait 2 ms until "clear display" command is complete
                JSR del_50us ; "
                RTS
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

del_50us        PSHX ; (2 E-clk) Protect the X register
eloop           LDX #300 ; (2 E-clk) Initialize the inner loop counter
iloop           NOP ; (1 E-clk) No operation
                DBNE X,iloop ; (3 E-clk) If the inner cntr not 0, loop again
                DBNE Y,eloop ; (3 E-clk) If the outer cntr not 0, loop again
                PULX ; (3 E-clk) Restore the X register
                RTS ; (5 E-clk) Else return  

;*******************************************************************

cmd2LCD:    BCLR LCD_CNTR,LCD_RS ; select the LCD Instruction Register (IR)
            JSR dataMov ; send data to IR
            RTS
	    	    
;*******************************************************************

putcLCD     BSET LCD_CNTR,LCD_RS ; select the LCD Data register (DR)
            JSR dataMov ; send data to DR
            RTS    
	    
;*******************************************************************

putsLCD     LDAA 1, X+ ; get one character from the string
            BEQ donePS ; reach NULL character?
            JSR putcLCD
            BRA putsLCD
            
donePS      RTS

;*******************************************************************

dataMov         BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
                STAA LCD_DAT ; send the 8 bits of data to LCD
                NOP
                NOP
                NOP
                BCLR LCD_CNTR,LCD_E ; pull the E signal low to complete the write operation
                LDY #1 ; adding this delay will complete the internal
                JSR del_50us ; operation for most instructions
                RTS 

;*******************************************************************
;Position the cursor

LCD_POS_CRSR    ORAA #%10000000 ; Set the high bit of the control word
                JSR cmd2LCD ; and set the cursor address
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
            
;*******************************************************************;
;        Update Display (Battery Voltage + Current State) 	    ;
;*******************************************************************;

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
	    
ISR_A	   MOVB #$01, TFLG1 ; initialize input capture for interrupt
	       INC ISR_CNT1 ; increment the first counter
	       RTI ; return to normal program execution

ISR_B	   MOVB #$02, TFLG1 ; initialize input capture for interrupt
	       INC ISR_CNT2  ; increment the second counter
	       RTI ; return to normal program execution
	   
;*******************************************************************;
; 			Interrupt Vectors 			    ;
;*******************************************************************;
            ORG $FFFE
            DC.W Entry ; Reset Vector
            
	    ORG $FFEE
	    DC.W ISR_A ; allocation of the first interrupt routine
	    
	    ORG $FFEC
	    DC.W ISR_B ; allocation of the second interrupt routine
	    
            ORG $FFDE
            DC.W TOF_ISR ; Timer Overflow Interrupt Vector
