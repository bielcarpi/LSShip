; Sistemes Digitals i Microprocessadors
; Curs 2022-23
; P2 FA - LSShip
; Biel Carpi i Alex Cano
LIST P=PIC18F4321 F=INHX32
#include <p18f4321.inc>
CONFIG  OSC=HSPLL  ;L?oscil.lador extern a 40MHz
CONFIG  PBADEN=DIG ;Volem que el PORTB sigui DIGital
CONFIG  WDT=OFF    ;Desactivem el WatchDog Timer


ORG 0x0000
GOTO    MAIN
ORG 0x0008
GOTO    INT_TMR0    ;Interruptions are only caused by TMR0
ORG 0x0018
RETFIE  FAST


;----------------- DEFINICIO VARIABLES RAM ---------------------
MODE    EQU 0X00 ;Bit 0: Manual, Bit 1: Creuer, Bit 2: Record, Bit 3: Pilot Auto, Bit 4: Enrere/!Endavant
    MANUAL  EQU 0
    CREUER  EQU 1
    RECORD  EQU 2
    AUTO    EQU 3
CURRENT_JOYSTICK_Y  EQU 0x01    ;Current value of Joystick Y (0 to 6). Middle is 3
CURRENT_JOYSTICK_Y_AUX  EQU 0x02
CURRENT_JOYSTICK_X  EQU 0x03    ;Current value of Joystick X (0 to 8). Middle is 4
CURRENT_JOYSTICK_X_AUX  EQU 0x04
COUNTER_REBOTS  EQU 0x05
COUNTER_REBOTS2 EQU 0x06
COUNTER_PWM_SERVO   EQU 0X07
COUNTER_TMR0_SERVO    EQU 0x08
COUNTER_TMR0_DC    EQU 0x20
FLAGS   EQU 0x09
    ENRERE  EQU 0               ;Whether we're going backwards or not
    RECORD_BUTTON_PUSHED   EQU 1    ;Whether the Record Button has been pushed
    DELAY_DONE   EQU 2          ;Whether the delay has been reached
    ALARM_ACTIVE EQU 3          ;Whether the alarm is active or not
NUMBER_SAVES    EQU 0x10
NUMBER_SAVES_AUX    EQU 0x11
DELAY_HIGH   EQU 0x12
DELAY_LOW   EQU 0x13
RECORD_BUTTON_TIME  EQU 0x14
CURRENT_DELAY_LOW   EQU 0x15
CURRENT_DELAY_HIGH  EQU 0x16
ALARM_TIMER_HIGH    EQU 0x17
ALARM_TIMER_LOW     EQU 0x18
ALARM_TIMER_COUNTER EQU 0x19    ;To count 0.5s which will make the alarm blink




;----------------- TAULES FLASH ---------------------
TAULA_7SEG  EQU 0x30
TAULA_PWM_DC    EQU 0x40
TAULA_LEDS  EQU 0x50

ORG TAULA_7SEG
DB b'10110101', b'01110101', b'00001101', b'01111101' ;H-M-L-0
DB b'00001101', b'01110101', b'10110101', b'00000000' ;L-M-H

ORG TAULA_PWM_DC
DB .4, .3, .2, .0   ;Ex. Step 0: S'encen quan COUNTER_TMR0 es mes petit que 4 (3 cops)
DB .2, .3, .4, .0   ;     Step 4: S'encen quan COUNTER_TMR0 es mes petit que 2 (nomes 1 cop)

ORG TAULA_LEDS
DB b'10000000', b'01000000', b'00100000', b'00010000'
DB b'00000000', b'00001000', b'00000100', b'00000010'
DB b'00000001', b'00000000'


INIT_VARS
   ;-------------------------------
   ;INICIALITZACIO DE VARIABLES
   ;-------------------------------
    CLRF    MODE,0
    CLRF    COUNTER_REBOTS,0
    CLRF    COUNTER_REBOTS2,0
    MOVLW   .4
    MOVWF   COUNTER_TMR0_SERVO,0
    MOVWF   COUNTER_TMR0_DC,0
    CLRF    NUMBER_SAVES,0
    CALL    CLEAR_BANK1
    CLRF    RECORD_BUTTON_TIME,0
    CLRF    COUNTER_PWM_SERVO,0
    CLRF    FLAGS,0
    CLRF    DELAY_HIGH,0
    CLRF    DELAY_LOW,0
    CLRF    ALARM_TIMER_HIGH,0
    CLRF    ALARM_TIMER_LOW,0
    CLRF    ALARM_TIMER_COUNTER,0
    RETURN

INIT_PORTS
   ;-------------------------------
   ;CONFIGURACIÓ DE PORTS
   ;-------------------------------

   ;BOTONS
   ;----------------------------------------------------------------------
   BSF   TRISB,0,0  ;Configurem el port RB0 d'entrada botó SAVE.
   BSF   TRISB,1,0  ;Configurem el port RB1 d'entrada botó MANUAL_MODE.
   BSF   TRISB,2,0  ;Configurem el port RB2 d'entrada botó RECORD_MODE.
   BCF   INTCON2,RBPU,0 ;Activem els pull-ups del port B.

   ;LEDS
   ;---------------------------------------------------------------------------
   BCF   TRISE,0,0  ;Configurem RGB COLOR RED
   BCF   TRISE,1,0  ;Configurem RGB COLOR BLUE
   BCF   TRISE,2,0  ;Configurem RGB COLOR GREEN
   CLRF  TRISD,0	;Configurem el 7Seg de sortida per al CURRENTSPEED[6..0].
   CLRF  TRISC,0    ;Barra de LEDs els [7..0]
   BCF   TRISB,3,0  ;Barra de LEDs [8][9]
   BCF   TRISA,4,0  ;Configurem LED VERMELL ALARMA

   ;MOTORS
   ;---------------------------------------------------------------------------
   BCF   TRISA,2,0  ;PWMDir (servo)
   BCF   TRISA,3,0  ;PWMSpeed[0] (Motor DC PWM Endarrere)
   BCF   TRISA,5,0  ;PWMSpeed[1] (Motor DC PWM Endavant) 

    ;CHECKINGS
   ;---------------------------------------------------------------------------
   BCF   TRISB,7,0  ;HIGH when recording mode runs
   BCF   TRISB,6,0  ;HIGH when auto mode runs
   BCF   LATB,7,0  ;HIGH when recording mode runs
   BCF   LATB,6,0  ;HIGH when auto mode runs
   ;This will allow us to compare recording time and automatic mode time

   ;JOYSTICK
   ;---------------------------------------------------------------------------
   BSF   TRISA,0,0  ;Eix X
   BSF   TRISA,1,0  ;Eix Y
   RETURN

INIT_INT
   ;-------------------------------
   ;CONFIGURACIO INTERRUPCIONS
   ;-------------------------------
   BSF      INTCON2,TMR0IP,0    ;TMR0 per flanc de pujada
   BCF	    RCON,IPEN,0	    ;Desactivem les prioritats
   MOVLW    b'11100000'
   MOVWF    INTCON,0	    ;Habilitem interrupcions globals i el TMR0
   RETURN

INIT_ADC
   ;-------------------------------
   ;CONFIGURACIO ADC
   ;-------------------------------
    MOVLW   b'00000001' ;Select AN0 & Enable A/D converter
    MOVWF   ADCON0,0
    MOVLW   b'00001101' ;PIC voltage reference & Enable AN0,AN1
    MOVWF   ADCON1,0
    MOVLW   b'00001100' ;Left justified 
    MOVWF   ADCON2,0
    RETURN

INIT_TMR0
   ;-------------------------------
   ;CONFIGURACIO TIMER0
   ;-------------------------------
    MOVLW   b'11000111'	 ; Configurem el TIMER0
    MOVWF   T0CON,0      
    CALL    CARREGA_TIMER ; Carreguem el TIMER0 per anar a 5ms
    RETURN
CARREGA_TIMER
    BCF	    INTCON,TMR0IF,0  ; Netejem el bit de causa d'interrupció
    MOVLW   .61	         ; 256-61 * (0.1us * 256 prescaler) = 5ms
    MOVWF   TMR0L,0  
    RETURN


INT_TMR0
    BTFSS   INTCON,TMR0IF,0 ;Si la interrupcio no es del TMR0, sortim
    RETFIE  FAST
    CALL    CONTA_RECORD_1S
    CALL    CONTA_DELAY_RECORDING
    CALL    CHECK_ALARM_TIMER
    CALL    SET_LEDS_X
    CALL    SET_LEDS_Y
    CALL    PWM_DC
    DCFSNZ  COUNTER_TMR0_DC,1,0
    CALL    LOAD_COUNTER_TMR0_DC
    CLRF    WREG,0
    CPFSEQ  COUNTER_TMR0_SERVO,0 ;Esperem 4 cops (20ms) per donar un altre pols al servo  
    DECF    COUNTER_TMR0_SERVO,1,0    
    CALL    CARREGA_TIMER   ;Tornem a carregar TMR0
    RETFIE  FAST

CHECK_ALARM_TIMER
    BTFSC   MODE,AUTO,0             ;If we're in mode auto or record, don't check alarm timer
    RETURN
    BTFSC   MODE,RECORD,0
    RETURN
    BTFSC   FLAGS,ALARM_ACTIVE,0    ;If Alarm already active, update alarm blinking and exit
    GOTO    ALARM_BLINK  
    INCF    ALARM_TIMER_LOW,1,0
    SETF    WREG,0
    BTFSC   STATUS,C,0
    INCF    ALARM_TIMER_HIGH,1,0
    ;If ALARM_TIMER_HIGH > 1min, ALARM ON
    MOVLW   .47     ;Each ALARM_TIMER_HIGH is 1,28s * 47 = 60,08s
    CPFSEQ  ALARM_TIMER_HIGH,0
    RETURN
    BSF     FLAGS,ALARM_ACTIVE,0
    RETURN
ALARM_BLINK
    INCF    ALARM_TIMER_COUNTER,1,0
    MOVLW   .99     ;Each ALARM_TIMER_COUNTER is 5ms * 100 = 500ms (0.5s)
    CPFSGT  ALARM_TIMER_COUNTER,0
    RETURN
    BTG     PORTA,4,0
    CLRF    ALARM_TIMER_COUNTER,0
    RETURN
    
CONTA_DELAY_RECORDING
    INCF    DELAY_LOW,1,0
    BTFSC   STATUS,C,0      ;If carry == 0 --> CALL, else carry == 1 --> DELAY_HIGH++
    INCF    DELAY_HIGH,1,0
    CALL    CHECK_DELAY_DONE
    CALL    CHECK_DELAY_1MIN
    RETURN

CHECK_DELAY_DONE
    MOVF    CURRENT_DELAY_HIGH,0   
    CPFSEQ  DELAY_HIGH,0    ;Compare delays. If delay_high not yet as big as current_delay_high, skip
    RETURN
    MOVF    CURRENT_DELAY_LOW,0   
    CPFSEQ  DELAY_LOW,0    ;Compare delays. If delay_low not yet as big as current_delay_low, skip
    RETURN
    BSF     FLAGS,DELAY_DONE,0
    RETURN

CHECK_DELAY_1MIN
    BTFSS   MODE,RECORD,0       ;If we're in mode record, check if DELAY_HIGH > 1MIN
    RETURN
    MOVLW   .47                 ;Each DELAY_HIGH is 1.28s * 47 = 60,08s
    CPFSEQ  DELAY_HIGH,0        ;If DELAY_HIGH == 1min, force exit recording
    RETURN
    CALL    EXIT_RECORD_MODE
    RETURN

CONTA_RECORD_1S
    BTFSS   FLAGS,RECORD_BUTTON_PUSHED,0    ;If record button not pushed, skip
    RETURN
    MOVLW   .200
    CPFSGT  RECORD_BUTTON_TIME,0        ;If record button time > 200, stop RECORD_BUTTON_TIME++
    INCF    RECORD_BUTTON_TIME,1,0
    RETURN


SET_LEDS_X
    MOVLW   .4
    CPFSEQ  CURRENT_JOYSTICK_X,0    ;Check if joystick X is 3
    GOTO    SET_LEDS_X_PORTC        ;If so, set leds port c
    CLRF    LATC,0                  ;If not, port c off and latb3 on
    BSF     LATB,3,0
    RETURN
SET_LEDS_X_PORTC
    BCF     LATB,3,0
    CLRF    TBLPTRU,0
    CLRF    TBLPTRH,0
    MOVLW   TAULA_LEDS
    ADDWF   CURRENT_JOYSTICK_X,0,0
    MOVWF   TBLPTRL,0
    TBLRD*
    MOVFF   TABLAT,LATC
    RETURN

SET_LEDS_Y
    CALL    SET_LEDS_Y_ALARM
    CLRF    TBLPTRU,0
    CLRF    TBLPTRH,0
    MOVLW   TAULA_7SEG
    ADDWF   CURRENT_JOYSTICK_Y,0,0
    MOVWF   TBLPTRL,0
    TBLRD*
    MOVFF   TABLAT,LATD
    RETURN
SET_LEDS_Y_ALARM
    MOVLW   .2
    CPFSGT  CURRENT_JOYSTICK_Y,0    ;If greater than 2 (2, 1 or 0), alarm ON
    GOTO    BACKWARDS_ON
    BCF     FLAGS,ENRERE,0
    BTFSC   FLAGS,ALARM_ACTIVE,0     ;If alarm active, don't deactivate backwards led
    RETURN
    BCF     LATA,4,0
    RETURN
BACKWARDS_ON
    BTFSC   FLAGS,ALARM_ACTIVE,0     ;If alarm active, don't activate backwards led
    RETURN
    BSF     LATA,4,0
    BSF     FLAGS,ENRERE,0
    RETURN
    

ESPERA_REBOTS_16_MILIS
  MOVLW .201
  MOVWF COUNTER_REBOTS2,0
  BUCLE_16_MILIS
    MOVLW .132
    MOVWF COUNTER_REBOTS,0
    BUCLE_16_MILIS2
        NOP
        NOP
        NOP
        DECFSZ  COUNTER_REBOTS,1,0
        GOTO    BUCLE_16_MILIS2
        DECFSZ  COUNTER_REBOTS2,1,0
        GOTO    BUCLE_16_MILIS
RETURN

ESPERA_250us
    MOVLW .255 ; Cargar el valor de COUNTER_PWM_SERVO para una espera de 250us
    MOVWF COUNTER_PWM_SERVO, 0
    LOOP_ESPERA_ACTIVA
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        DECFSZ COUNTER_PWM_SERVO, 1, 0
        GOTO LOOP_ESPERA_ACTIVA
    RETURN

PWM_SERVO
    ;HEM DE VEURE LA QUANTITAT DE ESPERES QUE HEM DE FER PER A LA POSICION DEL SERVO --> 0.5ms == 0º -- 2.5ms == 180º
    ; 180º / 8 direccions == 22.5º cada direccio
    ; 2000us entre els 180º -> 2000/8 == 250us
    MOVLW   .4
    MOVWF   COUNTER_TMR0_SERVO,0
    BSF     LATA, 2, 0      ;posem a 1 la sortida PW3
    MOVLW   .2          ;Minim 2 esperes en el step 0 (500us)
    MOVFF   CURRENT_JOYSTICK_X,CURRENT_JOYSTICK_X_AUX
    ADDWF   CURRENT_JOYSTICK_X_AUX,1,0
    LOOP_ESPERA_SERVO
        CALL ESPERA_250us 
        DECFSZ CURRENT_JOYSTICK_X_AUX, 1, 0  ;Decrementem els cops que ho ha de fer depenent del grau
        GOTO LOOP_ESPERA_SERVO 
    BCF LATA, 2, 0      ;posem a 0 la sortida
    RETURN

PWM_DC
    ;COUNTER_TMR0 can be 4,3,2 or 1. L -> 1 ON, M -> 1 2 ON, H -> 1 2 3 ON. 
    CLRF    TBLPTRU,0
    CLRF    TBLPTRH,0
    MOVLW   TAULA_PWM_DC
    ADDWF   CURRENT_JOYSTICK_Y,0,0
    MOVWF   TBLPTRL,0
    TBLRD*
    MOVF    TABLAT,0
    CPFSLT  COUNTER_TMR0_DC,0        ;Comparem W amb Taula_pwm_dc  ----   COUNTER_TMR0 < TABLAT?
    GOTO    TURN_OFF_DC
    GOTO    TURN_ON_DC
    RETURN
LOAD_COUNTER_TMR0_DC
    MOVLW   .4
    MOVWF   COUNTER_TMR0_DC,0
    RETURN
TURN_ON_DC
    BTFSS   FLAGS,ENRERE,0
    BSF     LATA,3,0
    BTFSC   FLAGS,ENRERE,0
    BSF     LATA,5,0
    RETURN
TURN_OFF_DC
    BCF     LATA,3,0
    BCF     LATA,5,0
    RETURN


CHECK_BUTTONS
    BTFSS   PORTB,0,0   ;Check if RB0 is pressed
    CALL    ONCLICK_MANUAL
    BTFSS   PORTB,1,0   ;Check if RB1 is pressed
    CALL    ONCLICK_RECORD
    BTFSS   PORTB,2,0   ;Check if RB2 is pressed
    CALL    ONCLICK_SAVE
    RETURN

ONCLICK_MANUAL
    CALL    CLEAR_ALARM_TIMER
    BTFSC   MODE,RECORD,0   ;If we're in record, skip
    RETURN
    CALL    ESPERA_REBOTS_16_MILIS
    BTFSC   PORTB,0,0       ;If it's not clicked, return
    RETURN
    BTFSC   MODE,MANUAL,0   ;If manual is ON
    GOTO    MODE_CREUER     ;Start mode creuer
    GOTO    MODE_MANUAL     ;Else, start mode manual

ONCLICK_RECORD
    CALL    CLEAR_ALARM_TIMER
    BTFSC   MODE,CREUER,0   ;If we're in creuer, skip
    RETURN
    CALL    ESPERA_REBOTS_16_MILIS
    BTFSC   PORTB,1,0       ;If it's not clicked, return
    RETURN
    BTFSC   MODE,RECORD,0
    GOTO    EXIT_RECORD_MODE
    BSF     FLAGS,RECORD_BUTTON_PUSHED,0
    GOTO    MODE_AUTOMATIC     ;Check if button is pressed more or less than 1s

ONCLICK_SAVE
    CALL    CLEAR_ALARM_TIMER
    BTFSS   MODE,RECORD,0   ;If we're not recording, skip
    RETURN
    CALL    ESPERA_REBOTS_16_MILIS
    BTFSC   PORTB,2,0       ;If it's not clicked, return
    RETURN
    CALL    SAVE_RECORDING
    GOTO    LOOP_SORTIDA_RB2
SAVE_RECORDING
    ;Save the recording
    MOVLW   .4
    MULWF   NUMBER_SAVES,0
    MOVFF   PRODL,FSR0L     ;Posicionem punter low a 4*NUMBER_SAVES + 2 (necessitem aquests 2 per guardar els primers valors de X i Y al entrar a recording)
    INCF    FSR0L,1,0   ; +1
    INCF    FSR0L,1,0   ; +1
    MOVLW   0x01
    MOVWF   FSR0H,0     ;Posicionem punters al banc 1
    MOVFF   DELAY_LOW,POSTINC0
    MOVFF   DELAY_HIGH,POSTINC0
    MOVFF   CURRENT_JOYSTICK_X,POSTINC0
    MOVFF   CURRENT_JOYSTICK_Y,POSTINC0
    CLRF    DELAY_LOW
    CLRF    DELAY_HIGH
    INCF    NUMBER_SAVES,1,0
    ;Check if saves == 30. If so, we'll act like we've clicked record button to exit the recording
    MOVLW   .30
    CPFSLT  NUMBER_SAVES,0
    GOTO    EXIT_RECORD
    RETURN

MODE_MANUAL
    BCF     LATB,7,0               ;End record checking
    CLRF    MODE,0
    BSF     MODE,MANUAL,0
    BSF     LATE,0,0
    BSF     LATE,1,0
    BCF     LATE,2,0
    CALL    CLEAR_ALARM_TIMER
    GOTO    LOOP_SORTIDA_RB0

MODE_CREUER
    CLRF    MODE,0
    BSF     MODE,CREUER,0
    BSF     LATE,0,0
    BCF     LATE,1,0
    BSF     LATE,2,0
    GOTO    LOOP_SORTIDA_RB0

MODE_AUTOMATIC
    CALL    LOOP_SORTIDA_RB1
    MOVLW   .199                    ;Mirem si record button time > 199 (5ms * 200 = 1s)
    CPFSGT  RECORD_BUTTON_TIME,0    ;Si es mes petit, mode record. Sino, mode automatic
    GOTO    MODE_RECORD             
    CLRF    MODE,0
    BSF     MODE,AUTO,0
    BCF     LATE,0,0
    BCF     LATE,1,0
    BCF     LATE,2,0
    BCF     FLAGS,RECORD_BUTTON_PUSHED,0
    CLRF    RECORD_BUTTON_TIME
    BSF     LATB,6,0               ;Start auto checking
    CALL    START_AUTOMATIC
    BCF     LATB,6,0               ;End auto checking
    GOTO    MODE_MANUAL

MODE_RECORD
    CLRF    MODE,0
    BSF     MODE,RECORD,0
    BCF     LATE,0,0
    BSF     LATE,1,0
    BSF     LATE,2,0
    BCF     FLAGS,RECORD_BUTTON_PUSHED,0
    CLRF    RECORD_BUTTON_TIME
    CALL    CLEAR_BANK1
    CLRF    NUMBER_SAVES
    ;Guardem els primers valors de JoystickX i Y al entrar al mode record
    CLRF    FSR0L,0
    MOVLW   0x01
    MOVWF   FSR0H,0     ;Posicionem punters a la primera @ del banc 1
    MOVFF   CURRENT_JOYSTICK_X,POSTINC0
    MOVFF   CURRENT_JOYSTICK_Y,POSTINC0
    CLRF    DELAY_LOW,0
    CLRF    DELAY_HIGH,0
    BSF     LATB,7,0               ;Start record checking
    RETURN

LOOP_SORTIDA_RB0
    BTFSS   PORTB,0,0
    GOTO    LOOP_SORTIDA_RB0
    RETURN
LOOP_SORTIDA_RB1
    BTFSS   PORTB,1,0
    GOTO    LOOP_SORTIDA_RB1
    RETURN
LOOP_SORTIDA_RB2
    BTFSS   PORTB,2,0
    GOTO    LOOP_SORTIDA_RB2
    RETURN

EXIT_RECORD_MODE
    ;Check if num_saves is > 0.
    CLRF    WREG,0
    CPFSGT  NUMBER_SAVES,0      ;If we have 0 saves, don't save the last one and exit record mode
    GOTO    EXIT_RECORD
    CALL    SAVE_RECORDING
EXIT_RECORD
    CALL    MODE_MANUAL
    CALL    LOOP_SORTIDA_RB1
    RETURN

START_AUTOMATIC
    CLRF    WREG,0
    CPFSGT  NUMBER_SAVES,0      ;Go to manual mode again if there is no recording
    GOTO    MODE_MANUAL
    ;Hem guardat de 4 en 4: JoysX, JoysY, Delay Low, Delay High
    CLRF    FSR0L,0
    MOVLW   0x01
    MOVWF   FSR0H,0     ;Posicionem punters a la primera @ del banc 1
    MOVFF   NUMBER_SAVES,NUMBER_SAVES_AUX
    READ_NEXT_POSITION
        MOVFF   POSTINC0,CURRENT_JOYSTICK_X
        MOVFF   POSTINC0,CURRENT_JOYSTICK_Y
        MOVFF   POSTINC0,CURRENT_DELAY_LOW
        MOVFF   POSTINC0,CURRENT_DELAY_HIGH
        CALL    WAIT_DELAY
        DECFSZ  NUMBER_SAVES_AUX,1,0        ;Decrement until we've looped through all positions
        GOTO    READ_NEXT_POSITION
        RETURN

WAIT_DELAY
    CLRF    DELAY_LOW
    CLRF    DELAY_HIGH
    BCF     FLAGS,DELAY_DONE,0
    WAIT
        CLRF    WREG,0
        CPFSGT  COUNTER_TMR0_SERVO,0  ;If counter timer0 is 0, pwm servo
        CALL    PWM_SERVO
        BTFSS   FLAGS,DELAY_DONE,0
        GOTO    WAIT
        RETURN

CLEAR_BANK1
    CLRF    FSR0L,0
    MOVLW   0x01
    MOVWF   FSR0H,0     ;Posicionem punters
    CLRF    WREG,0
    LOOP_RAM
        CLRF    POSTINC0,0
        INCF    WREG,0,0
        BTFSS   STATUS,C,0 
        GOTO    LOOP_RAM
        RETURN


CHECK_JOYSTICK_X
    BCF     ADCON0,CHS0,0   ;Select AN3
    CALL    ESPERA_ADC      ;Wait for ADC to convert
    CLRF    CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .5
    CPFSGT  ADRESH,0        ;Return if we're in step 0
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .1
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .28
    CPFSGT  ADRESH,0        ;Return if we're in step 1
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .2
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .69
    CPFSGT  ADRESH,0        ;Return if we're in step 2
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .3
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .110
    CPFSGT  ADRESH,0        ;Return if we're in step 3
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .4
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .145
    CPFSGT  ADRESH,0        ;Return if we're in step 4
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .5
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .180
    CPFSGT  ADRESH,0        ;Return if we're in step 5
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .6
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .210
    CPFSGT  ADRESH,0        ;Return if we're in step 6
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .7
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    MOVLW   .250
    CPFSGT  ADRESH,0        ;Return if we're in step 7
    GOTO    RETURN_JOYSTICK_X
    MOVLW   .8
    MOVWF   CURRENT_JOYSTICK_X_AUX,0
    GOTO    RETURN_JOYSTICK_X     ;Return in step 8
RETURN_JOYSTICK_X
    MOVF    CURRENT_JOYSTICK_X_AUX,0  ;Compare last value of joystick x with new one
    CPFSEQ  CURRENT_JOYSTICK_X,0    ;If they aren't equal, clear alarm timer
    CALL    CLEAR_ALARM_TIMER
    MOVFF   CURRENT_JOYSTICK_X_AUX,CURRENT_JOYSTICK_X
    RETURN

CHECK_JOYSTICK_Y
    BSF     ADCON0,CHS0,0   ;Select AN1
    CALL    ESPERA_ADC      ;Wait for ADC to convert
    CLRF    CURRENT_JOYSTICK_Y_AUX,0
    MOVLW   .4
    CPFSGT  ADRESH,0    ;Return if we're in step 0
    GOTO    RETURN_JOYSTICK_Y
    MOVLW   .1
    MOVWF   CURRENT_JOYSTICK_Y_AUX,0
    MOVLW   .55
    CPFSGT  ADRESH,0    ;Return if we're in step 1
    GOTO    RETURN_JOYSTICK_Y
    MOVLW   .2
    MOVWF   CURRENT_JOYSTICK_Y_AUX,0
    MOVLW   .120
    CPFSGT  ADRESH,0    ;Return if we're in step 2
    GOTO    RETURN_JOYSTICK_Y
    MOVLW   .3
    MOVWF   CURRENT_JOYSTICK_Y_AUX,0
    MOVLW   .136
    CPFSGT  ADRESH,0    ;Return if we're in step 3
    GOTO    RETURN_JOYSTICK_Y
    MOVLW   .4
    MOVWF   CURRENT_JOYSTICK_Y_AUX,0
    MOVLW   .201
    CPFSGT  ADRESH,0    ;Return if we're in step 4
    GOTO    RETURN_JOYSTICK_Y
    MOVLW   .5
    MOVWF   CURRENT_JOYSTICK_Y_AUX,0
    MOVLW   .250
    CPFSGT  ADRESH,0    ;Return if we're in step 5
    GOTO    RETURN_JOYSTICK_Y
    MOVLW   .6
    MOVWF   CURRENT_JOYSTICK_Y_AUX,0
    GOTO    RETURN_JOYSTICK_Y              ;Return if we're in step 6
RETURN_JOYSTICK_Y
    MOVF    CURRENT_JOYSTICK_Y_AUX,0  ;Compare last value of joystick y with new one
    CPFSEQ  CURRENT_JOYSTICK_Y,0    ;If they aren't equal, clear alarm timer
    CALL    CLEAR_ALARM_TIMER
    MOVFF   CURRENT_JOYSTICK_Y_AUX,CURRENT_JOYSTICK_Y
    RETURN

ESPERA_ADC
    BSF     ADCON0,GO_NOT_DONE,0 ;Start conversion
    ESPERA
        BTFSC   ADCON0,GO_NOT_DONE,0 ;Check if conversion is finished
        GOTO    ESPERA
    RETURN

CLEAR_ALARM_TIMER
    CLRF    ALARM_TIMER_HIGH,0
    CLRF    ALARM_TIMER_LOW,0
    CLRF    ALARM_TIMER_COUNTER,0
    BCF     FLAGS,ALARM_ACTIVE,0
    BTFSS   FLAGS,ENRERE,0
    BCF     LATA,4,0
    RETURN
   
;-------------------------------
;MAIN
;-------------------------------
MAIN
    CALL    INIT_VARS
    CALL    INIT_PORTS
    CALL    INIT_INT
    CALL    INIT_ADC
    CALL    INIT_TMR0
    CALL    MODE_MANUAL     ;Start with manual mode

;-------------------------------
;LOOP
;-------------------------------
LOOP
    CALL    CHECK_BUTTONS
    CALL    CHECK_JOYSTICK_X
    BTFSS   MODE,CREUER,0   ;Skip Joystick Y if we're in creuer
    CALL    CHECK_JOYSTICK_Y
    CLRF    WREG,0
    CPFSGT  COUNTER_TMR0_SERVO,0  ;If counter timer0 is 0, pwm servo
    CALL    PWM_SERVO
    GOTO    LOOP
    END