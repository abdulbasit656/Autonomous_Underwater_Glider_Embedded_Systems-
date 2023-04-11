/*
 * File:   main.c
 * 
 * Author01: BASIT AKRAM    (5161322)
 * Author02: ANKUR KOHLI    (5160903)
 * Author03: AMMAR IQBAL    (5183355)
 * 
 * Created on January 15, 2023, 5:24 PM
 */

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

#include "stdio.h"
#include <stdlib.h>
#include "xc.h"
#include "timer.h"
#include "spi.h"
#include"parser.h"
#include"circularBuffer.h"
#include"button.h"
#include"uart.h"
#include <string.h>
#include<math.h>

#define MAX_TASKS 6 // Define the number of task to perform

#define FOSC        7372800     // Frequency of board oscillator
#define FCY         1843200     // Number of clocks per seconds
#define BAUDRATE    9600        //baudrate of communication

// Board state definitions
#define STATE_CONTROLLED    0
#define STATE_TIMEOUT       1
#define STATE_SAFE          2

#define MAX_RPM_M1        +10000      // Maximum allowed propeller velocity
#define MIN_RPM_M1        -10000      // Minimum allowed propeller velocity

#define MIN_RPM_M2       -100        //minimum allowed rpm for m2
#define MAX_RPM_M2       +100        //maximum allowed rpm for m2

#define MIN_RPM_M3       -100        //minimum allowed rpm for m3
#define MAX_RPM_M3       +100        //maximum allowed rpm for m3

#define SPEED_M1            2
#define SPEED_M2            0.05
#define SPEED_M3            5

#define MAX_BATTERY_POS     100
#define MIN_BATTERY_POS     0

#define MAX_PITCH           +20
#define MIN_PITCH           -20

#define MIN_RUDDER_ANGLE    -30
#define MAX_RUDDER_ANGLE    30

#define MIN_VELOCITY        -5
#define MAX_VELOCITY        5

#define KP 1.0                          // Proportional gain
#define DT 0.3                          // Time step in seconds

#define NOTPRESSED          0            //S6FLAG NOT PRESSED
#define PRESSED             1               //S6FLAG PRESSED

void fromUartData();
int calculateDC(int rpm1, int rpm2, int rpm3);
void CalculatePWM();
int sendMCPWM();
int sendMCPOS();
void blinkD3();
void blinkD4();
void lcdOutput();

// Custom enum variable to store board state
// Board states: Controlled, Timeout, Safe mode
typedef enum {CONTROLLED, TIMEOUT, SAFE} state;
volatile state board_state = 0;

// Variables to store applied RPMs
int RPM_M1 = 0;     //initial RPM of motor 1
int RPM_M2 = 0;     //initial RPM of motor 2
int RPM_M3 = 0;     //initial RPM of motor 3

// Variable to store PWM duty cycle
double DC_M1 = 0.0;
double DC_M2 = 0.0;
double DC_M3 = 0.0;

//variable to store position of the rudder & battery 
double Actual_Pos = 0.0; // initial position in mm
double Actual_Rud = 0.0; // initial position in mm

// Flag for button S6
int S6flag = NOTPRESSED;

typedef struct {                //scheduler heartbeat structure
    int n;
    int N;
} heartbeat;

typedef struct {
    float speed;
    float pitch;
    float rudder;
}desired_ref;

parser_state pstate;
volatile circular_buffer_t inBuffer, outBuffer;
desired_ref dref;
heartbeat schedInfo[MAX_TASKS];

// parser initialization
void parser_setup(){
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
}

//put values in dref 
int parse_rlsen(const char* msg){
    int i = 0 ;
    extract_float(msg, &dref.speed);
    i = next_value(msg, i);
    extract_float(msg + i, &dref.pitch);
    i = next_value(msg, i);
    extract_float(msg + i, &dref.rudder);
    return 0;
}

//PWM configuration
void PWM_setup(){
    int f_pwm = 1000; // The frequency must be 1 kHz
    PTCONbits.PTMOD = 0; // free running
    PTCONbits.PTCKPS = 0b00; //prescaler
    PWMCON1bits.PEN1H = 1;
    PWMCON1bits.PEN2H = 1;
    PWMCON1bits.PEN3H = 1;
    
    int PTMR_Prescaler = 1;  // support variable for PTPER computation
    PTPER = FCY / (f_pwm * PTMR_Prescaler) - 1; //1842
    
    PDC1 = 0;                           //dutycycle 1 at beginning
    PDC2 = 0;                           //dutycycle 2 at beginning
    PDC3 = 0;                           //dutycycle 3 at beginning
    
    DTCON1bits.DTAPS = 0b00;    // dead time prescaler
    DTCON1bits.DTA = 6;         // dead time at least 3
    PTCONbits.PTEN = 1; // enable pwm
}

// Timer 2 ISR - Set motor velocity to zeros and blink led D4
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt () {
    //IEC0bits.T2IE = 0;         // Disable interrupt of timer t2
    IFS0bits.T2IF = 0;          // Reset interrupt flag for timer 2
    // Set timeout state
    board_state = TIMEOUT;
    // Set motor velocity to zero
    int rpm1 = 0;
    int rpm2 = 0;
    int rpm3 = 0;
    // Set duty cycle to zero
    calculateDC(rpm1, rpm2, rpm3);
    //blinkD4();
    T2CONbits.TON = 0;          // Stop the timer
    TMR2 = 0;                   // Reset the timer 
}
// Debouncing timer ISR
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;              // Reset interrupt flag of timer 3
    IFS1bits.INT1IF = 0;            // Reset interrupt flag of button s6
    IEC1bits.INT1IE = 1;            // Enable interrupt of button s6
    T3CONbits.TON = 0;              // Stop debouncing timer
}

// S6 buttons ISR
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
    IFS1bits.INT1IF = 0;            // Reset interrupt flag
    IEC1bits.INT1IE = 0;            // Disable interrupt of button s6
    
    // Toggle state of the flag
    S6flag = (S6flag + 1 ) % 2;     // If flag = 0 -> flag = 1. If flag = 1 -> flag = 0
    tmr_setup_period(3, 50);        // Start debouncing timer
}

void Heartbeat_setup(){
    // Set n = 0. The scheduler will update it on the go.   
    schedInfo[0].n = 0;
    schedInfo[1].n = 0;
    schedInfo[2].n = 0;
    schedInfo[3].n = 0;
    schedInfo[4].n = 0;
    schedInfo[5].n = 0;

    // Heartbeat is = 100, hence: 
    // N[i] = period of task / heartbeat
    
    schedInfo[0].N = 1;     // PWM values referesh              10 Hz (100ms)
    schedInfo[1].N = 2;     // Feedback PWM msg to pc            5 Hz (200ms)
    schedInfo[2].N = 1;     // Feedback POS msg to pc           10 Hz (100ms)
    schedInfo[3].N = 5;     // Blinking leds D3                  1 Hz (500ms on, 500ms off)
    schedInfo[4].N = 2;     // Blinking leds D4                 10 Hz (100ms on, 100ms off)
    schedInfo[5].N = 10;     // LCD update                       1 Hz (1000ms)
}

//scheduler synchronization
void scheduler() {
    int i;
    for (i = 0; i < MAX_TASKS; i++) {
        schedInfo[i].n++;
        if (schedInfo[i].n == schedInfo[i].N) {
            switch (i) {
                case 0:
                    fromUartData(); 
                    break;
                case 1:
                    sendMCPWM();
                    break;
                case 2:
                    sendMCPOS();
                    break;
                case 3:
                    blinkD3();
                    break;
                case 4:
                    blinkD4();
                    break;    
                case 5:
                    lcdOutput();
                    break;    
            }
            schedInfo[i].n = 0;
        }
    }
}

//**********TASK 01 FUNCTION**********//

void fromUartData(){
    
    char byte;
    UARTget();
    while (bufSize(&inBuffer)>0) {
        IEC1bits.U2RXIE = 0; // Disable interrupt of UART
        read_cb(&inBuffer, &byte);            //read data from buffer
        IEC1bits.U2RXIE = 1; // Enable interrupt of UART
        
        int ret = parse_byte(&pstate, byte);        //send data to parser to decode it
         
         if (ret == NEW_MESSAGE){ 
             //continue;
             if (strcmp(pstate.msg_type, "HLREF") == 0) {
                
                if(board_state != SAFE )
                {
                    if(board_state == TIMEOUT)    
                    {
                        // A new message (reference) arrived -> get out of timeout state
                        board_state = CONTROLLED; 
                    }
                    //CONTROLL MODE CODE
                    ret = parse_rlsen(pstate.msg_payload);      //parse payload to extract values
                    CalculatePWM();                                 //calculate RPMs and PWM
                    tmr2_restart_timer();
                }             
                //SAFE MODE
             }
         }
        else {
            UARTsend("  NO MESSAGE RECEIVED  ");
            }
    }
}

int COMPUTE_RPM_M1(float des_Speed) {
  return (int) round(des_Speed / SPEED_M1 * MAX_RPM_M1);
}

float COMPUTE_BATTERY_POS(float pitch_Angle) {
  return MIN_BATTERY_POS + (pitch_Angle - MIN_PITCH) / (MAX_PITCH - MIN_PITCH) * (MAX_BATTERY_POS - MIN_BATTERY_POS);
}

int COMPUTE_RPM_M2(float battery_Pos) {
  return (int) round((battery_Pos - MIN_BATTERY_POS) / (MAX_BATTERY_POS - MIN_BATTERY_POS) * MAX_RPM_M2);
}

int COMPUTE_RPM_M3(float rud_Angle) {
  return (int) round((rud_Angle - MIN_RUDDER_ANGLE) / (MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE) * MAX_RPM_M3);
}

float POS_CONTROL(float Actual_Pos, float Desired_Pos) {
  float Pos_error = 0.0; // Error signal
  float control_action = 0.0; // Control action
  
  // Check if desired position is within the range
  if (Desired_Pos < MIN_BATTERY_POS) Desired_Pos = MIN_BATTERY_POS;
  if (Desired_Pos > MAX_BATTERY_POS) Desired_Pos = MAX_BATTERY_POS;
  
  // Calculate the position error
  Pos_error = Desired_Pos - Actual_Pos;
    
  // Calculate the control action
  control_action = Pos_error * KP;
  RPM_M2 = COMPUTE_RPM_M2(control_action);
  // Update the current position
  Actual_Pos = Actual_Pos + (control_action) * DT;
 
  return Actual_Pos;
}

float RUDDER_CONTROL(float Actual_Rud, float Desired_Rud) {
  float Pos_error = 0.0; // Error signal
  float control_action = 0.0; // Control action
  
  // Check if desired rudder is within the range
  if (Desired_Rud < MIN_RUDDER_ANGLE) Desired_Rud = MIN_RUDDER_ANGLE;
  if (Desired_Rud > MAX_RUDDER_ANGLE) Desired_Rud = MAX_RUDDER_ANGLE;
  
  // Calculate the error signal
  Pos_error = Desired_Rud - Actual_Rud;
    
  // Calculate the control action
  control_action = Pos_error * KP;
  
  RPM_M3 = COMPUTE_RPM_M3(control_action);
    
  // Update the current rudder
  Actual_Rud = Actual_Rud + (control_action) * DT;
    
  return Actual_Rud;
}

// Function to compute the duty cycle range
int calculateDC(int rpm1, int rpm2, int rpm3){  
    float DC1 = 0.0, DC2 = 0.0, DC3 = 0.0;
    
    DC1 = (rpm1 == 0 ? 0.50 : (float)rpm1/10000.0);         //if rpm1 is 0 the dutycyle is 50%
    DC2 = (rpm1 == 0 ? 0.50 : (float)rpm2/100.0);           //if rpm2 is 0 the dutycyle is 50%    
    DC3 = (rpm1 == 0 ? 0.50 : (float)rpm3/100.0);           //if rpm3 is 0 the dutycyle is 50%
        
    if(DC1 > 0.95) DC1 = 0.95;                              //max rpm1 11000 -> 100% dutycycle but for 10000 -> 0.95%
    if(DC2 > 0.95) DC2 = 0.95;                              //max rpm2 110 -> 100% dutycycle but for 100 -> 0.95%
    if(DC3 > 0.95) DC2 = 0.95;                              //max rpm3 110 -> 100% dutycycle but for 100 -> 0.95%
    
    if(DC1 < 0.045) DC1 = 0.045;                              //min rpm1 -11000 -> 100% dutycycle but for -10000 -> 0.045%
    if(DC2 < 0.045) DC2 = 0.045;                              //min rpm2 -110 -> 100% dutycycle but for -100 -> 0.045%
    if(DC3 < 0.045) DC3 = 0.045;                              //min rpm3 -110 -> 100% dutycycle but for -100 -> 0.045%
        
    // Define the duty cycle
    DC_M1 = (DC1 * 2.0 * PTPER );
    DC_M2 = (DC2 * 2.0 * PTPER );
    DC_M3 = (DC3 * 2.0 * PTPER );
        
    // Assign it to the corresponding output pins of pwm register
    PDC1 = DC_M1;
    PDC2 = DC_M2;
    PDC3 = DC_M3;
    
    return 0;
}

void CalculatePWM(){
        // Motor 1 Control System
        RPM_M1 = COMPUTE_RPM_M1(dref.speed);         //send to calculateDC
        
        // Motor 2 control system
        float Desired_Pos = COMPUTE_BATTERY_POS(dref.pitch);
        if(Actual_Pos != Desired_Pos)
        {
            Actual_Pos = POS_CONTROL(Actual_Pos, Desired_Pos);
        }

        // Motor 3 Control System
        float Desired_Rud = dref.rudder;
        if(Actual_Rud != Desired_Rud){
            Actual_Rud = RUDDER_CONTROL(Actual_Rud, Desired_Rud);
        }
        
            // Saturate RPM if they're above the allowed threshold
        if(RPM_M1 > MAX_RPM_M1) RPM_M1 = MAX_RPM_M1;
        if(RPM_M2 > MAX_RPM_M2) RPM_M2 = MAX_RPM_M2;
        if(RPM_M3 > MAX_RPM_M3) RPM_M3 = MAX_RPM_M3;

        if(RPM_M1 < MIN_RPM_M1) RPM_M1 = MIN_RPM_M1;
        if(RPM_M2 < MIN_RPM_M2) RPM_M2 = MIN_RPM_M2;
        if(RPM_M3 < MIN_RPM_M3) RPM_M3 = MIN_RPM_M3;
        
        //calculate the dutycyle of each motor
        calculateDC(RPM_M1, RPM_M2, RPM_M3);
        
}

//**********TASK 02 FUNCTION**********//

int sendMCPWM() {
    // Message variable
    char MCPWM[21];

    sprintf(MCPWM, "$MCPWM,%d,%d,%d*", RPM_M1, RPM_M2, RPM_M3);
    // Send message to PC
    UARTsend(MCPWM);

    return 0;
}

//**********TASK 03 FUNCTION**********//

int sendMCPOS(){
    // Message variable
    char MCPOS[18];
    
    sprintf(MCPOS, "$MCPOS,%.1f,%.1f*", Actual_Rud, Actual_Pos);
    // Send message to PC
    UARTsend(MCPOS);

    return 0;
}

//**********TASK 04 FUNCTION**********//
void blinkD3() {
    LATBbits.LATB0 = !LATBbits.LATB0;       //Toggle LED D3
}

//**********TASK 05 FUNCTION**********//
void blinkD4(){
    if(board_state == TIMEOUT)        // If board is in timeout state
    {   // Blink led D4
        LATBbits.LATB1 = !LATBbits.LATB1;       //Toggle LED D4
    }
    else                        // If board is NOT in timeout state
        LATBbits.LATB1 = 0;     // Switch off led D4 
}

//**********TASK 06 FUNCTION**********//

void lcdOutput(){
    char printString[16];
   
    if (S6flag == NOTPRESSED)        // When S6 is not clicked
    {
        clear_LCD(0);
        clear_LCD(1);                    
        // Second part of first row: speed
        sprintf(printString, "Speed_M1: %.2f", (double)dref.speed);
        move_cursor_first_row(0);
        write_string_LCD(printString);
        // Second row: RPM_M1
        sprintf(printString, "RPM_M1: %d", RPM_M1);
        move_cursor_second_row(0);
        write_string_LCD(printString);
    }
    else if (S6flag == PRESSED)                 // When S6 is clicked
    {
        clear_LCD(0);
        clear_LCD(1);
        // First row: desired rudder and pitch values
        sprintf(printString, "R:%.2f P:%.2f", (double)dref.rudder, (double)dref.pitch);
        move_cursor_first_row(0);
        write_string_LCD(printString);
        
        // Second row: Actual rudder and pitch values 
        sprintf(printString, "R:%.2f P:%.2f", Actual_Rud, Actual_Pos);
        move_cursor_second_row(0);
        write_string_LCD(printString);
    }
}

//**********MAIN FUNCTION**********//

int main(void) {
    char in[15];                                    //input buffer from uart
    char out[35];                                   //output buffer to uart
    
    parser_setup();                                 //initialize parser
    Buffer_setup(&inBuffer, in, sizeof(in));        //input buffer initialization with relative size
    Buffer_setup(&outBuffer, out, sizeof(out));     //output buffer initialization with relative size
    UART_setup(BAUDRATE, &inBuffer, &outBuffer);    //setup UART with baudrate and in/out buffers
    Heartbeat_setup();                              //initialize the heartbeat 
    SPI_setup();                                    //initialize LCD
    PWM_setup();                                    //initialize PWM
    Button_setup();                                 //initialize button flag
    LedPins_setup();                                //initialize LED pins
    
    tmr_wait_ms(1, 1000);                           // Wait 1 second for setup
    tmr_setup_period(1, 100);                       // Set heartbeat of scheduler to 100ms -> 10 Hz
    
    tmr_setup_period(2, 5000);                      // Init timer for timeout mode
    IEC0bits.T2IE = 1;                              // Enabling the interrupt of TIMER2
    
    board_state = CONTROLLED;                //initially board is in Controlled state
    
    while(1){
        scheduler(schedInfo);
        tmr_wait_period(1);
    }
    return 0;
}