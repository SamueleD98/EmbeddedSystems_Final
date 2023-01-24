/*
 * File:   Assignment2.c
 * Author: utente
 *
 * Created on 9 dicembre 2022, 11.54
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

// Timer
#define TIMER1 1    
#define TIMER2 2    
#define TIMER3 3    
#define TIMER4 4    

#define STATE_DOLLAR (1)    // We discard everything until a dollar is found
#define STATE_TYPE (2)      // We are reading the type of msg until a comma is found
#define STATE_PAYLOAD (3)   // We read the payload until an asterix is found
#define NEW_MESSAGE (1)     // New message received and parsed completely
#define NO_MESSAGE (0)      // No new messages

/*For cb_in 7 would be enough but 21 is needed for cb_out
If the difference between the ideal sizes was higher,
a pointer to the custom size for each cb could have been used */
#define BUFFER_SIZE 21
typedef struct {
    char buffer[BUFFER_SIZE];
    int read_index;
    int write_index;
} CircularBuffer;
volatile CircularBuffer cb_in;
volatile CircularBuffer cb_out;

typedef struct{
    int state;
    char msg_type[6];       // Type is 5 chars 
    char msg_payload[5];    // Payload can't have more than 4 digits -..,dec
    int index_type;
    int index_payload;
} parser_state;

typedef struct{
    double current;
    double temperature;
} sensor_data;

//Scheduler
#define MAX_TASKS 3
#define HEARTBEAT_TIME 5  
typedef struct{
    int n;
    int N;
} heartbeat;

// Funzione scritta dal professore
int parse_byte(parser_state* ps, char byte){
    switch (ps->state){
        case STATE_DOLLAR:
            if(byte == '$'){
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if(byte == ','){
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0;           
            }else if(ps->index_type == 6){
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;           
            }else{
                ps->msg_type[ps->index_type] = byte;
                ps->index_type++;        
            }
            break;
        case STATE_PAYLOAD:
            if(byte == '*'){
                ps->state = STATE_DOLLAR;
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;     
            }else if(ps->index_payload == 15){
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;           
            }else{
                ps->msg_payload[ps->index_payload] = byte;
                ps->index_payload++;        
            }
            break;                   
    }
    return NO_MESSAGE;
}

int ExtractInteger(const char* str) {
	int i = 0, number = 0, sign = 1;
	if (str[i] == '-') {
		sign = -1;
		i++;
	}
	else if (str[i] == '+') {
		sign = 1;
		i++;
	}
	while (str[i] != ',' && str[i] != '\0') {
		number *= 10;                // Multiply the current number by 10;
		number += str[i] - '0';      // Converting character to decimal number
		i++;
	}
	return sign*number;
}

// Function to write on the buffer
void WriteBuffer(volatile CircularBuffer* cb, char char_rcv){
    cb->buffer[cb->write_index] = char_rcv;
    cb->write_index++;
    if(cb->write_index == BUFFER_SIZE){
        cb->write_index = 0;
    }
}

// Function to read from the buffer
int ReadBuffer(volatile CircularBuffer* cb, char* char_rcv){
    if(cb->read_index == cb->write_index){
        return 0;
    }
    *char_rcv = cb->buffer[cb->read_index];
    cb->read_index++;
    if(cb->read_index == BUFFER_SIZE){
        cb->read_index = 0;
    }
    return 1;
}

// Control loop
sensor_data Task1(parser_state* pstate, sensor_data mean, int n){
    char byte; 
    // Read buffer until empty
    while(ReadBuffer(&cb_in, &byte) == 1){
        // Parse the char
        int ret = parse_byte(pstate, byte);
        if ( ret == NEW_MESSAGE) {
            // If correct type
            if (strcmp(pstate->msg_type, "MCREF") == 0){
                /*In this case, no need of another parse function,
                the payload is directly the rpm value */
                int rpm = ExtractInteger(pstate->msg_payload);
                // Use it only if in the correct range
                if(rpm >= 0 && rpm <= 1000){
                    double volts = rpm * 0.005;
                    //PWM signal btw 0-5 V   
                    PDC2 = 2 * PTPER * volts / 5;   // Duty cycle
                }
            }
		}
    }
    
    while(ADCON1bits.DONE == 0);                                                // The conversion is already over
    int bitsT = ADCBUF1;                                                        // Retrieve the last converted value from the ADC
    float voltsT = bitsT * (5.00 / 1024.00);                                    // Retrieve the last converted value from bits to volts
    float temperature = voltsT * 100 - 50;                                      // Retrieve the last converted value from volts to degrees
    mean.temperature = (mean.temperature * n + temperature) / (n+1);            // Update the mean
    
    // For the current, same as before
    int bitsP = ADCBUF0;
    float voltsP = bitsP * (5.00 / 1024.00);
    float current = voltsP * 10 - 30;
    mean.current = (mean.current * n + current) / (n+1);
 
    // If current higher than +/- 15 A
    if (fabs(mean.current) > 15){
        LATBbits.LATB1 = 1;
    }else{
        LATBbits.LATB1 = 0;
    }  
    
    ADCON1bits.SAMP = 1; // Start sampling
    
    return mean;
}

// Build up the string to be sent
void Task2(sensor_data mean){
    char char_current[6];
    char char_temperature[6];
    char feedback[] = "$MCFBK,";
    
    // Convert in chars, just one decimal number
    sprintf(char_current, "%.1f", mean.current);
    sprintf(char_temperature, "%.1f", mean.temperature);
    
    // Write each char, one by one, on cb_out
    int i;
    for(i = 0; i < strlen(feedback); i++){
        WriteBuffer(&cb_out, feedback[i]);
    }
    for(i = 0; i < strlen(char_current); i++){
        WriteBuffer(&cb_out, char_current[i]);
    }
    WriteBuffer(&cb_out, ',');
    for(i = 0; i < strlen(char_temperature); i++){
        WriteBuffer(&cb_out, char_temperature[i]);
    }
    WriteBuffer(&cb_out, '*');
    
    /* Next we are going to send chars directly on UART
    so disable the interrupt on trasmision */ 
    IEC1bits.U2TXIE = 0;  
   
    /* Fill the transmit buffer with values from cb_out (if any, but there are for sure)
    This to restart the chain of interrupts on trasmission until cb_out is empty */
    while(U2STAbits.UTXBF == 0){
            char value;
            if(ReadBuffer(&cb_out, &value)==1){
                U2TXREG = value;
            }else{
                break;
            }
    }
    
    IEC1bits.U2TXIE = 1;    // Enable transmitter interrupt 
}

// Blink led
void Task3(){
    LATBbits.LATB0 = !LATBbits.LATB0; // ON/OFF led
}

// Function for setting a timer
void TMR_SetupPeriod(int Timer, int ms){
    // TMRx is incremented every internal clock cycle
    switch (Timer){
        case TIMER1:
            T1CONbits.TON = 0;  // Resets timer
            TMR1 = 0;   // Value timer counter (0 = reset)
            // Fcy = Fosc / 4 = 7372800 / 4 = 1843200 (number of clocks in 1 second)
            // In 1 second there would be 1843200 clocks steps
            // this is too high to be put in a 16 bit register (max 65535 clock steps)
            // If we set a prescaler of 1:64 we have 1843200/64 = 28800 clock steps, OK!
            PR1 = 28800*(long)ms/1000;  // Value the timer must count to
            T1CONbits.TCKPS = 0b10; // Prescaler 1:64
            T1CONbits.TON = 1;  // Starts the timer
            break;
            
        case TIMER2:
            T2CONbits.TON = 0;  // Resets timer
            TMR2 = 0;   // Value timer counter (0 = reset)
            // Fcy = Fosc / 4 = 7372800 / 4 = 1843200 (number of clocks in 1 second)
            // In 1 second there would be 1843200 clocks steps
            // this is too high to be put in a 16 bit register (max 65535 clock steps)
            // If we set a prescaler of 1:64 we have 1843200/64 = 28800 clock steps, OK!
            PR2 = 28800*(long)ms/1000;  // Value the timer must count to
            T2CONbits.TCKPS = 0b10; // Prescaler 1:64
            T2CONbits.TON = 1;  // Starts the timer
            break;
            
        case TIMER3:
            T3CONbits.TON = 0;  // Resets timer
            TMR3 = 0;   // Value timer counter (0 = reset)
            // Fcy = Fosc / 4 = 7372800 / 4 = 1843200 (number of clocks in 1 second)
            // In 1 second there would be 1843200 clocks steps
            // this is too high to be put in a 16 bit register (max 65535 clock steps)
            // If we set a prescaler of 1:64 we have 1843200/64 = 28800 clock steps, OK!
            PR2 = 28800*(long)ms/1000; // Value the timer must count to
            T3CONbits.TCKPS = 0b10; // Prescaler 1:64
            T3CONbits.TON = 1;  // Starts the timer
            break;
            
        case TIMER4:
            T4CONbits.TON = 0;  // Resets timer
            TMR4 = 0;   // Value timer counter (0 = reset)
            // Fcy = Fosc / 4 = 7372800 / 4 = 1843200 (number of clocks in 1 second)
            // In 1 second there would be 1843200 clocks steps
            // this is too high to be put in a 16 bit register (max 65535 clock steps)
            // If we set a prescaler of 1:64 we have 1843200/64 = 28800 clock steps, OK!
            PR2 = 28800*(long)ms/1000;  // Value the timer must count to
            T4CONbits.TCKPS = 0b10; // Prescaler 1:64
            T4CONbits.TON = 1;  // Starts the timer
            break;
    }
    return;
}

// Function to 'busy-wait' ms milliseconds
void TMR_Wait_ms(int Timer, int ms){
// IFSybits.TxIF signals an event related to the corresponding timer
    TMR_SetupPeriod(Timer, ms);
    
    switch (Timer){
    // IFS0bits T1-T2-T3
        case TIMER1:
            IFS0bits.T1IF = 0;
            // Do nothing, exit when T1 peripheral has expired
            while(IFS0bits.T1IF == 0);
            break;
        case TIMER2:
            IFS0bits.T2IF = 0;
            // Do nothing, exit when T2 peripheral has expired
            while(IFS0bits.T2IF == 0);
            break;
        case TIMER3:
            IFS0bits.T3IF = 0;
            // Do nothing, exit when T3 peripheral has expired
            while(IFS0bits.T3IF == 0);
            break;
    // IFS1bits T4    
        case TIMER4:
            IFS1bits.T4IF = 0;
            // Do nothing, exit when T4 peripheral has expired
            while(IFS1bits.T4IF == 0);
            break;
    }
}
// Function to wait until timer has expired
void TMR_WaitPeriod(int Timer){
    switch(Timer){
        case TIMER1:
            // Do nothing, exit when T1 peripheral has expired
            while(IFS0bits.T1IF == 0);
            IFS0bits.T1IF = 0;  // Flag = 0
            break;
        case TIMER2:
            // Do nothing, exit when T2 peripheral has expired
            while(IFS0bits.T2IF == 0);
            IFS0bits.T2IF = 0;  // Flag = 0
            break;
        case TIMER3:
            // Do nothing, exit when T3 peripheral has expired
            while(IFS0bits.T3IF == 0);
            IFS0bits.T3IF = 0;  // Flag = 0
            break;   
        case TIMER4:
            // Do nothing, exit when T4 peripheral has expired
            while(IFS1bits.T4IF == 0);
            IFS1bits.T4IF = 0;  // Flag = 0
            break;   
    }
}

// Interupt UART2 on receiving
void __attribute__ (( __interrupt__ , __auto_psv__ )) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0;    // Reset interrupt flag
    
    // With the current interrupt setting, this loop should be done just once
    while(U2STAbits.URXDA == 1){
        char value = U2RXREG;
        WriteBuffer(&cb_in, value);
    }
    
    // If an overflow error occurred
    if(U2STAbits.OERR == 1){
        // Clear the overflow notifier
        U2STAbits.OERR = 0;
    }
}

// Interupt UART2 on transmitting
void __attribute__ (( __interrupt__ , __auto_psv__ )) _U2TXInterrupt() {
    IFS1bits.U2TXIF = 0;    // Reset interrupt flag    
    while(U2STAbits.UTXBF == 0){
            char value;
            if(ReadBuffer(&cb_out, &value)==1){
                U2TXREG = value;
            }else{
                break;
            }
    }
}

int main(int argc, char** argv) {
    
    // Leds
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    LATBbits.LATB0 = 0;
    LATBbits.LATB1 = 0;
    
    // ADC (for "current" and temperature)
    // With the following setup we have a full sampling + conversion in 0.7 ms
    ADCON3bits.ADCS = 32;           // Longest Tad
    ADCON1bits.ASAM = 0;            // Manual start
    ADCON1bits.SSRC = 7;            // Conversion starts after time specified by SAMC
    ADCON3bits.SAMC = 31;           // Longest sample time
    ADCON2bits.CHPS = 1;            // CH0 and CH1
    ADCHSbits.CH0SA = 2;            // Positive input AN2 (potentiometer)   
    ADCHSbits.CH123SA = 1;          // Positive input AN3 (termometer)
    ADPCFG = 0xFFFF;                // Everything to digital
    ADPCFGbits.PCFG2 = 0;           // AN2 as analog input
    ADPCFGbits.PCFG3 = 0;           // AN3 as analog input
    ADCON1bits.SIMSAM = 0;          // Sample in sequence
    ADCON2bits.SMPI = 1;            // Number of inputs - 1
    ADCON1bits.ADON = 1;            // Turn on
   
    /*// Configuration SPI
    SPI1CONbits.MSTEN = 1;          // Master mode
    SPI1CONbits.MODE16 = 0;         // 8-bit mode
    SPI1CONbits.PPRE = 3;           // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6;           // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1;         // Enable SPI   
    */
    
    // Set cb indexes to zero
    cb_in.write_index = 0;
    cb_in.read_index = 0;
    cb_out.write_index = 0;
    cb_out.read_index = 0;
    
    // Configuration UART
    U2BRG = 11;                 // BAUD RATE REG: (7372800 / 4) / (16 * 9600) - 1
    U2MODEbits.UARTEN = 1;      // Enable UART
    U2STAbits.UTXEN = 1;        // Enable U2TX 
    U2STAbits.UTXISEL = 0b1;    // Interrupt when transmit buffer becomes empty
    U2STAbits.URXISEL = 0b00;   // Interrupt when a character is received    
    
    // PWM
    PTCONbits.PTCKPS = 0b00;    // prescaler
    PTCONbits.PTMOD = 0;        // Free running mode
    PWMCON1bits.PEN2H = 0b1;    // Set as output
    PTPER = 1842;               // Round of 1842.2 to the smallest integer
    PDC2 = 0;                   // Duty cycle: 0% = 0 V
    PTCONbits.PTEN = 1;
    
	// Parser initialization
    parser_state pstate;
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0;
	pstate.index_payload = 0;
    
    // Scheduling initialization
    heartbeat schedInfo[MAX_TASKS];
    schedInfo[0].n = 0; 
    schedInfo[0].N = 1;         // task1 is executed every heartbeat
    schedInfo[1].n = -1;
    schedInfo[1].N = 200;       // 1000Hz/HEARTBEAT_TIME;
    schedInfo[2].n = -3;
    schedInfo[2].N = 100;       // 500Hz/HEARTBEAT_TIME;
    
    // Current and temperature sensor data initialization
    sensor_data mean;   // Mean of the (at most) 5 values sampled each second
    mean.current = 0;
    mean.temperature = 0;
    int n = 0;          // Keeps track of number of samples averaged
    
    TMR_Wait_ms(TIMER1, 1000);  // Wait 1 second at startup 
    
    IEC1bits.U2TXIE = 1;        // Enable transmitter interrupt 
    IEC1bits.U2RXIE = 1;        // Enable receiver interrupt 
    ADCON1bits.SAMP = 1;        // Start first sampling
    
    TMR_SetupPeriod(TIMER1, HEARTBEAT_TIME);
    while(1) {
        // Scheduler        
        int i;
        for(i=0; i<MAX_TASKS; i++){
            schedInfo[i].n++;
            if(schedInfo[i].n >= schedInfo[i].N){
                switch(i){
                    case 0:
                        mean = Task1(&pstate, mean, n); 
                        // Update the mean values and rpm
                        n++;
                        break;
                    case 1:
                        Task2(mean); // Start sending the mean values
                        n = 0;
                        break;
                    case 2:
                        Task3();    // ON/OFF led
                        break;
                }
                schedInfo[i].n = 0;
            }
        }
        
        // If the timer has expired
        /*if(IFS0bits.T1IF == 1){
            // out of time
        }*/
        TMR_WaitPeriod(TIMER1);
    }
    return (EXIT_SUCCESS);
}