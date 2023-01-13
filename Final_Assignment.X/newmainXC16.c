/*
 * File:   newmainXC16.c
 * Author: Alice Rivi (5135011), Giacomo Lugano (5400573), Samuele Depalo (5153930)
 *
 * Created on 7 dicembre 2022, 10.45
 */

#include "xc.h"

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

#define STATE_DOLLAR (1) // in this state, we discard everything until a dollar is found
#define STATE_TYPE (2) // in this state, we are reading the type of msg until a comma is found
#define STATE_PAYLOAD (3) // in this state, we read the payload until an asterix is found
#define NEW_MESSAGE (1) // new message received and parsed completely
#define NO_MESSAGE (0) // no new messages

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
    char msg_type[6];   // type is 5 chars 
    char msg_payload[10]; // payload can't have more than ???? digits
    int index_type;
    int index_payload;
} parser_state;

typedef struct{
    double angular;
    double linear;
} cartesian_velocity;

typedef struct{
    double left;
    double right;
} motor_velocity;

//Scheduler
#define MAX_TASKS 4
#define HEARTBEAT_TIME 100  
typedef struct{
    int n;
    int N;
} heartbeat;

#define STANDARD_MODE 0
#define TIMEOUT_MODE 1
#define SAFE_MODE 2
volatile int state = STANDARD_MODE;

volatile bool button_s6_flag = false;
volatile bool button_s5_flag = false;


void tmr_wait_ms();
void tmr_setup_period();
void tmr_wait_period();
void write_str_LCD(); // for debug
void move_cursor();   // for debug
int read_buffer(volatile CircularBuffer*, char*);
void write_buffer(volatile CircularBuffer*, char);

int parse_byte(parser_state* ps, char byte);
void parse_hlref(const char* msg, cartesian_velocity* d_vel);
void set_rpm(motor_velocity);

int next_value(const char* msg, int i);
int extract_integer();

void task1();
void task2();
void task3();
void task4();

int main(int argc, char** argv) {
    
    // Leds
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    LATBbits.LATB0 = 0;
    LATBbits.LATB1 = 0;
    
    // ADC (for "current" and temperature)
    // with the following setup we have a full sampling + conversion in 0.7 ms
	// automatic start - automatic end??
    ADCON3bits.ADCS = 32; //longest Tad
    ADCON1bits.ASAM = 0; // manual start
    ADCON1bits.SSRC = 7; // conversion starts after time specified by SAMC
    ADCON3bits.SAMC = 31; // longest sample time
    ADCON2bits.CHPS = 1; // CH0 and CH1
	// Serve solo 1 canale
    ADCHSbits.CH0SA = 2; // positive input AN2 (potentiometer)   
    ADCHSbits.CH123SA = 1; // positive input AN3 (termometer)
    ADPCFG = 0xFFFF;    // everything to digital
    ADPCFGbits.PCFG2 = 0; // AN2 as analog input
    ADPCFGbits.PCFG3 = 0; // AN3 as analog input
    ADCON1bits.SIMSAM = 0; // sample in sequence
    ADCON2bits.SMPI = 1;  // number of inputs - 1
    ADCON1bits.ADON = 1; //turn on
   
    /*// Configuration SPI
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6; // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI   
    */
    
    // Set cb indexes to zero
    cb_in.write_index = 0;
    cb_in.read_index = 0;
    cb_out.write_index = 0;
    cb_out.read_index = 0;
    
    // Configuration UART
    U2BRG = 11; // BAUD RATE REG: (7372800 / 4) / (16 * 9600) - 1 
// Bound rate supporta almeno 10Hz di entrambe le reference ricez/trasmit (considera caso peggiore--> 9600 tantissimi, 300 troppo poco rischi di vedere valori vecchi)
    U2MODEbits.UARTEN = 1;  // enable UART
    U2STAbits.UTXEN = 1;    // enable U2TX 
    U2STAbits.UTXISEL = 0b1; // interrupt when transmit buffer becomes empty
    U2STAbits.URXISEL = 0b00;// interrupt when a character is received    
    // interrupts enabled later
    
    //PWM
// 2 PWM = 1 PWM bisogna solo settare high (?) diverso
    PTCONbits.PTCKPS = 0b00; // prescaler
    PTCONbits.PTMOD = 0; // free running mode
    PWMCON1bits.PEN2H = 0b1; // set as output
    PTPER = 1842; // round of 1842.2 to the smallest integer
    PDC2 = 0;  // duty cycle 0% -> 0 V
    PTCONbits.PTEN = 1;
    
	//parser initialization
    parser_state pstate;
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0;
	pstate.index_payload = 0;
    
    //scheduling initialization
    heartbeat schedInfo[MAX_TASKS];
    schedInfo[0].n = 0; 
    schedInfo[0].N = 1;     // 10 Hz
    schedInfo[1].n = 0; //?
    schedInfo[1].N = 2;     // 5 Hz
    schedInfo[2].n = 0; //?
    schedInfo[2].N = 5;     // 2 Hz
    schedInfo[3].n = 0; //?
    schedInfo[4].N = 10;    // 1 Hz
    
    double avg_temp = 0;
    int n = 0;  //keeps track of number of samples averaged
    int no_ref_counter = 0;
    bool ref_out_of_bound = false;
    //bool led_D4_flag = false;
    motor_velocity effective_v;
    
    tmr_wait_ms(TIMER1, 1000); // wait 1 second at startup 
    
    
    // Enable interrupts
    IEC0bits.INT0IE = 1; //S5
    IEC1bits.INT1IE = 1; //S6
    IEC0bits.T3IE = 1;
    IEC1bits.T4IE = 1;
    IEC1bits.U2TXIE = 1; // enable transmitter interrupt 
    IEC1bits.U2RXIE = 1; // enable receiver interrupt 
    
    ADCON1bits.SAMP = 1; // start first sampling
    
    tmr_setup_period(TIMER1, HEARTBEAT_TIME);
    while(1) {
        
        //scheduler        
        int i;
        for(i=0; i<MAX_TASKS; i++){
            schedInfo[i].n++;
            if(schedInfo[i].n >= schedInfo[i].N){
                switch(i){
                    case 0: // 10 Hz
                      
                        task1(&pstate, &avg_temp, n, &no_ref_counter, &ref_out_of_bound, &button_S6_flag, &effective_v);
                        n++;
                        break;
                    case 1: // 5 Hz
			task2(motor_velocity* effective_v);
                        break;
                    case 2: // 2 Hz
			task3();
                        // LATBbits.LATB0 = !LATBbits.LATB0;
                        break;                        
                    case 3: // 1 Hz
                        n = 0;  // Start a new set of temperatures to average
			// task4 ???
                        break;
                }
                schedInfo[i].n = 0;
            }
        }
        
        // If the timer has expired
        /*if(IFS0bits.T1IF == 1){
            // out of time
        }*/
        tmr_wait_period(TIMER1);
    }
    return (EXIT_SUCCESS);
}

motor_velocity compute_rpm(cartesian_velocity desired_v){
    // DO THE differential drive kinematic model, 
    // MAYBE DO THE FUNCTION AS A VOID ONE THAT EDITS A PASSED VARIABLE
    motor_velocity rpm;
    rpm.left = 50;
    rpm.right = 50;
    
    return rpm;
}

void task1(parser_state* pstate, double* avg_temp, int n, int* no_ref_counter, bool* led_D4_flag, bool* ref_out_of_bound, bool* button_S6_flag, motor_velocity* effective_v){
    char byte;
    
    //temperature stuff
    while(ADCON1bits.DONE == 0);   //actually it won't wait on this, the conversion is already over
    //retrieve the last converted value from the ADC,
    int bitsT = ADCBUF1;
    // from bits to volts
    float voltsT = bitsT * (5.00 / 1024.00);
    //from volts to degrees
    float temperature = voltsT * 100 - 50;
    //update the mean
    *avg_temp = (*avg_temp * n + temperature) / (n+1);
    
    if(state != SAFE_MODE){
        
        cartesian_velocity desired_v;
        bool new_ref = false;
        
        //read buffer until empty to check for new ref
        while(read_buffer(&cb_in, &byte) == 1){
            //parse the char
            int ret = parse_byte(pstate, byte);
            if ( ret == NEW_MESSAGE) {
                //if correct type
                if (strcmp(pstate->msg_type, "HLREF") == 0){
                    new_ref = true;
                    parse_hlref(pstate->msg_payload, &desired_v);
                }
            }
        }
        
        //velocity stuff
        if (new_ref) {
            state = STANDARD_MODE;
            //*led_D4_flag = false;
            LATBbits.LATB1 = 0;
            
            motor_velocity computed_v = compute_rpm(desired_v);
            *effective_v = computed_v;
            
            *ref_out_of_bound = false;
            
            // LEFT
            if(computed_v.left < -50 && computed_v.left > 50){ 
                effective_v->left = computed_v.left / abs(computed_v.left)*50;
                *ref_out_of_bound = true;   
            }
            if(computed_v.right < -50 && computed_v.right > 50){ 
                effective_v->right = computed_v.right / abs(computed_v.right)*50;
                *ref_out_of_bound = true;   
            }
            
            // change with the right PDC and stuff !!!!!!!!!!!
            set_rpm(*effective_v);
            PDC2 = PTPER * (1 + effective_v->right/60); // DUTY CYCLE TO CHECK !!!!
            PDC2 = PTPER * (1 + effective_v->left/60); // DUTY CYCLE TO CHECK !!!!
            
            // write second row
            char str2[] = "R: ";
            char char1[4], char2[4];      
            move_cursor(2,0);
            write_str_LCD(str2);
            if(!button_s6_flag){ 
                sprintf(char1, "%.1f", effective_v->left);
                sprintf(char2, "%.1f", effective_v->right);
                // "R: n1; n2" ni are rpms
            }else{
                sprintf(char1, "%.1f", desired_v.angular);
                sprintf(char2, "%.1f", desired_v.linear);
                // "R: angular_vel; linear_vel"    
            }
            write_str_LCD(char1);
            write_str_LCD("; ");
            write_str_LCD(char2);
        
        }else if(state == STANDARD_MODE){
            *no_ref_counter = *no_ref_counter + 1;     
            if(*no_ref_counter>=50){
                state = TIMEOUT_MODE;
                PDC2 = PTPER; // set velocity to zero
                //*led_D4_flag = true;
            }
        }else{ //TIMEOUT MODE
            LATBbits.LATB1 = !LATBbits.LATB1; // toggle D4
        }
        
        
        
        

    }else{ // safe mode
        
        //read buffer until empty
        while(read_buffer(&cb_in, &byte) == 1){
            //parse the char
            int ret = parse_byte(pstate, byte);
            if ( ret == NEW_MESSAGE) {
                //if correct type
                if (strcmp(pstate->msg_type, "HLENA") == 0){
                    state = STANDARD_MODE;
                    // write and send ack
                }
            }
        }

    }      
    
    // write first row
    move_cursor(1,0);
    char str1[] = "STATUS: ";
    write_str_LCD(str1);
    switch (state){
        case STANDARD_MODE:
            write_str_LCD("C"); //controlled
            break;
        case TIMEOUT_MODE:
            write_str_LCD("T"); //timeout
            break;
        case SAFE_MODE:
            write_str_LCD("H"); //halt
            break;
    }
    
    
    ADCON1bits.SAMP = 1; // start sampling
}

void task2(motor_velocity* effective_v){    //could it be better to pass the chars already?
    char str[18];
    char n1[4], n2[4];      
    sprintf(n1, "%.1f", effective_v->left);  
    sprintf(n2, "%.1f", effective_v->right);
           
    strcpy(str, "$MCFBK,");
    strcat(str, n1);
    strcat(str, ",");
    strcat(str, n2);
    strcat(str, ",");
    switch (state){
        case STANDARD_MODE:
            strcat(str, "0");
            break;
        case TIMEOUT_MODE:
            strcat(str, "1");
            break;
        case SAFE_MODE:
            strcat(str, "2");
            break;    
    }
    strcat(str, "*");
   
    // send in cb_out
    
}

void task3(){
    // toggle D3
    LATBbits.LATB0 = !LATBbits.LATB0;
}

void task4(bool* ref_out_of_bound, motor_velocity* computed_v, double* avg_temp){
    //$MCTEM,temp* where temp is the temperature
    char str1[18];
    char temp[4];      
    sprintf(temp, "%.1f", *avg_temp);  

    strcpy(str1, "$MCTEM,");
    strcat(str1, temp);
    strcat(str1, "*");
    
    
    //send in cb_out????
    
    
    
    //$MCALE,n1,n2*, where n1 and n2 are the compute wheel RPM values that are outside the maximum allowed values
    if(*ref_out_of_bound){
        char str2[18];
        char n1[4], n2[4];      
        sprintf(n1, "%.1f", computed_v->left);  
        sprintf(n2, "%.1f", computed_v->right);

        strcpy(str2, "$MCALE,");
        strcat(str2, n1);
        strcat(str2, ",");
        strcat(str2, n2);
        strcat(str2, "*");
        
        //send in cb_out????
    }
    
}

// ASSUMING VELOCITIES TO BE INTEGERS !!!!!!!!!!!!
void parse_hlref(const char* msg, cartesian_velocity* d_vel){
    int i=0;
    d_vel->angular = extract_integer(msg);
    i = next_value(msg, i);
    d_vel->linear = extract_integer(msg + i);
}

 void set_rpm(motor_velocity effective_v){
    PDC2 = PTPER * (1 + effective_v.right/60); // DUTY CYCLE TO CHECK !!!!
    PDC2 = PTPER * (1 + effective_v.left/60); // DUTY CYCLE TO CHECK !!!!
 }
            

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
            }else if(ps->index_payload == 15){                              // CHECK THESE NUMBERS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

int extract_integer(const char* str) {
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
		number *= 10; // multiply the current number by 10;
		number += str[i] - '0'; // converting character to decimal number
		i++;
	}
	return sign*number;
}

int next_value(const char* msg, int i){
    while (msg[i] != ',' && msg[i] != '\0') { i++; }
    if (msg[i] == ',')
        i++;
    return i;
}

// Function to write on the buffer
void write_buffer(volatile CircularBuffer* cb, char char_rcv){
    cb->buffer[cb->write_index] = char_rcv;
    cb->write_index++;
    if(cb->write_index == BUFFER_SIZE){
        cb->write_index = 0;
    }
}

// Function to read from the buffer
int read_buffer(volatile CircularBuffer* cb, char* char_rcv){
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

// Function to busy-wait ms milliseconds
void tmr_wait_ms(int timer, int ms){
    tmr_setup_period(timer, ms);
    switch (timer){
        case TIMER1:
            IFS0bits.T1IF = 0;
            while(IFS0bits.T1IF == 0){}
            break;
        case TIMER2:
            IFS0bits.T2IF = 0;
            while(IFS0bits.T2IF == 0){}
            break;
        case TIMER3:
            IFS0bits.T3IF = 0;
            while(IFS0bits.T3IF == 0){}
            break;
        case TIMER4:
            IFS1bits.T4IF = 0;
            while(IFS1bits.T4IF == 0){}
            break;
    }
}

// Function for setting up a timer
void tmr_setup_period(int timer, int ms) {
    switch (timer){
        case TIMER1:
            T1CONbits.TON = 0;
            TMR1 = 0; // reset the current value;
            PR1 = 28800*(long)ms/1000;
            T1CONbits.TCKPS = 0b10;
            T1CONbits.TON = 1;
            break;
        case TIMER2:
            T2CONbits.TON = 0;
            TMR2 = 0; // reset the current value;
            PR2 = 28800*(long)ms/1000;
            T2CONbits.TCKPS = 0b10;
            T2CONbits.TON = 1;
            break; 
        case TIMER3:
            T3CONbits.TON = 0;
            TMR3 = 0; // reset the current value;
            PR3 = 28800*(long)ms/1000;
            T3CONbits.TCKPS = 0b10;
            T3CONbits.TON = 1;
            break; 
        case TIMER4:
            T4CONbits.TON = 0;
            TMR4 = 0; // reset the current value;
            PR4= 28800*(long)ms/1000;
            T4CONbits.TCKPS = 0b10;
            T4CONbits.TON = 1;
            break; 
    }
    return;
}

// Function to wait until timer has expired
void tmr_wait_period(int timer){
    switch(timer){
        case TIMER1:
            while(IFS0bits.T1IF == 0);
            IFS0bits.T1IF = 0;
            break;
        case TIMER2:
            while(IFS0bits.T2IF == 0);
            IFS0bits.T2IF = 0;
            break;
        case TIMER3:
            while(IFS0bits.T3IF == 0);
            IFS0bits.T3IF = 0;
            break;   
        case TIMER4:
            while(IFS1bits.T4IF == 0);
            IFS1bits.T4IF = 0;
            break;   
    }
}

// Funtion to write on the LCD
void write_str_LCD(char* word){
    for(int i = 0; i < strlen(word); i++) {
        while(SPI1STATbits.SPITBF == 1); // wait until not full
        SPI1BUF = word[i]; // send the i-th character
    }
}

// Function to move the cursor
void move_cursor(int row, int offset){
    int Cursor; 
    // Check the args make sense
    if(row > 0 && row < 3 && offset >= 0 && offset <= 16){
        // FIRST ROW
        if (row == 1){
            Cursor = 0x80 + offset; // set the cursor to the specified offset
        }
        // SECOND ROW
        else if(row == 2){
            Cursor = 0xC0 + offset; // set the cursor to the specified offset
        }
        // Move the cursor
        while(SPI1STATbits.SPITBF == 1); // Wait until not full
        SPI1BUF = Cursor;
    }
}

// Function to clear a LCD row from 'offset' to the end of the row
void empty_row(int row, int offset){
    move_cursor(row,offset);
    for(int i = offset; i <= 16; i++) {
            while(SPI1STATbits.SPITBF == 1);    // Wait until not full
            SPI1BUF = ' ';  // Write spaces to 'clear' the LCD
        }
    move_cursor(row,offset);
}


// Interupt UART2 on receiving
void __attribute__ (( __interrupt__ , __auto_psv__ )) _U2RXInterrupt() {
    IFS1bits.U2RXIF = 0;    // reset interrupt flag
    
    //with the current interrupt setting, this loop should be done just once
    while(U2STAbits.URXDA == 1){
        char value = U2RXREG;
        write_buffer(&cb_in, value);
    }
    
    // If an overflow error occurred
    if(U2STAbits.OERR == 1){
        // Clear the overflow notifier
        U2STAbits.OERR = 0;
    }
}

// Interupt UART2 on transmitting
void __attribute__ (( __interrupt__ , __auto_psv__ )) _U2TXInterrupt() {
    IFS1bits.U2TXIF = 0;    // reset interrupt flag    
    while(U2STAbits.UTXBF == 0){
            char value;
            if(read_buffer(&cb_out, &value)==1){
                U2TXREG = value;
            }else{
                break;
            }
    }
}



// Interupt button S5
void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT0Interrupt() {
    IEC0bits.INT0IE = 0;    // disable interrupt
    IFS0bits.INT0IF = 0;    // reset interrupt flag
    tmr_setup_period(TIMER3, 20);
}
// Interupt button S6
void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT1Interrupt() {
    IEC1bits.INT1IE = 0;    // disable interrupt
    IFS1bits.INT1IF = 0;    // reset interrupt flag
    tmr_setup_period(TIMER4, 20);
}

// Interupt timer T3
void __attribute__ (( __interrupt__ , __auto_psv__ )) _T3Interrupt() {
    IFS0bits.T3IF = 0;      // reset interrupt flag
    T3CONbits.TON = 0;      // stop the timer
    
    // If the button is not pressed
    if (PORTEbits.RE8 == 1) {
        button_s5_flag = true;
        //PDC2 = PTPER;
        motor_velocity effective_v;
        effective_v.right = 0;
        effective_v.left = 0;
        set_rpm(effective_v);       
    }
    
    IFS0bits.INT0IF = 0;    // reset interrupt flag
    IEC0bits.INT0IE = 1;    // enable interrupt
    
}
// Interupt timer T4
void __attribute__ (( __interrupt__ , __auto_psv__ )) _T4Interrupt() {
    IFS1bits.T4IF = 0;      // reset interrupt flag
    T4CONbits.TON = 0;      // stop the timer

    // If the button is not pressed    
    if (PORTDbits.RD0 == 1) {
        button_s6_flag = !button_s6_flag;
    }
    IFS1bits.INT1IF = 0;    // reset interrupt flag
    IEC1bits.INT1IE = 1;    // enable interrupt
    
}
