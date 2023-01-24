/*
 * File:   Assignment1.c
 * Author: Alice Rivi, Samuele Depalo, Giacomo Lugano
 *
 * Created on 17 novembre 2022, 12.17
 */

#include <xc.h> // include processor files - each processor file is guarded.  
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


// dsPICDEM2 board oscillator (XT): 7.3728 MHz

#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// Timer
#define TIMER1 1    // 10ms period
#define TIMER2 2    // 1 second on startup and then 7ms algorithm simulation
#define TIMER3 3    // On S5 pression, disable interrupt for 20ms (to avoid bouncing) 
#define TIMER4 4    // On S6 pression, disable interrupt for 20ms (to avoid bouncing) 

// Circular buffer
#define BUFFER_SIZE 13
typedef struct {
    char Buffer[BUFFER_SIZE];
    int ReadIndex;
    int WriteIndex;
} CircularBuffer;

// As volatile to turn off the optimization 
volatile bool S6_Flag = false;
volatile bool S5_Flag = false;
volatile CircularBuffer CB;

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

void Algorithm(){
    TMR_Wait_ms(TIMER2,7);
}
// Funtion to write on the LCD
void Write_LCD(char* Word){
    for(int i = 0; i < strlen(Word); i++) {
        while(SPI1STATbits.SPITBF == 1);    // Wait until not full
        SPI1BUF = Word[i];  // Send the 'x' character
    }
}
// Function to send the characher to the UART
void Send_UART(char* Word){
    for(int i = 0; i < strlen(Word); i++) {
        while(SPI1STATbits.SPITBF == 1);    // Wait until not full
        U2TXREG = Word[i];  // Send the 'x' character
    }
}
// Function to move the cursor
void MoveCursor(int row, int offset){
    
    int Cursor; 
    if(row > 0 && row < 3 && offset >= 0 && offset <= 16){
        // FIRST ROW
        if (row == 1){
            Cursor = 0x80 + offset;
        }
        // SECOND ROW
        else if(row == 2){
            Cursor = 0xC0 + offset;
        }
        
        while(SPI1STATbits.SPITBF == 1); // Wait until not full
        SPI1BUF = Cursor;
    }
}
// Function to clear the LCD from cursor to the end of the row
void EmptyRow(int row, int offset){
    
    MoveCursor(row,offset);
    for(int i = offset; i < 16; i++) {
            while(SPI1STATbits.SPITBF == 1);    // Wait until not full
            SPI1BUF = ' ';  // Write spaces to 'clear' the LCD from previous characters
        }
    
    MoveCursor(row,offset);
}
// Function to write on the buffer
void Write_Buffer(volatile CircularBuffer* CB, char Char_rcv){
    CB->Buffer[CB->WriteIndex] = Char_rcv;
    CB->WriteIndex++;
    
    if(CB->WriteIndex == BUFFER_SIZE)
        CB->WriteIndex = 0;
}
// Function to read from the buffer
int Read_Buffer(volatile CircularBuffer* CB, char* Char_rcv){
    if(CB->ReadIndex == CB->WriteIndex)
        return 0;
        
    *Char_rcv = CB->Buffer[CB->ReadIndex];
    CB->ReadIndex++;
    
    if(CB->ReadIndex == BUFFER_SIZE)
        CB->ReadIndex = 0;
    return 1;
}
// Function to conver an int into a string
void ToString(char Str[], int Num)
{
    int i;
    int rem;
    int len = 0;
    int n = Num;

    while (n != 0)
    {
        len++;
        n /= 10;
    }
    for (i = 0; i < len; i++)
    {
        rem = Num % 10;
        Num = Num / 10;
        Str[len - (i + 1)] = rem + '0';
    }
    Str[len] = '\0';    // End of the string
}


int main(int argc, char** argv) {
    
    // Define button S5 and S6 as input (E8 and D0)
    TRISEbits.TRISE8 = 1;   // S5
    TRISDbits.TRISD0 = 1;   // S6
    
    // Enable interrupts
    IEC0bits.INT0IE = 1;    // S5
    IEC1bits.INT1IE = 1;    // S6
    IEC0bits.T3IE = 1;
    IEC1bits.T4IE = 1;
    
    // Configuration SPI
    SPI1CONbits.MSTEN = 1;  // Master mode
    SPI1CONbits.MODE16 = 0; // 8-bit mode
    SPI1CONbits.PPRE = 3;   // 1:1 primary prescaler
    SPI1CONbits.SPRE = 6;   // 2:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // Enable SPI   
    
    CB.WriteIndex = 0;
    CB.ReadIndex = 0;
    
    // Configuration UART
    U2BRG = 11; // BAUD RATE REG: (7372800 / 4) / (16 * 9600) - 1
    U2MODEbits.UARTEN = 1;  // Enable UART
    U2STAbits.UTXEN = 1;    // Enable U2TX (must be after UARTEN)
    IEC1bits.U2RXIE = 1;    // Enable interrupt
    
    TMR_Wait_ms(TIMER2, 1000);  // Wait 1 second at start-up 
    
    // Number of characters received from the UART2
    int CharCount = 0;
    /* Conversion of the number of characters received from the UART from 
     * integers to strings for sending and displaying */
    char Char_CharCount[3] = "0";

    //int Cursor = 0x80;  // First row
    int Offset_First = 0;

    MoveCursor(2,0);    // Second row
    Write_LCD("Char Recv: ");
    MoveCursor(1,0);    // First row
    
    bool Update_Flag = false;      // True when received at least a character -> updates count on 2nd row
    
    // Algorithm runs at 100 Hz
    TMR_SetupPeriod(TIMER1, 10);
    
    
    
    while(1) {
        
        Algorithm();
        /* If button S5 is pressed, send the current number of chars received 
         * to UART2 */
        if (S5_Flag){
            Send_UART(Char_CharCount);
            S5_Flag = false;
        }
        /* If button S6 is pressed, clear the first row and reset the characters 
        received counter */
        if (S6_Flag){
            EmptyRow(1,0);       // Clear all the first row
            EmptyRow(2,11);     // Clear the number of characters received from the UART2
            Offset_First = 0;
            CharCount = 0;
            
            S6_Flag = false;
        }
        
        // Char received
        char Char_rcv;
        /* The cycle will continue to run as long as the buffer contains a 
         * value related to the received character.*/
        while(Read_Buffer(&CB, &Char_rcv) == 1){
            Update_Flag = 1;    // Update second row char count
            CharCount++;                
            // FiIRST ROW
            // If a LF '/r' or CR '/n' character is received
            if(Char_rcv == '\r' || Char_rcv == '\n'){
                EmptyRow(1,1);   // Clear all the first row
                Offset_First = 0;
            }else{
                // If the cursor exceeds the last position
                if(Offset_First >= 16){
                    Offset_First = 0;
                    EmptyRow(1,0); // Clear all the first row
                    
                }
                MoveCursor(1,Offset_First);
                Write_LCD(&Char_rcv);  // Write on the LCD
                Offset_First++;
            }
            
            
        // SECOND ROW
        if (Update_Flag == 1){
            Update_Flag = 0;
            
            MoveCursor(2,11);
             // Convert an integer to a string to be displayed
            ToString(Char_CharCount, CharCount);
            Write_LCD(Char_CharCount);  // Write on the LCD
            
            MoveCursor(1,Offset_First);
        
            
        }
        
        // If an overflow error occurred
        if(U2STAbits.OERR == 1){
            U2STAbits.OERR = 0; // Clear the overflow notifier
        }
        
        if(IFS0bits.T1IF == 1){
            MoveCursor(1,0);    // First row
            Write_LCD("OUT OF TIME");  // Write on the LCD
            Offset_First = 16;  // The message will be overwritten as soon as new characters are received
        }
        
        TMR_WaitPeriod(TIMER1);
        }
    }
    return (EXIT_SUCCESS);
}



// Interupt timer T3
void __attribute__ (( __interrupt__ , __auto_psv__ )) _T3Interrupt() {
   
    IFS0bits.T3IF = 0;      // Reset interrupt flag
    T3CONbits.TON = 0;      // Stop the timer
    
    if (PORTEbits.RE8 == 1) // S5 press
        S5_Flag = true;
    
    IFS0bits.INT0IF = 0;    // Reset interrupt flag
    IEC0bits.INT0IE = 1;    // Enable interrupt
    
}

// Interupt timer T4
void __attribute__ (( __interrupt__ , __auto_psv__ )) _T4Interrupt() {
    
    IFS1bits.T4IF = 0;      // Reset interrupt flag
    T4CONbits.TON = 0;      // Stop the timer
    
    if (PORTDbits.RD0 == 1) // S6 presss
        S6_Flag = true;
    
    IFS1bits.INT1IF = 0;    // Reset interrupt flag
    IEC1bits.INT1IE = 1;    // Enable interrupt
    
}

// Interupt button S5
void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT0Interrupt() {
    
    IEC0bits.INT0IE = 0;    // Disable interrupt
    IFS0bits.INT0IF = 0;    // Reset interrupt flag
    
    TMR_SetupPeriod(TIMER3, 20);
}

// Interupt button S6
void __attribute__ (( __interrupt__ , __auto_psv__ )) _INT1Interrupt() {
    
    IEC1bits.INT1IE = 0;    // Disable interrupt
    IFS1bits.INT1IF = 0;    // Reset interrupt flag
    
    TMR_SetupPeriod(TIMER4, 20);
}

// Interupt UART2
void __attribute__ (( __interrupt__ , __auto_psv__ )) _U2RXInterrupt() {
   
    IFS1bits.U2RXIF = 0;    // Reset interrupt flag
    
    char VAL = U2RXREG;
    Write_Buffer(&CB, VAL);
}
