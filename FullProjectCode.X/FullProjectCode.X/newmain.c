/* 
 * File:   newmain.c
 * Author: Abo ALaa
 * Created on June 12, 2024
 */

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#include <stdio.h>
#include <string.h>
////////////////////////////(khaled ghazzal) ///////////////////////////////
#define _XTAL_FREQ 20000000
#define PRESCALAR 16
#define ADC_PIN 0
int Go = 0 ;
volatile int rate[10]; // array to hold last ten IBI values


float PWM_FREQ;
float PWM_PERIOD;
int counter;

void __init_adc() {
    /*
     * ADCON0 Register()
     * +---------------------------------------------------------------+
     * |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
     * +---------------------------------------------------------------+
     * |      ADCS     |          CSH          |GO/DONE|   -   | ADON  |
     * +---------------------------------------------------------------+
     */

    ADCON0bits.ADCS = 0b00; // set A/D conversion clock = fosc/2
    ADCON0bits.CHS = 0b000; // set all adc channels off
    ADCON0bits.ADON = 0; // a/d module is powered off

    /*
     * ADCON1 Register
     * +---------------------------------------------------------------+
     * |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
     * +---------------------------------------------------------------+
     * |  ADFM | ADCS2 |   -   |   -   |              PCFG             |
     * +---------------------------------------------------------------+
     */

    ADCON1bits.ADFM = 1; // set right justification for ADRESH
    ADCON1bits.ADCS2 = 0; // set A/D conversion clock = fosc/2
    ADCON1bits.PCFG = 0b0000; // set all pins as adc input    
}

int __adc_read(int adc_channel) {
    ADCON0bits.ADON = 1; // turn on a/d module
    ADCON0bits.CHS = (unsigned char) adc_channel; // turn on adc channel
    __delay_ms(10); // wait for capacitors to charge up
    ADCON0bits.GO = 1; // begin conversion
    while (ADCON0bits.GO_DONE == 1) {
        // wait for conversion to finish
    }
    int adc_value = (ADRESH << 8) + (ADRESL);
    return adc_value;
}

///////////////////////////// PWM Configuration(İbrahim Davutoğlu) ////////////////////////////////

void __set_pwm_freq(int f) {
    PWM_FREQ = f;
    PWM_PERIOD = 1 / PWM_FREQ;
    int PR2_value = (int) ((PWM_PERIOD * _XTAL_FREQ) / (4 * PRESCALAR) - 1);
    PR2 = (char) PR2_value;

}

void __init_pwm() {
    TRISCbits.TRISC2 = 0; // set RC2 as output
    T2CONbits.TMR2ON = 1; // set timer 2 on
    T2CONbits.T2CKPS = 0b10; // set pre scalar of 16
    CCP1CONbits.CCP1M = 0b1100; // set PWM mode of operation
}

void __set_duty_cycle(int duty_cycle) {
    float dc = (float) duty_cycle / 100;
    float dc_period = dc * PWM_PERIOD;
    char reg_value = (char) ((dc_period * _XTAL_FREQ) / PRESCALAR);
    /*
     * PWM has 10 bit resolution
     * 8 bits of MSB is stored in CCPR1L
     * 2 bits of LSB is stored in CCP1CON(5:4)
     */
    CCPR1L = reg_value >> 2;
    CCP1CONbits.CCP1X = (reg_value & 0b00000001);
    CCP1CONbits.CCP1Y = (reg_value & 0b00000010);
}

////////////////////////////////////////////////////////////////////////////////

void __init_interrupt__() {
    // setup for INTCON register
    GIE = 1; // enable global interrupt
    PEIE = 1; // enable peripheral interrupt
    // setup PIE1 register
    TMR1IE = 1; // enable timer1 overflow interrupt
    // setup PIR1 register 
    TMR1IF = 0; // clear timer1 overflow interrupt flag
}

void __init_timer__() {
    // setup T1CON register
    // set pre scale of 8
    T1CKPS0 = 1;
    T1CKPS1 = 1;
    TMR1CS = 0; // select internal clock
    TMR1ON = 1; // enable timer1 
}
/////////////////////////////////(mohamed elnour)///////////////////////////////////////////////
#define rs RD2
#define en RD3

#define R1 RB0
#define R2 RB1
#define R3 RB2
#define R4 RB3
#define C1 RB4
#define C2 RB5
#define C3 RB6
#define C4 RB7

void lcd_init();
void cmd(unsigned char a);
void dat(unsigned char b);
void show(unsigned char *s);
void lcd_delay();

unsigned char key();
void keyinit();

unsigned char keypad[4][4]={{'7','8','9','/'},{'4','5','6','*'},{'1','2','3','-'},{'C','0','=','+'}};
unsigned char rowloc,colloc;


void lcd_init()
{
    cmd(0x02);
    cmd(0x28);
    cmd(0x0e);
    cmd(0x06);
    cmd(0x80);
}

void cmd(unsigned char a)
{
    rs=0; 
    PORTD&=0x0F;
    PORTD|=(a&0xf0);
    en=1;
    lcd_delay();
    en=0;
    lcd_delay();
    PORTD&=0x0f;
    PORTD|=(a<<4&0xf0);
    en=1;
    lcd_delay();
    en=0;
    lcd_delay();
}

void dat(unsigned char b)
{
    rs=1; 
    PORTD&=0x0F;
    PORTD|=(b&0xf0);
    en=1;
    lcd_delay();
    en=0;
    lcd_delay();
    PORTD&=0x0f;
    PORTD|=(b<<4&0xf0);
    en=1;
    lcd_delay();
    en=0;
    lcd_delay();
}

void show(unsigned char *s)
{
    while(*s) {
        dat(*s++);
    }
}

void lcd_delay()
{
    unsigned int lcd_delay;
    for(lcd_delay=0;lcd_delay<=1000;lcd_delay++);
}

void keyinit()
{
    TRISB=0XF0;
    OPTION_REG&=0X7F;           //ENABLE PULL UP
}

unsigned char key()
{
    char no_push = 0 ;
    PORTB=0X00;
    //while(C1&&C2&&C3&&C4);
    while(!C1||!C2||!C3||!C4) {
        no_push = 1 ;
        R1=0;
        R2=R3=R4=1;
        if(!C1||!C2||!C3||!C4) {
            rowloc=0;
            break;
        }
        R2=0;R1=1;
        if(!C1||!C2||!C3||!C4) {
            rowloc=1;
            break;
        }
        R3=0;R2=1;
        if(!C1||!C2||!C3||!C4) {
            rowloc=2;
            break;
        }
        R4=0; R3=1;
        if(!C1||!C2||!C3||!C4){
            rowloc=3;
            break;
        }
    }
    if(C1==0&&C2!=0&&C3!=0&&C4!=0)
            colloc=0;
    else if(C1!=0&&C2==0&&C3!=0&&C4!=0)
            colloc=1;
    else if(C1!=0&&C2!=0&&C3==0&&C4!=0)
            colloc=2;
    else if(C1!=0&&C2!=0&&C3!=0&&C4==0)
            colloc=3;
    while(C1==0||C2==0||C3==0||C4==0);
    if( no_push == 0 ) return 255 ;
    return (keypad[rowloc][colloc]);
}
//////////////////////////////////(İbrahim Davutoğlu)/////////////////////////////////////////////
void show_num(int number){
    if( number >= 1000 ){
        dat( ( number/1000 + '0' ) );
        number %= 1000 ;
        dat( ( number/100 + '0' ) );
        number %= 100 ;
        dat( ( number/10 + '0' ) );
        number %= 10 ;
        dat( ( number/1 + '0' ) );
        number %= 1 ;
    }
    else if( number < 1000 && number >= 100 ){
        dat( ( number/100 + '0' ) );
        number %= 100 ;
        dat( ( number/10 + '0' ) );
        number %= 10 ;
        dat( ( number/1 + '0' ) );
        number %= 1 ;
        dat(' ');
    }
    else if( number < 100 && number >= 10 ){
        dat( ( number/10 + '0' ) );
        number %= 10 ;
        dat( ( number/1 + '0' ) );
        number %= 1 ;
        dat(' ');
        dat(' ');
    }
    else if( number < 10 && number >= 1 ){
        dat( ( number/1 + '0' ) );
        number %= 1 ;
        dat(' ');
        dat(' ');
        dat(' ');
    }
    else if(number == 0){
        dat('0');
        dat(' ');
        dat(' ');
        dat(' ');
    }
    return;
}
////////////////////////////////////////////////////////////////////////////////

int lastBeatTime = 0;      // Time of the last beat
int IBI = 600;             // Inter-Beat Interval initialized to 600ms
int Pulse = 0;             // Pulse indicator
int BPM = 0;               // Beats Per Minute
int Signal;                // Holds the analog signal value from the Pulse Sensor
int sampleCounter = 0;     // Used to determine the time between beats
int P = 512;               // Peak value
int T = 512;               // Trough value
int thresh = 512;          // Threshold value
int amp = 100;             // Amplitude value
int firstBeat = 1;         // Indicates if it's the first beat
int secondBeat = 0;        // Indicates if it's the second beat

void __interrupt() isr(void) {
    if (TMR1IF == 1) {
        TMR1H = 0x3C; // Timer 1 count for 1ms at 20MHz with prescaler 8
        TMR1L = 0xB0;
        TMR1IF = 0; // Clear Timer1 overflow interrupt flag
        
        sampleCounter += 2; // Keep track of time in ms
        int N = sampleCounter - lastBeatTime; // Time since the last beat
        Go = N ;

        Signal = __adc_read(ADC_PIN); // Read the Pulse Sensor signal

        // Find the peak and trough
        if (Signal < thresh && N > (IBI / 5) * 3) { // Avoid high freq noise
            if (Signal < T) {
                T = Signal; // Update trough
            }
        }
        
        if (Signal > thresh && Signal > P) { // Signal surges to beat
            P = Signal; // Update peak
        }

        // If it's time to detect a beat
        if (N > 250) { // 250ms per beat to avoid interference
            if ((Signal > thresh) && (N > (IBI / 5) * 3)) {
                Pulse = 1;
                IBI = sampleCounter - lastBeatTime;
                lastBeatTime = sampleCounter;

                if (secondBeat) {
                    secondBeat = 0;
                    // Calculate the BPM
                    BPM = 60000 / IBI;
                    int i;
                    for (i = 0; i <= 9; i++) { // seed the running total to get a realisitic BPM at startup
                        rate[i] = IBI;
                    }
                }

                if (firstBeat) {
                    firstBeat = 0;
                    secondBeat = 1;
                    return;
                }
                int runningTotal = 0; // clear the runningTotal variable
                int i;
                for (i = 0; i <= 8; i++) { // shift data in the rate array
                    rate[i] = rate[i + 1]; // and drop the oldest IBI value
                    runningTotal += rate[i]; // add up the 9 oldest IBI values
                }

                rate[9] = IBI; // add the latest IBI to the rate array
                runningTotal += rate[9]; // add the latest IBI to runningTotal
                runningTotal /= 10; // average the last 10 IBI values
                BPM = 60000 / runningTotal; // how many beats can fit into a minute? that's BPM!
            }
        }

        if (Signal < thresh && Pulse == 1) { // Beat finishes
            Pulse = 0;
            amp = P - T;
            thresh = amp / 2 + T;
            P = thresh;
            T = thresh;
        }

        if (N > 2500) { // No beat found, reset
            thresh = 512;
            P = 512;
            T = 512;
            lastBeatTime = sampleCounter;
            firstBeat = 1;
            secondBeat = 0;
            BPM = 0;
            IBI = 600;
            Pulse = 0;
            amp = 100;
        }
    }
}
/////////////////////////////////(khaled ghazzal)//////////////////////////////////////////////
#define lin1()  (cmd(0x80))
#define lin2()  (cmd(0xC0))

int IDs[10];
int Ages[10];
int BPMs[10];
int chossenIndex = -1 ;

void main()
{
    TRISD=0;
    lcd_init();
    keyinit();
    cmd(0x80);
    show("   Welcome TO   ");
    __delay_ms(500);
    cmd(0x01);
    cmd(0x80);
    show("   Heart Rate   ");
    cmd(0xC0);
    show("     Project    ");
    __delay_ms(500);
    cmd(0x01);
    cmd(0x80);
    show("Using PIC16F877A");
    __delay_ms(500);
    cmd(0x01);
    cmd(0x80);
    show("Press any key ....");
//    unsigned char n = key();
//    while(1){
//        if( n != 255 ){
//            cmd(0x01);
//            break;
//        }
//    }
    
    while(1)
    {
        int flag = 0 ;
        unsigned char k = key();
        if( k != 255 ){
            cmd(0x01);
            cmd(0x80);
            if( k == '/' ){
                show("Enter ID : ");
                IDagain:
                k = key();
                if( k >= '1' && k <= '9'){
                    dat(k);
                    chossenIndex = (int)( k - '0' );
                    IDs[chossenIndex] = chossenIndex ;
                }
                else{
                    goto IDagain ;
                }
                while(1){
                    k = key() ;
                    if( k == '=' ){
                        break;
                    }
                    else{
                        continue;
                    }
                }
            }
            if( k == '*' ){
                char agebuf[2] ;
                show("Enter Age : ");
                Ageagain1:
                k = key();
                if( k >= '1' && k <= '9' ){
                    dat(k);
                    agebuf[0] = k ;
                }
                else{
                    goto Ageagain1 ;
                }   
                Ageagain2:
                k = key();
                if( k >= '0' && k <= '9' ){
                    dat(k);
                    agebuf[1] = k;
                }
                else{
                    goto Ageagain2 ;
                }   
                int age = (int)( agebuf[0] - '0' )*10 + (int)( agebuf[1] - '0' ) ;
                Ages[chossenIndex] = age ;
                while(1){
                    k = key() ;
                    if( k == '=' ){
                        break;
                    }
                    else{
                        continue;
                    }
                }
            }
            if( k == '+' ){
                show("Start Calculating ");
                cmd(0xC0);
                show("BMP and IBI");
                __delay_ms(1000);
                cmd(0x01);
                __init_adc();
                TRISA = 0xFF;           // Set PORTA as input for ADC
                __init_adc();           // Initialize ADC
                __init_timer__();       // Initialize Timer
                __init_interrupt__();   // Initialize Interrupts
                Calcagain:
                k = key();
                if( k == '-' ){
                    flag = 1 ;
                    __delay_ms(500);
                    cmd(0x01);
                }
                else{
                    cmd(0x80);
                    show("BPM :- ");
                    show_num(BPM);
                    cmd(0xC0);
                    show("IBI :- ");
                    show_num(IBI);
                    goto Calcagain ;
                }
            }
            if( k == '-' && flag == 1 ){
                show("End Calculating ");
                __delay_ms(1000);
                cmd(0x01);
                flag = 0 ;
                BPMs[chossenIndex] = BPM ;
                ADCON0bits.ADON = 0; // Disable the ADC module
                PIE1bits.TMR1IE = 0; // Disable Timer1 overflow interrupt
                T1CONbits.TMR1ON = 0; // Disable Timer1
                GIE = 0;    // Disable global interrupts
                PEIE = 0;   // Disable peripheral interrupts
                IBI = 0 ;
                BPM = 0 ;
            }
            if( k == 'C' ){
                show("Show Data ");
                cmd(0xC0);
                show("Enter ID : ");
                Showagain:
                k = key();
                if( k >= '1' && k <= '9'){
                    dat(k);
                    __delay_ms(500);
                    chossenIndex = (int)( k - '0' );
                }
                else{
                    goto Showagain ;
                }
                int IDGlo = chossenIndex;
                int AgeGlo = Ages[chossenIndex];
                int BPMGlo = BPMs[chossenIndex];
                cmd(0x01);
                cmd(0x80);
                show("ID:");
                show_num(IDGlo);
                show("Age:");
                show_num(AgeGlo);
                cmd(0xC0);
                show("BPM:");
                show_num(BPMGlo);
                while(1){
                    k = key() ;
                    if( k == '=' ){
                        break;
                    }
                    else{
                        continue;
                    }
                }
            }
        }
    }   
}



