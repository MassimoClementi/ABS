//PROGRAM: ABS
//WRITTEN BY: Massimo Clementi
//DATA: 27/12/2015
//VERSION: 2.0
//FILE SAVED AS: ABS_Code.c
//FOR PIC: 18F4480
//CLOCK FREQUENCY: 16 MHz
//PROGRAM FUNCTION: Centralina che gestisce la frenata del veicolo secondo i
// valori che vengono forniti dalle altre centraline attraverso il protocollo 
// CANbus. Predisposizione di led di malfunzionamento del CANbus, arresto e
// trimmer su scheda per il controllo della correzione della frenata.

////////////////////////////            //////////////////////////////
//          I/O           //            //  COMBINAZIONI BIT CANBus //
//  RA0 => Trimmer (ADC)  //            //   00 = Nessuna frenata   //
//  RB0 => Warning LED    //            //   01 = Livello basso     //
//  RC0 => PWM TMR0       //            //   10 = Livello medio     //
//  CANRX/CANTX => CANBus //            //   11 = Livello alto      //
////////////////////////////            //////////////////////////////

// Implementazione di un bit del byte [1] del CanBus che permette la gestione
// analogica della frenata (per il radiocomando che necessita di un comando
// analogico). Al settaggio di questo bit il valore inviato nel primo byte sarà
// direttamente quello del valore di frenata.
//
// N.B: il bit deve essere presente per ogni pacchetto di dati del CANBus che si
//      desidera interpretare come analogico.
//
// TIMER0 => PWM
// 800us  => 0°
// 1500us => 90°
// 2200us => 180°
// 500ns ad ogni singolo incremento (prescaler 1:2)

#define USE_AND_MASKS

#include <xc.h>
#include "PIC18F4480_config.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "pwm.h"
#include "timers.h"
#define _XTAL_FREQ 16000000
#define HIGH 1
#define LOW 0

//////////////////////////////////////
//  10 GRADI DI AZIONE DEL SERVO    //
//      POSIZIONE HOME => 127       //
//      POSIZIONE BRAKE => 142      //
//////////////////////////////////////

#define brake_signal 0b00000000000000000000000000110 //(!!) impostare
#define status_id 0b00000000000000000000000000100 //(!!) impostare

//Prototitpi delle funzioni
void board_initialization(void);
void status_ok(void);
void ADC_Read(void);

//////////////////
//Declarations  //
//////////////////

//CANbus
CANmessage msg;
bit remote_frame;
bit Tx_retry;
unsigned long remote_frame_id = 0;
BYTE status_array [8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; //Sequenza di 101010...
unsigned char brake_signal_CAN;
unsigned char Analogic_Mode;

//PWM TMR0
unsigned long timer_on = 0;
unsigned long timer_off = 0;

//ADC
unsigned char read = 0;
int correction_factor = 0; //con segno
unsigned char home_position = 0;

//Program variables
unsigned char brake_value_inc = 0; //0-256 (fattore 17)
unsigned char brake_value = 0; //0-15
unsigned char brake_value_degree = 0; //0-180 gradi

//////////////////////////////////
//    INTERRUPT High Priority   //
//     Gestione PWM Virtuale    //
//////////////////////////////////

__interrupt(high_priority) void ISR_Alta(void) {
    if (INTCONbits.TMR0IF == HIGH) {
        PORTCbits.RC0 = ~PORTCbits.RC0;
        T0CONbits.TMR0ON = 0;
        if (PORTCbits.RC0 == 1) {
            timer_on = (((1400 * brake_value_degree) / 180) + 800)*2; //incrementi TMR0
            timer_off = 65536 - (40000 - timer_on);
            timer_on = 65536 - timer_on; //interrupt per overflow
            WriteTimer0(timer_on);
            T0CONbits.TMR0ON = 1;
        } else {
            WriteTimer0(timer_off);
            T0CONbits.TMR0ON = 1;
        }
        INTCONbits.TMR0IF = LOW;
    }
}

//////////////////////////////
//  INTERRUPT Low Priority  //
//      Gestione CANBus     //
/////////////////////////////

__interrupt(low_priority) void ISR_Bassa(void) {
    if ((PIR3bits.RXB0IF == HIGH) || (PIR3bits.RXB1IF == HIGH)) {
        if (CANisRxReady()) {
            CANreceiveMessage(&msg);
            if (msg.RTR == HIGH) {
                remote_frame_id = msg.identifier;
                remote_frame = HIGH;
            }
            if (msg.identifier == brake_signal) {
                brake_signal_CAN = msg.data[0];
                Analogic_Mode = msg.data[1];
            }
        }
        PIR3bits.RXB0IF = LOW;
        PIR3bits.RXB1IF = LOW;
    }
}

//////////////
//   MAIN   //
//////////////

int main(void) {
    board_initialization();
    while (1) {
       // CANisTXwarningON() = LOW; // Spegnimento led di Warning
        // CANisRXwarningON() = LOW; // del CANBus
        ADC_Read();

        if ((CANisTXwarningON() == HIGH) || (CANisRXwarningON() == HIGH)) {
            PORTBbits.RB0 = HIGH; //accendi led errore
        } else {
            PORTBbits.RB0 = LOW;
        }

        if ((remote_frame == HIGH) || (Tx_retry == HIGH)) {
            status_ok();
        }

        if (Analogic_Mode == 0) {
            if (brake_signal_CAN == 00) {
                brake_value_inc = 0;
            }
            if (brake_signal_CAN == 01) {
                brake_value_inc = 150;
            }
            if (brake_signal_CAN == 10) {
                brake_value_inc = 200;
            }
            if (brake_signal_CAN == 11) {
                brake_value_inc = 255;
            }
        } else {
            brake_value_inc = brake_signal_CAN;
        }

        brake_value = (brake_value_inc / 17) + home_position;
        brake_value_degree = (180 * brake_value) / 255;
    }
}

void status_ok(void) {
    if (CANisTxReady()) {
        if (remote_frame_id == status_id) {
            CANsendMessage(remote_frame_id, status_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            if (TXB0CONbits.TXABT || TXB1CONbits.TXABT) {
                Tx_retry = HIGH;
            } else {
                Tx_retry = LOW;
            }
        }
    } else {
        Tx_retry = HIGH;
    }
    remote_frame = LOW;
}

void ADC_Read(void) {
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO);
    read = ADRESH;
    correction_factor = read - 127;
    home_position = correction_factor / 4 + 127; //RICONTROLLARE /4
}

void board_initialization(void) {
    //Configurazione I/O
    LATA = 0x00;
    TRISA = 0xFF; //ALL IN
    LATB = 0x00;
    TRISB = 0b11111110; //RBO OUTPUT
    LATC = 0x00;
    TRISC = 0b11111110; //RC0 OUTPUT
    LATD = 0x00;
    TRISD = 0xFF;
    LATE = 0x00;
    TRISE = 0xFF;

    ADCON1 = 0x11111110;

    //Configurazione CANbus
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);

    //Azzero Flag Interrupts
    PIR3bits.RXB1IF = 0; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = 0; //azzera flag interrupt can bus buffer0
    INTCONbits.TMR0IF = 0;
    PIR2bits.TMR3IF = 0; //resetta flag interrupt timer 3

    //Config. Priorità
    RCONbits.IPEN = 1;
    IPR3bits.RXB1IP = 0; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorità per can
    INTCON2bits.TMR0IP = 1; //interrupt alta priorità timer0

    //Config. Registri
    T0CON = 0x80; //imposta timer0, prescaler 1:2

    //  Valori per forzare un interrupt del TMR0 all'avvio della periferica ed
    //  inizializarla correttamente. RC0 = 0 in modo che il primo ciclo venga
    //  eseguito il toggle nella ISR di gestione TMR0 e quindi venga eseguito il 
    //  calcolo di timer_on e timer_off in base al valore di break_value_degree
    //  fornito (in questo caso 90 gradi in modo che si metta in posizione centrale)
    TMR0H = 0xFF;
    TMR0L = 0xFE;
    PORTCbits.RC0 = 0;
    brake_value_degree = 90;
    INTCON2bits.INTEDG0 = 1; //interrupt su fronte di salita [SERVE?]

    //Configurazione ADC
    ADCON1 = 0b01110111;
    ADCON0bits.CHS2 = 0; //<--|
    ADCON0bits.CHS1 = 0; //<--|- CANALE 0 => RB0
    ADCON0bits.CHS0 = 0; //<--|
    ADCON2bits.ACQT2 = 1;
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 0;
    ADCON2bits.ADCS2 = 1;
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
    ADCON2bits.ADFM = 0; //Left Justified
    ADCON0bits.ADON = 1;

    //Enable Interrupts
    PIE3bits.RXB1IE = 1; //abilita interrupt ricezione can bus buffer1
    PIE3bits.RXB0IE = 1; //abilita interrupt ricezione can bus buffer0
    INTCONbits.TMR0IE = 1; //abilita interrupt timer 0
    INTCONbits.GIEH = 1; //abilita interrupt alta priorità
    INTCONbits.GIEL = 1; //abilita interrupt bassa priorità periferiche

    //Enable Timers
    T3CON = 0x01;
    T0CONbits.TMR0ON = 1;

    delay_ms(2);
}