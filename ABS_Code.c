//PROGRAM: ABS
//WRITTEN BY: Massimo Clementi
//DATA: 11/01/2016
//VERSION: 3.0
//FILE SAVED AS: ABS_Code.c
//FOR PIC: 18F4480
//CLOCK FREQUENCY: 16 MHz
//PROGRAM FUNCTION: Centralina che gestisce la frenata del veicolo secondo i
// valori che vengono forniti dalle altre centraline attraverso il protocollo 
// CANbus. Predisposizione di led di malfunzionamento del CANbus, arresto e
// trimmer su scheda per il controllo della correzione della frenata.
// [AGGIUNGERE DESCRIZIONE ENCODER]
// [AGGIUNGERE CONDIZIONE OVERFLOW TIMERS]
// [AGGIUNGERE PROGRAMMA COUNT]

////////////////////////////            //////////////////////////////
//   INPUT AND OUTPUTS    //            //  COMBINAZIONI BIT CANBus //
//  RA0 => Trimmer (ADC)  //            //   00 = Nessuna frenata   //
//  RA1 => Warning LED    //            //   01 = Livello basso     //
//  RB0 => ENCODER 1      //            //   10 = Livello medio     //
//  RB1 => ENCODER 2      //            //   11 = Livello alto      //
//  RB2/RB3 => CANBus     //            //////////////////////////////
//  RC0 => PWM TMR0       //
//  RC1 => Yellow LED     //
////////////////////////////

// Implementazione di un bit del byte[1] del CANBus che permette la gestione
// analogica della frenata (per il radiocomando che necessita di un comando
// analogico). Al settaggio di questo bit il valore inviato nel primo byte sar�
// direttamente quello del valore di frenata.
//
// N.B: il bit deve essere presente per ogni pacchetto di dati del CANBus che si
//      desidera interpretare come analogico.
//
// TIMER0 => PWM
// 800us  => 0�
// 1500us => 90�
// 2200us => 180�
// 500ns ad ogni singolo incremento (prescaler 1:2)
//
// ENCODER 1: INT1 (RB1)
//            TMR1
//            2us ad incremento
//
// ENCODER 2: INT2 (RB2)
//            TMR3
//            2us ad incremento

#define USE_AND_MASKS

#include <xc.h>
#include "PIC18F4480_config.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "pwm.h"
#include "timers.h"
#include "idCan.h"
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
#define speed 0b00000000000000000000000001000 //(!!) impostare
#define count_start 0b00000000000000000000000001010 //(!!) impostare
#define count_stop 0b00000000000000000000000001011 //(!!) impostare
#define wheel_diameter 11 //in cm

//Prototitpi delle funzioni
void board_initialization(void);
void remote_frame_handler(void);
void ADC_Read(void);

///////////////////
// Declarations  //
///////////////////

//CANbus
CANmessage msg;
bit remote_frame = LOW;
bit Tx_retry = LOW;
bit count_flag = LOW;
unsigned long remote_frame_id = 0;
BYTE status_array [8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; //Codice 1 => ABS
BYTE speed_array [8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
BYTE distance_array [8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char brake_signal_CAN = 0;
unsigned char Analogic_Mode = 0;

//PWM TMR0
unsigned long timer_on = 0;
unsigned long timer_off = 0;

//ENCODER 1
bit x = LOW;
bit ENC1_measure = LOW;
unsigned long gap_time_1 = 0;
unsigned char wheel_speed_1 = 0;
unsigned long int_counter_1 = 0;
unsigned int distance_1 = 0;

//ENCODER 2
bit y = LOW;
bit ENC2_measure = LOW;
unsigned long gap_time_2 = 0;
unsigned char wheel_speed_2 = 0;
unsigned long int_counter_2 = 0;
unsigned int distance_2 = 0;

//ADC
unsigned char read = 0;
int correction_factor = 0; //con segno
unsigned char home_position = 0;

//Program variables
unsigned char brake_value_inc = 0; //0-256 (fattore 17)
unsigned char brake_value = 0; //0-15
unsigned char brake_value_degree = 0; //0-180 gradi
unsigned char step = 0; //passo della ruota in cm


//////////////////////////////////
//    INTERRUPT High Priority   //
//////////////////////////////////

__interrupt(high_priority) void ISR_Alta(void) {
    //INTERRUPT PWM
    if (INTCONbits.TMR0IF == HIGH) {
        PORTCbits.RC0 = ~PORTCbits.RC0;
        if (PORTCbits.RC0 == 1) {
            timer_on = (((1400 * brake_value_degree) / 180) + 800)*2;
            timer_off = 65536 - (40000 - timer_on);
            timer_on = 65536 - timer_on;
            WriteTimer0(timer_on);
        } else {
            WriteTimer0(timer_off);
        }
        INTCONbits.TMR0IF = LOW;
    }

    //INTERRUPT ENCODER 1
    if (INTCONbits.INT0IF == HIGH) {
        if (x == LOW) {
            TMR1H = 0x00;
            TMR1L = 0x00;
            x = HIGH;
        } else {
            gap_time_1 = (TMR1H <<8) + TMR1L;
            gap_time_1 = gap_time_1 / 500; //in ms
            ENC1_measure = HIGH;
            TMR1H = 0x00;
            TMR1H = 0x00;
            if (count_flag == HIGH) {
                int_counter_1++;
            }
        }
        INTCONbits.INT0IF = LOW;
    }

    //INTERRUPT ENCODER 2
    if (INTCON3bits.INT1IF == HIGH) {
        if (y == LOW) {
            TMR3H = 0x00;
            TMR3L = 0x00;
            y = HIGH;
        } else {
            gap_time_2 = (TMR3H <<8) + TMR3L;
            gap_time_2 = gap_time_2 / 500; //in ms
            ENC2_measure = HIGH;
            TMR3H = 0x00;
            TMR3H = 0x00;
            if (count_flag == HIGH) {
                int_counter_2++;
            }
        }
        INTCON3bits.INT1IF = LOW;
    }
}


//////////////////////////////
//  INTERRUPT Low Priority  //
/////////////////////////////

__interrupt(low_priority) void ISR_Bassa(void) {
    //INTERRUPT CANBUS
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
    step = (wheel_diameter * (3,1415)) / 16;
    while (1) {
        ADC_Read();

        if ((CANisTXwarningON() == HIGH) || (CANisRXwarningON() == HIGH)) {
            PORTBbits.RB0 = HIGH; //accendi led errore
        } else {
            PORTBbits.RB0 = LOW;
        }

        if ((remote_frame == HIGH) || (Tx_retry == HIGH)) {
            remote_frame = LOW;
            remote_frame_handler();
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

        if (ENC1_measure == HIGH) {
            wheel_speed_1 = (step * 36) / gap_time_1;
            speed_array[1] = wheel_speed_1;
            ENC1_measure = LOW;
        }

        if (ENC2_measure == HIGH) {
            wheel_speed_2 = (step * 36) / gap_time_2;
            speed_array[0] = wheel_speed_2;
            ENC2_measure = LOW;
        }
    }
}


/////////////////////
//   Subroutines   //
////////////////////

void remote_frame_handler(void) {
    if (CANisTxReady()) {
        if (remote_frame_id == status_id) {
            CANsendMessage(remote_frame_id, status_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }
        if (remote_frame_id == speed) {
            CANsendMessage(remote_frame_id, speed_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }
        if (remote_frame_id == count_start) {
            int_counter_1 = 0;
            int_counter_2 = 0;
            count_flag = HIGH;
        }
        if (remote_frame_id == count_stop) {
            distance_1 = step * (int_counter_1);
            distance_2 = step * (int_counter_2);
            count_flag = LOW;
            distance_array[0] = distance_1;
            distance_array[1] = distance_1 >> 8;
            distance_array[2] = distance_2;
            distance_array[3] = distance_2 >> 8;
            CANsendMessage(remote_frame_id, distance_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }
        if (TXB0CONbits.TXABT || TXB1CONbits.TXABT) {
            Tx_retry = HIGH;
        } else {
            Tx_retry = LOW;
        }
    } else {
        Tx_retry = HIGH;
    }
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
    TRISB = 0b11111110; //RBO OUTPUT, RB1 e RB2 INPUT
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
    PIR3bits.RXB1IF = LOW; //azzera flag interrupt can bus buffer1
    PIR3bits.RXB0IF = LOW; //azzera flag interrupt can bus buffer0
    INTCONbits.TMR0IF = LOW; //azzera flag TMR0
    INTCONbits.INT0IF = LOW; //azzera flag INT0
    INTCON3bits.INT1IF = LOW; //azzera flag INT1
    PIR2bits.TMR3IF = LOW; //resetta flag interrupt timer 3

    //Config. Priorit�
    RCONbits.IPEN = 1;
    IPR3bits.RXB1IP = 0; //interrupt bassa priorit� per can
    IPR3bits.RXB0IP = 0; //interrupt bassa priorit� per can
    INTCON2bits.TMR0IP = 1; //interrupt alta priorit� timer0
    INTCON3bits.INT1IP = 1; //interrupt alta priorit� INT1

    //Config. Registri
    T0CON = 0x80; //imposta timer0, prescaler 1:2

    //  Valori per forzare un interrupt del TMR0 all'avvio della periferica ed
    //  inizializzarla correttamente. RC0 = 0 in modo che il primo ciclo venga
    //  eseguito il toggle nella ISR di gestione TMR0 e quindi venga eseguito il 
    //  calcolo di timer_on e timer_off in base al valore di break_value_degree
    //  fornito (in questo caso 90 gradi in modo che si metta in posizione centrale)
    TMR0H = 0xFF;
    TMR0L = 0xFE;
    PORTCbits.RC0 = 0;
    brake_value_degree = 90;
    T1CON = 00010000;
    T3CON = 01010000;
    INTCON2bits.INTEDG0 = 1;
    INTCON2bits.INTEDG1 = 1;

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
    INTCONbits.INT0IE = 1; //abilita interrupt INT0
    INTCON3bits.INT1IE = 1; //abilita interrupt INT1
    INTCONbits.GIEH = 1; //abilita interrupt alta priorit�
    INTCONbits.GIEL = 1; //abilita interrupt bassa priorit� periferiche

    //Enable Timers
    T1CONbits.TMR1ON = 1; //atttivazione TMR1
    T3CONbits.TMR3ON = 1; //attivazione TMR3
    T0CONbits.TMR0ON = 1; //attivazione TMR0
    delay_ms(2);
}