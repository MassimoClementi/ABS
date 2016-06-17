//PROGRAM: ABS
//WRITTEN BY: Massimo Clementi
//DATA: 11/01/2016
//VERSION: 3.0
//FILE SAVED AS: ABS_Code.c
//FOR PIC: 18F4480
//CLOCK FREQUENCY: 16 MHz

////////////////////////////            //////////////////////////////
//   INPUT AND OUTPUTS    //            //  CANBUS Bit Combinations //
//  RA0 => Trimmer (ADC)  //            //   00 = Nessuna frenata   //
//  RA1 => Warning LED    //            //   01 = Livello basso     //
//  RB0 => ENCODER 1      //            //   10 = Livello medio     //
//  RB1 => ENCODER 2      //            //   11 = Livello alto      //
//  RB2/RB3 => CANBus     //            //////////////////////////////
//  RC0 => PWM TMR0       //
//  RC1 => Yellow LED     //
////////////////////////////

#define USE_AND_MASKS
#define _XTAL_FREQ 16000000

#include <xc.h>
#include "PIC18F4480_config.h"
#include "CANlib.h"
#include "delay.c"
#include "delay.h"
#include "pwm.h"
#include "timers.h"
#include "idCan.h"
#include "math.h"

#define HIGH 1
#define LOW 0

#define wheel_diameter 11 //in cm
#define BRAKE_ANGLE_RATIO 1 //Default value: 17
#define AVRG_RTIO 7

//Prototitpi delle funzioni
void board_initialization(void);
void remote_frame_handler(void);
void ADC_Read(void);

///////////////////
// Declarations  //
///////////////////

//CANbus
CANmessage msg;
volatile bit remote_frame = LOW;
volatile bit Tx_retry = LOW;
volatile bit count_flag = LOW;
volatile bit steering_dir = LOW;
volatile unsigned char brake_signal_CAN = 0;
volatile unsigned char Analogic_Mode = 0;
volatile unsigned long remote_frame_id = 0;
volatile BYTE status_array [8] = 0;
volatile BYTE speed_array [8] = 0;
volatile BYTE distance_array [8] = 0;
volatile BYTE remote_frame_array [8] = 0x01;

//PWM TMR0
volatile unsigned long timer_on = 0;
volatile unsigned long timer_off = 0;

//ENCODER 1
volatile bit x = LOW;
volatile bit ENC1_measure = LOW;
volatile bit TMR1_overflow = LOW;
volatile unsigned int gap_time_1 = 0; //[ms]
volatile unsigned int int_counter_1_count = 0;
volatile unsigned int distance_1 = 0; //[cm]
volatile unsigned long wheel_speed_1 = 0; //[mm/s] MIN=26 MAX=3455
volatile unsigned long int_counter_1 = 0;
volatile unsigned long long wheel_speed_average_1 = 0;

//ENCODER 2
volatile bit y = LOW;
volatile bit ENC2_measure = LOW;
volatile bit TMR3_overflow = LOW;
volatile unsigned int gap_time_2 = 0; //[ms]
volatile unsigned int int_counter_2_count = 0;
volatile unsigned int distance_2 = 0; //[cm]
volatile unsigned long wheel_speed_2 = 0; //[mm/s] vedi sopra
volatile unsigned long int_counter_2 = 0;
volatile unsigned long long wheel_speed_average_2 = 0;

//Distance set function
volatile bit distance_set_flag = LOW;
volatile bit distance_reached_flag = LOW;
volatile unsigned int distance_set_value = 0; //[cm]
volatile unsigned int distance_set_counter_1 = 0;
volatile unsigned int distance_set_counter_2 = 0;
volatile unsigned long distance_actual_value = 0; //[cm]

//Program variables
volatile unsigned char home_position = 0;
volatile unsigned char brake_value_inc = 0; //0-256 (factor 17)
volatile unsigned char brake_value = 0; //0-15 [degree]
volatile unsigned char brake_value_degree = 0; //0-180 [degree]
volatile unsigned char step = 0; //[cm]


//////////////////////////////////
//    INTERRUPT High Priority   //
//////////////////////////////////

__interrupt(high_priority) void ISR_Alta(void) {
    //INTERRUPT PWM
    if (INTCONbits.TMR0IF == HIGH) {
        PORTCbits.RC0 = ~PORTCbits.RC0;
        if (PORTCbits.RC0 == HIGH) {
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
            gap_time_1 = (TMR1H << 8) + TMR1L;
            gap_time_1 = gap_time_1 / 500; //[ms]
            ENC1_measure = HIGH;
            TMR1H = 0x00;
            TMR1L = 0x00;
            if (count_flag == HIGH) {
                int_counter_1++;
            }
            if (distance_set_flag == HIGH) {
                distance_set_counter_1++;
                if ((steering_dir == 1)&&(distance_set_counter_1 > distance_set_value)) {
                    distance_reached_flag = HIGH;
                    PORTAbits.RA1 = HIGH;
                }
                if ((steering_dir == 0)&&(distance_set_counter_2 > distance_set_value)) {
                    distance_reached_flag = HIGH;
                    PORTAbits.RA1 = HIGH;
                }
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
            gap_time_2 = (TMR3H << 8) + TMR3L;
            gap_time_2 = gap_time_2 / 500; // [ms]
            ENC2_measure = HIGH;
            TMR3H = 0x00;
            TMR3L = 0x00;
            if (count_flag == HIGH) {
                int_counter_2++;
            }
            if (distance_set_flag == HIGH) {
                distance_set_counter_2++;
                if ((steering_dir == 1)&&(distance_set_counter_1 > distance_set_value)) {
                    distance_reached_flag = HIGH;
                    PORTAbits.RA1 = HIGH;
                }
                if ((steering_dir == 0)&&(distance_set_counter_2 > distance_set_value)) {
                    distance_reached_flag = HIGH;
                    PORTAbits.RA1 = HIGH;
                }
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
            if (msg.identifier == STEERING_CHANGE) {
                if (msg.data[0] >= 90) {
                    steering_dir = 1;
                } else {
                    steering_dir = 0;
                }
            }
            if (msg.identifier == BRAKE_SIGNAL) {
                brake_signal_CAN = msg.data[0];
                Analogic_Mode = msg.data[1];
            }
            if (msg.identifier == DISTANCE_SET) {
                distance_set_value = (msg.data[0]) / step;
                distance_set_counter_1 = 0;
                distance_set_counter_2 = 0;
                distance_reached_flag = LOW;
                distance_set_flag = HIGH;
                PORTAbits.RA1 = LOW;
            }
        }
        PIR3bits.RXB0IF = LOW;
        PIR3bits.RXB1IF = LOW;
    }

    //INTERRUPT TMR1
    if (PIR1bits.TMR1IF == HIGH) {
        TMR1_overflow = HIGH;
        TMR1H = 0x00;
        TMR1L = 0x00;
        PIR1bits.TMR1IF = LOW;
    }

    //INTERRUPT TMR3
    if (PIR2bits.TMR3IF == HIGH) {
        TMR3_overflow = HIGH;
        TMR3H = 0x00;
        TMR3L = 0x00;
        PIR2bits.TMR3IF = LOW;
    }
}

//////////////
//   MAIN   //
//////////////

int main(void) {
    board_initialization();
    step = 2; //[cm]
    home_position = 28;

    while (1) {
        //Warning CANBUS Management
        if ((CANisTXwarningON() == HIGH) || (CANisRXwarningON() == HIGH)) {
            PORTAbits.RA1 = HIGH;
            COMSTATbits.TXWARN = LOW;
            COMSTATbits.RXWARN = LOW;
        }

        //DISTANCE_SET Check
        if (distance_reached_flag == HIGH) {
            CANsendMessage(DISTANCE_SET, remote_frame_array, 8, CAN_CONFIG_STD_MSG & CAN_REMOTE_TX_FRAME & CAN_TX_PRIORITY_0);
            distance_reached_flag = LOW;
        }

        if ((remote_frame == HIGH) || (Tx_retry == HIGH)) {
            remote_frame = LOW;
            remote_frame_handler();
        }

        //Analogic/Digital brake
        if (Analogic_Mode == 0b00000000) {
            if (brake_signal_CAN == 0b00000000) {
                brake_value_inc = 0;
            }
            if (brake_signal_CAN == 0b00000001) {
                brake_value_inc = 150;
            }
            if (brake_signal_CAN == 0b00000010) {
                brake_value_inc = 200;
            }
            if (brake_signal_CAN == 0b00000011) {
                brake_value_inc = 255;
            }
        } else {
            brake_value_inc = brake_signal_CAN;
        }

        //Degree calculation
        if (((brake_value_inc / BRAKE_ANGLE_RATIO) + home_position) > 255) {
            brake_value = 255;
            brake_value_degree = 180;
        } else {
            brake_value = (brake_value_inc / BRAKE_ANGLE_RATIO) + home_position;
            brake_value_degree = (180 * brake_value) / 255;
        }

        //Wheel speed 1 calculation and packing
        if ((ENC1_measure == HIGH) || (TMR1_overflow == HIGH)) {
            if (TMR1_overflow == HIGH) {
                wheel_speed_1 = 0;
                TMR1_overflow = LOW;
            } else {
                wheel_speed_1 = (step * 10000) / gap_time_1;
            }

            wheel_speed_average_1 = wheel_speed_average_1 + wheel_speed_1;
            int_counter_1_count++;
            if (int_counter_1_count == AVRG_RTIO) {
                wheel_speed_1 = wheel_speed_average_1 / AVRG_RTIO;
                speed_array[3] = wheel_speed_1 >> 8;
                speed_array[2] = wheel_speed_1;
                wheel_speed_average_1 = 0;
                int_counter_1_count = 0;
                ENC1_measure = LOW;
            }
        }

        //Wheel speed 2 calculation and packing
        if ((ENC2_measure == HIGH) || (TMR3_overflow == HIGH)) {
            if (TMR3_overflow == HIGH) {
                wheel_speed_2 = 0;
                TMR3_overflow = LOW;
            } else {
                wheel_speed_2 = (step * 10000) / gap_time_2;
            }

            wheel_speed_average_2 = wheel_speed_average_2 + wheel_speed_2;
            int_counter_2_count++;
            if (int_counter_2_count == AVRG_RTIO) {
                wheel_speed_2 = wheel_speed_average_2 / AVRG_RTIO;
                speed_array[1] = wheel_speed_2 >> 8;
                speed_array[0] = wheel_speed_2;
                wheel_speed_average_2 = 0;
                int_counter_2_count = 0;
                ENC2_measure = LOW;
            }
        }
    }
}


/////////////////////
//   Subroutines   //
////////////////////

void remote_frame_handler(void) {
    if (CANisTxReady() == 1) {
        if (remote_frame_id == ECU_STATE_ABS) {
            status_array[0] = 0x01;
            CANsendMessage(ECU_STATE_ABS, status_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            PORTCbits.RC1 = ~PORTCbits.RC1;
        }

        if (remote_frame_id == ACTUAL_SPEED) {
            CANsendMessage(remote_frame_id, speed_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
            while (CANisTxReady() != HIGH);
            CANsendMessage(0xAA, speed_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }

        if (remote_frame_id == COUNT_START) {
            int_counter_1 = 0;
            int_counter_2 = 0;
            count_flag = HIGH;
        }

        if (remote_frame_id == COUNT_STOP) {
            distance_1 = step * (int_counter_1);
            distance_2 = step * (int_counter_2);
            count_flag = LOW;
            distance_array[0] = distance_1;
            distance_array[1] = distance_1 >> 8;
            distance_array[2] = distance_2;
            distance_array[3] = distance_2 >> 8;
            CANsendMessage(remote_frame_id, distance_array, 8, CAN_CONFIG_STD_MSG & CAN_NORMAL_TX_FRAME & CAN_TX_PRIORITY_0);
        }

        if (TXB0CONbits.TXABT == 1 || TXB1CONbits.TXABT == 1) {
            Tx_retry = HIGH;
        } else {
            Tx_retry = LOW;
        }
    } else {
        Tx_retry = HIGH;
    }
}

void board_initialization(void) {
    //I/O Configurations
    LATA = 0x00;
    TRISA = 0b11111101;
    LATB = 0x00;
    TRISB = 0b11111111;
    LATC = 0x00;
    TRISC = 0b11111100;
    LATD = 0x00;
    TRISD = 0xFF;
    LATE = 0x00;
    TRISE = 0xFF;

    //CANbus Configurations
    CANInitialize(4, 6, 5, 1, 3, CAN_CONFIG_LINE_FILTER_OFF & CAN_CONFIG_SAMPLE_ONCE & CAN_CONFIG_ALL_VALID_MSG & CAN_CONFIG_DBL_BUFFER_ON);

    //Clear Flag Interrupt 
    PIR3bits.RXB1IF = LOW;
    PIR3bits.RXB0IF = LOW;
    INTCONbits.TMR0IF = LOW;
    PIR1bits.TMR1IF = LOW;
    PIR2bits.TMR3IF = LOW;
    INTCONbits.INT0IF = LOW;
    INTCON3bits.INT1IF = LOW;


    //Priority Configurations
    RCONbits.IPEN = HIGH;
    IPR3bits.RXB1IP = LOW; //interrupt bassa priorità per can
    IPR3bits.RXB0IP = LOW; //interrupt bassa priorità per can
    INTCON2bits.TMR0IP = HIGH; //interrupt alta priorità timer0
    INTCON3bits.INT1IP = HIGH; //interrupt alta priorità INT1
    IPR1bits.TMR1IP = LOW; //interrupt bassa priorità timer1
    IPR2bits.TMR3IP = LOW; //interrupt bassa priorità timer3

    //Register Configurations
    T0CON = 0x80; //Prescaler 1:2
    TMR0H = 0xFF;
    TMR0L = 0xFE;
    PORTCbits.RC0 = LOW;
    brake_value_degree = 90;
    T1CON = 00110000;
    T3CON = 01110000;
    INTCON2bits.INTEDG0 = HIGH;
    INTCON2bits.INTEDG1 = HIGH;

    //Interrupts Enable
    PIE3bits.RXB1IE = HIGH;
    PIE3bits.RXB0IE = HIGH;
    INTCONbits.TMR0IE = HIGH;
    PIE1bits.TMR1IE = HIGH;
    PIE2bits.TMR3IE = HIGH;
    INTCONbits.INT0IE = HIGH;
    INTCON3bits.INT1IE = HIGH;
    INTCONbits.GIEH = HIGH;
    INTCONbits.GIEL = HIGH;

    //Timers Enable
    T1CONbits.TMR1ON = HIGH;
    T3CONbits.TMR3ON = HIGH;
    T0CONbits.TMR0ON = HIGH;
}