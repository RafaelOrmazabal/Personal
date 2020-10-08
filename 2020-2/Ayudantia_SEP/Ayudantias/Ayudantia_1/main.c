#include <msp430.h> 

#define ROJO 4
#define VERDE 5
#define AZUL 4
#define BOTON 1

#define APAGADO 0
#define TENUE 1
#define INTERMEDIO 2
#define FULL 3

#define NEXT 1
#define WAIT 0

//Rojo
volatile char next_state_1=WAIT;
volatile char next_state_2=WAIT;
volatile char next_state_3=WAIT;

#include "GPIO_config.h"
#include "Timer_config.h"
/**
 * main.c
 */
int main(void)
{
    char state_color_1=0;
    char state_color_2=0;
    char state_color_3=0;

	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	

	pin_init();
	PWM_init();
	timer_init();

	__bis_SR_register(GIE);

	while (1){
	//Dependiendo del estado, la intensidad de la luz del siguiente estado
	//ROJO:
	    if (next_state_1){
            switch (state_color_1){
                case APAGADO:
                    //Dependiendo del color que deseamos, se puede cambiar el registro
                    TA2CCR1=32;
                    state_color_1=TENUE;
                    break;
                case TENUE:
                    TA2CCR1=64;
                    state_color_1=INTERMEDIO;
                    break;
                case INTERMEDIO:
                    TA2CCR1=128;
                    state_color_1=FULL;
                    break;
                case FULL:
                    TA2CCR1=0;
                    state_color_1=APAGADO;
                    break;
                default:
                    TA2CCR1=0;
                    state_color_1=APAGADO;
                    break;
                }
                next_state_1=WAIT;
	    }
	//VERDE:
        if (next_state_2){
            switch (state_color_2){
                case APAGADO:
                    //Dependiendo del color que deseamos, se puede cambiar el registro
                    TA0CCR4=32;
                    state_color_2=TENUE;
                    break;
                case TENUE:
                    TA0CCR4=64;
                    state_color_2=INTERMEDIO;
                    break;
                case INTERMEDIO:
                    TA0CCR4=128;
                    state_color_2=FULL;
                    break;
                case FULL:
                    TA0CCR4=0;
                    state_color_2=APAGADO;
                    break;
                default:
                    TA0CCR4=0;
                    state_color_2=APAGADO;
                    break;
                }
                next_state_2=WAIT;
        }

    //AZUL:
        if (next_state_3){
            switch (state_color_3){
                case APAGADO:
                    //Dependiendo del color que deseamos, se puede cambiar el registro
                    TA0CCR3=32;
                    state_color_3=TENUE;
                    break;
                case TENUE:
                    TA0CCR3=64;
                    state_color_3=INTERMEDIO;
                    break;
                case INTERMEDIO:
                    TA0CCR3=128;
                    state_color_3=FULL;
                    break;
                case FULL:
                    TA0CCR3=0;
                    state_color_3=APAGADO;
                    break;
                default:
                    TA0CCR3=0;
                    state_color_3=APAGADO;
                    break;
                }
                next_state_3=WAIT;
        }
	}

	return 0;
}


#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void){
    //revisar que CC fue el que gatilló la interrupción
    switch (TA1IV){
        case 2:
            next_state_1=NEXT;
            break;
        case 4:
            next_state_1=NEXT;
            next_state_2=NEXT;
            break;
        case 0x0E:
            next_state_1=NEXT;
            next_state_2=NEXT;
            next_state_3=NEXT;
            break;
        default:
            next_state_1=NEXT;
            break;
        }
}
