/*
 * Timer_config.c
 *
 *  Created on: Oct 8, 2020
 *      Author: 7cg2r
 */

#include "Timer_config.h"
#include <msp430.h>

//Queremos que un ciclo de la PWM sea similar a los 60 Hz y que el contador vaya de 0 a 255
//Lo último permite que fijar el cilo de trabajo sea intuitivo

void PWM_init(){
    //Verde y Azul
    //Usamos el ACLK: 32kHz
    TA0CTL|=TASSEL0;
    TA0CTL&=~TASSEL1;

    //divisor de clock: 1/4
    TA0CTL|=ID0;
    TA0CTL&=~ID1;

    //Fijamos el tope 255
    TA0CCR0=255;

    //Topes iniciales
    TA0CCR3=0;
    TA0CCR4=0;

    //Modo de operación
    //Reset(1)/Set(0)
    TA0CCTL4|=OUTMOD0|OUTMOD1|OUTMOD2;
    TA0CCTL3|=OUTMOD0|OUTMOD1|OUTMOD2;

    //Rojo
    TA2CTL|=TASSEL0;
    TA2CTL&=~TASSEL1;

    //divisor de clock: 1/4
    TA2CTL|=ID0;
    TA2CTL&=~ID1;

    //Fijamos el tope 255
    TA2CCR0=255;

    //Topes iniciales
    TA2CCR1=0;

    //Modo de operación
    //Reset(1)/Set(0)
    TA2CCTL1|=OUTMOD0|OUTMOD1|OUTMOD2;

    //Ambos
    //Clock en up mode y partirlo
    TA0CTL|=MC0;
    TA0CTL&=~MC1;

    //Clock en up mode y partirlo
    TA2CTL|=MC0;
    TA2CTL&=~MC1;
}

//Regula los distintos led
void timer_init(){
    //Usamos el ACLK: 32kHz
    TA1CTL|=TASSEL0;
    TA1CTL&=~TASSEL1;

    //divisor de clock: 1/8
    TA1CTL|=ID1|ID0;

    //Queremos que un ciclo sea de 1.5 segundos
    TA1CCR0=1.5*4*1000;

    //3 topes: uno a los 0.5 segundo, otro a al segundo y el último en 1.5 segundos
    TA1CCR1=TA1CCR0/3;
    TA1CCR2=TA1CCR1*2;

    //Habilitar Interrupciones de CC
    TA1CCTL1|=CCIE;
    TA1CTL|=TAIE;
    TA1CCTL2|=CCIE;

    //Clock en up mode y partirlo
    TA1CTL|=MC0;
    TA1CTL&=~MC1;
}
