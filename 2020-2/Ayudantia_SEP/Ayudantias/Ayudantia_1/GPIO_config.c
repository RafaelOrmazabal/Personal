//Ayudantía timers e interrupciones
#define ROJO 4
#define VERDE 5
#define AZUL 4
#define BOTON 1

#include <msp430.h>

#include "GPIO_config.h"

void pin_init(){
    //Configurar luces

    P2DIR|=(1<<ROJO);
    P1DIR|=(1<<VERDE);
    P1DIR|=(1<<AZUL);

    //Función de PWM
    P2SEL|=(1<<ROJO);
    P1SEL|=(1<<VERDE);
    P1SEL|=(1<<AZUL);

}
