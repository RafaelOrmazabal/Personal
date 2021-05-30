#include <EEPROM.h>
//
volatile  int cont_aux=0;
//Pins
const int pin_1r = 12;
const int pin_2r = 11;
const int pin_1l = 14;
const int pin_2l = 15;

const int motorl_1 = 5;
const int motorl_2 = 6;
const int motorr_1 = 9;
const int motorr_2 = 10;
//Fin

//Parámetros físicos del motor
const int ticks_per_turn = 136;
const int drive_distance = 1000;
const int wheel_d = 67;// En milímetros
const float car_d = 168.239;
const float circumference = (wheel_d * 3.14);//diametro de la circumferencia
const float mm_step = circumference / ticks_per_turn; //mm por paso
//Fin

//Variables Controlador
//Sensor: Encoder
//pasos totales de la rueda derecha
volatile int cont_r = 0;
//pasos totales de la rueda izquierda
volatile int cont_l = 0;
//Diferencia de pasos entre interaciones
int d_cont_r=0;
int d_cont_l=0;
//Pasos en iteración anterior
int cont_r_ant = 0;
//Pasos en iteración anterior
int cont_l_ant = 0;
//Distancia que ha avanzado cada rueda en una iteración
int dist_r=0;
int dist_l=0;

//Estado estimado
int distancia_x = 0;
int distancia_y = 0;
double angulo = 0.0;//[rad]

//Velocidad instantanea de cada rueda
int speed_r=0;
int speed_l=0;
//Fin

//Controlador
//Referencia angulo [rad]
float ref=0;
//Referencia Distancia [mm]
int ref_d=1000;

//Error de ángulo
float e_a=0.0;
//Error Distancia
int e_d=0;
float e_d_aux=0.0;
//Control para velocidad igual
float speed_diff=0.0;

//Ganancias:
//Ángulo
float kp_a=60.0;
float kd_a=0.0;
//Integral
float ki_a=5.0;
float total=0.0;
float total_a=0.0;

//Distancia
float kp_d=0.8;
//Derivativo
float kd_d=0.45;
float prom_dist=0.0;
float dist_ant_0=0.0;
float dist_ant_1=0.0;
float dist_ant_2=0.0;
float rate_d=0.0;

//Integral
float ki_d=0.018;//4;
float total_i_d=0.0;
float wind_up=200.0;

//Diferencia de velocidad
float kp_v=1.0;
float ki_v=0.1;
float total_v=0.0;

//Ángulo chico
float kp_chico = 1000.0;
float ki_chico = 0.0;

//Señal a motores
int act_r=0;
int act_l=0;
float act_r_aux=0.0;
float act_l_aux=0.0;
//Fin

//Tiempo
//Manejo de tiempo para cálculo de velocidad y manejo del controlador
unsigned long time_act;
unsigned long time_ant = 0;
unsigned long d_time = 0;
//Fin



void setup() {
  Serial.begin(9600);

  pinMode(pin_1r,INPUT_PULLUP);
  pinMode(pin_2r,INPUT_PULLUP);
  pinMode(pin_1l,INPUT_PULLUP);
  pinMode(pin_2l,INPUT_PULLUP);
  pinMode(motorr_1, OUTPUT);
  pinMode(motorr_2, OUTPUT);
  pinMode(motorl_1, OUTPUT);
  pinMode(motorl_2, OUTPUT);
  pinMode(13,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pin_1r), count_right, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(pin_1l), count_left, CHANGE);

  motor_drive(0,0);
  
  delay(5000);
}

void loop() {
    
  time_act = micros();
  if (time_act-time_ant>35000){
        //cantidad de pasos que avanzó cada rueda
        d_cont_r=cont_r-cont_r_ant;
        d_cont_l=cont_l-cont_l_ant;
        cont_r_ant=cont_r;
        cont_l_ant=cont_l;
      
        //distancia que avanzó la rueda derecha
        dist_r=step_to_mm(d_cont_r);
        //distancia que avanzó la rueda izquierda
        dist_l=step_to_mm(d_cont_l);
      
        //tiempo que pasó
        d_time=time_act - time_ant;
      
        //velocidad instantanea de cada rueda
        //mm/s
        
        speed_r=measure_speed(d_time,dist_r);
        speed_l=measure_speed(d_time,dist_l);
      
        //estimación de estados
        estimar_estado(speed_r, speed_l, d_time);

        //Controlador
        e_d=ref_d-distancia_x;
        e_d_aux=float(e_d);
        e_a=ref-angulo;
        
        /*if (fabs(e_a)>(0.1)){
          total=total+ki_a*e_a*((float)d_time)/1000000.0;
          act_r_aux=kp_a*e_a+total;
          act_l_aux=-kp_a*e_a-total;
          Serial.println("aqui");

          //Revisar magnitudes

          if (act_r_aux>255.0){
            act_r_aux=255.0;
            }else if(act_r_aux<-255.0){
              act_r_aux = -255.0;
            }
          if (act_l_aux>255.0){
            act_l_aux=255.0;
            }else if(act_l_aux<-255.0){
              act_l_aux = -255.0;
            }

          //Pasar a int
          act_r= (int)act_r_aux;
          act_l= (int)act_l_aux;

          //Enviar señal de control
          motor_drive(act_r,act_l);
        }else*/ if (e_d_aux > (((float)ref_d) * 0.05)){
          
          //Integral
          total_i_d=total_i_d+ki_d*e_d*((float)d_time)/1000000.0;

          if(total_i_d > wind_up){
            total_i_d = wind_up;
          }

          //Deivativo
          //Filtro Pasabajos
          prom_dist=(float(distancia_x))+dist_ant_0+dist_ant_1+dist_ant_2;
          prom_dist=prom_dist/4.0;
          //Tasa de Cambio
          rate_d=prom_dist-dist_ant_0;
          rate_d=rate_d/d_time*1000000.0;
          //Actualizar Datos
          dist_ant_2=dist_ant_1;
          dist_ant_1=dist_ant_0;
          dist_ant_0=prom_dist;
          
          

          //Kp más Kd y Ki          
          act_r_aux=kp_d*e_d_aux + total_i_d + kp_chico*e_a-rate_d*kd_d;
          act_l_aux=kp_d*e_d_aux + total_i_d - kp_chico*e_a-rate_d*kd_d;


          //PID Anidado
          /*
          speed_diff=act_r_aux-act_l_aux;

          total_v=speed_diff*ki_v;
          act_r_aux=act_r_aux-speed_diff*kp_v;

          act_l_aux=act_l_aux+speed_diff*kp_v;
          */

          
          //Revisar magnitudes
          if (act_r_aux>255.0){
            act_r_aux=255.0;
            }else if(act_r_aux<-255.0){
              act_r_aux = -255.0;
            }
          if (act_l_aux>255.0){
            act_l_aux=255.0;
            }else if(act_l_aux<-255.0){
              act_l_aux = -255.0;
            }

          //Pasar a int
          act_r=(int)act_r_aux;
          act_l=(int)act_l_aux;

          //Enviar señal de control
          motor_drive(act_r,act_l);
        }else{
          motor_drive(0,0);
          digitalWrite(13,HIGH);
        }
        time_ant=time_act;
      
     
  }
}

void count_right(){
  //Serial.println(digitalRead(pin_1r));
  //Serial.println(digitalRead(pin_2r));

  
  int istate = digitalRead(pin_1r);
  int qstate = digitalRead(pin_2r);

  
  if(qstate == istate){
    cont_r++;
  }else{
    cont_r--;
  }
  //Serial.println(cont_r);
}

void count_left(){
  //Serial.println(digitalRead(pin_1l));
  //Serial.println(digitalRead(pin_2l));

  int istate = digitalRead(pin_1l);
  int qstate = digitalRead(pin_2l);

  
  if(qstate == istate){
    cont_l++;
  }else{
    cont_l--;
  }
  //Serial.println(cont_l);
}

void motor_drive(int velocidad_r, int velocidad_l){
   if(velocidad_r < 0){
    analogWrite(motorr_1, 0);
    analogWrite(motorr_2, abs(velocidad_r));
   }else{
    analogWrite(motorr_1, velocidad_r);
    analogWrite(motorr_2, 0);
   }

   if(velocidad_l < 0){
    analogWrite(motorl_2, 0);
    analogWrite(motorl_1, abs(velocidad_r));
   }else{
    analogWrite(motorl_2, velocidad_r);
    analogWrite(motorl_1, 0);
   }
   /*
   analogWrite(motorr_1, velocidad_r);
   analogWrite(motorl_2, velocidad_l);
   analogWrite(motorr_2, 0);
   analogWrite(motorl_1, 0);
   */
}

//función para pasar de pasos a cm
//basado en el código presente en:
//https://dronebotworkshop.com/robot-car-with-speed-sensors/
int step_to_mm(int steps){
  int result;
  float distance = steps*mm_step;
  result= (int)distance;
  return result;
}

int mm_to_step(int mm){
  int steps;
  float steps_measured = mm/mm_step;
  steps = (int)steps_measured;
  return steps;
}

int angle_to_step(int angle){
  
  float angle_d = ((float)angle)*3.14*car_d/360.0;
  int angle_d_mm = mm_to_step((int)angle_d);
  return angle_d_mm;
}

int measure_speed(unsigned long delta_t,int delta_d)
{
  int speed_wheel;
  double delta_t_aux=0.0;
  float delta_d_aux=0.0;
  delta_t_aux=((double)delta_t)/1000000.0;
  delta_d_aux= (float)delta_d;
  speed_wheel=(int)(delta_d_aux/(float)delta_t_aux);
  //Serial.println(speed_wheel);
  return speed_wheel;
  
}



void estimar_estado(int vel_r, int vel_l, unsigned long delta_t){
  double delta_t_aux = ((double)delta_t)/1000000.0;
  double angulo_aux = angulo + ((double)vel_r - (double)vel_l)*delta_t_aux/(((double)car_d));
  //Serial.println(angulo_aux);
  angulo = angulo_aux;
  float x_aux = (float)distancia_x + ((float)vel_r + (float)vel_l)*((float)delta_t_aux)*((float)cos(angulo_aux))/2.0;
  distancia_x = (int)x_aux;
  float y_aux = (float)distancia_y + ((float)vel_r + (float)vel_l)*((float)delta_t_aux)*((float)sin(angulo_aux))/2.0;
  distancia_y = (int)y_aux;

  Serial.println(distancia_x);
  //Serial.println(angulo);
}

void frenar(){
   analogWrite(motorr_1, 255);
   analogWrite(motorr_2, 255);
   analogWrite(motorl_1, 255);
   analogWrite(motorl_2, 255);
}
