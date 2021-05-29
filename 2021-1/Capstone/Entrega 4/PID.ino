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

const int change = 2;

//Variables Controlador
//Sensor: Encoder
//pasos totales de la rueda derecha
volatile int cont_r = 0;
//pasos totales de la rueda izquierda
volatile int cont_l = 0;
//Diferencia de pasas entre interaciones
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
double angulo = 0.0;

//Velocidad instantanea de cada rueda
int speed_r=0;
int speed_l=0;
//Fin

//Controlador
//Referencia ángulo [rad]
float ref=3.14;

//Error de ángulo
float e_a=0.0;

//Ganancias:
//Ángulo
float kp_a=60.0;
float kd_a=0.0;
//Integral
float ki_a=5.0;
float total=0.0;

//Señal a motores
int act_r=0;
int act_l=0;
float act_r_aux=0.0;
float act_l_aux=0.0;
//Fin

int ref_turn_r = 300;
int ref_turn_l = -300;


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

  attachInterrupt(digitalPinToInterrupt(pin_1r), count_right, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(pin_1l), count_left, CHANGE);

  motor_drive(0,0);
  
  delay(5000);
}

void loop() {
    
  time_act = micros();
  if (time_act-time_ant>10000){
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
        e_a=ref-angulo;
        
        if (abs(e_a)>(0.1)){
          total=total+ki_a*e_a*((float)d_time)/1000000.0;
          act_r_aux=kp_a*e_a+total;
          act_l_aux=-kp_a*e_a-total;

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

void straight_line(int distance){
  
  //pasos en iteración anterior
  int cont_r_ant = 0;

  //pasos en iteración anterior
  int cont_l_ant = 0;

  //delta de pasos
  int d_cont_r=0;
  int d_cont_l=0;

  int v_r = 50;

  int v_l = 50;

  //manejo de tiempo para cálculo de velocidad
  unsigned long time_act;
  unsigned long time_ant = 0;
  unsigned long d_time = 0;

  //distancia que ha avanzado cada rueda
  int dist_r=0;
  int dist_l=0;

  //velocidad instantanea de cada rueda
  int speed_r=0;
  int speed_l=0;

  //error de controlador
  int e_r=0;
  int e_l=0;

  //señal a motores
  int act_r=0;
  int act_l=0;
  float act_r_aux=0.0;
  float act_l_aux=0.0;

  //control para velocidad igual
  float speed_diff=0.0;

  //distancia a recorrer
  int total_steps = mm_to_step(distance);

  while((cont_r < total_steps)||(cont_l < total_steps)){

    //Serial.println(cont_r);
    //Serial.println(cont_l);
    
    time_act = micros();
    
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

    //Serial.println(speed_r);
    //Serial.println(speed_l);

    //control de velocidad
    e_r=ref-speed_r;
    e_l=ref-speed_l;


    act_r_aux=act_r_aux+((float)e_r)*1.0;
    act_l_aux=act_l_aux+((float)e_l)*1.0;
    
    speed_diff=act_r_aux-act_l_aux;

    
    act_r_aux=act_r_aux-speed_diff*0.1;

    act_l_aux=act_l_aux+speed_diff*0.1;
    

    if (act_r_aux>255.0){
      act_r_aux=255.0;
      }else if(act_r_aux<0.0){
        act_r_aux = 0.0;
      }
    if (act_l_aux>255.0){
      act_l_aux=255.0;
      }else if(act_l_aux<0.0){
        act_l_aux = 0.0;
      }

    act_r=(int)act_r_aux;
    act_l=(int)act_l_aux;
    

    //Serial.println(act_r);
    //Serial.println(act_l);
    
    motor_drive(act_r,act_l);
    
    time_ant=time_act;
    delay(10);
  }
  motor_drive(0,0);
  //Serial.println("ticks:");
  //Serial.println(cont_l);
  delay(100);
  cont_l = 0;
  cont_r = 0;
}

void turn(int angle){
  //pasos en iteración anterior
  int cont_r_ant = 0;

  //pasos en iteración anterior
  int cont_l_ant = 0;

  //delta de pasos
  int d_cont_r=0;
  int d_cont_l=0;

  int v_r = 50;

  int v_l = 50;

  //manejo de tiempo para cálculo de velocidad
  unsigned long time_act;
  unsigned long time_ant = 0;
  unsigned long d_time = 0;

  //distancia que ha avanzado cada rueda
  int dist_r=0;
  int dist_l=0;

  //velocidad instantanea de cada rueda
  int speed_r=0;
  int speed_l=0;

  //error de controlador
  int e_r=0;
  int e_l=0;

  //señal a motores
  int act_r=0;
  int act_l=0;
  float act_r_aux=0.0;
  float act_l_aux=0.0;

  //control para velocidad igual
  float speed_diff=0.0;

  int total_steps = angle_to_step(angle);
  
  while((cont_r < total_steps)||(cont_l > -total_steps)){
    time_act = micros();
    //Serial.println(cont_r);

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

    //control de velocidad
    e_r=ref_turn_r-speed_r;
    e_l=ref_turn_l-speed_l;

    act_r_aux=act_r_aux+((float)e_r)*1.0;
    act_l_aux=act_l_aux+((float)e_l)*1.0;
    
    speed_diff=act_r_aux-act_l_aux;

    //act_r_aux=act_r_aux-speed_diff*0.1;

    //act_l_aux=act_l_aux+speed_diff*0.1;
    

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

    act_r=(int)act_r_aux;
    act_l=(int)act_l_aux;
    

    //Serial.println(act_r);
    //Serial.println(act_l);
    
    motor_drive(act_r,act_l);
    
    time_ant=time_act;
    delay(10);
  }
  motor_drive(0,0);
  //Serial.println("ticks:");
  Serial.println(cont_l);
  delay(100);
  cont_l = 0;
  cont_r = 0;
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

  Serial.println(angulo);
}

void frenar(){
   analogWrite(motorr_1, 255);
   analogWrite(motorr_2, 255);
   analogWrite(motorl_1, 255);
   analogWrite(motorl_2, 255);
}
