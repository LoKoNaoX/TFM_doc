/**
 * http://domoticx.com/arduino-mega-shield-ramps/
 * ramps
 * https://www.prometec.net/ramps-14/
 * 
 * https://www.thingiverse.com/thing:1141146 POR SI ME HACE FALTA UNA REDUCTORA
 */
/**
 * Constantes
 */
#include <Wire.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo, later attatched to D9

int period;
float medida_espesor = 0.0;
float medida_espesor_previous_error, medida_espesor_error;

float factor_error = 1;
float kp=6; 
float ki=0.4; 
float kd=1; 
float medida_espesor_setpoint = 163;  //Should be the medida_espesor

float PID_p, PID_i, PID_d;
float PID_total = 60;
//VELOCIDAD DE LOS MOTOERES PASO A PASO
unsigned long previousMillis_fil = 0;
unsigned long previousMillis_bob = 0;
int velocidad_fil = 30;
int velocidad_bob = 290; //Se recomiend que velocidad_bob = 10 * velocidad_fil

//SENSOR ESPESOR
bool debug_espesor = false; //Para mostrar las trazas
int num_muestra = 15;       //Numero de muestras para calcular la media del valor (filtro) 
float espesor;              //Variable de salida del filtro
float media=0;              //Variable auxiliar, donde se almacena la media
int iterador = 0;           //Numero de muestra
float media_arr[15];        //Array donde se guardan los datos


//VENTILADORES


/**
 * Definicion De Pines
 */
#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38
#define X_MIN_PIN 3
#define X_MAX_PIN 2

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56
#define Y_MIN_PIN 14
#define Y_MAX_PIN 15

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62
#define Z_MIN_PIN 18
#define Z_MAX_PIN 19

#define E_STEP_PIN 26
#define E_DIR_PIN 28
#define E_ENABLE_PIN 24

#define Q_STEP_PIN 36
#define Q_DIR_PIN 34
#define Q_ENABLE_PIN 30

#define sensor_diameter_pin A4

const int Fans[3] = {16,17,23};// the number of the FAN pin

/**
 * FUNCION SETUP
 */
void setup()
{
 pinMode(Z_STEP_PIN , OUTPUT);
 pinMode(Z_DIR_PIN , OUTPUT);
 pinMode(Z_ENABLE_PIN , OUTPUT);
 pinMode(Y_STEP_PIN , OUTPUT);
 pinMode(Y_DIR_PIN , OUTPUT);
 pinMode(Y_ENABLE_PIN , OUTPUT);
 
 digitalWrite(Z_ENABLE_PIN , LOW);
 digitalWrite(Y_ENABLE_PIN , LOW);

 pinMode(Fans[0], OUTPUT);
 pinMode(Fans[1], OUTPUT);
 pinMode(Fans[2], OUTPUT);
 
 digitalWrite(Fans[0], HIGH);
 digitalWrite(Fans[1], HIGH);
 digitalWrite(Fans[2], HIGH);

 myservo.attach(11);  // attaches the servo on pin 9 to the servo object
 myservo.write(60);
 Serial.begin(115200);
 Serial.println("Referencia Planta Actuacion");
 

}
/**
 * FUNCIONES AUXILIARES  
 */
float sensor_espesor();
void avance_fil(bool motor);
void avance_bob(bool motor);

void loop ()
{
 unsigned long currentMillis = millis();

    
  if (currentMillis - previousMillis_fil >= velocidad_fil) {
    period = velocidad_fil;
    previousMillis_fil = currentMillis;
    avance_fil(true );
    medida_espesor = sensor_espesor();
  
    medida_espesor_error = (medida_espesor_setpoint - medida_espesor)*factor_error;   
    PID_p = kp * medida_espesor_error;
    float dist_diference = medida_espesor_error - medida_espesor_previous_error;     
    PID_d = kd*((medida_espesor_error - medida_espesor_previous_error)/period);

    if(PID_total>=50 && PID_total<=135)
    {
      if(-3 < medida_espesor_error && medida_espesor_error < 3)
      {
        PID_i = PID_i + (ki * medida_espesor_error);
      }
      else
      {
        PID_i = 0;
      }
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 50){PID_total = 50;}
    if(PID_total > 135) {PID_total = 135; } 
         
   
    Serial.print(medida_espesor_setpoint);
    Serial.print(" ");
    Serial.print(medida_espesor);
    Serial.print(" ");
    Serial.println(PID_total);
    myservo.write(PID_total);
    medida_espesor_previous_error = medida_espesor_error;
    
  }
  
  
  if (currentMillis - previousMillis_bob >= velocidad_bob) {
    previousMillis_bob = currentMillis;
    avance_bob(true);
  }

  
 /*for (int i = 0; i <= Ang2Step(90); i++)
 {
  digitalWrite(X_STEP_PIN , HIGH);
  delay(1);
  digitalWrite(X_STEP_PIN , LOW);
 }
 delay(1000);
*/
}


/*
 * FUNCIONES AUXILIARES
 */
float sensor_espesor()
{
  int sensor_diameter_value = analogRead(sensor_diameter_pin);   // realiza la lectura del sensor analógico
    
    if(debug_espesor) //Traza para ver el valor del sensor
    { 
      Serial.print("Sensor : ");
      Serial.println(sensor_diameter_value);
    }
    
     if(iterador >= num_muestra)      //Si el el numero de muestra es mayor o igual al deseado (15) 
     {
      for(int i=0;i<num_muestra;i++)//Suma todos los valores
      {
        media = media + media_arr[i];
      }
      media = media/num_muestra; //calculo la media
      espesor = media; // guarda la media en la variable de salida
      media = 0; //reset media
      iterador=0; //reset del iterador

      if(debug_espesor) //Traza para ver el valor medio del sensor
      {
        Serial.print("MEDIA Sensor MEDIA : ");
        Serial.println(espesor);
      }
      
    }
    else{ //Si el numero de muestra es menor que el deseado (15)
    media_arr[iterador]=sensor_diameter_value; //guardo en un array el valor
    iterador++; 
    }
    return espesor; //devuelvo el valor medio
}
void avance_fil(bool  motor)
{
  if (motor){
    digitalWrite(Z_DIR_PIN , LOW);
    digitalWrite(Z_STEP_PIN , HIGH);
    delay(1);
    digitalWrite(Z_STEP_PIN , LOW);
  }
}

void avance_bob(bool motor)
{
  if (motor){
    digitalWrite(Y_DIR_PIN , LOW);
    digitalWrite(Y_STEP_PIN , HIGH);
    delay(1);
    digitalWrite(Y_STEP_PIN , LOW);
  }
}
