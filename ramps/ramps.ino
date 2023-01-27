/**
 * http://domoticx.com/arduino-mega-shield-ramps/
 * ramps
 * https://www.prometec.net/ramps-14/
 * 
 * https://www.thingiverse.com/thing:1141146 POR SI ME HACE FALTA UNA REDUCTORA
 */

/*
 * LIBRERIAS
 */
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MLX90614.h>

/**
 * Constantes
 */
Servo myservo;  // create servo object to control a servo
Adafruit_MLX90614 termometroIR = Adafruit_MLX90614(); //Se instancia el objeto

/*
 * VARIABLES GLOBALES
 */
 
//VARIABLES PARA EL PID SERVO
int period_servo; //variable para controlar el periodo de actualización del PID servo
float medida_espesor = 0.0; //variable donde se guarda el valor del espesor actual
float medida_espesor_previous_error, medida_espesor_error; //Variables para guardar los errores, anteripor y presente  

float kp_servo=2; //Kp del servo
float ki_servo=0.05; //Ki del servo
float kd_servo=3; //Kd del servo
float medida_espesor_setpoint = 186;  //El valor deseado, que equivale a 1,75 mm de espesor (176 al principio)

float PID_servo_p, PID_servo_i, PID_servo_d; //Almacena los valores P, I y D del controlador
float PID_servo_total; //Suma de los valores
float PID_servo_map; //Se conversion de la suma para obtener una salida


//VELOCIDAD DE LOS MOTOERES PASO A PASO
unsigned long previousMillis_fil = 0; //Variable para el calculo de tiempo
unsigned long previousMillis_bob = 0; //Variable para el calculo de tiempo

int velocidad_fil = 30; //Velocidad del filamento
int velocidad_bob = 290; //Velocidad de la bobina. Se recomiend que velocidad_bob = 10 * velocidad_fil

//SENSOR ESPESOR
int num_muestra = 15;       //Numero de muestras para calcular la media del valor (filtro) 
float espesor;              //Variable de salida del filtro
float media=0;              //Variable auxiliar, donde se almacena la media
int iterador = 0;           //Numero de muestra
float media_arr[15];        //Array donde se guardan los datos


//PID TEMPERATURA
unsigned long previousMillis_temp = 0; //Variable para el calculo del periodo

float temperatura = 0.0;
float temperatura_previous_error, temperatura_error;

float kp_temp=15; 
float ki_temp=0.05; 
float kd_temp=1; 
float temperatura_setpoint = 25;  //Should be the temperatura

float PID_Temp_p, PID_Temp_i, PID_Temp_d;
float PID_Temp_total;
float PID_Temp_map;

/**
 * TRAZAS
 */
bool debug_espesor = false;  //SENSOR ESPESOR
bool debug_PID_servo = false; //PID SERVO
bool debug_PID_temp = true;  //PID TEMPERATURA

/**
 * Definicion De Pines
 */
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
 
 termometroIR.begin(); // Iniciar termómetro infrarrojo con Arduino

 myservo.attach(11);  // attaches the servo on pin 9 to the servo object
 myservo.write(60);
 Serial.begin(115200);
 if(debug_PID_servo)
 Serial.println("Referencia Medida_espesor Error PID_servo_p PID_servo_i PID_servo_d Actuacion");
 if(debug_PID_temp)
 Serial.println("Referencia Temperatura Error PID_temp_p PID_temp_i PID_temp_d Actuacion");
}

/**
 * FUNCIONES AUXILIARES  
 */
 
float sensor_espesor(); //Toma la medida del espesor
void avance_fil(bool motor); //Avanza un paso el motor del filamento
void avance_bob(bool motor); //Avanza un paso el motor de la bobina
void PID_Servo(); //PID del servomotor
int PID_Temp(unsigned long currentMillis); //PID de los ventiladores, devuelve el num de ventiladores activos
void ventiladores(int mapa); //Enciende los ventiladores segun el numero de ventiladores necesarios

/*
 * FUNCION PRINCIPAL LOOP
 */
void loop ()
{
 unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis_fil >= velocidad_fil) {
    period_servo = velocidad_fil;
    previousMillis_fil = currentMillis;
    avance_fil(true);
    medida_espesor = sensor_espesor();
    PID_Servo();  
  } 
  
  if (currentMillis - previousMillis_bob >= velocidad_bob) {
    previousMillis_bob = currentMillis;
    avance_bob(true);
  }
  
  int mapa = PID_Temp(currentMillis);
  ventiladores(mapa);
    
}


/*
 * FUNCIONES AUXILIARES
 */

////////////////////////////////////////////////////////////
///////////////////////VENTILADORES/////////////////////////
////////////////////////////////////////////////////////////
void ventiladores(int mapa)
{
  switch (mapa) {
   case 0:
     digitalWrite(Fans[0], LOW);
     digitalWrite(Fans[1], LOW);
     digitalWrite(Fans[2], LOW);
     break;
     
   case 1:
     digitalWrite(Fans[0], LOW);
     digitalWrite(Fans[1], HIGH);
     digitalWrite(Fans[2], LOW);
     break;

   case 2:
     digitalWrite(Fans[0], HIGH);
     digitalWrite(Fans[1], LOW);
     digitalWrite(Fans[2], HIGH);
     break;

   case 3:
     digitalWrite(Fans[0], HIGH);
     digitalWrite(Fans[1], HIGH);
     digitalWrite(Fans[2], HIGH);
     break;
  }
}

////////////////////////////////////////////////////////////
/////////////////////PID VENTILADORES///////////////////////
////////////////////////////////////////////////////////////
int PID_Temp(unsigned long currentMillis)
{
  
  temperatura = termometroIR.readObjectTempC();

  temperatura_error = (temperatura_setpoint - temperatura);   
  PID_Temp_p = kp_temp * temperatura_error;
  float period = currentMillis - previousMillis_temp;     
  PID_Temp_d = kd_temp*((temperatura_error - temperatura_previous_error)/period);

  if(PID_Temp_map>=0 && PID_Temp_map<=3)
  {
    if(-3 < temperatura_error && temperatura_error < 3)
    {
      PID_Temp_i = PID_Temp_i + (ki_temp * temperatura_error);
    }
    else
    {
      PID_Temp_i = 0;
    }
  }
  
  PID_Temp_total = PID_Temp_p + PID_Temp_i + PID_Temp_d;  
  PID_Temp_map = map(PID_Temp_total, -150, 3, 3, 0);
  
  if(PID_Temp_map < 0){PID_Temp_map = 0;}
  if(PID_Temp_map > 3) {PID_Temp_map = 3; } 
         
  if(debug_PID_temp)
  {
    Serial.print(temperatura_setpoint);
    Serial.print(" ");
    Serial.print(temperatura);
    Serial.print(" ");
    Serial.print(PID_Temp_p);
    Serial.print(" ");
    Serial.print(PID_Temp_i);
    Serial.print(" ");
    Serial.print(PID_Temp_d);
    Serial.print(" ");
    Serial.println(PID_Temp_map);
  }
   
  temperatura_previous_error = temperatura_error;
  previousMillis_temp = currentMillis;
  return PID_Temp_map;
}

////////////////////////////////////////////////////////////
////////////////////////PID SERVO///////////////////////////
////////////////////////////////////////////////////////////

 void PID_Servo()
 {
    medida_espesor_error = (medida_espesor_setpoint - medida_espesor);   
    PID_servo_p = kp_servo * medida_espesor_error;
    PID_servo_d = kd_servo*((medida_espesor_error - medida_espesor_previous_error)/period_servo);

    if(PID_servo_map>=50 && PID_servo_map<=135)
    {
      if(-3 < medida_espesor_error && medida_espesor_error < 3)
      {
        PID_servo_i = PID_servo_i + (ki_servo * medida_espesor_error);
      }
      else
      {
        PID_servo_i = 0;
      }
    }
  
    PID_servo_total = PID_servo_p + PID_servo_i + PID_servo_d;  
    PID_servo_map = map(PID_servo_total, -50, 4, 50, 135);
  
    if(PID_servo_map < 50){PID_servo_map = 50;}
    if(PID_servo_map > 135) {PID_servo_map = 135; } 
         
    if(debug_PID_servo)
    {
      Serial.print(medida_espesor_setpoint);
      Serial.print(" ");
      Serial.print(medida_espesor);
      Serial.print(" ");
      Serial.print(medida_espesor-medida_espesor_setpoint);
      Serial.print(" ");
      Serial.print(PID_servo_p);
      Serial.print(" ");
      Serial.print(PID_servo_i);
      Serial.print(" ");
      Serial.print(PID_servo_d);
      Serial.print(" ");
      Serial.println(PID_servo_map);
    }
    
    myservo.write(PID_servo_map);
    medida_espesor_previous_error = medida_espesor_error;
 }

////////////////////////////////////////////////////////////
///////////////LECTURA SENSOR ESPESOR///////////////////////
////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////
////////////////////AVANZA UN PASO//////////////////////////
////////////////////////////////////////////////////////////
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
