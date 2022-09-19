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
//VELOCIDAD DE LOS MOTOERES PASO A PASO
unsigned long previousMillis_fil = 0;
unsigned long previousMillis_bob = 0;
int velocidad_fil = 30;
int velocidad_bob = 290; //Se recomiend que velocidad_bob = 10 * velocidad_fil

//SENSOR ESPESOR
bool debug_espesor = true;
int num_muestra = 15;
float espesor;  
float media=0;
int iterador = 0;
float media_arr[15];

float espesor_deseado = 168; //Factor de conversion que equivale a 1,75 

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

const int Fans[3] = {16,17,23};// the number of the LED pin

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
 
 Serial.begin(9600);
 delay(1000);
}
/**
 * FUNCIONES AUXILIARES  
 */
float sensor_espesor();
void avance_fil();
void avance_bob();

void loop ()
{
 unsigned long currentMillis = millis();

    
  if (currentMillis - previousMillis_fil >= velocidad_fil) {
    previousMillis_fil = currentMillis;
    avance_fil();
    float espesor_ = sensor_espesor();
    
  }
  if (currentMillis - previousMillis_bob >= velocidad_bob) {
    previousMillis_bob = currentMillis;
    avance_bob();
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
  int sensor_diameter_value = analogRead(sensor_diameter_pin);   // realizar la lectura
    
    if(debug_espesor)
    { 
      Serial.print("Sensor : ");
      Serial.println(sensor_diameter_value);
    }
    
     if(iterador >= num_muestra)
     {
      for(int i=0;i<num_muestra;i++)
      {
        media = media + media_arr[i];
      }
      media = media/num_muestra;
      espesor = media;
      media = 0;
      iterador=0;

      if(debug_espesor)
      {
        Serial.print("MEDIA Sensor MEDIA : ");
        Serial.println(espesor);
      }
      
    }
    else{
    media_arr[iterador]=sensor_diameter_value;
    iterador++;
    }
    return espesor;
}
void avance_fil()
{
  digitalWrite(Z_DIR_PIN , LOW);
  digitalWrite(Z_STEP_PIN , HIGH);
  delay(1);
  digitalWrite(Z_STEP_PIN , LOW);
}

void avance_bob()
{
  digitalWrite(Y_DIR_PIN , LOW);
  digitalWrite(Y_STEP_PIN , HIGH);
  delay(1);
  digitalWrite(Y_STEP_PIN , LOW);
}
