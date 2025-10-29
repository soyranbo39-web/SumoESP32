#include <Arduino.h>
#include <algorithm>

const int PinSensor_Piso_Izquierdo = 4;   
const int PinSensor_Piso_Derecho = 15;    

const int PinSensor_Frontal_Izquierdo =  19;   
const int PinSensor_Frontal_Central = 16;     
const int PinSensor_Frontal_Derecho = 17;    

const int PinSensor_Lateral_Izquierdo = 18;   
const int PinSensor_Lateral_Derecho = 21;     

//Motor A (Derecho)
const int MotorA_IN1 = 25;
const int MotorA_IN2 = 26;
const int MotorA_PWM = 5;    

// Motor B (Izquierdo)
const int MotorB_IN1 = 22;
const int MotorB_IN2 = 23;
const int MotorB_PWM = 27;   

// Configuración PWM para ESP32
const int PWM_FREQ = 1000;       
const int PWM_RESOLUTION = 8;   
const int CANAL_MOTOR_A = 0;      
const int CANAL_MOTOR_B = 1;      


// Pines de control remoto / DIPs
const int Pin_Control_Remoto = 14;
const int Pin_DIP1 = 35;
const int Pin_DIP3 = 39;


bool robot_encendido = false;
int ultimo_estado_boton = HIGH;
unsigned long ultimo_cambio_boton = 0;
const unsigned long debounce_delay = 300; 


#ifndef BLANCO
#define BLANCO 600
#endif
int minIzq = 4095, maxIzq = 0;
int minDer = 4095, maxDer = 0;

int median3Analog(int pin) {
  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);
  return b;
}
int LeerSensor(int pin, int minVal, int maxVal) {
  int lectura = median3Analog(pin);
  lectura = constrain(lectura, minVal, maxVal);
  return map(lectura, minVal, maxVal, 0, 1000);
}
//usala en caso de que quieras lecturas rapidas  solo tienes que cambiarlo en en loop
bool Blanco(int sensorpiso) {
  int valor = LeerSensor(sensorpiso, 0, 4095);
  return valor < BLANCO;
}
bool DeteccionEnemigo(int sensor) {
  return digitalRead(sensor) == HIGH;
}

// Declaraciones de funciones
void Detenerse();

unsigned long tiempoAnterior = 0;
const unsigned long intervaloLectura = 20; 



bool ControlRemotoActivo() {
  int estado = digitalRead(Pin_Control_Remoto); 
  if (estado == LOW && ultimo_estado_boton == HIGH) {
    if (millis() - ultimo_cambio_boton > debounce_delay) {
      robot_encendido = !robot_encendido;
      ultimo_cambio_boton = millis();
    }
  }
  ultimo_estado_boton = estado;
  return robot_encendido;
}


bool VerificarApagado() {
  int estado = digitalRead(Pin_Control_Remoto);
  if (estado == LOW && robot_encendido) {
    if (millis() - ultimo_cambio_boton > debounce_delay) {
      robot_encendido = false;
      ultimo_cambio_boton = millis();
      return true;
    }
  }
  return false;
}


void DelayConVerificacion(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    if (VerificarApagado()) {
      Detenerse();
      return;
    }
    delay(10);
  }
}




//Motores
void Detenerse() {
  // Detener ambos motores completamente
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  ledcWrite(CANAL_MOTOR_A, 0);  
  
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
  ledcWrite(CANAL_MOTOR_B, 0);  
}

void IzquierdaVelocidad(int porcentaje_velocidad) {
 
  if (porcentaje_velocidad <= 0) {
    Detenerse();
    return;
  }
  if (porcentaje_velocidad >= 100) {
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, HIGH);
  digitalWrite(MotorB_IN2, LOW);
    return;
  }
 
  int tiempo_on = porcentaje_velocidad;    // ms encendido
  int tiempo_off = 100 - porcentaje_velocidad;  // ms apagado
  
  // Encender motores
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, HIGH);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_on);
  

  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_off);
}
void DerechoVelocidad(int porcentaje_velocidad) {
  
  if (porcentaje_velocidad <= 0) {
    Detenerse();
    return;
  }
  if (porcentaje_velocidad >= 100) {
 digitalWrite(MotorA_IN1, HIGH);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
    return;
  }
 
  int tiempo_on = porcentaje_velocidad;    // ms encendido
  int tiempo_off = 100 - porcentaje_velocidad;  // ms apagado
  
  // Encender motores
 digitalWrite(MotorA_IN1, HIGH);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_on);
  
  // Apagar motores
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_off);
}
void RetrocederVelocidad(int porcentaje_velocidad) {

  if (porcentaje_velocidad <= 0) {
    Detenerse();
    return;
  }
  if (porcentaje_velocidad >= 100) {
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, HIGH);
  digitalWrite(MotorB_IN1, HIGH);
  digitalWrite(MotorB_IN2, LOW);
    return;
  }

  int tiempo_on = porcentaje_velocidad;    // ms encendido
  int tiempo_off = 100 - porcentaje_velocidad;  // ms apagado
  
  // Encender motores
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, HIGH);
  digitalWrite(MotorB_IN1, HIGH);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_on);
  
  // Apagar motores
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_off);
}
void Atacar(int porcentaje_velocidad) {

  if (porcentaje_velocidad <= 0) {
    Detenerse();
    return;
  }
  if (porcentaje_velocidad >= 100) {
  
    digitalWrite(MotorA_IN1, HIGH);
    digitalWrite(MotorA_IN2, LOW);
    digitalWrite(MotorB_IN1, LOW);
    digitalWrite(MotorB_IN2, HIGH);
    return;
  }
 
  int tiempo_on = porcentaje_velocidad;    // ms encendido
  int tiempo_off = 100 - porcentaje_velocidad;  // ms apagado
  
  // Encender motores
  digitalWrite(MotorA_IN1, HIGH);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, HIGH);
  delay(tiempo_on);
  
  // Apagar motores
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_off);
}
void MoverAdelanteConVelocidad(int porcentaje_velocidad) {

  if (porcentaje_velocidad <= 0) {
    Detenerse();
    return;
  }
  if (porcentaje_velocidad >= 100) {
    digitalWrite(MotorA_IN1, HIGH);
    digitalWrite(MotorA_IN2, LOW);
    digitalWrite(MotorB_IN1, LOW);
    digitalWrite(MotorB_IN2, HIGH);
    return;
  }
  
  int tiempo_on = porcentaje_velocidad;    
  int tiempo_off = 100 - porcentaje_velocidad;  
  
  // Encender motores
  digitalWrite(MotorA_IN1, HIGH);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, HIGH);
  delay(tiempo_on);
  
  // Apagar motores
  digitalWrite(MotorA_IN1, LOW);
  digitalWrite(MotorA_IN2, LOW);
  digitalWrite(MotorB_IN1, LOW);
  digitalWrite(MotorB_IN2, LOW);
  delay(tiempo_off);
  
}


// void Sensores_piso(bool sensorpisoizquierda, bool sensorpisoderecha)
// {

//  if (!sensorpisoderecha && !sensorpisoizquierda)
//  {
//    // Si no hay detección
//    MoverAdelanteConVelocidad(30);
//  }
 
//   else if (sensorpisoderecha && !sensorpisoizquierda)
//   {
//     // Si solo el sensor derecho detecta
//   RetrocederVelocidad(100);
//     DelayConVerificacion(60);
//     DerechoVelocidad(100);
//     DelayConVerificacion(80);
//     IzquierdaVelocidad(100);
//     DelayConVerificacion(80);
//     return;
//   }
//   else if (sensorpisoizquierda && !sensorpisoderecha)
//   {
//     // Si solo el sensor izquierdo detecta
//     RetrocederVelocidad(100);
//     DelayConVerificacion(60);
//     IzquierdaVelocidad(100);
//     DelayConVerificacion(80);
//     return;
//   }
//   else {
//     // si ambos detectan
//      RetrocederVelocidad(100);
//     DelayConVerificacion(80);      
//     DerechoVelocidad(100);          
//     DelayConVerificacion(80);
//     return;
//   }
// }

// // movimiento de sensores frontales
// void Ataque(bool sensorfrontalderecho, bool sensorfrontalcentral, bool sensorfrontalizquierdo)
// {
  
//   if (sensorfrontalcentral  && !sensorfrontalderecho && !sensorfrontalizquierdo)
//   {
//     // Si solo el sensor central detecta
//        AtaqueEnemigo();
//         delay(500);
//     return;
//   }
//   else if (sensorfrontalderecho && !sensorfrontalizquierdo && !sensorfrontalcentral){
//     // Si solo el sensor derecho detecta
//      DerechoVelocidad(100);
//    DerechoVelocidad(100);    
   
//     return;
//   }

//   else if (sensorfrontalizquierdo && !sensorfrontalderecho && !sensorfrontalcentral){
//     // Si solo el sensor izquierdo detecta
//      IzquierdaVelocidad(100);
//     return;
//   }
//   else {
//     // Si ambos detectan
//     AtaqueEnemigo();
//     delay(500);
//     return;
//   }
  
 

  

// }


// void Movimientoslaterales(bool sensorlateralesderecho, bool sensorlateralizquierdo)
// {
 

//   if (sensorlateralesderecho && !sensorlateralizquierdo)
//   {
//     IzquierdaVelocidad(50);
//     return;
//   }
//   else if (sensorlateralizquierdo && !sensorlateralesderecho)
//   {
//   DerechoVelocidad(50); 
//     return;
//   }

// }



void setup() {
  Serial.begin(115200);
  delay(200);

  // Sensores
  pinMode(PinSensor_Piso_Izquierdo, INPUT);
  pinMode(PinSensor_Piso_Derecho, INPUT);

  pinMode(PinSensor_Frontal_Izquierdo, INPUT);
  pinMode(PinSensor_Frontal_Central, INPUT);
  pinMode(PinSensor_Frontal_Derecho, INPUT);
  pinMode(PinSensor_Lateral_Izquierdo, INPUT);
  pinMode(PinSensor_Lateral_Derecho, INPUT);

  pinMode(Pin_Control_Remoto, INPUT_PULLUP);
  pinMode(Pin_DIP1, INPUT_PULLUP);
  pinMode(Pin_DIP3, INPUT_PULLUP);


  pinMode(MotorA_IN1, OUTPUT);
  pinMode(MotorA_IN2, OUTPUT);
  pinMode(MotorB_IN1, OUTPUT);
  pinMode(MotorB_IN2, OUTPUT);

ledcSetup(CANAL_MOTOR_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CANAL_MOTOR_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MotorA_PWM, CANAL_MOTOR_A);
  ledcAttachPin(MotorB_PWM, CANAL_MOTOR_B);
  
}

void loop() {
  bool activo = ControlRemotoActivo();
  if (!activo) {
    Detenerse();
    delay(100);
    return;
  }
  unsigned long ahora = millis();
  if (ahora - tiempoAnterior < intervaloLectura) return;
  tiempoAnterior = ahora;

  //ejemplo del cambio 
//   bool piso2= Blanco(PinSensor_Piso_Derecho);

  int pisoIzq = LeerSensor(PinSensor_Piso_Izquierdo, 0, 4095);
  int pisoDer = LeerSensor(PinSensor_Piso_Derecho, 0, 4095);
  bool PisoI = pisoIzq < BLANCO;
  bool PisoD = pisoDer < BLANCO;

  if (PisoI && PisoD) {
    RetrocederVelocidad(200);
    DelayConVerificacion(60);
    DerechoVelocidad(120);
   DelayConVerificacion(80);
    return;
  } else if (PisoI) {
    RetrocederVelocidad(200);
    DelayConVerificacion(60);
    DerechoVelocidad(120);
    DelayConVerificacion(80);
    return;
  } else if (PisoD) {
    RetrocederVelocidad(200);
    DelayConVerificacion(60);
    IzquierdaVelocidad(120);
    DelayConVerificacion(80);
    return;
  }

  bool frontalIzq = digitalRead(PinSensor_Frontal_Izquierdo);
  bool frontalDer = digitalRead(PinSensor_Frontal_Derecho);
  bool central = digitalRead(PinSensor_Frontal_Central);

  if (central) {
    Atacar(180);
    DelayConVerificacion(500);
    return;
  } else if (frontalIzq) {
    IzquierdaVelocidad(150);
    DelayConVerificacion(60); 
    return;
  } else if (frontalDer) {
    DerechoVelocidad(150);
    DelayConVerificacion(60); 
    return;
  }

 //puedes comentar esta sección si no te funciona 
  bool lateralIzq = digitalRead(PinSensor_Lateral_Izquierdo);
  bool lateralDer = digitalRead(PinSensor_Lateral_Derecho);

  if (lateralIzq) {
    IzquierdaVelocidad(120);
    DelayConVerificacion(50);
    return;
  } else if (lateralDer) {
    DerechoVelocidad(120);
    DelayConVerificacion(50);
    return;
  }

 
  MoverAdelanteConVelocidad(40);
}

// void loop() {
//   bool activo = ControlRemotoActivo();
//   if (!activo) {
//     Detenerse();
//     delay(100);
//     return;
//   }
// int lecturaizq=LeerSensor(PinSensor_Piso_Izquierdo, Sensor_min, Sensor_max);
// int lecturader=LeerSensor(PinSensor_Piso_Derecho, Sensor_min, Sensor_max);

//   bool PisoIzq = Blanco(lecturaizq<BLANCO);
//   bool PisoDer = Blanco(lecturader<BLANCO);
//     if (PisoIzq || PisoDer) {
//         Sensores_piso(PisoIzq, PisoDer);
//         return;
//     }


//   bool FrontalDer = DeteccionEnemigo(PinSensor_Frontal_Derecho);
//   bool FrontalIzq = DeteccionEnemigo(PinSensor_Frontal_Izquierdo);
//   bool Central    = DeteccionEnemigo(PinSensor_Frontal_Central);
//   if (FrontalDer || Central || FrontalIzq) {
//     Ataque(FrontalDer, Central, FrontalIzq);
//     return;
//   }

//   bool LateralDer = DeteccionEnemigo(PinSensor_Lateral_Derecho);
//   bool LateralIzq = DeteccionEnemigo(PinSensor_Lateral_Izquierdo);
//   if (LateralDer || LateralIzq) {
//     Movimientoslaterales(LateralDer, LateralIzq);
//     return;
//   }

//   MoverAdelanteConVelocidad(30);