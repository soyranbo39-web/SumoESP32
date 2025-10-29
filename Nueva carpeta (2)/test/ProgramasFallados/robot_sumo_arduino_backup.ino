#include <Arduino.h>

// Declaraciones de funciones
void Detenerse();
void AtaqueEnemigo();
void MoverAdelante();
void Retroceder();
void MoverDerecha();
void MoverIzquierda();
void ProcesarEvento();
int LecturaAnalogicaSensoresPiso(int pinsensor, int numEjemplos);
void LecturaSensores_Piso();
void LecturaSensoresLaterales();
void LecturaSensoresFrontal();

//Sensores de distancia
const int pinSensor_Lateral_derecho = 15;
const int pinSensor_Lateral_izquierdo = 14;

const int pinSensor_Frontal_izquierdo = 21;
const int pinSensor_Frontal_derecho = 16;
const int pinSensor_Superior = 18;

// sensores de piso
const int pinSensor_Piso_derecho = 17;
const int pinSensor_Piso_izquierdo = 19;

//Motor A
const int MA1A = 25;  // Motor A IN1 - GPIO25
const int MA2A = 26;  // Motor A IN2 - GPIO26
const int PWMA = 5;   // PWM Motor A - GPIO5

//Motor B  
const int MA1B = 22;  // Motor B IN1 - GPIO22
const int MA2B = 23;  // Motor B IN2 - GPIO23
const int PWMB = 10;  // PWM Motor B - GPIO10


//Variables de velocidad de acciones 
const int Velocidad_movimiento_seguir = 160;
const int Velocidad_estandar = 180;
const int Velocidad_normal = 120;
const int Velocidad_maxima = 250;

//Deteccion de enemigos 
bool enemigo_delante = false;
bool enemigo_derecha = false;
bool enemigo_izquierda = false;
bool enemigo_superior_izquierda = false;
bool enemigo_superior_derecha = false;

//Deteccion de piso
bool piso_derecho = false;
bool piso_izquierdo = false;

//variables de deteccion de borde del dojo
int Borde_blanco=60;
const int Rango_entre_encerado_y_apagado=20;

//variables para la lectura de sensores de piso
const int Ejemplos=2; // numero de lecturas 
const unsigned long Tiempo_entre_ejemplos=10; // tiempo entre lecturas
unsigned long tiempo_ultima_muestra=0;// tiempo de la ultima muestra

//valriables para la dracion de los movimientos
const unsigned long Duracion_movimiento=200; 
unsigned long tiempo_inicio_movimiento=0;

//vairbales para debug
int debug_sensor_lateral_derecho=0;
int debug_sensor_lateral_izquierdo=0;

//Eventos donde se realizara los movimientos de det-eccion de borde
bool evento_blanco_derecho = false;
bool evento_blanco_izquierdo = false;
bool evento_blanco_ambos = false;

//eventos de deteccion de enemigos
bool evento_enemigo_derecha = false;
bool evento_enemigo_izquierda = false;
bool evento_enemigo_frontal = false;
bool evento_enemigo_frontal_derecha = false;
bool evento_enemigo_frontal_izquierda = false;
bool prev_colision_derecha_lateral = false;
bool prev_colision_izquierda_lateral = false;
bool prev_colision_frontal_derecho = false;
bool prev_colision_frontal_izquierdo = false;
bool prev_colision_frontal_superior = false;
unsigned long tiempo_ultimo_evento_enemigo_lateral_derecha = 0;
unsigned long tiempo_ultimo_evento_enemigo_lateral_izquierda = 0;
unsigned long tiempo_ultimo_evento_enemigo_frontal_superior = 0;
unsigned long tiempo_ultimo_evento_enemigo_frontal_derecho = 0;
unsigned long tiempo_ultimo_evento_enemigo_frontal_izquierdo = 0;
const unsigned long Tiempo_entre_eventos_enemigos_lateral = 40; 
const unsigned long Tiempo_entre_eventos_enemigos_frontal = 40;

//Duraccion de eventos de deteccion de enemigos
const unsigned long Duracion_evento_enemigo_frontal = 300;
const unsigned long Duracion_evento_enemigo_lateral = 200;

//Duracion de eventos de deteccion de borde
const unsigned long Duracion_evento_borde_un_sensor = 300;
const unsigned long Duracion_evento_borde_ambos_sensores = 100;

//Duracion actual de la accion
unsigned long Duracion_actual=0;

// Lectura analogica de sensores de piso 
int LecturaAnalogicaSensoresPiso(int pinsensor, int numEjemplos)
{
    long suma=0;
    for(int i=0;i<numEjemplos;i++)
    {
        suma+=analogRead(pinsensor);
           delayMicroseconds(1);
    }
    return (int)(suma / numEjemplos);
}
void Detenerse() {
    // Detener completamente ambos motores
    digitalWrite(MA1A, LOW); digitalWrite(MA2A, LOW);  // Motor A parado
    digitalWrite(MA1B, LOW); digitalWrite(MA2B, LOW);  // Motor B parado
    ledcWrite(0, 0);    // Sin velocidad motor A (canal 0)
    ledcWrite(1, 0);    // Sin velocidad motor B (canal 1)
}

void AtaqueEnemigo() {
    // Avanzar con fuerza para empujar al enemigo
    digitalWrite(MA1A, HIGH); digitalWrite(MA2A, LOW); // Motor A adelante
    digitalWrite(MA1B, HIGH); digitalWrite(MA2B, LOW); // Motor B adelante
    ledcWrite(0, Velocidad_normal);  // Velocidad 120 PWM (canal 0)
    ledcWrite(1, Velocidad_normal);  // Velocidad 120 PWM (canal 1)
}

void MoverAdelante() {
    // Movimiento normal hacia adelante
    digitalWrite(MA1A, HIGH); digitalWrite(MA2A, LOW); // Motor A adelante
    digitalWrite(MA1B, HIGH); digitalWrite(MA2B, LOW); // Motor B adelante
    ledcWrite(0, Velocidad_normal);  // Velocidad 120 PWM (canal 0)
    ledcWrite(1, Velocidad_normal);  // Velocidad 120 PWM (canal 1)
}

void Retroceder() {
    // Retroceder con mayor velocidad para escapar rápidamente
    digitalWrite(MA1A, LOW); digitalWrite(MA2A, HIGH); // Motor A atrás
    digitalWrite(MA1B, LOW); digitalWrite(MA2B, HIGH); // Motor B atrás
    ledcWrite(0, Velocidad_estandar); // Velocidad 180 PWM (canal 0)
    ledcWrite(1, Velocidad_estandar); // Velocidad 180 PWM (canal 1)
}

void MoverDerecha() {
    // Girar a la derecha: motor izquierdo adelante, motor derecho atrás
    digitalWrite(MA1A, HIGH); digitalWrite(MA2A, LOW); // Motor A (izquierdo) adelante
    digitalWrite(MA1B, LOW); digitalWrite(MA2B, HIGH); // Motor B (derecho) atrás
    ledcWrite(0, Velocidad_normal);  // Velocidad 120 PWM (canal 0)
    ledcWrite(1, Velocidad_normal);  // Velocidad 120 PWM (canal 1)
}

void MoverIzquierda() {
    // Girar a la izquierda: motor izquierdo atrás, motor derecho adelante
    digitalWrite(MA1A, LOW); digitalWrite(MA2A, HIGH); // Motor A (izquierdo) atrás
    digitalWrite(MA1B, HIGH); digitalWrite(MA2B, LOW); // Motor B (derecho) adelante
    ledcWrite(0, Velocidad_normal);  // Velocidad 120 PWM (canal 0)
    ledcWrite(1, Velocidad_normal);  // Velocidad 120 PWM (canal 1)
}



// Variables de etapas eliminadas - ya no son necesarias
// Los sensores laterales solo giran, los frontales solo atacan

//control de prioridad de eventos
//control de prioridad
typedef enum
{
    PRIORIDAD_NINGUNA = 0,
    PRIORIDAD_UN_BLANCO = 4,
    PRIORIDAD_AMBOS_BLANCOS = 3,
    PRIORIDAD_Deteccion_FRONTAL = 2,
    PRIORIDAD_Deteccion_LATERAL = 1
} PrioridadEvento;
bool evento_en_proceso = false;
unsigned long tiempo_evento_en_proceso = 0;
PrioridadEvento prioridad_actual = PRIORIDAD_NINGUNA;

void detenerAccion() {
    MoverAdelante();
    evento_en_proceso = false;
    prioridad_actual = PRIORIDAD_NINGUNA;
    Duracion_actual = 0;
}

//Funciones de control de motores
unsigned long obtenerDuracionPorDefecto(PrioridadEvento prio, int lado) {
    (void)lado;
    switch (prio) {
        case PRIORIDAD_Deteccion_LATERAL: return Duracion_evento_enemigo_lateral;
        case PRIORIDAD_AMBOS_BLANCOS: return Duracion_evento_borde_ambos_sensores;
        case PRIORIDAD_UN_BLANCO: return Duracion_evento_borde_un_sensor;
        case PRIORIDAD_Deteccion_FRONTAL: return  Duracion_evento_enemigo_frontal;
        default: return Duracion_movimiento;
    }
}


// Declaracion de funciones donde se procesa y se aplica la logica de eventos

void InicioEvento(PrioridadEvento nuevaPrioridad, int prioridad, long duracion)
{
  evento_en_proceso = true;
  tiempo_evento_en_proceso = millis();
  prioridad_actual = nuevaPrioridad;
  // establecer duración de la accion
  if (duracion > 0) Duracion_actual = (unsigned long)duracion;
  else Duracion_actual = obtenerDuracionPorDefecto(nuevaPrioridad, prioridad);
        if (nuevaPrioridad == PRIORIDAD_Deteccion_LATERAL ) {
        // Sensores laterales
        if (prioridad == 0) {
            // Chocó por derecha > girar derecha para enfrentar al enemigo
            MoverDerecha();
        } else {
            // Chocó por izquierda > girar izquierda para enfrentar al enemigo
            MoverIzquierda();
        }
    }
    else if (nuevaPrioridad == PRIORIDAD_AMBOS_BLANCOS) {
        // Ambos sensores en borde > retroceder y girar para volver al centro
        Retroceder();
        MoverIzquierda();
    }
    else if (nuevaPrioridad == PRIORIDAD_UN_BLANCO) {
        // Un sensor en blanco > apartarse: si derecho detecta -> girar izquierda
        if (prioridad == 0) {
            MoverIzquierda();
        } else if (prioridad == 1) {
            MoverDerecha();
        }
    }
    else if (nuevaPrioridad == PRIORIDAD_Deteccion_FRONTAL) {
        // Sensores frontales
        if (prioridad == 2) {
            // Sensor frontal derecho > girar derecha para seguir al enemigo
            MoverDerecha();
        } else if (prioridad == 3) {
            // Sensor frontal izquierdo > girar izquierda para seguir al enemigo
            MoverIzquierda();
        } else if (prioridad == 4) {
            // Sensor frontal superior > atacar directamente
            AtaqueEnemigo();
        }
    }
}

// Finalizacion de eventos
void FinEvento()
{
  evento_en_proceso = false;
  prioridad_actual = PRIORIDAD_NINGUNA;
}

// Decide la prioridad actual revisando eventos en orden estratégico
PrioridadEvento eventoPrioridadActual(int *lado)
{
    
    *lado = -2;
    if (evento_blanco_derecho) { *lado = 0; return PRIORIDAD_UN_BLANCO; }
    if (evento_blanco_izquierdo) { *lado = 1; return PRIORIDAD_UN_BLANCO; }
    if (evento_blanco_ambos) { *lado = -1; return PRIORIDAD_AMBOS_BLANCOS; }

    // Sensor frontal superior (mayor prioridad) > atacar
    if (evento_enemigo_frontal) { *lado = 4; return  PRIORIDAD_Deteccion_FRONTAL; }
    // Sensores frontales laterales (menor prioridad) > seguir/perseguir
    if ( evento_enemigo_frontal_derecha) { *lado = 2; return PRIORIDAD_Deteccion_FRONTAL; }
    if (evento_enemigo_frontal_izquierda) { *lado = 3; return PRIORIDAD_Deteccion_FRONTAL; }

    if (evento_enemigo_derecha) { *lado = 0; return PRIORIDAD_Deteccion_LATERAL; }
    if (evento_enemigo_izquierda) { *lado = 1; return PRIORIDAD_Deteccion_LATERAL; }

    return PRIORIDAD_NINGUNA;
}

// Funciones de movimiento
void ProcesarEvento()
{
     // si hay acción en curso, comprobar fin o interrupción
    if ( evento_en_proceso) {
        // Si no estamos en maniobra por etapas, usar la duración normal y preempción
        if (millis() - tiempo_evento_en_proceso >= Duracion_actual) {
            // terminar acción
            MoverAdelante();
            // limpiar eventos ya consumidos
            evento_enemigo_derecha = false;
            evento_enemigo_izquierda = false;
            evento_blanco_ambos = false;
            evento_blanco_derecho = false;
            evento_blanco_izquierdo = false;
            evento_enemigo_frontal_derecha = false;
            evento_enemigo_frontal_izquierda = false;
            evento_enemigo_frontal = false;
            return;
        }
        // comprobar interrupción por evento de mayor prioridad
        int lado; 
        PrioridadEvento mas_alta = eventoPrioridadActual(&lado);
        if (mas_alta > prioridad_actual && mas_alta != PRIORIDAD_NINGUNA) {
            // interrumpir acción actual con evento de mayor prioridad
            MoverAdelante();
            InicioEvento(mas_alta, lado, -1);
            // limpiar evento iniciado
            if (mas_alta == PRIORIDAD_Deteccion_LATERAL) {
                if (lado==0)  evento_enemigo_derecha = false;
                else evento_enemigo_izquierda = false;
            } else if (mas_alta == PRIORIDAD_AMBOS_BLANCOS) {
                evento_blanco_ambos = false;
            } else if (mas_alta == PRIORIDAD_UN_BLANCO) {
                if (lado==0) evento_blanco_derecho = false;
                else evento_blanco_izquierdo = false;
            }
        }
        return;
    }

    

    // si no hay acción activa, iniciar la de mayor prioridad
    int lado; 
    PrioridadEvento mas_alta = eventoPrioridadActual(&lado);
    if (mas_alta != PRIORIDAD_NINGUNA) {
        InicioEvento(mas_alta, lado, -1);
        // consumir evento
        if (mas_alta == PRIORIDAD_Deteccion_LATERAL) {
            if (lado==0) evento_enemigo_derecha = false;
            else evento_enemigo_izquierda = false;
        } else if (mas_alta == PRIORIDAD_AMBOS_BLANCOS) {
            evento_blanco_ambos = false;
        } else if (mas_alta == PRIORIDAD_UN_BLANCO) {
            if (lado==0) evento_blanco_derecho = false;
            else evento_blanco_izquierdo = false;
        }
    }
}


//sirve para que se actualice la lectura de los sensores de piso
void LecturaSensores_Piso()
{
     debug_sensor_lateral_derecho = LecturaAnalogicaSensoresPiso(pinSensor_Piso_derecho, Ejemplos);
    debug_sensor_lateral_izquierdo = LecturaAnalogicaSensoresPiso(pinSensor_Piso_izquierdo, Ejemplos);
    int umbral_on = Borde_blanco + Rango_entre_encerado_y_apagado/2;
    int umbral_off = Borde_blanco - Rango_entre_encerado_y_apagado/2;

    bool nueva_derecha = piso_derecho;
    if (!nueva_derecha && debug_sensor_lateral_derecho >= umbral_on) nueva_derecha = true;
    else if (nueva_derecha && debug_sensor_lateral_derecho <= umbral_off) nueva_derecha = false;

    bool nueva_izquierda = piso_izquierdo;
    if (!nueva_izquierda && debug_sensor_lateral_izquierdo >= umbral_on) nueva_izquierda = true;
    else if (nueva_izquierda && debug_sensor_lateral_izquierdo <= umbral_off) nueva_izquierda = false;


    evento_blanco_derecho  = (!piso_derecho && nueva_derecha);
    evento_blanco_izquierdo = (!piso_izquierdo && nueva_izquierda);
    bool previo = (piso_derecho && piso_izquierdo);
    bool nuevo  = (nueva_derecha && nueva_izquierda);
    evento_blanco_ambos = (!previo && nuevo);

    piso_derecho = nueva_derecha;
    piso_izquierdo = nueva_izquierda;

}
void LecturaSensoresLaterales()
{
    bool derecha_Lateral = digitalRead(pinSensor_Lateral_derecho);
    bool izquierda_Lateral = digitalRead(pinSensor_Lateral_izquierdo);

    unsigned long tiempo_actual = millis();

   if (derecha_Lateral && !prev_colision_derecha_lateral && (tiempo_actual - tiempo_ultimo_evento_enemigo_frontal_derecho >= Tiempo_entre_eventos_enemigos_frontal)) {
      evento_enemigo_derecha = true;
        tiempo_ultimo_evento_enemigo_frontal_derecho = tiempo_actual;
        prev_colision_derecha_lateral = true;
    } else if (!derecha_Lateral) {
        prev_colision_derecha_lateral = false;
    }

    if (izquierda_Lateral && !prev_colision_izquierda_lateral && (tiempo_actual - tiempo_ultimo_evento_enemigo_frontal_izquierdo >= Tiempo_entre_eventos_enemigos_frontal)) {
        evento_enemigo_izquierda = true;
        tiempo_ultimo_evento_enemigo_frontal_izquierdo = tiempo_actual;
        prev_colision_izquierda_lateral = true;
    } else if (!izquierda_Lateral) {
        prev_colision_izquierda_lateral = false;
    }

   
    enemigo_derecha = derecha_Lateral;
    enemigo_izquierda = izquierda_Lateral;

}

void LecturaSensoresFrontal()
{
    bool derecha_frontal = digitalRead(pinSensor_Frontal_derecho);
    bool izquierda_frontal = digitalRead(pinSensor_Frontal_izquierdo);
    bool superior = digitalRead(pinSensor_Superior);
    unsigned long tiempo_actual = millis();

    if (derecha_frontal && !prev_colision_frontal_derecho && (tiempo_actual - tiempo_ultimo_evento_enemigo_frontal_derecho >= Tiempo_entre_eventos_enemigos_frontal)) {
       evento_enemigo_frontal_derecha= true;
        tiempo_ultimo_evento_enemigo_frontal_derecho= tiempo_actual;
        prev_colision_frontal_derecho = true;
    } else if (!derecha_frontal) {
        prev_colision_frontal_derecho = false;
    }

    if (izquierda_frontal && !prev_colision_frontal_izquierdo && (tiempo_actual - tiempo_ultimo_evento_enemigo_frontal_izquierdo >= Tiempo_entre_eventos_enemigos_frontal)) {
        evento_enemigo_frontal_izquierda = true;
        tiempo_ultimo_evento_enemigo_frontal_izquierdo = tiempo_actual;
        prev_colision_frontal_izquierdo = true;
    } else if (!izquierda_frontal) {
        prev_colision_frontal_izquierdo = false;
    }

    if (superior && !prev_colision_frontal_superior && (tiempo_actual - tiempo_ultimo_evento_enemigo_frontal_superior >= Tiempo_entre_eventos_enemigos_frontal)) {
        evento_enemigo_frontal= true;
        tiempo_ultimo_evento_enemigo_frontal_superior = tiempo_actual;
        prev_colision_frontal_superior = true;
    } else if (!superior) {
        prev_colision_frontal_superior = false;
    }

    enemigo_superior_derecha = derecha_frontal;
    enemigo_superior_izquierda = izquierda_frontal;
    enemigo_delante = superior;

}



void setup() {
    // Inicializar comunicación serial para debug
    Serial.begin(115200);
    Serial.println("=== INICIANDO PRUEBA DE SENSORES ESP32 ===");
    Serial.println("Robot Sumo - Verificación de sensores");
    Serial.println("======================================");
    
    pinMode(pinSensor_Lateral_derecho, INPUT);   
    pinMode(pinSensor_Lateral_izquierdo, INPUT); 
    pinMode(pinSensor_Frontal_derecho, INPUT);   
    pinMode(pinSensor_Frontal_izquierdo, INPUT); 
    pinMode(pinSensor_Superior, INPUT);          

    pinMode(pinSensor_Piso_derecho, INPUT);     
    pinMode(pinSensor_Piso_izquierdo, INPUT);    

    // Configurar pines de motores como OUTPUT
    pinMode(MA1A, OUTPUT);
    pinMode(MA2A, OUTPUT);
    pinMode(MA1B, OUTPUT);
    pinMode(MA2B, OUTPUT);
    
    // Configurar PWM para ESP32
    // Canal 0 para Motor A, Canal 1 para Motor B
    ledcSetup(0, 1000, 8); // Canal 0, 1kHz, 8 bits resolución
    ledcSetup(1, 1000, 8); // Canal 1, 1kHz, 8 bits resolución
    ledcAttachPin(PWMA, 0); // Asociar pin PWMA con canal 0
    ledcAttachPin(PWMB, 1); // Asociar pin PWMB con canal 1
    
    // Inicializar motores apagados
    digitalWrite(MA1A, LOW);
    digitalWrite(MA2A, LOW);
    digitalWrite(MA1B, LOW);
    digitalWrite(MA2B, LOW);
    ledcWrite(0, 0); // Motor A a 0
    ledcWrite(1, 0); // Motor B a 0
    
    Serial.println("Configuración completada - Iniciando lectura de sensores...");
    delay(2000);
}

void loop() {

   

   // Leer sensores de piso cada 10ms para detectar bordes blancos
    unsigned long tiempo_actual = millis();
    if (tiempo_actual - tiempo_ultima_muestra >= Tiempo_entre_ejemplos) {
        tiempo_ultima_muestra = tiempo_actual;
        LecturaSensores_Piso();  // Detectar líneas blancas del dohyo
    }
    
    // Leer sensores de colisión continuamente
    LecturaSensoresLaterales();  // Detectar contacto por los lados
    LecturaSensoresFrontal();    // Detectar enemigo al frente
    
    // Procesar eventos y ejecutar estrategias de combate
    ProcesarEvento();
}

