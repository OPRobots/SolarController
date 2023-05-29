/**
 * @file Main.ino
 * @author Álex Santos (@robotaleh)
 * @brief
 * @version 0.2
 * @date 2023-05-16
 *
 * @copyright Copyright (c) 2023
 *
 */

// INCLUDES
#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// CONFIGURACIÓN
#define SERVO_CENTRO 1400  // Barco: 1160ms  || Coche: 1625ms
#define SERVO_AMPLITUD 600 // Barco: 275ms || Coche 300ms
#define CALIBRACION_V 2366 // 23.66v es la tensión máxima para 1023 (5v)
#define CALIBRACION_A 25000
bool usa_mppt = true;
bool usa_descartes_giros_bruscos = true;
bool usa_descartes_velocidades_bruscos = true;

#define TENSION_ANALOG_MINIMA 0
#define TENSION_ANALOG_MAXIMA 932

#define CONSUMO_BARCO_ANALOG_MINIMA 0
#define CONSUMO_BARCO_ANALOG_MAXIMA 1023

// PINOUT
#define PIN_RAXIS_X 2
#define PIN_LAXIS_Y 3

// #define PIN_ENCODER_A 2
// #define PIN_ENCODER_B 3

#define PIN_SERVO_1 9
#define PIN_SERVO_2 10

#define PIN_MOTOR_1 11
#define PIN_MOTOR_2 12

#define PIN_VOLTIMETRO A0
#define PIN_AMPERIMETRO A1

#define PIN_LED 13

// VARIABLES
Servo motorBrushless;
Servo motorServo;

// VARIABLES ENCODERS
volatile unsigned long ticks_encoder_a = 0;
volatile unsigned long ticks_encoder_b = 0;

// VARIABLES DATOS
#define NUMERO_MEDIDAS 30
#define TIEMPO_MPPT 125
int medida_actual = 0;
bool lectura_inicial = true;
int V[NUMERO_MEDIDAS];
int V_media = 0;
int V_anterior = 0;
int A[NUMERO_MEDIDAS];
int A_media = 0;
long millisLeerDatos = 0;
int vel_anterior = 0;
int giro_anterior = 0;
int count_velocidades_descartados = 0;
int count_giros_descartados = 0;

#define NUMERO_MEDIDAS_VELOCIDAD_LIMITE 20
#define TIEMPO_VELOCIDAD_LIMITE 10
bool lectura_inicial_velocidad_limite = true;
int velocidad_limite[NUMERO_MEDIDAS_VELOCIDAD_LIMITE];
long millisLeerVelocidadLimite = 0;

#define NUMERO_MEDIDAS_GIRO 6
#define TIEMPO_GIRO 10
bool lectura_inicial_giro = true;
int giro[NUMERO_MEDIDAS_GIRO];
long millisLeerGiro = 0;

// VARIABLES PULSEIN
#define PULSEIN_LAXIS_Y 0
#define PULSEIN_RAXIS_X 1
int arr_pulseIn[2];

// VARIABLES MPPT
#define DELTA_VELOCIDAD 30
float W = 0;
float W_anterior = 0;
int velocidad = 0;
int velocidad_mppt = 0;
long ultimoMPPT = 0;

void setup() {

  // Iniciar Serial
  Serial.begin(115200);

  // Pines de entrada
  pinMode(PIN_LAXIS_Y, INPUT);
  pinMode(PIN_RAXIS_X, INPUT);

  // // Iniciar Encoders
  // pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_a, CHANGE);
  // pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoder_b, CHANGE);

  delay(3000); // Para forzar desconexión del esc al reiniciar

  // Iniciar Motores
  motorBrushless.attach(PIN_MOTOR_1);
  motorServo.attach(PIN_SERVO_1);
  motorServo.write(SERVO_CENTRO);

  // Configurar ESC
  motorBrushless.writeMicroseconds(1000);
  delay(5000);
  // motorBrushless.writeMicroseconds(2000);

  // Iniciar mediciones
  pinMode(PIN_VOLTIMETRO, INPUT);
  pinMode(PIN_AMPERIMETRO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
}

long millisPrint = 0;

void loop() {
  leer_pulseIn();
  leer_datos();

  int velocidad_limite = calcular_media_velocidad_limite();
  int giro = calcular_media_giro();

  // Añade una DEADZONE de 50 a la velocidad_limite
  if (abs(1000 - velocidad_limite) < 50) {
    velocidad_limite = 1000;
  } else if (abs(1500 - velocidad_limite) < 50) {
    velocidad_limite = 1490;
  } else if (abs(2000 - velocidad_limite) < 50) {
    velocidad_limite = 2000;
  }

  if (usa_mppt && abs(1500 - velocidad_limite) > 50) {
    velocidad = calcular_mppt(velocidad_limite);
  } else {
    velocidad_mppt = 0;
    velocidad = velocidad_limite;
  }

  // Añade una DEADZONE de 50 a la velocidad
  if (abs(1000 - velocidad) < 50) {
    velocidad = 1000;
  } else if (abs(1500 - velocidad) < 50) {
    velocidad = 1490;
  } else if (abs(2000 - velocidad) < 50) {
    velocidad = 2000;
  }

  motorBrushless.writeMicroseconds(velocidad);
  motorServo.writeMicroseconds(giro);
  return;

  if (millis() - millisPrint >= 100) {

    Serial.print(" V: ");
    Serial.print(leer_tension());
    Serial.print("\t - \t");
    //   // Serial.print(analogRead(PIN_VOLTIMETRO));
    //   // Serial.print(" - ");
    Serial.print(calcular_media_V());
    Serial.print("\t A: ");
    Serial.print(leer_consumo());
    Serial.print("\t - \t");
    //   // Serial.print(analogRead(PIN_AMPERIMETRO));
    //   // Serial.print(" - ");
    //   // Serial.print(0);
    //   // Serial.print(" ");
    Serial.print(calcular_media_A());
    Serial.print(" ");
    //   // Serial.print(5000);
    //   // Serial.print("\t");
    //   // Serial.print("\n");

    //   // Serial.print(" MPPT: ");
    //   // Serial.print(arr_ibus[IBUS_SWITCH2]);

    //   Serial.print("\t V LIM: ");
    //   Serial.print(velocidad_limite);

    Serial.print("\t V MPPT: ");
    Serial.print(velocidad_mppt);
    Serial.print("\t V: ");
    Serial.print(velocidad);
    Serial.print("\t G: ");
    Serial.print(giro);
    Serial.print("\n");
    millisPrint = millis();
  }
}

void leer_pulseIn() {
  arr_pulseIn[PULSEIN_RAXIS_X] = pulseIn(PIN_RAXIS_X, HIGH);
  arr_pulseIn[PULSEIN_LAXIS_Y] = pulseIn(PIN_LAXIS_Y, HIGH);
  // Serial.print(arr_pulseIn[PULSEIN_LAXIS_Y]);
  // Serial.print(" ");
  // Serial.println(arr_pulseIn[PULSEIN_RAXIS_X]);
}

void leer_datos() {
  if (millis() - millisLeerDatos > 10 || lectura_inicial) {
    if (lectura_inicial) {
      for (int medida = 0; medida < NUMERO_MEDIDAS; medida++) {
        V[medida] = leer_tension();
        A[medida] = leer_consumo();
      }
      lectura_inicial = false;
    } else {
      for (int medida = 0; medida < NUMERO_MEDIDAS - 1; medida++) {
        V[medida] = V[medida + 1];
        A[medida] = A[medida + 1];
      }
      V[NUMERO_MEDIDAS - 1] = leer_tension();
      A[NUMERO_MEDIDAS - 1] = leer_consumo();
    }
    millisLeerDatos = millis();
  }

  if (millis() - millisLeerVelocidadLimite >= TIEMPO_VELOCIDAD_LIMITE || lectura_inicial_velocidad_limite) {
    if (lectura_inicial_velocidad_limite) {
      for (int medida = 0; medida < NUMERO_MEDIDAS_VELOCIDAD_LIMITE; medida++) {
        velocidad_limite[medida] = map(arr_pulseIn[PULSEIN_LAXIS_Y], 1030, 1880, 1000, 2000);
      }
      lectura_inicial_velocidad_limite = false;
    } else {
      int vel_actual = map(arr_pulseIn[PULSEIN_LAXIS_Y], 1030, 1880, 1000, 2000);

      // Descarta cambios en la velocidad que sean demasiado bruscos
      if (usa_descartes_velocidades_bruscos && abs(vel_actual - vel_anterior) > 400 && count_velocidades_descartados <= 0) {
        count_velocidades_descartados++;
      } else {
        count_velocidades_descartados = 0;

        for (int medida = 0; medida < NUMERO_MEDIDAS_VELOCIDAD_LIMITE - 1; medida++) {
          velocidad_limite[medida] = velocidad_limite[medida + 1];
        }
        velocidad_limite[NUMERO_MEDIDAS_VELOCIDAD_LIMITE - 1] = vel_actual;
      }
    }
    millisLeerVelocidadLimite = millis();
  }

  if (millis() - millisLeerGiro >= TIEMPO_GIRO || lectura_inicial_giro) {
    if (lectura_inicial_giro) {
      for (int medida = 0; medida < NUMERO_MEDIDAS_GIRO; medida++) {
        giro[medida] = map(arr_pulseIn[PULSEIN_RAXIS_X], 1060, 1970, SERVO_CENTRO - SERVO_AMPLITUD, SERVO_CENTRO + SERVO_AMPLITUD);
        giro_anterior = giro[medida];
      }
      lectura_inicial_giro = false;
    } else {
      int giro_actual = map(arr_pulseIn[PULSEIN_RAXIS_X], 1060, 1970, SERVO_CENTRO - SERVO_AMPLITUD, SERVO_CENTRO + SERVO_AMPLITUD);

      // Descarta cambios en el giro que sean demasiado bruscos
      if (usa_descartes_giros_bruscos && abs(giro_actual - giro_anterior) > 400 && count_giros_descartados <= 0) {
        count_giros_descartados++;
      } else {
        count_giros_descartados = 0;
        // Serial.println(giro_actual);
        //  Solo se añade el giro a la media si no es demasiado brusco
        if (giro_actual < SERVO_CENTRO - SERVO_AMPLITUD) {
          giro_actual = SERVO_CENTRO - SERVO_AMPLITUD;
        } else if (giro_actual > SERVO_CENTRO + SERVO_AMPLITUD) {
          giro_actual = SERVO_CENTRO + SERVO_AMPLITUD;
        }
        for (int medida = 0; medida < NUMERO_MEDIDAS_GIRO - 1; medida++) {
          giro[medida] = giro[medida + 1];
        }
        giro[NUMERO_MEDIDAS_GIRO - 1] = giro_actual;
        giro_anterior = giro_actual;
      }
    }
    millisLeerGiro = millis();
  }
}

int calcular_mppt(int velocidad_limite) {

  if (millis() - ultimoMPPT > TIEMPO_MPPT) {
    ultimoMPPT = millis();

    A_media = calcular_media_A();
    V_media = calcular_media_V();
    W = (V_media / 100.0f) * (A_media / 100.0f);

    // Serial.print(A_media);
    // Serial.print("\t");
    // Serial.print(V_media);
    // Serial.print("\t");
    // Serial.print(W);
    // ultimoMPPT = millis();
    // return velocidad_limite;

    if (W > W_anterior) {
      if (V_media > V_anterior) {
        velocidad_mppt += DELTA_VELOCIDAD;
      } else {
        velocidad_mppt -= DELTA_VELOCIDAD;
      }
    } else {
      if (V_media > V_anterior) {
        velocidad_mppt -= DELTA_VELOCIDAD;
      } else {
        velocidad_mppt += DELTA_VELOCIDAD;
      }
    }
    // Serial.print("\t");
    // Serial.println(velocidad_mppt);

    velocidad_mppt = constrain(velocidad_mppt, 75, 500);
    V_anterior = V_media;
    W_anterior = W;
    if (velocidad_limite != 1500) {
      if (velocidad_limite > 1500) {
        return constrain(1500 + velocidad_mppt, 1500, velocidad_limite);
      } else {
        return constrain(1500 - velocidad_mppt, velocidad_limite, 1500);
      }
    } else {
      return velocidad_limite;
    }
  }
  return velocidad;
}

int leer_tension() {
  int v = map(analogRead(PIN_VOLTIMETRO), 0, 1024, 0, CALIBRACION_V);
  return v > 0 ? v : 0;
}

int leer_consumo() {
  int a = constrain(map(analogRead(PIN_AMPERIMETRO), 512, 1023, 0, CALIBRACION_A), 0, CALIBRACION_A);
  return a > 0 ? a : 0;
  // return ((analogRead(PIN_AMPERIMETRO) * (20000/1023))-20000)/100;
}

int calcular_media_V() {
  long suma_V = 0;
  for (int medida = 0; medida < NUMERO_MEDIDAS; medida++) {
    suma_V += V[medida];
  }
  return suma_V / NUMERO_MEDIDAS;
}

int calcular_media_A() {
  long suma_A = 0;
  for (int medida = 0; medida < NUMERO_MEDIDAS; medida++) {
    suma_A += A[medida];
  }
  return suma_A / NUMERO_MEDIDAS;
}

int calcular_media_velocidad_limite() {
  long suma_velocidad_limite = 0;
  for (int medida = 0; medida < NUMERO_MEDIDAS_VELOCIDAD_LIMITE; medida++) {
    suma_velocidad_limite += velocidad_limite[medida];
  }
  return suma_velocidad_limite / NUMERO_MEDIDAS_VELOCIDAD_LIMITE;
}

int calcular_media_giro() {
  long suma_giro = 0;
  for (int medida = 0; medida < NUMERO_MEDIDAS_GIRO; medida++) {
    suma_giro += giro[medida];
  }
  return suma_giro / NUMERO_MEDIDAS_GIRO;
}

// void encoder_a() { ticks_encoder_a++; }

// void encoder_b() { ticks_encoder_b++; }