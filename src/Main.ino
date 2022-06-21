/**
 * @file Main.ino
 * @author Álex Santos (@robotaleh)
 * @brief
 * @version 0.1
 * @date 2021-03-24
 *
 * @copyright Copyright (c) 2021
 *
 */

// INCLUDES
#include "FlySkyIBus.h"
#include <Servo.h>
#include <SoftwareSerial.h>

// CONFIGURACIÓN
#define SERVO_CENTRO 1160 // Barco: 1160  || Coche: 1500
#define SERVO_AMPLITUD 250

#define TENSION_ANALOG_MINIMA 0
#define TENSION_ANALOG_MAXIMA 932

#define CONSUMO_BARCO_ANALOG_MINIMA 0
#define CONSUMO_BARCO_ANALOG_MAXIMA 1023

// PINOUT
#define PIN_MANDO_IBUS_RX 5
#define PIN_MANDO_IBUS_TX 4

#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 3

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
SoftwareSerial IBusSerial(PIN_MANDO_IBUS_RX, PIN_MANDO_IBUS_TX);

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

// VARIABLES IBUS
#define IBUS_RAXIS_X 0
#define IBUS_LAXIS_Y 1
#define IBUS_RAXIS_Y 2
#define IBUS_LAXIS_X 3
#define IBUS_SWITCH2 4
#define IBUS_SWITCH3 5
int arr_ibus[6];

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
  // IBusSerial.begin(115200);
  IBus.begin(Serial);

  // Iniciar Encoders
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_a, CHANGE);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoder_b, CHANGE);

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
  IBus.loop();

  leer_IBus();
  leer_datos();

  // Se reciben -1 en el IBus hasta que el mando se conecte al menos una vez
  if (arr_ibus[IBUS_SWITCH2] == -1) {
    motorBrushless.writeMicroseconds(1500);
    motorServo.writeMicroseconds(SERVO_CENTRO);
    return;
  }

  int velocidad_limite = map(arr_ibus[IBUS_RAXIS_Y], 0, 100, 1500, 2000) - map(constrain(arr_ibus[IBUS_LAXIS_Y], 0, 50), 0, 50, 500, 0);
  int giro = map(arr_ibus[IBUS_LAXIS_X], 0, 100, SERVO_CENTRO - SERVO_AMPLITUD, SERVO_CENTRO + SERVO_AMPLITUD);

  // Añade una DEADZONE de 50 a la velocidad_limite
  if (abs(1000 - velocidad_limite) < 50) {
    velocidad_limite = 1000;
  } else if (abs(1500 - velocidad_limite) < 50) {
    velocidad_limite = 1500;
  } else if (abs(2000 - velocidad_limite) < 50) {
    velocidad_limite = 2000;
  }

  if (arr_ibus[IBUS_SWITCH2] == 1 && abs(1500 - velocidad_limite) > 50) {
    velocidad = calcular_mppt(velocidad_limite);
  } else {
    velocidad_mppt = 0;
    velocidad = velocidad_limite;
  }

  // Añade una DEADZONE de 50 a la velocidad
  if (abs(1000 - velocidad) < 50) {
    velocidad = 1000;
  } else if (abs(1500 - velocidad) < 50) {
    velocidad = 1500;
  } else if (abs(2000 - velocidad) < 50) {
    velocidad = 2000;
  }

  motorBrushless.writeMicroseconds(velocidad);
  motorServo.writeMicroseconds(giro);
  // return;

  // if (millis() - millisPrint >= 100) {

  //   Serial.print(" V: ");
  //   Serial.print(leer_tension());
  //   Serial.print("\t - \t");
  //   // Serial.print(analogRead(PIN_VOLTIMETRO));
  //   // Serial.print(" - ");
  //   Serial.print(calcular_media_V());
  //   Serial.print("\t A: ");
  //   Serial.print(leer_consumo());
  //   Serial.print("\t - \t");
  //   // Serial.print(analogRead(PIN_AMPERIMETRO));
  //   // Serial.print(" - ");
  //   // Serial.print(0);
  //   // Serial.print(" ");
  //   Serial.print(calcular_media_A());
  //   // Serial.print(" ");
  //   // Serial.print(5000);
  //   // Serial.print("\t");
  //   // Serial.print("\n");

  //   // Serial.print(" MPPT: ");
  //   // Serial.print(arr_ibus[IBUS_SWITCH2]);

  //   Serial.print("\t V LIM: ");
  //   Serial.print(velocidad_limite);

  //   Serial.print("\t V MPPT: ");
  //   Serial.print(velocidad_mppt);
  //   Serial.print("\t V: ");
  //   Serial.print(velocidad);
  //   Serial.print("\t G: ");
  //   Serial.print(giro);
  //   Serial.print("\n");
  //   millisPrint = millis();
  // }
}

void leer_IBus() {
  for (int channel = 0; channel < 6; channel++) {
    int val = IBus.readChannel(channel);

    switch (channel) {
      case IBUS_LAXIS_Y:
        val = map(val, 1000, 2000, 0, 100);
        if (abs(100 - val) < 5) {
          val = 100;
        } else if (abs(50 - val) < 10) {
          val = 50;
        } else if (abs(val) < 5) {
          val = 0;
        }
        break;
      case IBUS_RAXIS_Y:
        val = map(val, 1000, 2000, 0, 100);
        if (abs(100 - val) < 5) {
          val = 100;
        } else if (abs(val) < 5) {
          val = 0;
        }
        break;
      case IBUS_SWITCH2:
        val = map(val, 1000, 2000, 0, 1);
        break;
      case IBUS_SWITCH3:
        val = map(val, 1000, 2000, 0, 2);
        break;
      default:
        val = map(val, 1000, 2000, 0, 100);
        if (abs(100 - val) < 5) {
          val = 100;
        } else if (abs(50 - val) < 5) {
          val = 50;
        } else if (abs(val) < 5) {
          val = 0;
        }
        break;
    }
    arr_ibus[channel] = val;

    // Serial.print(" ");
    // Serial.print(arr_ibus[channel]);
    // if (channel == 5) {
    //   Serial.print("\n");
    // }
  }
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
}

int calcular_mppt(int velocidad_limite) {

  if (millis() - ultimoMPPT > TIEMPO_MPPT) {
    ultimoMPPT = millis();

    A_media = calcular_media_A();
    V_media = calcular_media_V();
    W = (V_media / 100.0f) * (A_media / 100.0f);


    Serial.print(A_media);
    Serial.print("\t");
    Serial.print(V_media);
    Serial.print("\t");
    Serial.print(W);
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
    Serial.print("\t");
    Serial.println(velocidad_mppt);

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
  int v = map(analogRead(PIN_VOLTIMETRO), 0, 1024, 0, 2366); // 23.66v es la tensión máxima para 1023 (5v)
  return v > 0 ? v : 0;
}

int leer_consumo() {
  int a = constrain(map(analogRead(PIN_AMPERIMETRO), 512, 1023, 0, 25000), 0, 30000);
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

void encoder_a() { ticks_encoder_a++; }

void encoder_b() { ticks_encoder_b++; }