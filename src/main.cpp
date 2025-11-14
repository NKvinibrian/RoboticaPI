#include <Arduino.h>

/*
  Seguidor de linha (preta) com 3 sensores IR e 2x 28BYJ-48 + ULN2003
  Lógica:
    - Esquerda ativa  -> virar à ESQUERDA (pivô)
    - Centro  ativo   -> seguir RETO
    - Direita ativa   -> virar à DIREITA (pivô)
    - Sem leitura     -> seguir RETO

  Ajustes:
    - Defina os pinos dos sensores e dos motores conforme sua montagem.
    - Para sensores analógicos, ajuste SENSOR_THRESHOLD conforme ambiente.
*/

#define DEBUG 1  // 1 para imprimir leituras no Serial

// ----------------------------- Sensores -----------------------------
const uint8_t PIN_S_LEFT   = A3;
const uint8_t PIN_S_CENTER = A4;
const uint8_t PIN_S_RIGHT  = A5;

// Threshold analógico (0..1023). Ativa acima de 500
const int SENSOR_THRESHOLD = 500;

// ----------------------------- Motores -----------------------------
// LEFT motor (ULN2003 IN1..IN4)
const uint8_t L_IN1 = 8;
const uint8_t L_IN2 = 9;
const uint8_t L_IN3 = 10;
const uint8_t L_IN4 = 11;

// RIGHT motor (ULN2003 IN1..IN4)
const uint8_t R_IN1 = 5;
const uint8_t R_IN2 = 6;
const uint8_t R_IN3 = 7;
const uint8_t R_IN4 = 12;

// Tempo entre meia-etapas (quanto menor, mais rápido).
// Para 28BYJ-48, valores típicos 800–2000 us. Comece com 1200.
const unsigned int STEP_DELAY_US_STRAIGHT = 1200;
const unsigned int STEP_DELAY_US_TURN     = 1000;  // um pouco mais rápido nos giros

// Quantidade de meia-etapas por iteração (pode refinar o "ganho" da curva)
const uint8_t STRAIGHT_STEPS_PER_LOOP = 1;
const uint8_t TURN_STEPS_PER_LOOP     = 2;

// ---------------------- Sequência meia-etapa ------------------------
const uint8_t HALFSTEP_SEQ[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

struct Stepper28BYJ {
  uint8_t pins[4];
  int8_t  idx;        // 0..7
  bool    invert;     // inverte o sentido lógico do motor
};

// Monte aqui conforme sua fiação
// Invertido globalmente para que goStraight() avance para frente
Stepper28BYJ motorLeft  = { {L_IN1, L_IN2, L_IN3, L_IN4}, 0, true };
Stepper28BYJ motorRight = { {R_IN1, R_IN2, R_IN3, R_IN4}, 0, false };

// ----------------------------- Funções ------------------------------
void writeCoils(const Stepper28BYJ& m, const uint8_t pat[4]) {
  digitalWrite(m.pins[0], pat[0]);
  digitalWrite(m.pins[1], pat[1]);
  digitalWrite(m.pins[2], pat[2]);
  digitalWrite(m.pins[3], pat[3]);
}

void stepOnce(Stepper28BYJ &m, int dir /* +1 fwd, -1 bwd */) {
  int realDir = m.invert ? -dir : dir;
  m.idx = (m.idx + (realDir > 0 ? 1 : -1) + 8) % 8;
  writeCoils(m, HALFSTEP_SEQ[m.idx]);
}

void stepBoth(int dirLeft, int dirRight, uint8_t steps, unsigned int delayUs) {
  for (uint8_t i = 0; i < steps; i++) {
    if (dirLeft  != 0) stepOnce(motorLeft,  dirLeft);
    if (dirRight != 0) stepOnce(motorRight, dirRight);
    delayMicroseconds(delayUs);
  }
}

// Movimentos de alto nível
void goStraight() {
  stepBoth(+1, +1, STRAIGHT_STEPS_PER_LOOP, STEP_DELAY_US_STRAIGHT);
}

void turnLeftPivot() {
  // Pivô: esquerda para trás, direita para frente
  stepBoth(-1, +1, TURN_STEPS_PER_LOOP, STEP_DELAY_US_TURN);
}

void turnRightPivot() {
  // Pivô: esquerda para frente, direita para trás
  stepBoth(+1, -1, TURN_STEPS_PER_LOOP, STEP_DELAY_US_TURN);
}

// ----------------------------- Setup/Loop ---------------------------
enum Mode : uint8_t { MODE_STRAIGHT, MODE_LEFT, MODE_RIGHT, MODE_IDLE };
Mode lastMode = MODE_STRAIGHT;

void setup() {
  // Sensores analógicos: apenas INPUT (sem pull-up)
  pinMode(PIN_S_LEFT,   INPUT);
  pinMode(PIN_S_CENTER, INPUT);
  pinMode(PIN_S_RIGHT,  INPUT);

  for (uint8_t p : motorLeft.pins)  pinMode(p, OUTPUT);
  for (uint8_t p : motorRight.pins) pinMode(p, OUTPUT);

  // Inicializa bobinas desligadas (opcional manter torque: comente as linhas abaixo)
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  digitalWrite(R_IN3, LOW); digitalWrite(R_IN4, LOW);

#if DEBUG
  Serial.begin(115200);
  delay(300);
  Serial.println(F("Seguidor de linha - 28BYJ-48 + ULN2003"));
  Serial.print(F("THRESHOLD=")); Serial.println(SENSOR_THRESHOLD);
#endif
}

void loop() {
  // Leituras analógicas 0..1023
  int adcL = analogRead(PIN_S_LEFT);
  int adcC = analogRead(PIN_S_CENTER);
  int adcR = analogRead(PIN_S_RIGHT);

  // Ativação acima do threshold
  bool L = adcL > SENSOR_THRESHOLD;
  bool C = adcC > SENSOR_THRESHOLD;
  bool R = adcR > SENSOR_THRESHOLD;

  Mode mode;

  // Prioridade: se centro vê linha, siga reto; senão ajuste para o lado que vê.
  if (C) {
    mode = MODE_STRAIGHT;
  } else if (L && !R) {
    mode = MODE_LEFT;
  } else if (R && !L) {
    mode = MODE_RIGHT;
  } else if (L && R) {
    // Linha larga / cruzamento: tente seguir reto
    mode = MODE_STRAIGHT;
  } else {
    // Nada detectado: seguir reto (pedido do usuário)
    mode = MODE_STRAIGHT;
  }

#if DEBUG
  static uint32_t t0 = 0;
  uint32_t now = millis();
  if (now - t0 > 100) {
    t0 = now;
    Serial.print(F("adc L=")); Serial.print(adcL);
    Serial.print(F(" C=")); Serial.print(adcC);
    Serial.print(F(" R=")); Serial.print(adcR);
    Serial.print(F(" | act L=")); Serial.print(L);
    Serial.print(F(" C=")); Serial.print(C);
    Serial.print(F(" R=")); Serial.print(R);
    Serial.print(F(" | mode="));
    if (mode == MODE_STRAIGHT) Serial.println(F("STRAIGHT"));
    else if (mode == MODE_LEFT) Serial.println(F("LEFT"));
    else if (mode == MODE_RIGHT) Serial.println(F("RIGHT"));
    else Serial.println(F("IDLE"));
  }
#endif

  // Executa movimento curto (não bloqueia por muito tempo) e reavalia
  switch (mode) {
    case MODE_STRAIGHT: goStraight(); break;
    case MODE_LEFT:     turnLeftPivot(); break;
    case MODE_RIGHT:    turnRightPivot(); break;
    default:            goStraight(); break; // opção segura: avance devagar
  }

  lastMode = mode;
}
