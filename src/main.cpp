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

// ----------------------------- LED onboard (heartbeat) -----------------------------
const uint8_t PIN_LED = LED_BUILTIN; // LED interno da placa
const unsigned long LED_BLINK_MS = 500; // ms entre piscadas

// ----------------------------- Sensores -----------------------------
const uint8_t PIN_S_LEFT   = A3;
const uint8_t PIN_S_CENTER = A4;
const uint8_t PIN_S_RIGHT  = A5;

// Threshold analógico (0..1023). Ativa acima de 200
const int SENSOR_THRESHOLD = 120;

// ----------------------------- Ultrassom ----------------------------
// HC-SR04: TRIG -> saída, ECHO -> entrada
// Escolhidos pinos livres no Uno: A0 (TRIG), A1 (ECHO)
const uint8_t PIN_US_TRIG = A0;
const uint8_t PIN_US_ECHO = A1;
// Distância (cm) para parar o robô quando houver obstáculo à frente
const uint16_t ULTRASONIC_ALERT_CM = 50;   // desvio de obstáculo quando menor que 25 cm
// Timeout do pulseIn em microssegundos (≈ 20 ms ~ 3.4 m máx.)
const unsigned long ULTRASONIC_TIMEOUT_US = 20000UL;

// ----------------------------- Motores (HW-095 H-bridge) -----------------------------
// HW-095 uses 4 inputs: IN1, IN2 control Motor A; IN3, IN4 control Motor B
// O usuário conectou IN1..IN4 aos pinos 8,9,10,11 respectivamente.
// Mapeamento: LEFT motor = Motor A (IN1, IN2 -> 8,9)
//             RIGHT motor = Motor B (IN3, IN4 -> 10,11)
const uint8_t M_LEFT_IN1  = 8;  // IN1
const uint8_t M_LEFT_IN2  = 9;  // IN2
const uint8_t M_RIGHT_IN1 = 10; // IN3
const uint8_t M_RIGHT_IN2 = 11; // IN4
const uint8_t M_LEFT_EN   = 5;  // ENA (D5)
const uint8_t M_RIGHT_EN  = 6;  // ENB (D6)

// Duração do pulso para o movimento em ms. Mantemos curtos para reavaliação rapida.
// Aumentados para teste de torque: pulsos mais longos permitem o motor atingir velocidade/torque maiores.
const unsigned int MOVE_PULSE_MS_STRAIGHT = 30; // anteriormente 30
const unsigned int MOVE_PULSE_MS_TURN     = 45; // anteriormente 45

// Motor power (0..255). Ajuste aqui para reduzir/ aumentar velocidade.
const uint8_t MOTOR_POWER = 160; // 180 Valor bom para não ultrapassar a linha muito rápido,

// Direções: +1 frente, -1 tras, 0 para parado

// Funções de controle simples para motores DC via H-bridge
void setLeft(int dir) {
  if (dir > 0) {
    digitalWrite(M_LEFT_IN1, HIGH);
    digitalWrite(M_LEFT_IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(M_LEFT_IN1, LOW);
    digitalWrite(M_LEFT_IN2, HIGH);
  } else {
    digitalWrite(M_LEFT_IN1, LOW);
    digitalWrite(M_LEFT_IN2, LOW);
  }
}

void setRight(int dir) {
  if (dir > 0) {
    digitalWrite(M_RIGHT_IN1, HIGH);
    digitalWrite(M_RIGHT_IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(M_RIGHT_IN1, LOW);
    digitalWrite(M_RIGHT_IN2, HIGH);
  } else {
    digitalWrite(M_RIGHT_IN1, LOW);
    digitalWrite(M_RIGHT_IN2, LOW);
  }
}

// Movimentos de alto nível: agora setam a direção e deixam os motores ligados até que outra ação mude isso.
// Removemos os delays e stop para manter os motores acionados continuamente enquanto o modo permanecer.
void goStraight() {
  setLeft(+1);
  setRight(+1);
}

void turnLeftPivot() {
  // Pivô: esquerda para trás, direita para frente
  setLeft(-1);
  setRight(+1);
}

void turnRightPivot() {
  // Pivô: esquerda para frente, direita para trás
  setLeft(+1);
  setRight(-1);
}

// Desenergiza as bobinas/saídas para parar o robô
void stopMotors() {
  digitalWrite(M_LEFT_IN1, LOW);  digitalWrite(M_LEFT_IN2, LOW);
  digitalWrite(M_RIGHT_IN1, LOW); digitalWrite(M_RIGHT_IN2, LOW);
}

// Medição simples do HC-SR04 (bloqueante porém rápida)
unsigned int measureDistanceCm() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  unsigned long dur = pulseIn(PIN_US_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (dur == 0) return 0; // sem eco dentro do timeout -> fora de alcance OU muito perto / mau alinhado
  // Fórmula aproximada: cm = dur / 58
  return (unsigned int)(dur / 58UL);
}

// ----------------------------- Setup/Loop ---------------------------
enum Mode : uint8_t { MODE_STRAIGHT, MODE_LEFT, MODE_RIGHT, MODE_IDLE };
Mode lastMode = MODE_STRAIGHT;

// Estados da sequência de desvio de obstáculo
enum AvoidState : uint8_t {
  AVOID_NONE = 0,      // não desviando
  AVOID_LEFT,          // sair da linha girando para a esquerda
  AVOID_FORWARD,       // andar reto afastado do obstáculo
  AVOID_RIGHT,         // virar à direita para voltar em direção à linha
  AVOID_RETURN         // seguir reto tentando reencontrar a linha
};

bool inAvoidMode = false;
AvoidState avoidState = AVOID_NONE;
unsigned long avoidStartMs = 0;   // início de todo o desvio
unsigned long avoidStateStartMs = 0; // início do estado atual de desvio

// Tempos (em ms) de cada fase do desvio - ajuste conforme o robô
const unsigned long AVOID_LEFT_MS    = 400;  // quanto tempo gira para a esquerda saindo da linha
const unsigned long AVOID_FORWARD_MS = 1200;  // quanto tempo anda reto longe do obstáculo
const unsigned long AVOID_RIGHT_MS   = 450;  // quanto tempo gira para a direita voltando na direção da linha
const unsigned long AVOID_TOTAL_TIMEOUT_MS = 6000; // tempo máximo total tentando desviar

void setup() {
  // Sensores analógicos: apenas INPUT (sem pull-up)
  pinMode(PIN_S_LEFT,   INPUT);
  pinMode(PIN_S_CENTER, INPUT);
  pinMode(PIN_S_RIGHT,  INPUT);

  // Ultrassom
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);

  // Motores (H-bridge)
  pinMode(M_LEFT_IN1, OUTPUT);
  pinMode(M_LEFT_IN2, OUTPUT);
  pinMode(M_RIGHT_IN1, OUTPUT);
  pinMode(M_RIGHT_IN2, OUTPUT);
  // EN pins (mantém habilitados permanentemente)
  pinMode(M_LEFT_EN, OUTPUT);
  pinMode(M_RIGHT_EN, OUTPUT);
  // Use PWM on EN pins so podemos controlar potência por software.
  // Se quiser habilitar sempre na máxima potência, analogWrite com 255 equivale a HIGH.
  analogWrite(M_LEFT_EN, MOTOR_POWER);
  analogWrite(M_RIGHT_EN, MOTOR_POWER);

  // Inicializa saídas desligadas
  stopMotors();

  // LED onboard
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Serial: inicializa somente se DEBUG ativo (evita overhead quando não necessário)
#if DEBUG
  Serial.begin(115200);
  // Aguarda um breve momento para o monitor Serial se conectar (útil ao usar USB)
  delay(50);
  Serial.println(F("[BOOT] Sistema inicializado"));
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

  // Leitura ultrassônica (amostragem periódica para evitar bloqueio longo)
  static uint32_t lastUSms = 0;
  static unsigned int distCm = 0;
  uint32_t now = millis();

  // LED heartbeat (não bloqueante)
  static uint32_t lastLedMs = 0;
  static bool ledState = false;
  if (now - lastLedMs >= LED_BLINK_MS) {
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    lastLedMs = now;
  }

  // Mede distância ~25 vezes por segundo. Filtra leituras 0 para não "perder" o obstáculo quando está muito perto.
  if (now - lastUSms >= 40) { // ~25 Hz
    unsigned int d = measureDistanceCm();
    lastUSms = now;
    // Só atualiza se houve eco válido (> 0). Leituras 0 acontecem quando o objeto está muito perto ou não há eco.
    if (d > 0) {
      distCm = d;
    }
  }

  // Histerese simples para estado de obstáculo
  static bool obstacle = false;
  if (distCm > 0) {
    if (!obstacle && distCm <= ULTRASONIC_ALERT_CM) obstacle = true;
    else if (obstacle && distCm > (ULTRASONIC_ALERT_CM + 3)) obstacle = false;
  }

  Mode mode;

  // ---------------- Lógica de desvio de obstáculo ----------------
  // Entrada no modo de desvio: borda de subida de "obstacle"
  static bool prevObstacle = false;
  if (obstacle && !prevObstacle && !inAvoidMode) {
    // Começa a sequência de desvio: sair pela esquerda
    inAvoidMode = true;
    avoidState = AVOID_LEFT;
    avoidStartMs = now;
    avoidStateStartMs = now;
  }

  // Se tempo total de desvio estourar, aborta e para o robô
  if (inAvoidMode && (now - avoidStartMs >= AVOID_TOTAL_TIMEOUT_MS)) {
    inAvoidMode = false;
    avoidState = AVOID_NONE;
    mode = MODE_IDLE;
  }

  if (inAvoidMode) {
    // Sequência de desvio (ignora seguidor de linha normal enquanto estiver aqui)
    switch (avoidState) {
      case AVOID_LEFT:
        // Gira para a esquerda para sair da linha
        mode = MODE_LEFT;
        if (now - avoidStateStartMs >= AVOID_LEFT_MS) {
          avoidState = AVOID_FORWARD;
          avoidStateStartMs = now;
        }
        break;

      case AVOID_FORWARD:
        // Anda reto um pouco para passar o obstáculo
        mode = MODE_STRAIGHT;
        if (now - avoidStateStartMs >= AVOID_FORWARD_MS) {
          avoidState = AVOID_RIGHT;
          avoidStateStartMs = now;
        }
        break;

      case AVOID_RIGHT:
        // Gira para a direita para voltar em direção à linha
        mode = MODE_RIGHT;
        if (now - avoidStateStartMs >= AVOID_RIGHT_MS) {
          avoidState = AVOID_RETURN;
          avoidStateStartMs = now;
        }
        break;

      case AVOID_RETURN:
        // Anda reto tentando reencontrar a linha
        mode = MODE_STRAIGHT;
        // Se o centro enxergar a linha novamente, sai do modo de desvio
        if (C) {
          inAvoidMode = false;
          avoidState = AVOID_NONE;
        }
        break;

      default:
        // Qualquer outro caso, sai do modo de desvio por segurança
        inAvoidMode = false;
        avoidState = AVOID_NONE;
        mode = MODE_STRAIGHT;
        break;
    }
  } else {
    // ---------------- Seguidor de linha normal ----------------
    if (obstacle) {
      // Se ainda há obstáculo mas não estamos em desvio, por segurança para o robô
      mode = MODE_IDLE;
    } else {
      // Prioridade: se centro vê linha, siga reto; senão ajuste para o lado que vê.
      if (C) {
        mode = MODE_STRAIGHT;
      } else if (L && !R) {
        // inverter direções: sensor ESQUERDO agora força correção que antes era associada a MODE_RIGHT
        mode = MODE_RIGHT;
      } else if (R && !L) {
        // inverter direções: sensor DIREITO agora força correção que antes era associada a MODE_LEFT
        mode = MODE_LEFT;
      } else if (L && R) {
        // Linha larga / cruzamento: tente seguir reto
        mode = MODE_STRAIGHT;
      } else {
        // Nada detectado: seguir reto (pedido do usuário)
        mode = MODE_STRAIGHT;
      }
    }
  }

  prevObstacle = obstacle;

#if DEBUG
  static uint32_t t0 = 0;
  if (now - t0 > 100) {
    t0 = now;
    Serial.print(F("adc L=")); Serial.print(adcL);
    Serial.print(F(" C=")); Serial.print(adcC);
    Serial.print(F(" R=")); Serial.print(adcR);
    Serial.print(F(" | act L=")); Serial.print(L);
    Serial.print(F(" C=")); Serial.print(C);
    Serial.print(F(" R=")); Serial.print(R);
    Serial.print(F(" | DIST=")); Serial.print(distCm); Serial.print(F(" cm"));
    Serial.print(F(" obst=")); Serial.print(obstacle);
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
    case MODE_IDLE:     stopMotors(); break;
    default:            goStraight(); break; // opção segura: avance devagar
  }

  lastMode = mode;
}
