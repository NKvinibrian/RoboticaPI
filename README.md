# Robótica PI – Seguidor de Linha (3 sensores IR + 2x 28BYJ-48)

Projeto para Arduino UNO com:
- 2 motores 28BYJ-48 via drivers ULN2003 (esquerdo e direito)
- 3 sensores IR agora lidos como ANALÓGICOS (A0, A1, A2)
- Lógica de seguidor de linha com threshold (padrão 500)

O código principal está em `src/main.cpp`.

## Pinout (ajuste conforme sua fiação)
- Sensores (saída analógica AO → entradas analógicas do Arduino):
  - Esquerda: A0
  - Centro:  A1
  - Direita: A2
  - Threshold padrão: valores ADC > 500 são tratados como “ATIVO”. Ajuste `SENSOR_THRESHOLD` no código para calibrar.
- Motores (saídas para IN1..IN4 dos ULN2003):
  - Motor ESQUERDO (ULN2003 IN1..IN4): D8, D9, D10, D11
  - Motor DIREITO (ULN2003 IN1..IN4): D5, D6, D7, D12

A direção “frente” foi ajustada no código (campo `invert` de cada motor). Se precisar inverter novamente, edite `motorLeft.invert` e/ou `motorRight.invert` em `main.cpp`.

## Log Serial
- O log está ATIVADO (`#define DEBUG 1`).
- Baud: 115200.
- Saída inclui valores ADC (0..1023) e flags interpretadas (L/C/R) com base no threshold.

Exemplo:
```
adc L=640 C=275 R=860 | act L=1 C=0 R=1 | mode=RIGHT
```

## Como compilar e carregar

### Usando PlatformIO (recomendado)
- Passos no VS Code:
  1. Abra a pasta do projeto.
  2. “Build” para compilar (env padrão: `uno`).
  3. “Upload” para carregar.
  4. “Monitor” (115200) para ver os logs.
- Via CLI:
  ```bash
  pio run
  pio run -t upload
  pio device monitor -b 115200
  ```
- Outras placas: use `-e megaatmega1280` ou altere `default_envs` no `platformio.ini`.

### Usando Arduino IDE
1. Abra `src/main.cpp` (ou copie para um `.ino`).
2. Placa: “Arduino Uno” (ou ajuste para sua placa).
3. Porta: selecione a correta.
4. Verificar → Carregar.
5. Monitor Serial em 115200 baud.

## Testar no Wokwi (https://wokwi.com)
- `diagram.json` define um UNO vazio. Para simular, adicione no Wokwi sensores com saída analógica (ou potênciometros) ligados a A0/A1/A2 e drivers ULN2003 + 28BYJ-48 aos pinos listados acima.
- Inicie a simulação e abra o Serial (115200) para ver `adc` e `act`.

## Ajustes de comportamento
- Threshold: `SENSOR_THRESHOLD` (padrão 500). Acima de 500 ativa o lado correspondente.
  - Regras: `adcC > threshold` → reto; `adcL > threshold` (sem `adcR`) → vira esquerda; `adcR > threshold` (sem `adcL`) → vira direita.
- Velocidade: `STEP_DELAY_US_STRAIGHT`, `STEP_DELAY_US_TURN`.
- Passos por loop: `STRAIGHT_STEPS_PER_LOOP`, `TURN_STEPS_PER_LOOP`.

## Troubleshooting
- Nada no Serial: confira 115200 e porta correta; no Wokwi, abra a aba Serial.
- Direção invertida: ajuste `invert` nos structs `motorLeft`/`motorRight`.
- Oscilações no ADC: use fiação curta, GND comum e, se necessário, média móvel/filtragem (pode ser adicionada).
- Sensores só digitais (sem AO): não use este modo; use a saída DO e leitura digital (ou substitua por sensores com AO).

## Estrutura
```
platformio.ini
src/main.cpp
diagram.json
include/
lib/
test/
```

Atualize este README conforme você evoluir a fiação/diagrama.
