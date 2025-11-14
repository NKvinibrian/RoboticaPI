# Robótica PI – Seguidor de Linha (3 sensores IR + 2x 28BYJ-48)

Projeto para Arduino UNO com:
- 2 motores 28BYJ-48 via drivers ULN2003 (esquerdo e direito)
- 3 sensores IR lidos como ANALÓGICOS (A3, A4, A5)
- Lógica de seguidor de linha com threshold (padrão 500) e curvas suaves quando centro+lado ativos

O código principal está em `src/main.cpp`.

## Pinout (ajuste conforme sua fiação)
- Sensores (saída analógica AO → entradas analógicas do Arduino):
  - Esquerda: A3
  - Centro:  A4
  - Direita: A5
  - Threshold padrão: valores ADC > 500 são tratados como “ATIVO”. Ajuste `SENSOR_THRESHOLD` no código para calibrar.
- Motores (saídas para IN1..IN4 dos ULN2003):
  - Motor ESQUERDO (ULN2003 IN1..IN4): D8, D9, D10, D11
  - Motor DIREITO (ULN2003 IN1..IN4): D5, D6, D7, D12

A direção “frente” foi ajustada no código pelo campo `invert` de cada motor. Se ficar invertido no seu hardware, altere `motorLeft.invert` e/ou `motorRight.invert` em `main.cpp`.

## Regras de movimento
- Centro ativo → seguir reto.
- Lado esquerdo ativo (sem centro/direito) → virar à esquerda (pivô).
- Lado direito ativo (sem centro/esquerdo) → virar à direita (pivô).
- Centro + Esquerda ativos → curva suave à esquerda.
- Centro + Direita ativos → curva suave à direita.
- Nada detectado → seguir reto.

Curva suave usa menos passos por ciclo e/ou um delay mais alto, resultando em giro menos agressivo.

## Log Serial
- Log ATIVADO (`#define DEBUG 1`).
- Baud: 115200.
- Saída inclui valores ADC (0..1023) e flags interpretadas (L/C/R) com base no threshold, além do modo atual.

Exemplo:
```
adc L=640 C=275 R=860 | act L=1 C=0 R=1 | mode=RIGHT
```

## Como compilar e carregar

### Usando PlatformIO (recomendado)
- No VS Code:
  1. Abra a pasta do projeto.
  2. Use “Build” para compilar (env padrão: `uno`).
  3. “Upload” para carregar.
  4. “Monitor” (115200) para ver logs.
- Via CLI:
  ```bash
  pio run
  pio run -t upload
  pio device monitor -b 115200
  ```
- Outras placas: altere `default_envs` no `platformio.ini` ou use `-e` na CLI.

### Usando Arduino IDE
1. Abra `src/main.cpp` (ou copie para um `.ino`).
2. Placa: “Arduino Uno” (ou ajuste para sua placa).
3. Porta: selecione a correta.
4. Verificar → Carregar.
5. Monitor Serial em 115200 baud.

## Testar no Wokwi (https://wokwi.com)
- Abra o projeto e use o `diagram.json` como base (já contém um UNO).
- Adicione no editor do Wokwi:
  - 3 fontes analógicas (p. ex., potênciometros) nos pinos A3/A4/A5 para simular os sensores IR (AO).
  - 2 drivers ULN2003 + motores 28BYJ-48 conectados aos pinos D8–D11 (esquerdo) e D5–D7, D12 (direito).
- Inicie a simulação e abra o Serial (115200) para ver `adc`/`act`/`mode`.
- Dica: se preferir, pode usar sensores `wokwi-ir-reflect` em modo analógico.

## Dúvidas comuns de ligação
- GND comum é obrigatório: a bateria pode alimentar tudo, mas o GND da bateria deve estar em comum com o GND do Arduino, dos sensores e dos drivers. Sem GND comum, o Arduino não “enxerga” os sinais.
- Alimentar sensores direto da bateria (VCC/GND) não é problema se o GND for comum ao Arduino e a tensão for compatível (normalmente 5V). Se for maior que 5V, use regulador.
- Saída digital (DO) vs analógica (AO):
  - DO é binário e depende do comparador/potenciômetro do módulo (pode precisar ajustar o trimpot). Em pinos 2/3/4 funciona bem se nível lógico for 0/5V e GND comum.
  - AO entrega um valor proporcional (ADC 0..1023). É o que o código usa em A3/A4/A5.

## Ajustes de comportamento
- Threshold: `SENSOR_THRESHOLD` (padrão 500). Acima de 500 ativa o lado correspondente.
- Velocidade e “torque aparente” dos 28BYJ-48:
  - Aumente `STEP_DELAY_US_*` para reduzir velocidade e evitar perda de passo (parece mais torque).
  - Diminua para mais velocidade (pode perder torque se muito baixo).
  - Aumente `*_STEPS_PER_LOOP` para ações mais fortes por iteração.
  - Para mais força por passo, você pode trocar a sequência para “full-step de 2 fases” (fica como melhoria futura).
- Direção: ajuste os campos `invert` em `motorLeft`/`motorRight` se precisar.

## Troubleshooting rápido
- Nada no Serial: confira 115200 e a porta correta; no Wokwi, abra a aba Serial.
- Leituras estranhas: verifique GND comum, ruído na fiação dos sensores e fontes; se oscilar, considere média móvel simples no código.
- Motores tremendo/parados: aumente `STEP_DELAY_US_STRAIGHT/TURN` (ex.: 1500–2000 µs) e confira a ordem dos pinos IN1..IN4.

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
