# Projeto Robótica PI


## Estrutura do Repositório
```
platformio.ini      # Configuração PlatformIO (env: uno default, mega opcional)
src/main.cpp        # Código fonte principal (pisca LED + log serial)
diagram.json        # Diagrama Wokwi (Arduino UNO)
include/            # Cabeçalhos (vazio por enquanto)
lib/                # Bibliotecas locais (vazio)
test/               # Testes (placeholder)
```

## Requisitos
- VS Code + extensão PlatformIO IDE (ou CLI PlatformIO instalada)
- Ou Arduino IDE (>= 2.x)
- Opcional: Conta no Wokwi (https://wokwi.com) para simulação online

## Compilar e Executar com PlatformIO
### Via Extensão (VS Code)
1. Abra a pasta do projeto no VS Code.
2. Na aba PlatformIO (barra lateral), clique em "Build" para compilar.
3. Conecte a placa (ex: Arduino UNO) e clique em "Upload".
4. Abra "Monitor" para ver as mensagens: `LED: ON` / `LED: OFF`.

### Via Linha de Comando
No diretório do projeto:
```bash
# Compilar ambiente padrão (uno)
pio run

# Fazer upload para a placa conectada
tio? # (opcional) verificar portas seriais disponíveis
pio run -t upload

# Abrir monitor serial (9600 baud)
pio device monitor -b 9600
```

### Selecionar Outra Placa
O arquivo `platformio.ini` já contém um ambiente para MEGA (ATmega1280). Para usar:
```bash
pio run -e megaatmega1280 -t upload
pio device monitor -b 9600 -e megaatmega1280
```
Ou altere `default_envs = uno` para `default_envs = megaatmega1280`.

## Compilar com Arduino IDE
1. Abra o Arduino IDE.
2. Arquivo > Abrir... e selecione `src/main.cpp` (ou copie o conteúdo para um novo sketch `.ino`).
   - Se preferir, renomeie para `RoboticaPI.ino` e coloque em uma pasta `RoboticaPI/` seguindo o padrão do Arduino.
3. Ferramentas > Placa > selecione "Arduino Uno" (ou "Arduino Mega or Mega 2560" caso ajuste para MEGA; o código usa `LED_BUILTIN` e é compatível).
4. Ferramentas > Porta > selecione a porta serial correta.
5. Clique em "Verificar" (compilar) e depois "Carregar".
6. Abra o Monitor Serial (9600 baud) para ver a saída.

## Simulação no Wokwi
O arquivo `diagram.json` descreve um Arduino UNO simples (sem conexões extras). Passos:

### Método 1: Criar Projeto Manualmente
1. Acesse https://wokwi.com e clique em "New Project" > Arduino Uno.
2. Substitua o código padrão pelo conteúdo de `src/main.cpp`.
3. Clique no botão de engrenagem (ou menu) e selecione "Edit Diagram".
4. Cole o conteúdo de `diagram.json` no editor de diagrama (substituindo o existente) e salve.
5. Pressione "Start Simulation". O console mostrará as mensagens do Serial. O LED virtual (L) pisca.

### Método 2: Importar `diagram.json`
1. Dentro de um projeto Wokwi já criado, abra o painel de arquivos (ícone de pasta).
2. Adicione/importe `diagram.json` (arrastar e soltar ou botão Upload) com o mesmo conteúdo deste repositório.
3. Garanta que o tipo de placa no diagrama é `wokwi-arduino-uno` (já definido).
4. Inicie a simulação.

### Ajustar Intervalo de Pisca
Edite em `main.cpp`:
```cpp
const unsigned long INTERVALO_MS = 500; // ajuste aqui
```
Clique em "Start Simulation" novamente (ou reinicie) para ver a nova taxa.

## Uso do `diagram.json`
Campos principais:
- `parts`: lista de componentes (aqui apenas `wokwi-arduino-uno`).
- `connections`: vazio (nenhum fio extra). Adicione itens ao inserir sensores, LEDs externos, etc.

Para adicionar um LED externo futuramente (exemplo rápido):
1. No Wokwi, clique em "+" e adicione um LED e um resistor.
2. Conecte pino digital (ex: 8) -> resistor -> LED -> GND.
3. Atualize o código para usar `pinMode(8, OUTPUT);` e `digitalWrite(8, HIGH/LOW);`.
4. Salve o novo diagrama exportado se quiser sincronizar com o repo.

## Monitor Serial (Mensagens Esperadas)
```
Teste LED iniciado. LED deve piscar a cada 500 ms.
LED: ON
LED: OFF
...
```
Se nada aparece:
- Verifique baud = 9600.
- Confirme cabo/detecção da porta.
- Reinicie a simulação (Wokwi) ou pressione reset físico.

## Troubleshooting
| Problema | Causa Comum | Solução |
|----------|-------------|---------|
| Upload falha (PlatformIO) | Porta errada | Ajuste `upload_port` ou desconecte outros dispositivos |
| Nada no Serial | Baud incorreto | Use 9600 no monitor |
| LED não pisca | Código travando | Evite `delay()` longo (não usado aqui) |
| Simulação sem log | Esqueceu de abrir Serial console no Wokwi | Abra aba Serial no painel |

## Licença
Definir uma licença (ex: MIT) conforme necessidade.

---
Atualize este README conforme novos componentes forem adicionados ao `diagram.json`.
