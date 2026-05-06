# 🤖 Projeto: Robô Seguidor de Linha Pro - Arduino Mega 2560

## 📍 Mapeamento de Hardware (NÃO ALTERAR)
- **Microcontrolador:** Arduino Mega 2560
- **Sensores de Linha (IR):** QTR-8 (8 sensores) nos pinos Digitais 24, 25, 26, 27, 28, 29, 30, 31.
- **Motores:** 4 motores DC via Motor Shield (M1, M2, M3, M4).
- **Comunicação I2C:** Multiplexador TCA9548A.
  - Porta 0: Giroscópio/Acelerômetro GY-521.
  - Porta 1: Sensor RGB TCS34725 (Direito).
  - Porta 2: Sensor RGB TCS34725 (Esquerdo).
  - Porta 3: Display OLED SSD1306.

## ⚙️ Arquitetura do Software (FSM)
O robô opera através de uma Máquina de Estados Finitos:
1. `ESTADO_CALIBRACAO`: Giro oscilatório para leitura de máx/mín do QTR-8.
2. `ESTADO_LINHA`: Controle PID para seguir diversas formas (V, W, U, L).
3. `ESTADO_VERDE`: Giro preciso (90° ou 180°) usando GY-521 após detecção de cor.
4. `ESTADO_VERMELHO`: Parada total (Stop).
5. `ESTADO_OBSTACULO`: Desvio via Sensores Ultrassônicos.

## 🧠 Lógica de Decisão (Regras de Negócio)
- **PID:** Erro baseado na posição central (3500) do QTR-8. Tratamento de perda de linha usando `ultimoErro`.
- **Giroscópio (Z/Yaw):** Referência absoluta para curvas de 90°.
- **Sensores de Cor:** 
  - Verde (Esq + Dir): Inverter sentido (180°).
  - Verde (Só um lado): Girar 90° para o lado correspondente.
  - Vermelho: Bloqueio total até limpeza do sensor.

## 🛠️ Bibliotecas Utilizadas
- `QTRSensors.h` (Pololu)
- `Adafruit_TCS34725.h` (Cores)
- `Adafruit_SSD1306.h` (OLED)
- `MPU6050_light.h` ou `Adafruit_MPU6050.h` (Giroscópio)

### 🟢 Lógica do Verde (Tomada de Decisão)
1. Detectou Verde -> Salva posição atual do Giroscópio (Yaw).
2. Entra no `ESTADO_VERDE`.
3. Gira os motores em sentidos opostos até que o Giroscópio aponte +90°, -90° ou 180°.
4. Ao atingir o ângulo, limpa as variáveis do PID (zera a integral) e volta para `ESTADO_LINHA`.