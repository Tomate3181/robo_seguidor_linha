# 🤖 Projeto: Robô Seguidor de Linha Pro - Arduino Mega 2560

## 📍 Mapeamento de Hardware
- **Microcontrolador:** Arduino Mega 2560
- **Motores:** 4 motores DC via Motor Shield (M1, M2, M3, M4).
- **Sensor de Refletância (QRE-8D):**
  - 1: vermelho - 30
  - 2: branco - 31
  - 3: laranja - 32
  - 4: verde - 33
  - 5: amarelo - 34
  - 6: cinza - 35
  - 7: azul - 36
  - 8: vermelho - 37
- **Sensor Ultrassônico (HC-SR04):**
  - **Frente:** TRIG - amarelo - 48 | ECHO - laranja - 49
  - **Esquerda:** TRIG - amarelo - 50 | ECHO - laranja - 51
  - **Direita:** TRIG - amarelo - 52 | ECHO - laranja - 53
- **Comunicação I2C (Multiplexador TCA9548A):**
  - **Porta 0 (Display OLED):** SDK - roxo - SD0 | SCK - azul - SC0
  - **Porta 1 (Giroscópio GY-521):** SDA - roxo - SD1 | SCL - azul - SC1
  - **Porta 2 (Sensor RGB TCS34725 - Direita):** SDA - roxo - SD2 | SCL - azul - SC2
  - **Porta 3 (Sensor RGB TCS34725 - Esquerda):** SDA - roxo - SD3 | SCL - azul - SC3

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