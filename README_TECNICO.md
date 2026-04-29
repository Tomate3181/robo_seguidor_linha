### Especificações do Robô Seguidor de Linha (Mega 2560)

**1. Mapeamento de Hardware:**
* **Sensores IR (QTR-8):** Portas Digitais **24, 25, 26, 27, 28, 29, 30, 31**.
* **Multiplexador I2C (TCA9548A):**
    * `Porta 0`: Giroscópio/Acelerômetro (GY-521).
    * `Porta 1`: Sensor RGB Direito (TCS34725).
    * `Porta 2`: Sensor RGB Esquerdo (TCS34725).
    * `Porta 3`: Display OLED (SSD1306).
* **Motores (4 Unidades):** Controlados via Shield (M1, M2, M3, M4).
* **Ultrassônico:** (A definir pinos conforme conexão).

**2. Arquitetura do Software:**
* **Padrão:** Máquina de Estados Finitos (FSM).
* **Estados:** `ESTADO_CALIBRACAO`, `ESTADO_LINHA` (PID), `ESTADO_VERDE`, `ESTADO_VERMELHO`, `ESTADO_OBSTACULO`.
* **Regra de Ouro:** Código Não-Bloqueante. Proibido o uso de `delay()`, exceto na calibração inicial.

**3. Lógica de Cores:**
* `VERDE (Esq + Dir)`: Giro 180° via Giroscópio.
* `VERDE (Apenas um)`: Giro 90° para o lado detectado.
* `VERMELHO`: `StopAll()` e aguardar limpeza da leitura.