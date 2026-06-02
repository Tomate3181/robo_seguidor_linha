# Sistema Avançado de Desvio de Obstáculos Omini-Direcional com Fusão de Sensores
## Para Robô Seguidor de Linha (Arduino Mega 2560)

Este documento serve como uma especificação técnica detalhada, arquitetura de software e guia de implementação para engenheiros de sistemas e Inteligências Artificiais. O objetivo é configurar um sistema de desvio de obstáculos dinâmico, eficiente e determinístico para um robô seguidor de linha de alta performance baseado no microcontrolador **ATmega2560**.

---

## 1. Arquitetura do Hardware e Distribuição Sensorial

O robô utiliza uma matriz de sensores redundantes e complementares para mapeamento espacial imediato e controle de atitude.

* **Processador Central:** Arduino Mega 2560 (frequência de clock de 16 MHz, múltiplos timers de hardware e portas de interrupção disponíveis).
* **Arranjo Ultrassônico (3x Sensores HC-SR04 ou similar):**
    * *Sensor Frontal ($0^\circ$):* Dedicado à varredura de obstruções na trajetória retilínea da linha.
    * *Sensor Lateral Esquerdo ($-90^\circ$):* Responsável por monitorar a proximidade lateral do objeto e detectar o ponto de ultrapassagem caso o desvio ocorra pela esquerda.
    * *Sensor Lateral Direito ($+90^\circ$):* Responsável por monitorar a proximidade lateral do objeto e detectar o ponto de ultrapassagem caso o desvio ocorra pela direita.
* **Unidade de Medição Inercial - IMU (1x GY-521 / MPU6050):**
    * Posicionado o mais próximo possível do centro de massa (CoM) do chassi.
    * *Foco:* Eixo Z do Giroscópio ($\omega_z$ - rotação/Yaw) para integração de ângulo em tempo real, mitigando erros de odometria por derrapagem.
* **Matriz de Navegação de Superfície:** Sensores Infravermelhos (IR) reflexivos de alta velocidade e sensores RGB operando via multiplexador digital/analógico (ex: CD74HC4067).

---

## 2. Máquina de Estados Finitos (FSM)

O controle do robô deve ser estruturado obrigatoriamente através de uma Máquina de Estados Finitos Assíncrona para evitar o bloqueio de loops de controle síncronos e garantir determinismo.

### Detalhamento dos Estados:

1.  **`ESTADO_LINHA`:**
    * **Execução:** Algoritmo de controle proporcional, integral e derivativo (PID) processando a leitura dos sensores de linha.
    * **Condição de Transição:** Se a distância do `Ultrassônico Frontal` for menor que o limiar parametrizado ($D_{front} \le 15\text{ cm}$), armazena o estado atual, cessa a malha PID, zera o registrador do ângulo de Yaw do GY-521 e transita para `ESTADO_GIRO_INICIAL`.
2.  **`ESTADO_GIRO_INICIAL`:**
    * **Execução:** Aplica torque diferencial nos motores para rotacionar o robô sobre seu próprio eixo (ex: rotação horária ou anti-horária dependendo da estratégia de pista).
    * **Condição de Transição:** Integra continuamente a velocidade angular $\omega_z$. Quando o ângulo absoluto de Yaw ($\theta$) atingir o limite determinado ($\theta \ge \theta_{alvo}$ ou $\theta \le -\theta_{alvo}$, ex: $45^\circ$), para os motores e transita para `ESTADO_CONTORNO_LATERAL`.
3.  **`ESTADO_CONTORNO_LATERAL`:**
    * **Execução:** Descreve uma trajetória linear/tangencial adjacente ao obstáculo. Monitora continuamente o sensor ultrassônico lateral que ficou apontado para o objeto (ex: se girou para a esquerda, monitora o sensor direito).
    * **Condição de Transição:** Enquanto $D_{lateral} \le Limiar_{objeto}$ (ex: 25 cm), o robô mantém a progressão linear. No instante em que $D_{lateral} > Limiar_{escape}$ por um número $N$ de iterações consecutivas (sinalizando que o robô ultrapassou a barreira física do obstáculo), limpa o registrador do giroscópio e transita para `ESTADO_BUSCA_LINHA`.
4.  **`ESTADO_BUSCA_LINHA`:**
    * **Execução:** Curva em arco reverso aproximando-se diagonalmente da pista original.
    * **Condição de Transição:** Varredura em alta frequência dos sensores IR/RGB de borda e centro. Assim que o sensor de contorno ou os sensores centrais interceptarem o gradiente da linha com histerese válida, interrompe a manobra de busca, reinicia as variáveis integrais e derivativas do PID e retorna para `ESTADO_LINHA`.

---

## 3. Diretrizes de Implementation de Código (Práticas Recomendadas)

Para garantir que o firmware execute de forma eficiente no ATmega2560, a IA deve seguir as seguintes restrições algorítmicas:

### A. Leituras de Sensores Não-Bloqueantes
* **Proibido:** O uso da função nativa `pulseIn()` do ecossistema Arduino, pois ela suspende a execução do thread único por até dezenas de milissegundos esperando pelo eco.
* **Solução:** Implementar leituras baseadas em temporizadores de hardware ou usar a biblioteca `NewPing` configurada no modo de varredura por interrupção de timer (*Timer Interrupt Event Driven*).

### B. Tratamento de Dados da IMU (GY-521)
* **Filtro de Fusão:** O sinal bruto do giroscópio apresenta ruído de alta frequência e desvio estático (*drift*). Deve ser implementado um **Filtro Complementar** de primeira ordem ou Filtro de Kalman simplificado para acoplar os dados do acelerômetro e giroscópio:
    $$\theta_{atual} = \alpha \cdot (\theta_{anterior} + \omega_z \cdot \Delta t) + (1 - \alpha) \cdot \text{Acc}_y$$
    *(Onde $\alpha \approx 0.98$ e $\Delta t$ é o tempo exato de amostragem determinado por `micros()`).*
* Para simplificação matemática otimizada, pode ser empregada a biblioteca `MPU6050_light` realizando chamadas estritas do método `.update()` em cada ciclo do loop principal.

### C. Controle de Velocidade e Tração Diferencial
* Durante a execução dos estados de desvio (`ESTADO_GIRO_INICIAL`, `ESTADO_CONTORNO_LATERAL` e `ESTADO_BUSCA_LINHA`), o valor base de PWM fornecido aos motores de passo ou motores DC via Ponte H deve ser reduzido em pelo menos $30\%$ a $50\%$ em relação à velocidade nominal do PID de linha.
* **Justificativa:** Reduzir a derrapagem física (slip) dos pneus com a superfície, garantindo que o deslocamento calculado pela IMU seja fiel ao vetor de deslocamento real do robô.

---

## 4. Estrutura de Código Sugerida (Template C++)

```cpp
#include <Wire.h>
#include <MPU6050_light.h>
#include <NewPing.h>

// Definições de Pinos e Sensores Ultrassônicos
// definição dos pinos ja feita no arquivo config.h
#define MAX_DISTANCE 200

NewPing sonarF(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonarE(TRIGGER_PIN_E, ECHO_PIN_E, MAX_DISTANCE);
NewPing sonarD(TRIGGER_PIN_D, ECHO_PIN_D, MAX_DISTANCE);

MPU6050 mpu(Wire);
unsigned long cronometroEspera = 0;


void loop() {
  mpu.update();
  
  switch (estadoAtual) {
    case ESTADO_LINHA:
      executarPIDLinha();
      if (ObterDistanciaFiltrada(sonarF) <= 15) {
        ConfigurarMotores(0, 0); // Parada imediata
        cronometroEspera = millis();
        estadoAtual = ESTADO_GIRO_INICIAL;
      }
      break;

    case ESTADO_GIRO_INICIAL:
      // Executa rotação controlada baseada no Giroscópio
      ConfigurarMotores(-100, 100); // Rotação sobre o eixo para a esquerda
      if (mpu.getAngleZ() >= 45.0) { 
        ConfigurarMotores(0, 0);
        estadoAtual = ESTADO_CONTORNO_LATERAL;
      }
      break;

    case ESTADO_CONTORNO_LATERAL:
      ConfigurarMotores(120, 120); // Anda para frente contornando
      // Monitora o sensor ultrassônico lateral direito
      if (ObterDistanciaFiltrada(sonarD) > 30) {
        estadoAtual = ESTADO_BUSCA_LINHA;
      }
      break;

    case ESTADO_BUSCA_LINHA:
      ConfigurarMotores(100, -100); // Curva reversa para a direita buscando a linha
      if (VerificarSensoresLinha()) {
        ConfigurarMotores(0, 0);
        ReiniciarConstantesPID();
        estadoAtual = ESTADO_LINHA;
      }
      break;
  }
}

// Funções de Abstração de Hardware (Stubs)
void executarPIDLinha() { /* Código PID Existente */ }
bool VerificarSensoresLinha() { /* Retorna true se detectar a linha */ return false; }
void ReiniciarConstantesPID() { /* Zera o erro acumulado da integral */ }
void ConfigurarMotores(int pwmEsquerdo, int pwmDireito) { /* Controle da Ponte H */ }
int ObterDistanciaFiltrada(NewPing &sonar) { int d = sonar.ping_cm(); return (d == 0) ? MAX_DISTANCE : d; }

```

---

## 5. Casos de Exceção e Tratamento de Falhas (Fail-safe)

1. **Bloqueio por Ruído Ultrassônico (Leitura Falsa de Zero):** A biblioteca `NewPing` retorna `0` quando a leitura excede o tempo limite de eco ou falha. O algoritmo deve tratar o valor `0` convertendo-o para a distância máxima (`MAX_DISTANCE`), evitando disparos falsos de desvio de obstáculos no meio da pista aberta.
2. **Derrapagem Acentuada no Giro:** Caso o robô patine e o giroscópio falhe em atingir o ângulo exato, deve ser implementado um *Timeout Watchdog* de tempo (ex: se em 2.5 segundos no estado `ESTADO_GIRO_INICIAL` o robô não mudar de estado, ele força a transição por segurança).
3. **Perda Total da Linha na Busca:** Se no estado `ESTADO_BUSCA_LINHA` o ângulo acumulado de retorno da IMU passar de um limite crítico inverso (ex: $-90^\circ$) sem encontrar a linha, o robô deve entrar em modo de varredura circular de emergência de $360^\circ$ ou parar completamente para evitar evasão da pista.

## Estados do robo

os estados do robo são esses aqui que estão no codigo config.h:

```cpp
enum EstadoRobo {
  ESTADO_CALIBRACAO,
  ESTADO_LINHA,
  ESTADO_VERDE,
  ESTADO_VERMELHO,
  ESTADO_OBSTACULO
};
```