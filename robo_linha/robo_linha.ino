#include <Wire.h>
#include "config.h"
#include "display_utils.h"
#include "motores.h"
#include "sensores.h"

// Variável global para armazenar o estado atual da Máquina de Estados Finitos (FSM)
EstadoRobo estadoAtual = ESTADO_CALIBRACAO;

// Variáveis de controle de linha
ModoLinha modoLinha = SEGUINDO;
int ultimoLado = 0;
int ultimoErro = 0;
int contadorFalhas = 0;
unsigned long tempoInicioInsistir = 0;
unsigned long tempoInicioGap = 0;

// Variável para armazenar o tipo de giro determinado pelo sensor RGB
int tipoGiro = 0;


// ==============================================================================
// FUNÇÃO DO MULTIPLEXADOR I2C (TCA9548A)
// ==============================================================================
// Seleciona o canal I2C (0 a 7) para comunicar com um dispositivo específico
// Esta função DEVE ser chamada antes de qualquer comunicação I2C (OLED, GY-521, TCS)
void tcaselect(uint8_t i) {
  if (i > 7) return; // Segurança: Evita selecionar um canal inexistente
  
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);
  
  // Inicializa o barramento I2C
  Wire.begin(); 
  
  Serial.println(F("======================================="));
  Serial.println(F("   Iniciando Robo Seguidor de Linha    "));
  Serial.println(F("======================================="));
  
  // Configuração inicial de pinos avulsos (ex: Ultrassom)
  pinMode(PINO_TRIG, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  
  // Inicialização do Display OLED
  initDisplay();
  atualizarStatus("Sistema", "Iniciado!");
  
  // Inicialização dos módulos
  initMotores();
  initSensores();
}

// ==============================================================================
// LOOP PRINCIPAL (Máquina de Estados)
// ==============================================================================
void loop() {
  // REGRA DE OURO: Código não-bloqueante. Não utilize delay() no loop principal!
  
  switch (estadoAtual) {
    case ESTADO_CALIBRACAO:
      // Executa a calibração dos sensores e do giroscópio
      executarCalibracao();
      break;

    case ESTADO_LINHA: {
      // Leitura da posição da linha e dos sensores
      uint16_t position = qtr.readLineBlack(sensorValues);
      bool vendoLinha = false;
      for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
        if (sensorValues[i] > 200) {
          vendoLinha = true;
          break;
        }
      }

      // Verifica as cores a cada 50ms para intersecções/obstáculos
      verificarCores();

      // Se verificarCores alterou o estado (achou verde/vermelho), saímos do case
      if (estadoAtual != ESTADO_LINHA) {
        break;
      }

      // Máquina de estados interna para controle da linha (Não-bloqueante)
      switch (modoLinha) {
        case SEGUINDO:
          if (!vendoLinha) {
            modoLinha = INSISTINDO;
            tempoInicioInsistir = millis();
          } else {
            contadorFalhas = 0; 
            int erro = position - 3500;
            
            // Memoriza o lado para curvas fechadas
            if (erro > 500) ultimoLado = 1;       
            else if (erro < -500) ultimoLado = -1; 

            // Cálculo do PID
            int P = erro * KP;
            int D = (erro - ultimoErro) * KD;
            int ajuste = P + D;
            ultimoErro = erro;

            controlarRodas(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste);
          }
          break;

        case INSISTINDO:
          // Tenta insistir na curva por 150ms
          if (millis() - tempoInicioInsistir < 150) {
            if (ultimoLado == 1) controlarRodas(180, -100); 
            else controlarRodas(-100, 180);

            // Se reencontrou a linha (sensores centrais)
            if (vendoLinha && (sensorValues[3] > 400 || sensorValues[4] > 400)) {
              modoLinha = SEGUINDO; 
            }
          } else {
            // Falhou em encontrar a linha, passa para o tratamento de Gap
            contadorFalhas++;
            if (contadorFalhas >= 3) {
              modoLinha = GAP_RE_AJUSTE;
              tempoInicioGap = millis();
            } else {
              modoLinha = GAP_AVANCA;
              tempoInicioGap = millis();
            }
          }
          break;

        case GAP_AVANCA:
          // Avança pelo tempo determinado para buscar a linha após um gap
          if (millis() - tempoInicioGap < TEMPO_PARA_12CM) {
            controlarRodas(VELOCIDADE_GAP, VELOCIDADE_GAP);
            if (vendoLinha && (sensorValues[3] > 400 || sensorValues[4] > 400)) {
              modoLinha = SEGUINDO; // Encontrou a linha do outro lado do gap
            }
          } else {
            // Não encontrou andando para frente, recua
            controlarRodas(-VELOCIDADE_GAP, -VELOCIDADE_GAP);
            if (vendoLinha && (sensorValues[3] > 400 || sensorValues[4] > 400)) {
              modoLinha = SEGUINDO;
            }
          }
          break;

        case GAP_RE_AJUSTE:
          // Dá ré por 2 segundos após falhar várias vezes no gap
          if (millis() - tempoInicioGap < 2000) {
            controlarRodas(-100, -100);
          } else {
            contadorFalhas = 0;
            modoLinha = SEGUINDO; // Retoma a tentativa de seguir
          }
          break;
      }
      break;
    }

    case ESTADO_VERDE:
      // Lógica de intersecção/curva acionada por sensor(es) verde(s):
      // - 1 Sensor: Giro de 90° (para a direção do sensor) usando odometria/giroscópio
      // - 2 Sensores: Giro de 180° usando giroscópio
      break;

    case ESTADO_VERMELHO:
      // Lógica acionada ao detectar a cor vermelha:
      // - Parar os motores (StopAll)
      // - Aguardar liberação
      break;

    case ESTADO_OBSTACULO:
      // Lógica de desvio quando o ultrassom detecta um objeto no caminho
      break;

    default:
      // Fallback de segurança para garantir que o robô não trave em um estado inexistente
      estadoAtual = ESTADO_LINHA;
      break;
  }
}
