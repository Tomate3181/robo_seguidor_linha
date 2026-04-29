#include <Wire.h>
#include "config.h"
#include "display_utils.h"
#include "motores.h"
#include "sensores.h"

// Variável global para armazenar o estado atual da Máquina de Estados Finitos (FSM)
EstadoRobo estadoAtual = ESTADO_CALIBRACAO;

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

    case ESTADO_LINHA:
      // Lógica principal: Algoritmo PID para seguir a linha usando os 8 sensores IR
      // - Lê a posição da linha
      // - Calcula o erro
      // - Ajusta as velocidades dos motores
      break;

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
