#ifndef SENSORES_H
#define SENSORES_H

#include <QTRSensors.h>
#include <Adafruit_TCS34725.h>
#include "config.h"
#include "display_utils.h"
#include "motores.h"

// Assinatura do multiplexador (definido em robo_linha.ino)
extern void tcaselect(uint8_t i);

// Objeto da biblioteca QTRSensors
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORES_IR];

// Objetos dos sensores RGB TCS34725
Adafruit_TCS34725 tcsDir = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcsEsq = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

// Variável para controle não-bloqueante de leitura de cor
unsigned long ultimaLeituraCor = 0;

// Referências às variáveis globais em robo_linha.ino
extern EstadoRobo estadoAtual;
extern int tipoGiro;

// ==============================================================================
// FUNÇÕES DOS SENSORES
// ==============================================================================

void initSensores() {
  // Configurando sensores IR como RC
  qtr.setTypeRC();
  qtr.setSensorPins(PINOS_IR, NUM_SENSORES_IR);
  
  // Inicializa Sensor RGB Direito (Porta 1)
  tcaselect(CANAL_TCS_DIR);
  if (!tcsDir.begin()) {
    Serial.println(F("ERRO: TCS Direito não encontrado!"));
  }
  
  // Inicializa Sensor RGB Esquerdo (Porta 2)
  tcaselect(CANAL_TCS_ESQ);
  if (!tcsEsq.begin()) {
    Serial.println(F("ERRO: TCS Esquerdo não encontrado!"));
  }
}

// ==============================================================================
// LÓGICA DE VALIDAÇÃO DE CORES
// ==============================================================================

bool ehVermelho(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 80) return false;  // Ignora se estiver muito escuro
  if (r < 100) return false; // Ignora se tiver pouco vermelho absoluto

  // O Vermelho (R) precisa ser consideravelmente maior que o G e o B.
  float margem = 1.25; 
  if (r > (g * margem) && r > (b * margem)) {
    return true; 
  }
  return false;
}

bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 80) return false; 
  if (g < 100) return false;

  float margem = 1.185; 
  if (g > (r * margem) && g > (b * margem)) {
    return true; 
  }
  return false;
}

void verificarCores() {
  // Executa a leitura apenas a cada 50ms para não atrasar o controle PID da linha
  if (millis() - ultimaLeituraCor < 50) return;
  ultimaLeituraCor = millis();
  
  uint16_t rD, gD, bD, cD;
  uint16_t rE, gE, bE, cE;
  
  // Leitura RGB Direito
  tcaselect(CANAL_TCS_DIR);
  tcsDir.getRawData(&rD, &gD, &bD, &cD);
  
  // Leitura RGB Esquerdo
  tcaselect(CANAL_TCS_ESQ);
  tcsEsq.getRawData(&rE, &gE, &bE, &cE);
  
  // Validação comparativa (Aplica as margens seguras para evitar falso positivo)
  bool verdeDir = ehVerde(rD, gD, bD, cD);
  bool vermelhoDir = ehVermelho(rD, gD, bD, cD);
  
  bool verdeEsq = ehVerde(rE, gE, bE, cE);
  bool vermelhoEsq = ehVermelho(rE, gE, bE, cE);
  
  // Regras de Decisão FSM
  if (vermelhoDir || vermelhoEsq) {
    estadoAtual = ESTADO_VERMELHO;
    atualizarStatus("COR", "VERMELHO");
    return;
  }
  
  if (verdeDir && verdeEsq) {
    estadoAtual = ESTADO_VERDE;
    tipoGiro = 180;
    atualizarStatus("VERDE DUPLO", "Giro 180");
    return;
  } else if (verdeDir) {
    estadoAtual = ESTADO_VERDE;
    tipoGiro = 90;
    atualizarStatus("VERDE DIR", "Giro 90");
    return;
  } else if (verdeEsq) {
    estadoAtual = ESTADO_VERDE;
    tipoGiro = -90;
    atualizarStatus("VERDE ESQ", "Giro -90");
    return;
  }
}

void executarCalibracao() {
  // Feedback visual no OLED
  atualizarStatus("Sistema", "CALIBRANDO...");
  
  unsigned long tempoInicio = millis();
  unsigned long ultimoInversao = millis();
  bool sentidoGiro = true; // true = Horário, false = Anti-horário
  
  // Aciona os motores para começar o giro sobre o próprio eixo
  rotacionarEixo(sentidoGiro, VELOCIDADE_BASE);
  
  // Loop de calibração que dura exatamente 5 segundos (5000 ms)
  while (millis() - tempoInicio < 5000) {
    qtr.calibrate();
    
    // Lógica de oscilação: Inverte o sentido dos motores a cada 500ms
    if (millis() - ultimoInversao >= 500) {
      sentidoGiro = !sentidoGiro; // Inverte o valor booleano
      rotacionarEixo(sentidoGiro, VELOCIDADE_BASE); // Aplica a nova direção
      ultimoInversao = millis();  // Reseta o temporizador de inversão
    }
  }
  
  // Fim da calibração
  pararMotores();
  
  // Feedback de conclusão no OLED
  atualizarStatus("CALIBRACAO", "OK");
  
  // Apenas nesta rotina específica do Setup inicial o uso de delay() é aceitável
  delay(1000); 
  
  // Transição de Estado
  estadoAtual = ESTADO_LINHA;
}

#endif // SENSORES_H
