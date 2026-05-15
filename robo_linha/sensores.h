#ifndef SENSORES_H
#define SENSORES_H

#include <QTRSensors.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050_light.h>
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

// Objeto do Giroscópio
MPU6050 mpu(Wire);

// Variável para controle não-bloqueante de leitura de cor
unsigned long ultimaLeituraCor = 0;

// Variáveis dinâmicas para calibração de luminosidade (ignorar sombras)
uint16_t limiarLuminosidadeDir = 80;
uint16_t limiarLuminosidadeEsq = 80;

// Referências às variáveis globais em robo_linha.ino
extern EstadoRobo estadoAtual;
extern int tipoGiro;
extern float anguloInicial;

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
  
  // Inicializa Giroscópio (Porta 0)
  tcaselect(CANAL_GY521);
  byte status = mpu.begin();
  if(status != 0) {
    Serial.println(F("ERRO: MPU6050 não encontrado!"));
  } else {
    Serial.println(F("Calculando offsets do MPU... Não mova o robô!"));
    atualizarStatus("Giroscopio", "Calibrando");
    // O delay aqui é aceitável pois está no setup
    delay(1000); 
    mpu.calcOffsets(true,true);
    atualizarStatus("Giroscopio", "OK");
  }
}

// ==============================================================================
// LÓGICA DE VALIDAÇÃO DE CORES
// ==============================================================================

bool ehVermelho(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  if (c < limiarC) return false;  // Ignora se estiver muito escuro (provável linha preta ou sombra)
  if (r < 80) return false; // Ignora se tiver pouco vermelho absoluto

  // O Vermelho (R) precisa ser consideravelmente maior que o G e o B para evitar confusões com branco/amarelo.
  float margem = 1.35; 
  if (r > (g * margem) && r > (b * margem)) {
    return true; 
  }
  return false;
}

bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  if (c < limiarC) return false;  // Ignora fundo preto (reduz chance de ler verde nas sombras)
  if (g < 80) return false;

  float margem = 1.35; // Aumentamos a precisão da proporção: o verde tem que ser dominante
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
  
  // Validação comparativa com os novos limiares dinâmicos
  bool verdeDir = ehVerde(rD, gD, bD, cD, limiarLuminosidadeDir);
  bool vermelhoDir = ehVermelho(rD, gD, bD, cD, limiarLuminosidadeDir);
  
  bool verdeEsq = ehVerde(rE, gE, bE, cE, limiarLuminosidadeEsq);
  bool vermelhoEsq = ehVermelho(rE, gE, bE, cE, limiarLuminosidadeEsq);
  
  // Regras de Decisão FSM
  if (vermelhoDir || vermelhoEsq) {
    estadoAtual = ESTADO_VERMELHO;
    atualizarStatus("COR", "VERMELHO");
    return;
  }
  
  if (verdeDir && verdeEsq) {
    estadoAtual = ESTADO_VERDE;
    tipoGiro = 180;
    anguloInicial = mpu.getAngleZ(); // Salva o ângulo atual
    atualizarStatus("VERDE DUPLO", "Giro 180");
    return;
  } else if (verdeDir) {
    estadoAtual = ESTADO_VERDE;
    tipoGiro = 90;
    anguloInicial = mpu.getAngleZ(); // Salva o ângulo atual
    atualizarStatus("VERDE DIR", "Giro 90");
    return;
  } else if (verdeEsq) {
    estadoAtual = ESTADO_VERDE;
    tipoGiro = -90;
    anguloInicial = mpu.getAngleZ(); // Salva o ângulo atual
    atualizarStatus("VERDE ESQ", "Giro -90");
    return;
  }
}

void executarCalibracao() {
  // Feedback visual no OLED
  atualizarStatus("Sistema", "CALIBRANDO...");
  
  unsigned long tempoInicio = millis();
  unsigned long ultimoInversao = millis();
  unsigned long ultimaLeituraCorCalib = 0;
  bool sentidoGiro = true; // true = Horário, false = Anti-horário
  
  uint16_t maxCDir = 0;
  uint16_t maxCEsq = 0;
  
  // Aciona os motores para começar o giro sobre o próprio eixo
  rotacionarEixo(sentidoGiro, VELOCIDADE_BASE);
  
  // Loop de calibração que dura exatamente 5 segundos (5000 ms)
  while (millis() - tempoInicio < 5000) {
    qtr.calibrate();
    
    // Leitura de cor em background para achar a luminosidade do fundo branco da pista
    if (millis() - ultimaLeituraCorCalib > 100) {
      ultimaLeituraCorCalib = millis();
      uint16_t r, g, b, c;
      
      tcaselect(CANAL_TCS_DIR);
      tcsDir.getRawData(&r, &g, &b, &c);
      if (c > maxCDir) maxCDir = c;
      
      tcaselect(CANAL_TCS_ESQ);
      tcsEsq.getRawData(&r, &g, &b, &c);
      if (c > maxCEsq) maxCEsq = c;
    }
    
    // Lógica de oscilação: Inverte o sentido dos motores a cada 500ms
    if (millis() - ultimoInversao >= 500) {
      sentidoGiro = !sentidoGiro; // Inverte o valor booleano
      rotacionarEixo(sentidoGiro, VELOCIDADE_BASE); // Aplica a nova direção
      ultimoInversao = millis();  // Reseta o temporizador de inversão
    }
  }
  
  // Fim da calibração
  pararMotores();
  
  // Define o limiar de corte baseando-se em 30% da luminosidade encontrada no branco
  limiarLuminosidadeDir = maxCDir * 0.30;
  limiarLuminosidadeEsq = maxCEsq * 0.30;
  
  // Garante um piso de 80 caso a leitura esteja escura demais ou com problemas
  if (limiarLuminosidadeDir < 80) limiarLuminosidadeDir = 80;
  if (limiarLuminosidadeEsq < 80) limiarLuminosidadeEsq = 80;
  
  // Feedback de conclusão no OLED
  atualizarStatus("CALIBRACAO", "OK");
  
  // Apenas nesta rotina específica do Setup inicial o uso de delay() é aceitável
  delay(1000); 
  
  // Transição de Estado
  estadoAtual = ESTADO_LINHA;
}

#endif // SENSORES_H
