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

// ESTRUTURA PARA SALVAR A ASSINATURA RGB DO VERDE CALIBRADO
struct AssinaturaCor {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
};

AssinaturaCor verdeCalibradoDir = {0, 0, 0, 0};
AssinaturaCor verdeCalibradoEsq = {0, 0, 0, 0};

// Referências às variáveis globais em robo_linha.ino
extern EstadoRobo estadoAtual;
extern int tipoGiro;
extern float anguloInicial;

// ==============================================================================
// FUNÇÕES DOS SENSORES
// ==============================================================================

void initSensores() {
  qtr.setTypeRC();
  qtr.setSensorPins(PINOS_IR, NUM_SENSORES_IR);
  
  tcaselect(CANAL_TCS_DIR);
  if (!tcsDir.begin()) {
    Serial.println(F("ERRO: TCS Direito não encontrado!"));
  }
  
  tcaselect(CANAL_TCS_ESQ);
  if (!tcsEsq.begin()) {
    Serial.println(F("ERRO: TCS Esquerdo não encontrado!"));
  }
  
  tcaselect(CANAL_GY521);
  byte status = mpu.begin();
  if(status != 0) {
    Serial.println(F("ERRO: MPU6050 não encontrado!"));
  } else {
    Serial.println(F("MPU Encontrado. Pronto para calibracao de pista."));
  }
}

// ==============================================================================
// LÓGICA DE VALIDAÇÃO DE CORES
// ==============================================================================

bool ehVermelho(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  if (c < limiarC) return false;  
  if (r < 80) return false; 

  float margem = 1.35; 
  if (r > (g * margem) && r > (b * margem)) {
    return true; 
  }
  return false;
}

float calcularHue(float r, float g, float b) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  float delta = maxVal - minVal;
  if (delta == 0) return 0;
  
  float hue = 0;
  if (maxVal == r) {
    hue = 60.0 * ((g - b) / delta);
  } else if (maxVal == g) {
    hue = 60.0 * ((b - r) / delta + 2.0);
  } else if (maxVal == b) {
    hue = 60.0 * ((r - g) / delta + 4.0);
  }
  if (hue < 0) hue += 360.0;
  return hue;
}

float calcularSaturacao(float r, float g, float b) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  if (maxVal == 0) return 0;
  return (maxVal - minVal) / maxVal;
}

bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  // Filtro de Ruído: Se C < limiar_minimo, ignorar leitura (luz insuficiente/fora da pista).
  if (c < limiarC) return false;
  
  // Filtro absoluto de ruído do sensor (valores muito baixos são lixo)
  if (g < 50) return false;
  if (c > 60000) return false;
  
  float hue = calcularHue(r, g, b);
  float sat = calcularSaturacao(r, g, b);

  // Range Hue Verde: [95.0 - 165.0]
  bool hueValido = (hue >= 95.0 && hue <= 165.0);

  // CONDIÇÃO 1: Verde Inegável (Fita Escura centralizada no sensor)
  // Saturação Mínima: 0.35 (Fita central)
  bool verdeForte = (hueValido && sat >= 0.35 && g > (r * 1.25) && g > (b * 1.25));

  if (verdeForte) {
    return true;
  }

  // CONDIÇÃO 2: Verde de Borda (Metade fita, metade chão branco)
  // Saturação Mínima: 0.18 (Borda)
  bool verdeBorda = (hueValido && sat >= 0.18 && g > (r * 1.10) && g > (b * 1.10));

  if (verdeBorda) {
    return true;
  }

  return false;
}

void verificarCores() {
  if (millis() - ultimaLeituraCor < 10) return; // Tempo de loop ajustado para 10ms
  ultimaLeituraCor = millis();
  
  uint16_t rD, gD, bD, cD;
  uint16_t rE, gE, bE, cE;
  
  tcaselect(CANAL_TCS_DIR);
  tcsDir.getRawData(&rD, &gD, &bD, &cD);
  
  tcaselect(CANAL_TCS_ESQ);
  tcsEsq.getRawData(&rE, &gE, &bE, &cE);
  
  bool verdeDirLeitura = ehVerde(rD, gD, bD, cD, limiarLuminosidadeDir);
  bool vermelhoDirLeitura = ehVermelho(rD, gD, bD, cD, limiarLuminosidadeDir);
  
  bool verdeEsqLeitura = ehVerde(rE, gE, bE, cE, limiarLuminosidadeEsq);
  bool vermelhoEsqLeitura = ehVermelho(rE, gE, bE, cE, limiarLuminosidadeEsq);
  
  // ==========================================================================
  // FILTRO DE ESTABILIDADE (Debouncing / Votação)
  // ==========================================================================
  static uint8_t contadorVerdeDir = 0;
  static uint8_t contadorVerdeEsq = 0;
  static uint8_t contadorVermelhoDir = 0;
  static uint8_t contadorVermelhoEsq = 0;

  // Atualizando os contadores DIR
  if (verdeDirLeitura) contadorVerdeDir++; else contadorVerdeDir = 0;
  if (vermelhoDirLeitura) contadorVermelhoDir++; else contadorVermelhoDir = 0;
  
  // Atualizando os contadores ESQ
  if (verdeEsqLeitura) contadorVerdeEsq++; else contadorVerdeEsq = 0;
  if (vermelhoEsqLeitura) contadorVermelhoEsq++; else contadorVermelhoEsq = 0;

  // Validando as leituras (exige 3 leituras consecutivas)
  bool verdeDir = (contadorVerdeDir >= 3);
  bool verdeEsq = (contadorVerdeEsq >= 3);
  bool vermelhoDir = (contadorVermelhoDir >= 3);
  bool vermelhoEsq = (contadorVermelhoEsq >= 3);

  // ==========================================================================
  // DEBUGGER NO MONITOR SERIAL (A cada 300ms para não travar o Arduino)
  // ==========================================================================
  static unsigned long tempoUltimoPrint = 0;
  if (millis() - tempoUltimoPrint > 300) {
    tempoUltimoPrint = millis();
    
    Serial.println(F("\n--- [DEBUGGER TCS34725] ---"));
    // Infos do Sensor Direito
    Serial.print(F("DIR -> R:")); Serial.print(rD);
    Serial.print(F(" G:")); Serial.print(gD);
    Serial.print(F(" B:")); Serial.print(bD);
    Serial.print(F(" Hue:")); Serial.print(calcularHue(rD, gD, bD), 1);
    Serial.print(F(" Sat:")); Serial.print(calcularSaturacao(rD, gD, bD), 2);
    Serial.print(F(" | VERDE? ")); Serial.print(verdeDirLeitura ? F("[SIM Leitura]") : F("nao"));
    Serial.print(F(" -> Valido: ")); Serial.println(verdeDir ? F("SIM") : F("NAO"));
    
    // Infos do Sensor Esquerdo
    Serial.print(F("ESQ -> R:")); Serial.print(rE);
    Serial.print(F(" G:")); Serial.print(gE);
    Serial.print(F(" B:")); Serial.print(bE);
    Serial.print(F(" Hue:")); Serial.print(calcularHue(rE, gE, bE), 1);
    Serial.print(F(" Sat:")); Serial.print(calcularSaturacao(rE, gE, bE), 2);
    Serial.print(F(" | VERDE? ")); Serial.print(verdeEsqLeitura ? F("[SIM Leitura]") : F("nao"));
    Serial.print(F(" -> Valido: ")); Serial.println(verdeEsq ? F("SIM") : F("NAO"));
  }
  // ==========================================================================

  if (vermelhoDir || vermelhoEsq) {
    contadorVermelhoDir = 0;
    contadorVermelhoEsq = 0;
    pararMotores();
    estadoAtual = ESTADO_VERMELHO;
    atualizarStatus("COR", "VERMELHO");
    return;
  }
  
  bool detectouVerde = false;

  if (verdeDir && verdeEsq) {
    tipoGiro = 180;
    atualizarStatus("VERDE DUPLO", "Alinhando 180");
    detectouVerde = true;
  } else if (verdeDir) {
    tipoGiro = 90;
    atualizarStatus("VERDE DIR", "Alinhando 90");
    detectouVerde = true;
  } else if (verdeEsq) {
    tipoGiro = -90;
    atualizarStatus("VERDE ESQ", "Alinhando -90");
    detectouVerde = true;
  }

  if (detectouVerde) {
    contadorVerdeDir = 0;
    contadorVerdeEsq = 0;
    
    // Avança um pouco para alinhar o eixo de rotação do robô com a interseção antes de girar
    controlarRodas(130, 130); 
    delay(350); // Aumentado para o robô adentrar mais no cruzamento
    pararMotores();
    delay(50); 

    tcaselect(CANAL_GY521);
    mpu.update();
    anguloInicial = mpu.getAngleZ(); 
    
    estadoAtual = ESTADO_VERDE; 
  }
}

void executarCalibracao() {
  pararMotores(); 
  Serial.println(F("\n====== [CALIBRAÇÃO MANUAL EXPANDIDA] ======"));
  
  uint16_t maxCDir = 0;
  uint16_t maxCEsq = 0;
  
  // ==========================================================================
  // FASE 1: LINHA PRETA E FUNDO BRANCO (5 SEGUNDOS)
  // ==========================================================================
  unsigned long tempoInicio = millis();
  int segundosRestantes = 5;
  
  while (millis() - tempoInicio < 5000) {
    int tempoPassado = (millis() - tempoInicio) / 1000;
    if (5 - tempoPassado != segundosRestantes) {
      segundosRestantes = 5 - tempoPassado;
      String msgTempo = "Fundo/Linha: " + String(segundosRestantes) + "s";
      atualizarStatus("Fase 1/3", msgTempo.c_str());
    }
    
    qtr.calibrate();
    
    uint16_t r, g, b, c;
    tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&r, &g, &b, &c); if (c > maxCDir) maxCDir = c;
    tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&r, &g, &b, &c); if (c > maxCEsq) maxCEsq = c;
    delay(10);
  }

  // ==========================================================================
  // FASE 2: CALIBRAÇÃO COM GATILHO DE MAIOR GREEN DOMINANTE (5 SEGUNDOS)
  // ==========================================================================
  tempoInicio = millis();
  segundosRestantes = 5;
  Serial.println(F("[FASE 2] PASSE O SENSOR SOBRE O QUADRADO VERDE..."));
  
  while (millis() - tempoInicio < 5000) {
    int tempoPassado = (millis() - tempoInicio) / 1000;
    if (5 - tempoPassado != segundosRestantes) {
      segundosRestantes = 5 - tempoPassado;
      String msgTempo = "Passe no VERDE: " + String(segundosRestantes) + "s";
      atualizarStatus("Fase 2/3", msgTempo.c_str());
    }
    
    uint16_t rD, gD, bD, cD;
    uint16_t rE, gE, bE, cE;
    
    tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&rD, &gD, &bD, &cD);
    tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&rE, &gE, &bE, &cE);
    
    // GATILHO INTELIGENTE DIREITO:
    // Só atualiza se o 'Green' atual for maior que o recorde anterior E o 'Green' for maior que o Red e Blue (provando que não é o branco da pista)
    if (gD > verdeCalibradoDir.g && gD > rD && gD > bD) {
      verdeCalibradoDir.r = rD;
      verdeCalibradoDir.g = gD;
      verdeCalibradoDir.b = bD;
      verdeCalibradoDir.c = cD;
    }
    
    // GATILHO INTELIGENTE ESQUERDO:
    if (gE > verdeCalibradoEsq.g && gE > rE && gE > bE) {
      verdeCalibradoEsq.r = rE;
      verdeCalibradoEsq.g = gE;
      verdeCalibradoEsq.b = bE;
      verdeCalibradoEsq.c = cE;
    }
    delay(10);
  }
  
  Serial.println(F("\n--- MAPA DA ASSINATURA DO VERDE GRAVADA ---"));
  Serial.print(F("[DIR] R:")); Serial.print(verdeCalibradoDir.r); Serial.print(F(" G:")); Serial.print(verdeCalibradoDir.g); Serial.print(F(" B:")); Serial.println(verdeCalibradoDir.b);
  Serial.print(F("[ESQ] R:")); Serial.print(verdeCalibradoEsq.r); Serial.print(F(" G:")); Serial.print(verdeCalibradoEsq.g); Serial.print(F(" B:")); Serial.println(verdeCalibradoEsq.b);

  // ==========================================================================
  // FASE 3: POSICIONAMENTO NA LINHA (5 SEGUNDOS)
  // ==========================================================================
  tempoInicio = millis();
  segundosRestantes = 5;
  Serial.println(F("[FASE 3] COLOQUE O ROBÔ PARADO NA LINHA DE LARGADA..."));
  
  while (millis() - tempoInicio < 5000) {
    int tempoPassado = (millis() - tempoInicio) / 1000;
    if (5 - tempoPassado != segundosRestantes) {
      segundosRestantes = 5 - tempoPassado;
      String msgTempo = "Alinhe na pista: " + String(segundosRestantes) + "s";
      atualizarStatus("Fase 3/3", msgTempo.c_str());
    }
    delay(50);
  }

  // ==========================================================================
  // FASE 4: REGISTRO ESTÁTICO DO GIROSCÓPIO
  // ==========================================================================
  atualizarStatus("IMU", "Gravando Zeros...");
  tcaselect(CANAL_GY521);
  delay(50);
  
  mpu.calcOffsets(true, true);
  
  limiarLuminosidadeDir = maxCDir * 0.15; // Reduzido para não ignorar o verde escuro
  limiarLuminosidadeEsq = maxCEsq * 0.15;
  if (limiarLuminosidadeDir < 40) limiarLuminosidadeDir = 40; // Piso menor para fitas que refletem pouca luz
  if (limiarLuminosidadeEsq < 40) limiarLuminosidadeEsq = 40;
  
  // Casos de erro/segurança (se você não passar no verde na calibração por engano)
  if (verdeCalibradoDir.g == 0) { verdeCalibradoDir.r = 45; verdeCalibradoDir.g = 100; verdeCalibradoDir.b = 50; }
  if (verdeCalibradoEsq.g == 0) { verdeCalibradoEsq.r = 45; verdeCalibradoEsq.g = 100; verdeCalibradoEsq.b = 50; }
  
  Serial.println(F("====== [CALIBRAÇÃO CONCLUÍDA COM SUCESSO] ======\n"));
  atualizarStatus("CALIBRACAO", "PRONTO! CORRE");
  delay(1000); 
  
  estadoAtual = ESTADO_LINHA;
}

#endif // SENSORES_H