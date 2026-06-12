#ifndef SENSORES_H
#define SENSORES_H

#include <QTRSensors.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050_light.h>
#include <NewPing.h>
#include "config.h"

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

// Objetos Ultrassônicos
NewPing sonarFrente(PINO_TRIG_FRENTE, PINO_ECHO_FRENTE, MAX_DISTANCE);
NewPing sonarEsq(PINO_TRIG_ESQ, PINO_ECHO_ESQ, MAX_DISTANCE);
NewPing sonarDir(PINO_TRIG_DIR, PINO_ECHO_DIR, MAX_DISTANCE);

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
// LÓGICA DE VALIDAÇÃO DE ULTRASSOM
// ==============================================================================

int obterDistanciaFiltrada(NewPing &sonar) {
  int d = sonar.ping_cm();
  return (d == 0) ? MAX_DISTANCE : d;
}

// ==============================================================================
// FUNÇÃO AUXILIAR DO BOTÃO
// ==============================================================================
void esperarBotao() {
  // Espera o botão ser pressionado (HIGH -> LOW)
  while (digitalRead(PINO_BOTAO) == HIGH) {
    delay(10);
  }
  delay(50); // Debounce (filtro de ruído mecânico)
  
  // Espera o botão ser solto (LOW -> HIGH)
  while (digitalRead(PINO_BOTAO) == LOW) {
    delay(10);
  }
  delay(50); // Debounce
}

// ==============================================================================
// LÓGICA DE VALIDAÇÃO DE CORES
// ==============================================================================

bool ehVermelho(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  if (c < limiarC) return false;  
  if (r < 80) return false; 
  float margem = 1.35; 
  if (r > (g * margem) && r > (b * margem)) return true; 
  return false;
}

float calcularHue(float r, float g, float b) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  float delta = maxVal - minVal;
  if (delta == 0) return 0;
  
  float hue = 0;
  if (maxVal == r) hue = 60.0 * ((g - b) / delta);
  else if (maxVal == g) hue = 60.0 * ((b - r) / delta + 2.0);
  else if (maxVal == b) hue = 60.0 * ((r - g) / delta + 4.0);
  if (hue < 0) hue += 360.0;
  return hue;
}

float calcularSaturacao(float r, float g, float b) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  if (maxVal == 0) return 0;
  return (maxVal - minVal) / maxVal;
}

// ==============================================================================
// LÓGICA DE VALIDAÇÃO DE CORES SUPER RIGOROSA (Sem falso positivo)
// ==============================================================================
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  // Ignora escuro total ou sombras
  if (c < limiarC) return false;
  
  // Ignora lixo do sensor
  if (g < 50 || c > 60000) return false;
  
  // REGRA DE OURO: No verde real, o 'G' TEM que ser a cor dominante. 
  // Se R ou B forem maiores ou iguais ao G, não é verde (provavelmente é branco ou cinza)
  if (r >= g || b >= g) return false;
  
  // O Verde tem que ser pelo menos 15% mais forte que o vermelho e o azul
  if (g < (r * 1.15)) return false;
  if (g < (b * 1.15)) return false;

  float hue = calcularHue(r, g, b);
  float sat = calcularSaturacao(r, g, b);

  // Range de Verde restrito
  bool hueValido = (hue >= 90.0 && hue <= 170.0);
  
  // Aumentamos a saturação para 0.25 (O antigo 0.18 deixava o chão branco ser lido como verde)
  bool satValida = (sat >= 0.25); 

  if (hueValido && satValida) {
    return true;
  }
  
  return false;
}

// ==============================================================================
// NOVA LEITURA DE CRUZAMENTO (Super Rápida - Não quebra as curvas de 90º)
// ==============================================================================
bool avaliarInterseccao() {
  // Para o robô para a leitura não sair borrada
  pararMotores();
  delay(80); // Tempo super rápido apenas para o chassi parar de tremer
  
  uint16_t rD, gD, bD, cD;
  uint16_t rE, gE, bE, cE;
  
  tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&rD, &gD, &bD, &cD);
  tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&rE, &gE, &bE, &cE);
  
  bool verdeDir = ehVerde(rD, gD, bD, cD, limiarLuminosidadeDir);
  bool verdeEsq = ehVerde(rE, gE, bE, cE, limiarLuminosidadeEsq);
  
  // Se achou o verde, resolve a curva!
  if (verdeDir || verdeEsq) {
    if (verdeDir && verdeEsq) {
      tipoGiro = 180;
    } else if (verdeDir) {
      tipoGiro = 65;
    } else if (verdeEsq) {
      tipoGiro = -65;
    }
    
    // Achou o verde! Dá mais um passinho para alinhar o eixo das rodas com o cruzamento
    controlarRodas(100, 100); 
    delay(150); 
    pararMotores();
    delay(50); 

    tcaselect(CANAL_GY521);
    mpu.update();
    anguloInicial = mpu.getAngleZ(); 
    
    estadoAtual = ESTADO_VERDE; 
    return true; 
  }

  // Verifica Vermelho
  if (ehVermelho(rD, gD, bD, cD, limiarLuminosidadeDir) || ehVermelho(rE, gE, bE, cE, limiarLuminosidadeEsq)) {
    estadoAtual = ESTADO_VERMELHO;
    return true;
  }
  
  // SE CHEGOU AQUI: Era só uma curva preta grossa de 90 graus!
  // REMOVIDO A RÉ! Apenas dá um mini-pulo pra frente para o PID "engolir" a curva sem perder momento.
  controlarRodas(100, 100);
  delay(40);
  
  return false; // Retorna falso para o PID voltar a trabalhar imediatamente
}


// ==============================================================================
// NOVA CALIBRAÇÃO MANUAL VIA BOTÃO
// ==============================================================================
void executarCalibracao() {
  pararMotores(); 
  Serial.println(F("\n====== [CALIBRAÇÃO MANUAL COM BOTÃO] ======"));
  
  uint16_t maxCDir = 0;
  uint16_t maxCEsq = 0;
  
  // ---------------------------------------------------------
  // FASE 1: LINHA PRETA E FUNDO BRANCO
  // ---------------------------------------------------------
  Serial.println(F("[FASE 1] Mova a frente do robô sobre a linha e o fundo branco."));
  Serial.println(F("Quando terminar, APERTE O BOTÃO."));
  
  while (digitalRead(PINO_BOTAO) == HIGH) { // Enquanto não apertar...
    qtr.calibrate();
    
    uint16_t r, g, b, c;
    tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&r, &g, &b, &c); if (c > maxCDir) maxCDir = c;
    tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&r, &g, &b, &c); if (c > maxCEsq) maxCEsq = c;
    delay(10);
  }
  esperarBotao(); // Aguarda você soltar o botão e faz o filtro mecânico
  Serial.println(F("-> IR Calibrado!"));

  // ---------------------------------------------------------
  // FASE 2: GATILHO DO VERDE
  // ---------------------------------------------------------
  Serial.println(F("\n[FASE 2] Coloque os DOIS sensores RGB sobre a fita VERDE."));
  Serial.println(F("Mexa um pouquinho para ele pegar a cor, e APERTE O BOTÃO."));
  
  while (digitalRead(PINO_BOTAO) == HIGH) {
    uint16_t rD, gD, bD, cD;
    uint16_t rE, gE, bE, cE;
    
    tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&rD, &gD, &bD, &cD);
    tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&rE, &gE, &bE, &cE);
    
    if (gD > verdeCalibradoDir.g && gD > rD && gD > bD) {
      verdeCalibradoDir.r = rD; verdeCalibradoDir.g = gD; verdeCalibradoDir.b = bD; verdeCalibradoDir.c = cD;
    }
    if (gE > verdeCalibradoEsq.g && gE > rE && gE > bE) {
      verdeCalibradoEsq.r = rE; verdeCalibradoEsq.g = gE; verdeCalibradoEsq.b = bE; verdeCalibradoEsq.c = cE;
    }
    delay(10);
  }
  esperarBotao();
  
  Serial.println(F("--- MAPA DA ASSINATURA DO VERDE GRAVADA ---"));
  Serial.print(F("[DIR] R:")); Serial.print(verdeCalibradoDir.r); Serial.print(F(" G:")); Serial.print(verdeCalibradoDir.g); Serial.print(F(" B:")); Serial.println(verdeCalibradoDir.b);
  Serial.print(F("[ESQ] R:")); Serial.print(verdeCalibradoEsq.r); Serial.print(F(" G:")); Serial.print(verdeCalibradoEsq.g); Serial.print(F(" B:")); Serial.println(verdeCalibradoEsq.b);

  // ---------------------------------------------------------
  // FASE 4: POSICIONAMENTO FINAL
  // ---------------------------------------------------------
  Serial.println(F("\n[FASE 4] Posicione o robô na LARGADA."));
  Serial.println(F("Não toque no robô! APERTE O BOTÃO e afaste a mão para calibrar o Giroscópio."));
  esperarBotao();
  
  Serial.println(F("Calibrando MPU6050 (NAO MEXA)..."));
  tcaselect(CANAL_GY521);
  delay(100);
  mpu.calcOffsets(true, true);
  
  limiarLuminosidadeDir = maxCDir * 0.15; 
  limiarLuminosidadeEsq = maxCEsq * 0.15;
  if (limiarLuminosidadeDir < 40) limiarLuminosidadeDir = 40; 
  if (limiarLuminosidadeEsq < 40) limiarLuminosidadeEsq = 40;
  
  if (verdeCalibradoDir.g == 0) { verdeCalibradoDir.r = 45; verdeCalibradoDir.g = 100; verdeCalibradoDir.b = 50; }
  if (verdeCalibradoEsq.g == 0) { verdeCalibradoEsq.r = 45; verdeCalibradoEsq.g = 100; verdeCalibradoEsq.b = 50; }
  
  Serial.println(F("\n====== [CALIBRAÇÃO CONCLUÍDA! LARGANDO...] ======\n"));
  delay(1000); 
  
  estadoAtual = ESTADO_LINHA;
}
#endif // SENSORES_H