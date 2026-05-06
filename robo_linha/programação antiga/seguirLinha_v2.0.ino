#include <QTRSensors.h>
#include <Wire.h>
#include <SoftwareWire.h>
#include "Adafruit_TCS34725.h"
#include "AFMotor_R4.h"

// --- MOTORES ---
AF_DCMotor m_tras_dir(1); AF_DCMotor m_tras_esq(2);
AF_DCMotor m_frente_dir(3); AF_DCMotor m_frente_esq(4);

// --- SENSORES DE LINHA ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
QTRSensors qtr;

// --- SENSORES RGB ---
Adafruit_TCS34725 tcsEsq = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
SoftwareWire softWire(22, 23);

// --- MEMÓRIA DOS SENSORES RGB (Filtro de Falso Verde) ---
#define COR_BRANCO 0
#define COR_PRETO 1
int ultimoFundoEsq = COR_BRANCO;
int ultimoFundoDir = COR_BRANCO;

// --- PARÂMETROS LINHA ---
#define VELOCIDADE_BASE 150
float Kp = 0.12; 
int ultimoLado = 0; 

// --- CONFIGURAÇÃO DO GAP ---
#define VELOCIDADE_GAP 130
const int tempoPara12cm = 1250;
int contadorFalhas = 0; 

// --- PARÂMETROS CURVA VERDE ---
#define VEL_CURVA 140
#define TEMPO_ALINHAR_RODA 150 
#define TEMPO_CURVA_CEGA_90 300  
#define TEMPO_CURVA_CEGA_180 550 


void setup() {
  Serial.begin(115200); 
  
  if (!tcsEsq.begin()) {
    Serial.println("ERRO: Sensor RGB Esquerdo nao encontrado!");
  }
  softWire.begin();
  configuraSensorDireito(); 

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){24, 25, 26, 27, 28, 29, 30, 31}, SensorCount);
  qtr.setEmitterPin(255);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) { 
    qtr.calibrate(); 
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void loop() {
  // 1. LEITURA DOS SENSORES RGB
  uint16_t rE, gE, bE, cE;
  uint16_t rD, gD, bD, cD;
  
  tcsEsq.getRawData(&rE, &gE, &bE, &cE);
  lerTudoDireito(&rD, &gD, &bD, &cD);

  // --- NOVA CHECAGEM: VERMELHO PARA PARAR O ROBÔ ---
  bool vermelhoEsq = ehVermelho(rE, gE, bE, cE);
  bool vermelhoDir = ehVermelho(rD, gD, bD, cD);

  // Se qualquer um dos sensores ler vermelho, encerra a prova
  if (vermelhoEsq || vermelhoDir) {
    pararRoboParaSempre();
  }

  // --- CHECAGEM DO VERDE ---
  bool verdeEsq = ehVerde(rE, gE, bE, cE);
  bool verdeDir = ehVerde(rD, gD, bD, cD);

  // 2. ATUALIZA A MEMÓRIA DE FUNDO (Apenas se não estiver em cima do verde ou vermelho)
  if (!verdeEsq && !vermelhoEsq) {
    if (ehPreto(cE)) ultimoFundoEsq = COR_PRETO;
    else ultimoFundoEsq = COR_BRANCO;
  }
  
  if (!verdeDir && !vermelhoDir) {
    if (ehPreto(cD)) ultimoFundoDir = COR_PRETO;
    else ultimoFundoDir = COR_BRANCO;
  }

  // 3. VALIDAÇÃO INTELIGENTE (Ignorar falso verde após preto)
  bool verdeValidoEsq = verdeEsq && (ultimoFundoEsq == COR_BRANCO);
  bool verdeValidoDir = verdeDir && (ultimoFundoDir == COR_BRANCO);

  // 4. TOMA DECISÃO BASEADA NAS CORES VALIDADAS
  if (verdeValidoEsq && verdeValidoDir) {
    Serial.println("BECO SEM SAIDA (180 Graus)");
    fazerCurva(180);
    return;
  } 
  else if (verdeValidoDir) {
    Serial.println("VERDE DIREITA (90 Graus Dir)");
    fazerCurva(90);  
    return;
  } 
  else if (verdeValidoEsq) {
    Serial.println("VERDE ESQUERDA (90 Graus Esq)");
    fazerCurva(-90); 
    return;
  }

  // 5. SE NÃO TEM VERDE E NÃO É VERMELHO, SEGUE A LINHA NORMALMENTE
  uint16_t position = qtr.readLineBlack(sensorValues);
  bool vendoLinha = false;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 200) vendoLinha = true;
  }

  if (!vendoLinha) {
    insistirNaCurva();
  } else {
    contadorFalhas = 0; 
    int erro = position - 3500;
    if (erro > 500) ultimoLado = 1;       
    else if (erro < -500) ultimoLado = -1; 

    int ajuste = erro * Kp;
    controlarRodas(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste);
  }
}

// ========================================================
// RECONHECIMENTO DE CORES
// ========================================================
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

bool ehPreto(uint16_t c) {
  if (c < 90) return true;
  return false;
}

// ========================================================
// FUNÇÃO DE PARADA TOTAL (FIM DE PROVA)
// ========================================================
void pararRoboParaSempre() {
  Serial.println("!!! VERMELHO DETECTADO !!!");
  Serial.println("Encerrando execucao e parando motores...");
  
  // Zera todos os motores
  controlarRodas(0, 0);
  
  // Entra em um loop infinito (o robô "morre" aqui até o botão reset ser apertado)
  while (true) {
    // Pode colocar o LED do Arduino para piscar avisando que parou
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

// ========================================================
// LÓGICA DE CURVAS E RESTANTE DO CÓDIGO (Mantidos)
// ========================================================
void fazerCurva(int angulo) {
  controlarRodas(VELOCIDADE_BASE, VELOCIDADE_BASE);
  delay(TEMPO_ALINHAR_RODA); 

  if (angulo == 90) { // DIREITA
    controlarRodas(-VEL_CURVA, VEL_CURVA); 
    delay(TEMPO_CURVA_CEGA_90); 
    while(true) {
      controlarRodas(-VEL_CURVA, VEL_CURVA);
      qtr.readLineBlack(sensorValues);
      if (sensorValues[3] > 400 || sensorValues[4] > 400) break;
    }
  } 
  else if (angulo == -90) { // ESQUERDA
    controlarRodas(VEL_CURVA, -VEL_CURVA); 
    delay(TEMPO_CURVA_CEGA_90); 
    while(true) {
      controlarRodas(VEL_CURVA, -VEL_CURVA);
      qtr.readLineBlack(sensorValues);
      if (sensorValues[3] > 400 || sensorValues[4] > 400) break;
    }
  }
  else if (angulo == 180) { // MEIA VOLTA
    controlarRodas(-VEL_CURVA, VEL_CURVA); 
    delay(TEMPO_CURVA_CEGA_180); 
    while(true) {
      controlarRodas(-VEL_CURVA, VEL_CURVA);
      qtr.readLineBlack(sensorValues);
      if (sensorValues[3] > 400 || sensorValues[4] > 400) break;
    }
  }
  
  controlarRodas(0, 0);
  delay(50);

  ultimoFundoEsq = COR_BRANCO;
  ultimoFundoDir = COR_BRANCO;
}

void configuraSensorDireito() {
  softWrite8(0x00, 0x01); 
  delay(3);
  softWrite8(0x00, 0x01 | 0x02); 
  softWrite8(0x01, 0xF6); 
  softWrite8(0x0F, 0x01); 
}

void softWrite8(uint8_t reg, uint8_t value) {
  softWire.beginTransmission(0x29);
  softWire.write(0x80 | reg);
  softWire.write(value);
  softWire.endTransmission();
}

void lerTudoDireito(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
  softWire.beginTransmission(0x29);
  softWire.write(0x80 | 0x14); 
  softWire.endTransmission();

  softWire.requestFrom(0x29, 8);
  *c = softWire.read() | (softWire.read() << 8);
  *r = softWire.read() | (softWire.read() << 8);
  *g = softWire.read() | (softWire.read() << 8);
  *b = softWire.read() | (softWire.read() << 8);
}

void insistirNaCurva() {
  unsigned long start = millis();
  while (millis() - start < 150) { 
    if (ultimoLado == 1) controlarRodas(180, -100); 
    else controlarRodas(-100, 180);                 

    qtr.readLineBlack(sensorValues);
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 400) return; 
    }
  }
  tratarGap();
}

void tratarGap() {
  contadorFalhas++;

  if (contadorFalhas >= 3) {
    unsigned long tempoRe = millis();
    while (millis() - tempoRe < 2000) { controlarRodas(-100, -100); }
    contadorFalhas = 0;
    return;
  }

  unsigned long tempoInicio = millis();
  bool encontrou = false;

  while (millis() - tempoInicio < tempoPara12cm) {
    controlarRodas(VELOCIDADE_GAP, VELOCIDADE_GAP);
    qtr.readLineBlack(sensorValues);
    if (sensorValues[3] > 400 || sensorValues[4] > 400) {
      encontrou = true;
      break;
    }
  }

  if (!encontrou) {
    while (true) {
      controlarRodas(-VELOCIDADE_GAP, -VELOCIDADE_GAP);
      qtr.readLineBlack(sensorValues);
      if (sensorValues[3] > 400 || sensorValues[4] > 400) break;
    }
  }
}

void controlarRodas(int vDir, int vEsq) {
  if (vDir >= 0) {
    int v = constrain(vDir, 0, 255);
    m_frente_dir.setSpeed(v); m_tras_dir.setSpeed(v);
    m_frente_dir.run(FORWARD); m_tras_dir.run(FORWARD);
  } else {
    int v = constrain(abs(vDir), 0, 255);
    m_frente_dir.setSpeed(v); m_tras_dir.setSpeed(v);
    m_frente_dir.run(BACKWARD); m_tras_dir.run(BACKWARD);
  }
  if (vEsq >= 0) {
    int v = constrain(vEsq, 0, 255);
    m_frente_esq.setSpeed(v); m_tras_esq.setSpeed(v);
    m_frente_esq.run(FORWARD); m_tras_esq.run(FORWARD);
  } else {
    int v = constrain(abs(vEsq), 0, 255);
    m_frente_esq.setSpeed(v); m_tras_esq.setSpeed(v);
    m_frente_esq.run(BACKWARD); m_tras_esq.run(BACKWARD);
  }
}