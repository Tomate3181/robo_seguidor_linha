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
// INTEGRATIONTIME_24MS + GAIN_4X: configuração calibrada para sensor a ~3mm do chão.
// Com esses parâmetros os valores medidos foram:
//   Preto:       ESQ C~282  | DIR C~211
//   Silver Tape: ESQ C~1297 | DIR C~829
//   Verde:       ESQ C~721  | DIR C~501
//   Vermelho:    ESQ C~715  | DIR C~487
//   Branco:      ESQ C~4118 | DIR C~2605
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

// ==============================================================================
// LIMIAR MÍNIMO DE LUMINOSIDADE PARA LEITURAS DE COR (Verde / Vermelho)
// ==============================================================================
// Com GAIN_4X a 3mm:
//   Preto:    ESQ C~282, DIR C~211  → abaixo do limiar → ignora cor
//   Verde:    ESQ C~721, DIR C~501  → acima do limiar → processa cor
//   Vermelho: ESQ C~715, DIR C~487  → acima do limiar → processa cor
// Limiar definido a ~60% do Clear mínimo das cores úteis:
//   DIR: 487 * 0.6 ≈ 292 → arredondado para 300 (acima do preto DIR ~211)
//   ESQ: 715 * 0.6 ≈ 429 → arredondado para 400 (acima do preto ESQ ~282)
// ==============================================================================
uint16_t limiarLuminosidadeDir = 300;
uint16_t limiarLuminosidadeEsq = 400;

// ==============================================================================
// CALIBRAÇÃO DE LUMINOSIDADE DA SILVER TAPE (canal Clear do TCS34725)
// ==============================================================================
// Valores-padrão medidos com sensor a ~3mm do chão, GAIN_4X, 24ms:
//   Preto:        ESQ C~282,  DIR C~211
//   Silver Tape:  ESQ C~1297, DIR C~829   ← referência desta calibração
//   Verde:        ESQ C~721,  DIR C~501
//   Vermelho:     ESQ C~715,  DIR C~487
//   Branco:       ESQ C~4118, DIR C~2605
//
// A prata está ~4.6x acima do preto e ~3.2x abaixo do branco.
// Sobrescritos em executarCalibracao() Fase 3 ao vivo.
// ==============================================================================
int lumCinzaEsqCalibrado = 1297; // Clear do sensor ESQ sobre a silver tape
int lumCinzaDirCalibrado = 829;  // Clear do sensor DIR sobre a silver tape

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
// DETECÇÃO DE VERMELHO
// ==============================================================================
// Dados medidos (GAIN_4X, sensor a 3mm):
//   Vermelho: ESQ R=462 G=170 B=130  |  DIR R=326 G=111 B=92
//   Razão R/G: ESQ=2.72, DIR=2.94  →  threshold conservador: R > G * 1.8
//   Razão R/B: ESQ=3.55, DIR=3.54  →  threshold conservador: R > B * 2.0
//
// Preto (C~282) e branco (C~4118): Clear > limiarC filtra ambos.
// Verde: G domina → R < G → rejeita automaticamente pela condição R > G*1.8.
// ==============================================================================
bool ehVermelho(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  // Rejeita escuro (preto, sombra)
  if (c < limiarC) return false;

  // R deve dominar com boa margem sobre G e B
  // Threshold 1.8x (conservador vs 2.72x medido) → cobre variações de iluminação
  if (r > (g * 1.6f) && r > (b * 1.6f)) return true;

  return false;
}

// ==============================================================================
// CÁLCULO DE HUE (Matiz) — usado por ehVerde()
// Converte RGB normalizado para ângulo de matiz [0°, 360°]
// ==============================================================================
float calcularHue(float r, float g, float b) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  float delta  = maxVal - minVal;
  if (delta == 0) return 0;

  float hue = 0;
  if      (maxVal == r) hue = 60.0f * ((g - b) / delta);
  else if (maxVal == g) hue = 60.0f * ((b - r) / delta + 2.0f);
  else                  hue = 60.0f * ((r - g) / delta + 4.0f);
  if (hue < 0) hue += 360.0f;
  return hue;
}

float calcularSaturacao(float r, float g, float b) {
  float maxVal = max(r, max(g, b));
  float minVal = min(r, min(g, b));
  if (maxVal == 0) return 0;
  return (maxVal - minVal) / maxVal;
}

// ==============================================================================
// DETECÇÃO DE VERDE
// ==============================================================================
// Dados medidos (GAIN_4X, sensor a 3mm):
//   Verde: ESQ R=151 G=426 B=168  |  DIR R=124 G=279 B=124
//   Razão G/R: ESQ=2.82, DIR=2.25  →  threshold conservador: G > R * 1.8
//   Razão G/B: ESQ=2.54, DIR=2.25  →  threshold conservador: G > B * 1.8
//
// Cinza/Silver: ESQ R=421 G=517 B=356  →  G/R=1.23, G/B=1.45
//   → Não atinge G > R*1.8 → rejeita corretamente
//
// Branco: ESQ R=1297 G=1642 B=1059  →  G/R=1.27, G/B=1.55
//   → Não atinge G > R*1.8 → rejeita corretamente
//
// Preto: Clear ~282 < limiarC → rejeita antes de calcular razões
//
// Hue do verde medido: ~118° (ESQ) e ~120° (DIR) → faixa [95°, 155°] cobre com margem
// Saturação: ESQ=0.65, DIR=0.56 → threshold 0.4 rejeita brancos/cinzas (sat~0.2)
// ==============================================================================
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t limiarC) {
  // Rejeita escuro (preto, sombra)
  if (c < limiarC) return false;

  // Rejeita saturação de sensor (branco com muita luz, C acima de 65000)
  if (c > 60000) return false;

  // REGRA PRIMÁRIA: G deve dominar com margem clara sobre R e B
  // Threshold 1.8x (conservador vs 2.25x mínimo medido)
  // Isso rejeita cinza (G/R~1.23) e branco (G/R~1.27) automaticamente
  if (g <= (r * 1.6f)) return false;
  if (g <= (b * 1.6f)) return false;

  // Confirmação por Hue: verde real fica entre 95° e 155°
  float hue = calcularHue((float)r, (float)g, (float)b);
  if (hue < 95.0f || hue > 155.0f) return false;

  // Confirmação por saturação: verde tem saturação alta, cinza/branco têm baixa
  // Verde medido: sat~0.56-0.65 | Cinza/Branco: sat~0.19-0.22
  float sat = calcularSaturacao((float)r, (float)g, (float)b);
  if (sat < 0.40f) return false;

  return true;
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
  // FASE 3: SILVER TAPE — Calibração da Luminosidade (Clear)
  // ---------------------------------------------------------
  // Coloque AMBOS os sensores RGB diretamente sobre a fita prata.
  // O maior valor de Clear registrado durante a janela será salvo como
  // referência. A tolerância (±TOLERANCIA_CINZA) é aplicada em runtime.
  Serial.println(F("\n[FASE 3] Coloque os DOIS sensores RGB sobre a FITA PRATA (Silver Tape)."));
  Serial.println(F("Mexa levemente para cobrir variações de superficie. APERTE O BOTÃO."));

  uint16_t melhorClearEsq = 0;
  uint16_t melhorClearDir = 0;

  while (digitalRead(PINO_BOTAO) == HIGH) {
    uint16_t r, g, b, c;

    tcaselect(CANAL_TCS_DIR);
    tcsDir.getRawData(&r, &g, &b, &c);
    if (c > melhorClearDir) melhorClearDir = c;

    tcaselect(CANAL_TCS_ESQ);
    tcsEsq.getRawData(&r, &g, &b, &c);
    if (c > melhorClearEsq) melhorClearEsq = c;

    delay(10);
  }
  esperarBotao();

  // Só sobrescreve se capturou algo razoável (> 50 counts = não estava tapado)
  if (melhorClearDir > 50) {
    lumCinzaDirCalibrado = (int)melhorClearDir;
  }
  if (melhorClearEsq > 50) {
    lumCinzaEsqCalibrado = (int)melhorClearEsq;
  }

  Serial.println(F("--- CALIBRAÇÃO SILVER TAPE GRAVADA ---"));
  Serial.print(F("[DIR] Clear calibrado: ")); Serial.println(lumCinzaDirCalibrado);
  Serial.print(F("[ESQ] Clear calibrado: ")); Serial.println(lumCinzaEsqCalibrado);

  // Imprime a última leitura de R/G/B para ajuste do LIMIAR_ACROMIA_SILVER
  {
    uint16_t r, g, b, c;
    tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&r, &g, &b, &c);
    Serial.print(F("[DIR] R:")); Serial.print(r); Serial.print(F(" G:")); Serial.print(g);
    Serial.print(F(" B:")); Serial.print(b); Serial.print(F(" -> acromia(max-min)="));
    Serial.println(max(r, max(g, b)) - min(r, min(g, b)));
    tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&r, &g, &b, &c);
    Serial.print(F("[ESQ] R:")); Serial.print(r); Serial.print(F(" G:")); Serial.print(g);
    Serial.print(F(" B:")); Serial.print(b); Serial.print(F(" -> acromia(max-min)="));
    Serial.println(max(r, max(g, b)) - min(r, min(g, b)));
  }

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
  // Com GAIN_4X a 3mm, maxCDir (branco) ≈ 2605, maxCEsq ≈ 4118.
  // 15% disso: DIR ≈ 390, ESQ ≈ 617 — acima do preto (DIR~211, ESQ~282)
  // e abaixo do verde (DIR~501, ESQ~721). Garante piso mínimo de 300/400.
  if (limiarLuminosidadeDir < 300) limiarLuminosidadeDir = 300;
  if (limiarLuminosidadeEsq < 400) limiarLuminosidadeEsq = 400;
  
  if (verdeCalibradoDir.g == 0) { verdeCalibradoDir.r = 45; verdeCalibradoDir.g = 100; verdeCalibradoDir.b = 50; }
  if (verdeCalibradoEsq.g == 0) { verdeCalibradoEsq.r = 45; verdeCalibradoEsq.g = 100; verdeCalibradoEsq.b = 50; }
  
  Serial.println(F("\n====== [CALIBRAÇÃO CONCLUÍDA! LARGANDO...] ======\n"));
  delay(1000); 
  
  estadoAtual = ESTADO_LINHA;
}
#endif // SENSORES_H