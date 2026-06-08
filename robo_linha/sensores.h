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

AssinaturaCor cinzaCalibradoDir = {0, 0, 0, 0};
AssinaturaCor cinzaCalibradoEsq = {0, 0, 0, 0};

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
// VALIDAÇÃO CROMÁTICA DO CINZA (SILVER TAPE) VIA TCS34725
// ==============================================================================
// Estratégia em DUAS CAMADAS:
//   CAMADA 1 (Física): A silver tape é altamente reflexiva → 'c' será ALTO.
//                      Se c for baixo, não pode ser prata. Rejeita imediatamente.
//   CAMADA 2 (Cromática): Compara as proporções R/G/B normalizadas com a calibração.
//                         O cinza é neutro: R≈G≈B em proporções (~33% cada).
//
// CORREÇÕES DO BUG ORIGINAL:
//   - REMOVIDO: 'if (calib.c == 0) return true' → era um fail-safe perigoso.
//     Se o usuário não calibrou, o sistema não deve aceitar qualquer coisa como cinza.
//   - AJUSTADO: limiar de c de 80 para 200 (ignora ruído real do sensor em escuro).
//   - ADICIONADO: critério de luminosidade positiva (c > 500) como reforço para
//     confirmar superfície brilhante, característica física da silver tape.
bool ehCinzaRGB(uint16_t r, uint16_t g, uint16_t b, uint16_t c, AssinaturaCor &calib) {

  // --- CAMADA 1: Verificação Física de Reflexividade ---
  // Luminosidade mínima: evita ler sombras, buracos ou superfícies escuras
  if (c < 200) return false;

  // A silver tape metálica é MUITO reflexiva. Se c for baixo demais, não é prata.
  // Este critério funciona INDEPENDENTE da calibração cromática.
  bool altaReflexividade = (c > 500);

  // --- CAMADA 2: Verificação Cromática (só se calibração foi feita) ---
  // Se não foi calibrado (c_calib == 0), usa apenas a reflexividade como critério
  if (calib.c == 0) {
    // Sem calibração: o cinza neutro deve ter proporções R≈G≈B (tolerância 8%)
    uint32_t somaAtual = r + g + b;
    if (somaAtual == 0) return false;
    float rN = (float)r / somaAtual;
    float gN = (float)g / somaAtual;
    float bN = (float)b / somaAtual;
    // Um cinza neutro perfeito seria 0.333 cada. Tolerância de 10% para prata.
    bool cinzaNeutro = (abs(rN - 0.333f) < 0.10f &&
                        abs(gN - 0.333f) < 0.10f &&
                        abs(bN - 0.333f) < 0.10f);
    return (altaReflexividade && cinzaNeutro);
  }

  // Com calibração: compara proporções cromáticas com a assinatura gravada
  uint32_t somaCalib = calib.r + calib.g + calib.b;
  if (somaCalib == 0) return altaReflexividade; // Calibração inválida: usa só reflexividade

  uint32_t somaAtual = r + g + b;
  if (somaAtual == 0) return false;

  float rCalib = (float)calib.r / somaCalib;
  float gCalib = (float)calib.g / somaCalib;
  float bCalib = (float)calib.b / somaCalib;

  float rAtual = (float)r / somaAtual;
  float gAtual = (float)g / somaAtual;
  float bAtual = (float)b / somaAtual;

  // Tolerância cromática aumentada de 12% para 15%:
  // A prata metálica pode variar bastante com ângulo e luz ambiente
  float tolerancia = 0.15f;

  bool cromaticaOk = (abs(rAtual - rCalib) < tolerancia &&
                      abs(gAtual - gCalib) < tolerancia &&
                      abs(bAtual - bCalib) < tolerancia);

  // Aceita se AMBOS os critérios concordam (mais seguro)
  // OU se a reflexividade for altíssima E a cromática estiver perto
  return (altaReflexividade && cromaticaOk);
}


// ==============================================================================
// DETECÇÃO DE SILVER TAPE (ENTRADA DO RESGATE)
// ==============================================================================
// A fita cinza/prata cobre a barra de sensores inteira na entrada da zona de resgate.
// O sensor QTR-RC retorna valores de 0 a ~2500 µs:
//   ~0–150   → branco (muito reflexivo)
//   ~200–900 → CINZA / PRATA (faixa alvo)
//   ~1000+   → preto (pouco reflexivo)
//
// DIAGNÓSTICO DO BUG ORIGINAL:
// A amplitude < 180 era o principal culpado. A silver tape tem reflexividade
// variável dependendo do ângulo de incidência da luz e da posição do sensor,
// então a amplitude real entre sensores pode facilmente superar 300.
// Além disso, a janela 280–750 excluía leituras válidas abaixo de 280.
bool detectouSilverTape(uint16_t *valores) {
  uint16_t menorValor = 3000; // Inicializa com valor máximo possível
  uint16_t maiorValor = 0;
  uint32_t somaValores = 0;
  uint8_t sensoresNaFaixaCinza = 0; // Quantos sensores leram na faixa da prata

  for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
    if (valores[i] < menorValor) menorValor = valores[i];
    if (valores[i] > maiorValor) maiorValor = valores[i];
    somaValores += valores[i];

    // Conta quantos sensores individuais estão na faixa do cinza/prata
    if (valores[i] >= 150 && valores[i] <= 1000) {
      sensoresNaFaixaCinza++;
    }
  }

  uint16_t mediaValores = somaValores / NUM_SENSORES_IR;
  uint16_t amplitude    = maiorValor - menorValor;

  // Critério de detecção DUPLO (mais robusto):
  // 1. A MÉDIA da barra precisa estar na faixa cinza/prata (200 a 900)
  // 2. A AMPLITUDE entre sensores não pode ser altíssima (< 350 cobre variações reais)
  //    - Isso ainda exclui leituras com metade preta + metade branca (amplitude > 600)
  // 3. A MAIORIA dos sensores precisa estar individualmente na faixa (>= 5 de 8)
  bool mediaNaFaixa       = (mediaValores >= 200 && mediaValores <= 900);
  bool amplitudeTolerada  = (amplitude < 350);
  bool maioriaNaFaixa     = (sensoresNaFaixaCinza >= 5);

  if (mediaNaFaixa && amplitudeTolerada && maioriaNaFaixa) {
    return true;
  }
  return false;
}


// ==============================================================================
// NOVA LEITURA DE COR (Varredura Contínua e Retorno de Segurança)
// ==============================================================================
bool avaliarInterseccao() {
  unsigned long tempoInicio = millis();
  
  int votosVerdeDir = 0;
  int votosVerdeEsq = 0;
  int votosVermelho = 0;
  
  bool achouVerde = false;
  bool achouVermelho = false;
  
  // 1. Avança devagar para "varrer" o chão à frente
  controlarRodas(80, 80);
  
  // 2. Fica filmando o chão por 250ms (Tempo suficiente pro sensor passar sobre toda a fita)
  while (millis() - tempoInicio < 250) { 
    uint16_t rD, gD, bD, cD;
    uint16_t rE, gE, bE, cE;
    
    tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&rD, &gD, &bD, &cD);
    tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&rE, &gE, &bE, &cE);
    
    // Verifica Lado Direito
    if (ehVerde(rD, gD, bD, cD, limiarLuminosidadeDir)) votosVerdeDir++;
    else votosVerdeDir = 0; // Se piscar outra cor, zera. Exige leitura CONSECUTIVA!
    
    // Verifica Lado Esquerdo
    if (ehVerde(rE, gE, bE, cE, limiarLuminosidadeEsq)) votosVerdeEsq++;
    else votosVerdeEsq = 0;
    
    // Verifica Vermelho
    if (ehVermelho(rD, gD, bD, cD, limiarLuminosidadeDir) || ehVermelho(rE, gE, bE, cE, limiarLuminosidadeEsq)) votosVermelho++;
    else votosVermelho = 0;
    
    // Se achou a cor 2 VEZES SEGUIDAS, confirmamos a detecção imediatamente!
    if (votosVerdeDir >= 2 || votosVerdeEsq >= 2) {
      achouVerde = true;
      break; 
    }
    if (votosVermelho >= 2) {
      achouVermelho = true;
      break;
    }
  }
  
  pararMotores(); // Acabou a varredura
  
  // --- DECISÕES DA VARREDURA ---
  
  if (achouVermelho) {
    estadoAtual = ESTADO_VERMELHO;
    return true;
  }
  
  if (achouVerde) {
    if (votosVerdeDir >= 2 && votosVerdeEsq >= 2) {
      tipoGiro = 180;
    } else if (votosVerdeDir >= 2) {
      tipoGiro = 70;
    } else if (votosVerdeEsq >= 2) {
      tipoGiro = -70;
    }
    
    // Achou o verde! Dá mais um passinho para alinhar o eixo das rodas com o cruzamento
    controlarRodas(100, 100); 
    delay(100); 
    pararMotores();
    delay(50); 

    tcaselect(CANAL_GY521);
    mpu.update();
    anguloInicial = mpu.getAngleZ(); 
    
    estadoAtual = ESTADO_VERDE; 
    return true; 
  }

  // ===== O MOONWALK (RÉ DE SEGURANÇA) =====
  // Se chegou aqui, a varredura durou os 250ms completos e não viu NADA verde.
  // Isso significa que era apenas uma curva de 90 graus preta!
  // Como andamos pra frente e perdemos a quina, damos ré pela mesma quantidade de tempo
  // para devolver os sensores infravermelhos exatamente em cima da curva!
  controlarRodas(-80, -80);
  delay(250); 
  pararMotores();
  
  // Retorna falso para a FSM do loop voltar a caçar a linha com o PID (que agora verá a curva de 90!)
  return false; 
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
  // FASE 3: GATILHO DA FITA CINZA (SILVER TAPE)
  // ---------------------------------------------------------
  Serial.println(F("\n[FASE 3] Coloque os DOIS sensores RGB sobre a fita CINZA (Silver Tape)."));
  Serial.println(F("Mexa um pouquinho para ele pegar a cor, e APERTE O BOTÃO."));
  
  while (digitalRead(PINO_BOTAO) == HIGH) {
    uint16_t rD, gD, bD, cD;
    uint16_t rE, gE, bE, cE;
    
    tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&rD, &gD, &bD, &cD);
    tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&rE, &gE, &bE, &cE);
    
    // Filtra ruído (escuro) e captura o cinza
    if (cD > 80 && cE > 80) {
      cinzaCalibradoDir.r = rD; cinzaCalibradoDir.g = gD; cinzaCalibradoDir.b = bD; cinzaCalibradoDir.c = cD;
      cinzaCalibradoEsq.r = rE; cinzaCalibradoEsq.g = gE; cinzaCalibradoEsq.b = bE; cinzaCalibradoEsq.c = cE;
    }
    delay(10);
  }
  esperarBotao();
  
  Serial.println(F("--- MAPA DA ASSINATURA DO CINZA GRAVADA ---"));
  Serial.print(F("[DIR] R:")); Serial.print(cinzaCalibradoDir.r); Serial.print(F(" G:")); Serial.print(cinzaCalibradoDir.g); Serial.print(F(" B:")); Serial.println(cinzaCalibradoDir.b);
  Serial.print(F("[ESQ] R:")); Serial.print(cinzaCalibradoEsq.r); Serial.print(F(" G:")); Serial.print(cinzaCalibradoEsq.g); Serial.print(F(" B:")); Serial.println(cinzaCalibradoEsq.b);

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