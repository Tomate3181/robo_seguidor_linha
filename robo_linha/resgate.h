#ifndef RESGATE_H
#define RESGATE_H

/*
 * ==============================================================================
 * MÓDULO: resgate.h
 * DESCRIÇÃO: Gerencia toda a lógica de detecção, validação e navegação
 *            dentro da Zona de Resgate da OBR.
 *
 * FLUXO DE ESTADOS:
 *   ESTADO_LINHA
 *     └─► (≥5 sensores IR cravados em 1000) ──► ESTADO_VALIDANDO_SILVER_TAPE
 *           │
 *           ├─► (verde detectado durante 250ms) ──► ESTADO_VERDE / ESTADO_LINHA
 *           ├─► (IR caiu após 250ms) ──────────────► ESTADO_LINHA (falso positivo)
 *           └─► (todos os 8 cravados após 250ms) ──► ESTADO_ZONA_RESGATE
 *                 │
 *                 ├─► (linha preta nos centrais = cruzamento em cruz) ──► ESTADO_LINHA
 *                 └─► (branco total = resgate confirmado)
 *                       └─► FSM interna: AVANCANDO → VARRENDO → DEPOSITANDO → SAINDO
 *
 * REGRAS DE OURO:
 *   - Proibido delay(). Toda temporização usa millis().
 *   - Não redeclara pinos nem objetos de hardware.
 *   - Reutiliza controlarRodas(), pararMotores(), qtr, sonarFrente,
 *     sonarEsq, sonarDir, tcsDir, tcsEsq e mpu de motores.h / sensores.h.
 * ==============================================================================
 */

#include <Arduino.h>
#include "config.h"

// Inclui sensores.h antes (traz qtr, mpu, sonarFrente, sonarEsq, sonarDir,
// tcsDir, tcsEsq, obterDistanciaFiltrada, ehVerde, limiarLuminosidade*)
#include "sensores.h"

// Referências externas (declaradas em robo_linha.ino)
extern EstadoRobo estadoAtual;
extern ModoLinha  modoLinha;
extern int        ultimoErro;
extern int        contadorFalhas;
extern void       tcaselect(uint8_t i);

// Funções externas de controle de motores (declaradas em motores.h)
extern void controlarRodas(int velDir, int velEsq);
extern void pararMotores();

// ==============================================================================
// CONSTANTES CONFIGURÁVEIS — ajuste conforme testes em pista
// ==============================================================================

// --- Validação da Silver Tape ---
const int           VELOCIDADE_TESTE_SILVER    = 110;  // PWM durante o avanço de validação (fase IR)
const unsigned long TEMPO_VALIDACAO_SILVER      = 250; // Janela de confirmação IR em ms (fase 1)

// --- Posicionamento RGB sobre a fita ---
// Após a barra IR cravar todos os 8 sensores, o robô precisa avançar
// TEMPO_POSICIONA_RGB ms a velocidade reduzida para que os sensores TCS34725
// (montados levemente atrás da barra IR) fiquem fisicamente sobre a fita prata.
// Este valor já está declarado em config.h como: const unsigned long TEMPO_POSICIONA_RGB = 200;

const int           VELOCIDADE_POSICIONA_RGB   = 80;   // PWM reduzido durante o posicionamento

// --- Fusão de Sensores: janela relativa ao Clear calibrado ---
// Detalhada na FASE_FUSAO_SENSORES. Constantes FATOR_MIN_SILVER e FATOR_MAX_SILVER
// estão definidas em config.h com base nos valores medidos (GAIN_4X, sensor a 3mm).

// --- Filtro do Cruzamento em Cruz ---
const int           VELOCIDADE_FILTRO_CRUZ     = 100;  // PWM do avanço de confirmação
const unsigned long TEMPO_FILTRO_CRUZ_MS       = 200;  // Duração do avanço (ms)
const int           LIMIAR_IR_CRUZ             = 700;  // Valor mínimo para considerar "linha preta"
const int           SENSORES_CENTRAIS_CRUZ     = 2;    // Qtd. mínima de sensores centrais vendo preto

// --- Navegação na Sala de Resgate (Ultrassom) ---
const float         DIST_PAREDE_FRENTE_CM      = 15.0; // Distância de frenagem/parada frontal (cm)
const float         DIST_PAREDE_LATERAL_CM     = 18.0; // Distância alvo de seguimento lateral (cm)
const float         DIST_LATERAL_TOLERANCIA_RES = 4.0; // Banda morta lateral (cm)
const int           VEL_RESGATE_BASE           = 100;  // Velocidade base dentro da sala
const int           VEL_RESGATE_GIRO           = 90;   // Velocidade do giro de 90° no resgate
const unsigned long TIMEOUT_GIRO_RESGATE_MS    = 3000; // Timeout de segurança do giro (ms)
const unsigned long TIMEOUT_SONAR_RESGATE_MS   = 60;   // Intervalo entre leituras do sonar (ms)

// ==============================================================================
// ENUMERAÇÃO DA FSM INTERNA DO RESGATE
// ==============================================================================
enum EstadoResgate {
  RES_AGUARDANDO_INICIO,    // Estado neutro / aguardando entrada na zona
  RES_FILTRO_CRUZ,          // Avanço pós-silver para detectar cruzamento em +
  RES_NAVEGANDO,            // Navegação autônoma por ultrassom
  RES_GIRANDO_DIREITA,      // Giro de 90° para a direita (parede frontal)
  RES_GIRANDO_ESQUERDA,     // Giro de 90° para a esquerda (parede frontal)
  RES_CONCLUIDO             // Resgate encerrado (reservado para expansão futura)
};

// ==============================================================================
// ENUMERAÇÃO DA MÁQUINA DE ABORTO POR VERDE (sub-FSM de VALIDANDO_SILVER_TAPE)
// ==============================================================================
// Quando verde é detectado durante a validação, o robô precisa recuar para
// reposicionar os sensores TCS sobre o ladrilho antes de disparar o giro.
// Esta enum controla as fases dessa manobra de forma 100% não-bloqueante.
enum EstadoAbortoPorVerde {
  ABORTO_INATIVO,        // Nenhum aborto em andamento
  ABORTO_RECUANDO,       // Fase 1: ré para reposicionar sobre o ladrilho verde
  ABORTO_LENDO_COR,      // Fase 2: parado, aguarda estabilização e lê os TCS
  ABORTO_PREPARANDO_GIRO // Fase 3: avança mini-passo e captura Yaw → ESTADO_VERDE
};

// ==============================================================================
// ENUMERAÇÃO DAS FASES INTERNAS DA VALIDAÇÃO DA SILVER TAPE
// ==============================================================================
//
// A validação ocorre em 3 fases sequenciais não-bloqueantes:
//
//  FASE_IR_AVANCANDO  ──────────────────────────────────────────────────────
//    O robô avança a VELOCIDADE_TESTE_SILVER durante TEMPO_VALIDACAO_SILVER ms.
//    Nesse período, monitora:
//      • Verde detectado pelo RGB → aborta para sub-FSM EstadoAbortoPorVerde
//      • IR caiu abaixo de 5 sensores cravados → falso positivo → ESTADO_LINHA
//    Se o tempo expirar sem aborto → conta sensores cravados:
//      • Todos 8 → Silver tape confirmada → avança para FASE_POSICIONA_RGB
//      • Menos de 8 → falso positivo → ESTADO_LINHA
//
//  FASE_POSICIONA_RGB  ─────────────────────────────────────────────────────
//    O robô avança suavemente durante TEMPO_POSICIONA_RGB ms para que os
//    sensores TCS34725 (fisicamente atrás da barra IR) fiquem sobre a fita.
//    Continua monitorando verde. Ao expirar → FASE_FUSAO_SENSORES
//
//  FASE_FUSAO_SENSORES  ────────────────────────────────────────────────────
//    Lê o canal Clear (luminosidade) de ambos os TCS34725.
//    Aplica a condição de fusão dupla:
//      |clearESQ - lumCinzaEsqCalibrado| ≤ TOLERANCIA_CINZA  AND
//      |clearDIR - lumCinzaDirCalibrado| ≤ TOLERANCIA_CINZA
//    → Ambos dentro da faixa: Silver Tape confirmada → ESTADO_ZONA_RESGATE
//    → Pelo menos um fora:    Preto ou cruzamento    → ESTADO_LINHA
//
enum FaseValidacaoSilver {
  FASE_IR_AVANCANDO,    // Fase 1: avanço + monitoramento IR + filtro de verde
  FASE_POSICIONA_RGB,   // Fase 2: avanço para posicionar os TCS sobre a fita
  FASE_FUSAO_SENSORES   // Fase 3: leitura dos TCS + decisão por fusão de sensores
};

// ==============================================================================
// VARIÁVEIS DE ESTADO DO MÓDULO (escopo de arquivo — não poluem o global)
// ==============================================================================

static EstadoResgate estadoResgate         = RES_AGUARDANDO_INICIO;

// Validação da Silver Tape
static unsigned long tempoInicioValidacao  = 0;
static FaseValidacaoSilver faseValidacao   = FASE_IR_AVANCANDO; // Fase atual da validação
static unsigned long tempoInicioPosiciona  = 0; // Timestamp de início do posicionamento RGB

// -----------------------------------------------------------------------
// Aborto por Verde: variáveis da sub-FSM não-bloqueante
// -----------------------------------------------------------------------
static EstadoAbortoPorVerde estadoAborto   = ABORTO_INATIVO;

// Duração da ré de reposicionamento.
// Calculada em runtime como: (tempoAvancado * 90%) para compensar o avanço
// feito antes da detecção de verde. O cálculo usa variável local por fase.
static unsigned long tempoInicioAbortoRe   = 0; // timestamp de início da ré de reposicionamento
static unsigned long duracaoReAborto       = 0; // ms de ré calculados para compensar o avanço

// Fase de leitura de cor: aguarda estabilização óptica após parar
const unsigned long PAUSA_ESTABILIZACAO_COR_MS = 60; // equivalente ao delay(80) de avaliarInterseccao
static unsigned long tempoInicioPausaCor   = 0;

// Fase de mini-avanço de alinhamento (equivalente ao delay(150)+delay(50) de avaliarInterseccao)
const int           VEL_MINI_AVANCO        = 100;
const unsigned long TEMPO_MINI_AVANCO_MS   = 150;
static unsigned long tempoInicioMiniAvanco = 0;

// Filtro do cruzamento em cruz
static unsigned long tempoInicioCruz       = 0;

// Navegação por ultrassom
static unsigned long tempoUltimoSonarRes   = 0;
static unsigned long tempoInicioGiroRes    = 0;
static float         anguloBaseGiroRes     = 0.0;
static int           ladoGiroRes           = 1; // +1 = direita, -1 = esquerda

// ==============================================================================
// FUNÇÕES AUXILIARES INTERNAS
// ==============================================================================

/*
 * lerSensoresCor()
 * Verifica se ALGUM dos dois sensores TCS34725 está detectando verde.
 * Retorna true se verde for encontrado em qualquer sensor.
 */
static bool lerSensoresCor_verdeDetectado() {
  uint16_t rD, gD, bD, cD;
  uint16_t rE, gE, bE, cE;

  tcaselect(CANAL_TCS_DIR);
  tcsDir.getRawData(&rD, &gD, &bD, &cD);

  tcaselect(CANAL_TCS_ESQ);
  tcsEsq.getRawData(&rE, &gE, &bE, &cE);

  bool verdeDir = ehVerde(rD, gD, bD, cD, limiarLuminosidadeDir);
  bool verdeEsq = ehVerde(rE, gE, bE, cE, limiarLuminosidadeEsq);

  return (verdeDir || verdeEsq);
}

/*
 * contarSensoresCravados1000()
 * Conta quantos sensores da barra IR estão lendo exatamente 1000.
 * Usa o array global sensorValues (última leitura do qtr.readLineBlack).
 */
static int contarSensoresCravados1000() {
  int contagem = 0;
  for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
    if (sensorValues[i] == 1000) contagem++;
  }
  return contagem;
}

/*
 * contarSensoresCentraisPretos()
 * Avalia os 4 sensores centrais (índices 2, 3, 4, 5) para detectar
 * continuidade de linha preta — sinal de cruzamento em cruz (+).
 */
static int contarSensoresCentraisPretos() {
  int contagem = 0;
  for (uint8_t i = 2; i <= 5; i++) {
    if (sensorValues[i] >= LIMIAR_IR_CRUZ) contagem++;
  }
  return contagem;
}

// ==============================================================================
// FUNÇÕES PÚBLICAS — VALIDAÇÃO DA SILVER TAPE
// ==============================================================================

/*
 * iniciarValidacaoSilverTape()
 * Chamada UMA VEZ pelo ESTADO_LINHA ao detectar ≥5 sensores cravados em 1000.
 * Reseta todas as variáveis de fase e transita para ESTADO_VALIDANDO_SILVER_TAPE.
 */
inline void iniciarValidacaoSilverTape() {
  tempoInicioValidacao = millis();
  faseValidacao        = FASE_IR_AVANCANDO; // Começa pela fase de confirmação IR
  estadoAborto         = ABORTO_INATIVO;    // Garante que nenhum aborto anterior ficou pendente
  estadoAtual          = ESTADO_VALIDANDO_SILVER_TAPE;
  Serial.println(F("[RESGATE] Possivel Silver Tape detectada. Iniciando validacao (Fase 1: IR)..."));
}

/*
 * executarValidacaoSilverTape()
 * Chamada a cada ciclo do loop() enquanto o robô está em ESTADO_VALIDANDO_SILVER_TAPE.
 *
 * ┌─────────────────────────────────────────────────────────────────────────────┐
 * │  FLUXO DA MÁQUINA DE VALIDAÇÃO (3 fases não-bloqueantes)                   │
 * │                                                                             │
 * │  FASE 1 — FASE_IR_AVANCANDO (0 → TEMPO_VALIDACAO_SILVER ms)               │
 * │    • Avança a VELOCIDADE_TESTE_SILVER                                       │
 * │    • A cada ciclo: monitora verde (aborto) e conta IR cravados              │
 * │    • Verde detectado → sub-FSM EstadoAbortoPorVerde                         │
 * │    • Ao expirar: todos 8 IR = 1000 → FASE 2 | caso contrário → LINHA       │
 * │                                                                             │
 * │  FASE 2 — FASE_POSICIONA_RGB (0 → TEMPO_POSICIONA_RGB ms)                 │
 * │    • Avança a VELOCIDADE_POSICIONA_RGB (mais lento) para alinhar TCS       │
 * │    • Continua monitorando verde                                             │
 * │    • Ao expirar → FASE 3                                                    │
 * │                                                                             │
 * │  FASE 3 — FASE_FUSAO_SENSORES (executa uma única vez)                      │
 * │    • Para os motores                                                        │
 * │    • Lê Clear dos dois TCS34725                                             │
 * │    • Condição de fusão dupla (AND):                                         │
 * │        |clearESQ - lumCinzaEsqCalibrado| ≤ TOLERANCIA_CINZA                │
 * │        |clearDIR - lumCinzaDirCalibrado| ≤ TOLERANCIA_CINZA                │
 * │    • Ambos dentro → Silver confirmada → ESTADO_ZONA_RESGATE                │
 * │    • Pelo menos um fora → preto/cruzamento → ESTADO_LINHA                  │
 * └─────────────────────────────────────────────────────────────────────────────┘
 */
inline void executarValidacaoSilverTape() {

  // ============================================================
  // CAMINHO B: sub-FSM de aborto por verde (verde detectado
  // em ciclo anterior — executa ré, leitura e giro de forma
  // não-bloqueante enquanto estadoAtual ainda é VALIDANDO)
  // ============================================================
  if (estadoAborto != ABORTO_INATIVO) {

    switch (estadoAborto) {

      // ----------------------------------------------------------
      // Fase 1: Ré para reposicionar os sensores TCS sobre o verde
      // ----------------------------------------------------------
      case ABORTO_RECUANDO:
        if (millis() - tempoInicioAbortoRe < duracaoReAborto) {
          controlarRodas(-VELOCIDADE_TESTE_SILVER, -VELOCIDADE_TESTE_SILVER);
        } else {
          controlarRodas(0, 0);
          tempoInicioPausaCor = millis();
          estadoAborto        = ABORTO_LENDO_COR;
          Serial.println(F("[RESGATE-ABORTO] Re concluida. Aguardando estabilizacao optica..."));
        }
        break;

      // ----------------------------------------------------------
      // Fase 2: Pausa para estabilização óptica (chassis parado)
      // ----------------------------------------------------------
      case ABORTO_LENDO_COR: {
        controlarRodas(0, 0);

        if (millis() - tempoInicioPausaCor >= PAUSA_ESTABILIZACAO_COR_MS) {
          uint16_t rD, gD, bD, cD;
          uint16_t rE, gE, bE, cE;

          tcaselect(CANAL_TCS_DIR);
          tcsDir.getRawData(&rD, &gD, &bD, &cD);
          tcaselect(CANAL_TCS_ESQ);
          tcsEsq.getRawData(&rE, &gE, &bE, &cE);

          bool verdeDir = ehVerde(rD, gD, bD, cD, limiarLuminosidadeDir);
          bool verdeEsq = ehVerde(rE, gE, bE, cE, limiarLuminosidadeEsq);

          if (verdeDir || verdeEsq) {
            if      (verdeDir && verdeEsq) tipoGiro =  180;
            else if (verdeDir)             tipoGiro =   65;
            else                           tipoGiro =  -65;

            Serial.print(F("[RESGATE-ABORTO] Verde confirmado. Tipo de giro: "));
            Serial.println(tipoGiro);

            controlarRodas(VEL_MINI_AVANCO, VEL_MINI_AVANCO);
            tempoInicioMiniAvanco = millis();
            estadoAborto          = ABORTO_PREPARANDO_GIRO;
          } else {
            Serial.println(F("[RESGATE-ABORTO] Cor nao confirmada apos recuo. Retornando para linha."));
            tempoInicioValidacao = 0;
            estadoAborto         = ABORTO_INATIVO;
            faseValidacao        = FASE_IR_AVANCANDO;
            ultimoErro           = 0;
            contadorFalhas       = 0;
            modoLinha            = SEGUINDO;
            estadoAtual          = ESTADO_LINHA;
          }
        }
        break;
      }

      // ----------------------------------------------------------
      // Fase 3: Mini-avanço de alinhamento + captura do Yaw
      // ----------------------------------------------------------
      case ABORTO_PREPARANDO_GIRO:
        if (millis() - tempoInicioMiniAvanco < TEMPO_MINI_AVANCO_MS) {
          controlarRodas(VEL_MINI_AVANCO, VEL_MINI_AVANCO);
        } else {
          controlarRodas(0, 0);

          tcaselect(CANAL_GY521);
          mpu.update();
          anguloInicial = mpu.getAngleZ();

          ultimaLeituraCor     = millis() + 1500;
          tempoInicioValidacao = 0;
          estadoAborto         = ABORTO_INATIVO;
          faseValidacao        = FASE_IR_AVANCANDO;

          estadoAtual = ESTADO_VERDE;
          Serial.println(F("[RESGATE-ABORTO] Alinhado. Disparando ESTADO_VERDE."));
        }
        break;

      default:
        estadoAborto = ABORTO_INATIVO;
        break;
    }

    return; // Enquanto o aborto está em andamento, não executa o restante
  }

  // ============================================================
  // CAMINHO A: fluxo normal de validação por fases
  // ============================================================

  switch (faseValidacao) {

    // ------------------------------------------------------------------
    // FASE 1: FASE_IR_AVANCANDO
    // Avança durante TEMPO_VALIDACAO_SILVER ms confirmando que o IR mantém
    // ≥5 sensores cravados (sinal de superfície altamente refletiva).
    // Monitora verde a cada ciclo para aborto imediato.
    // ------------------------------------------------------------------
    case FASE_IR_AVANCANDO: {

      // Mantém avanço suave durante a janela de validação IR
      controlarRodas(VELOCIDADE_TESTE_SILVER, VELOCIDADE_TESTE_SILVER);

      // --- FILTRO CRÍTICO: verde interrompe imediatamente ---
      if (lerSensoresCor_verdeDetectado()) {
        unsigned long tempoAvancadoMs = millis() - tempoInicioValidacao;

        Serial.print(F("[RESGATE-ABORTO] Verde detectado na Fase IR apos "));
        Serial.print(tempoAvancadoMs);
        Serial.println(F("ms. Iniciando re proporcional..."));

        controlarRodas(0, 0);

        // Calcula ré proporcional ao avanço (90% do tempo avançado)
        duracaoReAborto = (tempoAvancadoMs * 90UL) / 100UL;
        if (duracaoReAborto < 40) duracaoReAborto = 40;

        tempoInicioAbortoRe = millis();
        estadoAborto        = ABORTO_RECUANDO;
        return;
      }

      // --- Checagem ao final da janela de tempo ---
      if (millis() - tempoInicioValidacao >= TEMPO_VALIDACAO_SILVER) {

        // Leitura precisa e atualizada dos sensores IR
        qtr.readLineBlack(sensorValues);
        int cravados = contarSensoresCravados1000();

        if (cravados == NUM_SENSORES_IR) {
          // Todos os 8 IR cravados: superfície totalmente refletiva confirmada.
          // Avança para a Fase 2 para posicionar os TCS34725 sobre a fita.
          Serial.println(F("[RESGATE] Fase 1 OK (8/8 IR). Avancando para posicionar RGB (Fase 2)..."));
          tempoInicioPosiciona = millis();
          faseValidacao        = FASE_POSICIONA_RGB;
        } else {
          // IR caiu: falso positivo (curva grossa, sujeira, etc.)
          tempoInicioValidacao = 0;
          faseValidacao        = FASE_IR_AVANCANDO;
          estadoAtual          = ESTADO_LINHA;
          Serial.print(F("[RESGATE] Fase 1 FALHOU (apenas "));
          Serial.print(cravados);
          Serial.println(F("/8 IR). Falso positivo. Retornando para linha."));
        }
      }
      break;
    }

    // ------------------------------------------------------------------
    // FASE 2: FASE_POSICIONA_RGB
    // Avança suavemente durante TEMPO_POSICIONA_RGB ms para que os
    // sensores TCS34725 (montados atrás da barra IR) fiquem fisicamente
    // sobre a fita prata. Continua monitorando verde para aborto.
    // ------------------------------------------------------------------
    case FASE_POSICIONA_RGB: {

      // Avanço lento para posicionamento preciso
      controlarRodas(VELOCIDADE_POSICIONA_RGB, VELOCIDADE_POSICIONA_RGB);

      // --- Filtro de verde continua ativo nesta fase ---
      if (lerSensoresCor_verdeDetectado()) {
        unsigned long tempoTotal = (millis() - tempoInicioValidacao);

        Serial.print(F("[RESGATE-ABORTO] Verde detectado na Fase RGB apos "));
        Serial.print(tempoTotal);
        Serial.println(F("ms totais. Iniciando re proporcional..."));

        controlarRodas(0, 0);

        // Recua o tempo total acumulado nas duas fases (IR + posicionamento)
        duracaoReAborto = (tempoTotal * 90UL) / 100UL;
        if (duracaoReAborto < 40) duracaoReAborto = 40;

        tempoInicioAbortoRe = millis();
        estadoAborto        = ABORTO_RECUANDO;
        return;
      }

      // --- Posicionamento concluído: vai para a fusão de sensores ---
      if (millis() - tempoInicioPosiciona >= TEMPO_POSICIONA_RGB) {
        controlarRodas(0, 0); // Para antes da leitura para evitar blur óptico
        Serial.println(F("[RESGATE] Fase 2 OK. Sensores RGB posicionados. Iniciando fusao (Fase 3)..."));
        faseValidacao = FASE_FUSAO_SENSORES;
        // A Fase 3 executa na próxima iteração do loop
      }
      break;
    }

    // ------------------------------------------------------------------
    // FASE 3: FASE_FUSAO_SENSORES
    // Lê o canal Clear dos dois TCS34725 e aplica a fusão por janela relativa.
    //
    // LÓGICA (derivada das medições reais com GAIN_4X a 3mm):
    //
    //   Superfície  | Clear ESQ | Clear DIR | Razão vs prata ESQ | Razão vs prata DIR
    //   ------------|-----------|-----------|--------------------|-----------------
    //   Preto       |    ~282   |    ~211   |       0.22x        |       0.25x
    //   Silver Tape |   ~1297   |    ~829   |       1.00x        |       1.00x   ← alvo
    //   Verde       |    ~721   |    ~501   |       0.56x        |       0.60x
    //   Vermelho    |    ~715   |    ~487   |       0.55x        |       0.59x
    //   Branco      |   ~4118   |   ~2605   |       3.17x        |       3.14x
    //
    //   Verde e vermelho ficam abaixo de 0.60x → já foram filtrados pelo
    //   monitoramento contínuo de verde nas fases anteriores, mas a fusão
    //   os rejeita por via de regra (Clear < FATOR_MIN_SILVER * ref).
    //
    //   CONDIÇÃO DE ACEITAÇÃO (AND em AMBOS os sensores):
    //     Clear >= lumCalibrado * FATOR_MIN_SILVER  (0.45 → rejeita preto, verde, vermelho)
    //     Clear <= lumCalibrado * FATOR_MAX_SILVER  (2.20 → rejeita branco puro)
    //
    //   Por que AND e não OR?
    //     A prata cobre os dois sensores simultaneamente. Exigir concordância
    //     de ambos elimina reflexos localizados (borda da fita, sujeira pontual).
    // ------------------------------------------------------------------
    case FASE_FUSAO_SENSORES: {

      uint16_t rD, gD, bD, clearDIR;
      uint16_t rE, gE, bE, clearESQ;

      tcaselect(CANAL_TCS_DIR);
      tcsDir.getRawData(&rD, &gD, &bD, &clearDIR);
      tcaselect(CANAL_TCS_ESQ);
      tcsEsq.getRawData(&rE, &gE, &bE, &clearESQ);

      // Janela relativa ao valor calibrado em pista
      uint16_t minDIR = (uint16_t)(lumCinzaDirCalibrado * FATOR_MIN_SILVER);
      uint16_t maxDIR = (uint16_t)(lumCinzaDirCalibrado * FATOR_MAX_SILVER);
      uint16_t minESQ = (uint16_t)(lumCinzaEsqCalibrado * FATOR_MIN_SILVER);
      uint16_t maxESQ = (uint16_t)(lumCinzaEsqCalibrado * FATOR_MAX_SILVER);

      bool dirOK = (clearDIR >= minDIR) && (clearDIR <= maxDIR);
      bool esqOK = (clearESQ >= minESQ) && (clearESQ <= maxESQ);

      // Debug compacto para Serial Monitor em pista
      Serial.print(F("[FUSAO] DIR C=")); Serial.print(clearDIR);
      Serial.print(F(" janela=[")); Serial.print(minDIR); Serial.print(F(",")); Serial.print(maxDIR);
      Serial.print(F("] ")); Serial.print(dirOK ? F("OK") : F("FAIL"));
      Serial.print(F(" | ESQ C=")); Serial.print(clearESQ);
      Serial.print(F(" janela=[")); Serial.print(minESQ); Serial.print(F(",")); Serial.print(maxESQ);
      Serial.print(F("] ")); Serial.println(esqOK ? F("OK") : F("FAIL"));

      if (dirOK && esqOK) {
        pararMotores();
        tempoInicioValidacao = 0;
        faseValidacao        = FASE_IR_AVANCANDO;
        estadoResgate        = RES_FILTRO_CRUZ;
        estadoAtual          = ESTADO_ZONA_RESGATE;
        Serial.println(F("[RESGATE] SILVER TAPE CONFIRMADA! Entrando na Zona de Resgate."));

      } else {
        tempoInicioValidacao = 0;
        faseValidacao        = FASE_IR_AVANCANDO;
        ultimoErro           = 0;
        contadorFalhas       = 0;
        modoLinha            = SEGUINDO;
        estadoAtual          = ESTADO_LINHA;
        Serial.println(F("[RESGATE] Fusao rejeitada. Retornando para linha."));
      }
      break;
    }

    default:
      faseValidacao = FASE_IR_AVANCANDO;
      break;
  }
}

// ==============================================================================
// FUNÇÕES PÚBLICAS — ZONA DE RESGATE
// ==============================================================================

/*
 * executarRotinaResgate()
 * Chamada a cada ciclo do loop() enquanto o robô está em ESTADO_ZONA_RESGATE.
 * Contém a FSM interna do resgate com os sub-estados do enum EstadoResgate.
 */
inline void executarRotinaResgate() {

  switch (estadoResgate) {

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_FILTRO_CRUZ
    // Avança um pouco após cruzar a fita prata para confirmar que não é
    // um cruzamento em formato de cruz (+).
    // --------------------------------------------------------------------------
    case RES_FILTRO_CRUZ: {
      static bool filtroIniciado = false;

      if (!filtroIniciado) {
        tempoInicioCruz  = millis();
        filtroIniciado   = true;
        Serial.println(F("[RESGATE] Filtro de cruzamento em cruz: avancando..."));
      }

      // Avança enquanto o tempo não esgotou
      if (millis() - tempoInicioCruz < TEMPO_FILTRO_CRUZ_MS) {
        controlarRodas(VELOCIDADE_FILTRO_CRUZ, VELOCIDADE_FILTRO_CRUZ);

        // Durante o avanço, monitora os sensores centrais
        qtr.readLineBlack(sensorValues);
        if (contarSensoresCentraisPretos() >= SENSORES_CENTRAIS_CRUZ) {
          // Linha preta nos centrais: é um cruzamento em +, não é o resgate!
          pararMotores();
          filtroIniciado = false;
          estadoResgate  = RES_AGUARDANDO_INICIO; // Reseta a FSM interna

          // Zera variáveis do PID para retomada limpa
          ultimoErro     = 0;
          contadorFalhas = 0;
          modoLinha      = SEGUINDO;

          estadoAtual = ESTADO_LINHA;
          Serial.println(F("[RESGATE] Cruzamento em CRUZ detectado! Abortando resgate. Voltando para linha."));
          return;
        }

      } else {
        // Tempo esgotado: leitura definitiva dos sensores
        pararMotores();
        qtr.readLineBlack(sensorValues);

        // Verifica se os sensores centrais estão vendo branco (linha sumiu = resgate real)
        bool brancoTotal = true;
        for (uint8_t i = 2; i <= 5; i++) {
          if (sensorValues[i] >= LIMIAR_IR_CRUZ) {
            brancoTotal = false;
            break;
          }
        }

        filtroIniciado = false; // Reseta a flag para próxima entrada

        if (brancoTotal) {
          // Branco total nos centrais: resgate CONFIRMADO!
          Serial.println(F("[RESGATE] Branco total confirmado. Resgate validado! Iniciando navegacao."));
          tempoUltimoSonarRes = millis();
          estadoResgate = RES_NAVEGANDO;
        } else {
          // Ainda tem linha: cruzamento em cruz tardio ou ruído
          ultimoErro     = 0;
          contadorFalhas = 0;
          modoLinha      = SEGUINDO;
          estadoResgate  = RES_AGUARDANDO_INICIO;
          estadoAtual    = ESTADO_LINHA;
          Serial.println(F("[RESGATE] Linha ainda presente apos filtro. Abortando. Voltando para linha."));
        }
      }
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_NAVEGANDO
    // Navegação por malha fechada usando os três ultrassônicos.
    // Estratégia: avança enquanto há espaço frontal; ao encontrar parede,
    // gira para o lado com mais espaço.
    // --------------------------------------------------------------------------
    case RES_NAVEGANDO: {
      // Throttle de leitura do sonar para não ultrapassar ~20 Hz do HC-SR04
      if (millis() - tempoUltimoSonarRes > TIMEOUT_SONAR_RESGATE_MS) {
        tempoUltimoSonarRes = millis();

        // Lê os três ultrassônicos
        int distFrente = obterDistanciaFiltrada(sonarFrente);
        int distEsq    = obterDistanciaFiltrada(sonarEsq);
        int distDir    = obterDistanciaFiltrada(sonarDir);

        Serial.print(F("[RESGATE-NAV] F:")); Serial.print(distFrente);
        Serial.print(F(" E:"));              Serial.print(distEsq);
        Serial.print(F(" D:"));              Serial.println(distDir);

        // --- Decisão primária: parede frontal? ---
        if (distFrente <= DIST_PAREDE_FRENTE_CM) {
          // Parede à frente: para e escolhe o lado com mais espaço para girar
          controlarRodas(0, 0);

          // Captura o Yaw atual como base para o giro de 90°
          tcaselect(CANAL_GY521);
          mpu.update();
          anguloBaseGiroRes = mpu.getAngleZ();

          if (distDir >= distEsq) {
            // Mais espaço à direita: gira para a direita
            ladoGiroRes   = 1;
            estadoResgate = RES_GIRANDO_DIREITA;
            Serial.println(F("[RESGATE-NAV] Parede frontal! Girando 90 graus para DIREITA."));
          } else {
            // Mais espaço à esquerda: gira para a esquerda
            ladoGiroRes   = -1;
            estadoResgate = RES_GIRANDO_ESQUERDA;
            Serial.println(F("[RESGATE-NAV] Parede frontal! Girando 90 graus para ESQUERDA."));
          }
          tempoInicioGiroRes = millis();
          return;
        }

        // --- Decisão secundária: correção lateral suave (seguimento de parede) ---
        // Mantém o robô centralizado no corredor usando feedback dos sonares laterais
        float erroLateral = distDir - distEsq; // Positivo = mais longe da direita → deriva para esq
        int ajusteLateral = (int)(erroLateral * 1.5); // Ganho proporcional simples
        ajusteLateral     = constrain(ajusteLateral, -30, 30);

        // Avança com correção lateral
        controlarRodas(VEL_RESGATE_BASE + ajusteLateral, VEL_RESGATE_BASE - ajusteLateral);
      }
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_GIRANDO_DIREITA
    // Gira 90° para a direita usando o Yaw do MPU6050 como referência.
    // Convenção do MPU neste projeto: giro à DIREITA → Yaw DIMINUI (negativo).
    // --------------------------------------------------------------------------
    case RES_GIRANDO_DIREITA: {
      tcaselect(CANAL_GY521);
      mpu.update();
      float yawAtual  = mpu.getAngleZ();
      float anguloAlvo = anguloBaseGiroRes - 90.0; // Giro à direita: subtrai 90°
      float erro      = anguloAlvo - yawAtual;

      if (abs(erro) > 3.0) {
        // Proporcional ao erro, limitado para não gerar inércia excessiva
        int velGiro = constrain(80 + (int)(abs(erro) * 1.5), 80, 160);
        // Girar à DIREITA: roda esquerda para frente, direita para trás
        controlarRodas(-velGiro, velGiro);

        // Timeout de segurança
        if (millis() - tempoInicioGiroRes > TIMEOUT_GIRO_RESGATE_MS) {
          Serial.println(F("[RESGATE-GIRO] TIMEOUT no giro direita! Retomando navegacao."));
          controlarRodas(0, 0);
          tempoUltimoSonarRes = millis();
          estadoResgate = RES_NAVEGANDO;
        }
      } else {
        // Giro concluído
        controlarRodas(0, 0);
        Serial.println(F("[RESGATE-GIRO] Giro 90 graus DIREITA concluido."));
        tempoUltimoSonarRes = millis();
        estadoResgate = RES_NAVEGANDO;
      }
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_GIRANDO_ESQUERDA
    // Gira 90° para a esquerda usando o Yaw do MPU6050 como referência.
    // Convenção do MPU neste projeto: giro à ESQUERDA → Yaw AUMENTA (positivo).
    // --------------------------------------------------------------------------
    case RES_GIRANDO_ESQUERDA: {
      tcaselect(CANAL_GY521);
      mpu.update();
      float yawAtual   = mpu.getAngleZ();
      float anguloAlvo = anguloBaseGiroRes + 90.0; // Giro à esquerda: soma 90°
      float erro       = anguloAlvo - yawAtual;

      if (abs(erro) > 3.0) {
        int velGiro = constrain(80 + (int)(abs(erro) * 1.5), 80, 160);
        // Girar à ESQUERDA: roda direita para frente, esquerda para trás
        controlarRodas(velGiro, -velGiro);

        if (millis() - tempoInicioGiroRes > TIMEOUT_GIRO_RESGATE_MS) {
          Serial.println(F("[RESGATE-GIRO] TIMEOUT no giro esquerda! Retomando navegacao."));
          controlarRodas(0, 0);
          tempoUltimoSonarRes = millis();
          estadoResgate = RES_NAVEGANDO;
        }
      } else {
        controlarRodas(0, 0);
        Serial.println(F("[RESGATE-GIRO] Giro 90 graus ESQUERDA concluido."));
        tempoUltimoSonarRes = millis();
        estadoResgate = RES_NAVEGANDO;
      }
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_CONCLUIDO
    // Reservado para expansão futura (depositar vítimas, sair da sala, etc.)
    // Por ora, para o robô e aguarda.
    // --------------------------------------------------------------------------
    case RES_CONCLUIDO:
      controlarRodas(0, 0);
      break;

    // Segurança: estado inválido → para o robô
    default:
      controlarRodas(0, 0);
      break;
  }
}

#endif // RESGATE_H
