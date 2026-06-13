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

// --- Navegação na Sala de Resgate (Ultrassom + Giroscópio) ---
// Velocidades: usa as constantes de config.h (VELOCIDADE_RESGATE_RETO / _GIRO)
// para garantir precisão. Ver config.h para justificativa dos valores.

const float         DIST_PAREDE_FRENTE_CM       = 12.0; // Distância de frenagem/parada frontal (cm)
const float         DIST_PAREDE_LATERAL_CM      = 18.0; // Distância alvo de centralização lateral (cm)
const float         DIST_LATERAL_TOLERANCIA_RES =  4.0; // Banda morta lateral (cm) — evita correção contínua
const unsigned long TIMEOUT_GIRO_RESGATE_MS     = 3500; // Timeout de segurança do giro (ms)
const unsigned long TIMEOUT_SONAR_RESGATE_MS    =  55;  // Intervalo entre ciclos de leitura sonar (~18 Hz)

// --- Alinhamento em linha reta com o giroscópio (malha fechada de Yaw) ---
// Enquanto avança, o robô monitora o desvio angular em relação ao Yaw de referência.
// A correção é proporcional ao erro de ângulo — análogo ao PID de linha.
const float         KP_YAW_RESGATE             =  2.5f; // Ganho proporcional do controlador de Yaw
const float         TOLERANCIA_YAW_RESGATE     =  1.5f; // Erro angular (graus) tolerado sem correção
const int           CORRECAO_YAW_MAX           =   25;  // Correção máxima em PWM (evita oscilação)

// --- Filtro de média móvel para os ultrassônicos (anti-ruído de eco) ---
// Cada leitura de sonar é armazenada em um buffer circular de N amostras.
// A distância usada é a mediana das N amostras — imune a spikes isolados.
const uint8_t       TAM_FILTRO_SONAR           =    3;  // Número de amostras no buffer (3 leituras)

// --- Detecção da saída da zona de resgate (portal = linha preta no IR) ---
// O robô varre continuamente a barra QTR durante a navegação.
// Considera saída confirmada quando ≥ SENSORES_SAIDA_MIN dos 4 sensores
// centrais lerem acima de LIMIAR_IR_SAIDA por TEMPO_CONFIRMA_SAIDA_MS seguidos.
const int           LIMIAR_IR_SAIDA            =  700;  // Valor QTR para considerar "linha preta"
const uint8_t       SENSORES_SAIDA_MIN         =    3;  // Sensores centrais mínimos para confirmar saída
const unsigned long TEMPO_CONFIRMA_SAIDA_MS    =   80;  // Janela de debounce da detecção de saída (ms)

// ==============================================================================
// ENUMERAÇÃO DA FSM INTERNA DO RESGATE
// ==============================================================================
enum EstadoResgate {
  RES_AGUARDANDO_INICIO,    // Estado neutro / aguardando entrada na zona
  RES_FILTRO_CRUZ,          // Avanço pós-silver para detectar cruzamento em +
  RES_ENTRADA_SALA,         // Captura Yaw de referência e inicializa buffers (executa 1x)
  RES_NAVEGANDO,            // Navegação autônoma: linha reta (Yaw) + sonar filtrado
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
static float         anguloReferenciaReto  = 0.0; // Yaw salvo ao entrar na sala / após cada giro
static int           ladoGiroRes           = 1;   // +1 = direita, -1 = esquerda

// Buffers circulares para filtro de média móvel dos três sonares
static int  bufFrente[TAM_FILTRO_SONAR];
static int  bufEsq[TAM_FILTRO_SONAR];
static int  bufDir[TAM_FILTRO_SONAR];
static uint8_t idxBufSonar = 0;
static bool  bufInicializado = false;

// Detecção da saída (portal de linha preta)
static unsigned long tempoDetectouSaida    = 0;  // Momento da primeira detecção estável
static bool          saidaEmDebounce       = false; // Flag de janela de confirmação

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
// FUNÇÕES AUXILIARES — ZONA DE RESGATE
// ==============================================================================

/*
 * inicializarBuffersSonar()
 * Preenche todos os buffers circulares com a distância máxima para garantir
 * que o filtro não produza leituras falsas no primeiro ciclo dentro da sala.
 */
static void inicializarBuffersSonar() {
  for (uint8_t i = 0; i < TAM_FILTRO_SONAR; i++) {
    bufFrente[i] = MAX_DISTANCE;
    bufEsq[i]    = MAX_DISTANCE;
    bufDir[i]    = MAX_DISTANCE;
  }
  idxBufSonar    = 0;
  bufInicializado = true;
}

/*
 * lerSonarFiltrado()
 * Lê os três sonares, atualiza o buffer circular e retorna a mediana
 * (valor do meio após ordenação dos 3 elementos).
 *
 * POR QUE MEDIANA E NÃO MÉDIA?
 *   Um único eco falso (ex: 0 cm ou 60 cm) não contamina a mediana.
 *   A média seria puxada pelo outlier; a mediana descarta o extremo.
 *   Com N=3: ordena e retorna o elemento central.
 *
 * Preenche as variáveis passadas por referência com os valores filtrados.
 */
static void lerSonarFiltrado(int &outFrente, int &outEsq, int &outDir) {
  // Lê uma nova amostra de cada sensor e armazena no slot atual do buffer
  bufFrente[idxBufSonar] = obterDistanciaFiltrada(sonarFrente);
  bufEsq   [idxBufSonar] = obterDistanciaFiltrada(sonarEsq);
  bufDir   [idxBufSonar] = obterDistanciaFiltrada(sonarDir);
  idxBufSonar = (idxBufSonar + 1) % TAM_FILTRO_SONAR; // Avança índice circular

  // --- Mediana de 3 elementos (ordenação em rede — 3 comparações, sem laço) ---
  // Copia os buffers para variáveis temporárias antes de ordenar
  int f[3] = { bufFrente[0], bufFrente[1], bufFrente[2] };
  int e[3] = { bufEsq[0],    bufEsq[1],    bufEsq[2]    };
  int d[3] = { bufDir[0],    bufDir[1],    bufDir[2]     };

  // Rede de ordenação ótima para N=3 (Batcher's odd-even merge)
  #define SWAP_IF_MAIOR(a, b) if ((a) > (b)) { int t = (a); (a) = (b); (b) = t; }
  SWAP_IF_MAIOR(f[0], f[1]); SWAP_IF_MAIOR(f[1], f[2]); SWAP_IF_MAIOR(f[0], f[1]);
  SWAP_IF_MAIOR(e[0], e[1]); SWAP_IF_MAIOR(e[1], e[2]); SWAP_IF_MAIOR(e[0], e[1]);
  SWAP_IF_MAIOR(d[0], d[1]); SWAP_IF_MAIOR(d[1], d[2]); SWAP_IF_MAIOR(d[0], d[1]);
  #undef SWAP_IF_MAIOR

  outFrente = f[1]; // Elemento central = mediana
  outEsq    = e[1];
  outDir    = d[1];
}

/*
 * verificarSaidaResgate()
 * Verifica continuamente se a barra IR detectou a linha preta do portal de saída.
 *
 * LÓGICA DE DEBOUNCE:
 *   Ao detectar ≥ SENSORES_SAIDA_MIN sensores centrais acima de LIMIAR_IR_SAIDA,
 *   inicia uma janela de confirmação de TEMPO_CONFIRMA_SAIDA_MS ms.
 *   Se após esse tempo o IR ainda confirmar a linha, a saída é aceita.
 *   Se o IR oscilar durante a janela, o debounce é resetado (ignora ruído).
 *
 * POR QUE USAR OS CENTRAIS (índices 2–5)?
 *   Os sensores das bordas (0, 1, 6, 7) podem ver reflexos laterais das
 *   paredes brancas do corredor de resgate. Os centrais são mais confiáveis
 *   para detectar a linha preta que cruza o corredor na direção de saída.
 *
 * Retorna true se a saída foi confirmada com debounce.
 */
static bool verificarSaidaResgate() {
  qtr.readLineBlack(sensorValues);

  // Conta sensores centrais (2, 3, 4, 5) com leitura de linha preta
  uint8_t contagem = 0;
  for (uint8_t i = 2; i <= 5; i++) {
    if (sensorValues[i] >= LIMIAR_IR_SAIDA) contagem++;
  }

  bool linhaPresenteAgora = (contagem >= SENSORES_SAIDA_MIN);

  if (linhaPresenteAgora) {
    if (!saidaEmDebounce) {
      // Primeira detecção — inicia janela de confirmação
      saidaEmDebounce    = true;
      tempoDetectouSaida = millis();
      return false; // Ainda não confirmado
    }
    // Linha persistiu: verifica se a janela de debounce expirou
    if (millis() - tempoDetectouSaida >= TEMPO_CONFIRMA_SAIDA_MS) {
      return true; // SAÍDA CONFIRMADA
    }
  } else {
    // Linha sumiu antes do debounce — reseta (era ruído ou reflexo)
    saidaEmDebounce = false;
  }

  return false;
}

/*
 * resetarEstadoNavegacaoResgate()
 * Limpa todas as variáveis de tempo e estado da navegação interna do resgate.
 * Chamada tanto na transição de saída quanto em reinícios de segurança.
 */
static void resetarEstadoNavegacaoResgate() {
  estadoResgate       = RES_AGUARDANDO_INICIO;
  tempoUltimoSonarRes = 0;
  tempoInicioGiroRes  = 0;
  anguloBaseGiroRes   = 0.0;
  anguloReferenciaReto = 0.0;
  ladoGiroRes         = 1;
  saidaEmDebounce     = false;
  tempoDetectouSaida  = 0;
  bufInicializado     = false;
}

// ==============================================================================
// FUNÇÕES PÚBLICAS — ZONA DE RESGATE
// ==============================================================================

/*
 * executarRotinaResgate()
 * Chamada a cada ciclo do loop() enquanto o robô está em ESTADO_ZONA_RESGATE.
 *
 * ARQUITETURA DA FSM INTERNA:
 *
 *   RES_FILTRO_CRUZ
 *     └─► (branco total após avanço)  ──► RES_ENTRADA_SALA
 *     └─► (linha preta nos centrais)  ──► ESTADO_LINHA (falso positivo)
 *
 *   RES_ENTRADA_SALA  (NOVO)
 *     └─► Salva Yaw de referência ("linha reta" = 0°) e inicializa buffers
 *     └─► Transita imediatamente para RES_NAVEGANDO
 *
 *   RES_NAVEGANDO
 *     ├─► Avança em linha reta com correção proporcional do Yaw (malha fechada)
 *     ├─► Varre IR continuamente → linha preta central = SAÍDA confirmada
 *     ├─► Sonar filtrado por mediana (buffer N=3) a cada TIMEOUT_SONAR_RESGATE_MS
 *     └─► Parede frontal ≤ DIST_PAREDE_FRENTE_CM → RES_GIRANDO_DIREITA ou _ESQUERDA
 *
 *   RES_GIRANDO_DIREITA / RES_GIRANDO_ESQUERDA
 *     ├─► Giro de 90° validado pelo Yaw do MPU6050 (proporcional ao erro)
 *     ├─► Varre IR durante o giro → saída confirmada mesmo girando
 *     └─► Ao concluir: salva novo Yaw de referência → RES_NAVEGANDO
 *
 * SAÍDA DA ZONA:
 *   Qualquer sub-estado pode chamar verificarSaidaResgate().
 *   Ao confirmar: limpa variáveis, muda estadoAtual → ESTADO_LINHA.
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
        tempoInicioCruz = millis();
        filtroIniciado  = true;
        Serial.println(F("[RESGATE] Filtro de cruzamento em cruz: avancando..."));
      }

      // Avança enquanto o tempo não esgotou
      if (millis() - tempoInicioCruz < TEMPO_FILTRO_CRUZ_MS) {
        controlarRodas(VELOCIDADE_FILTRO_CRUZ, VELOCIDADE_FILTRO_CRUZ);

        // Monitora os sensores centrais durante o avanço
        qtr.readLineBlack(sensorValues);
        if (contarSensoresCentraisPretos() >= SENSORES_CENTRAIS_CRUZ) {
          // Linha preta nos centrais: é um cruzamento em +, não é a sala de resgate
          pararMotores();
          filtroIniciado = false;
          estadoResgate  = RES_AGUARDANDO_INICIO;

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

        bool brancoTotal = true;
        for (uint8_t i = 2; i <= 5; i++) {
          if (sensorValues[i] >= LIMIAR_IR_CRUZ) {
            brancoTotal = false;
            break;
          }
        }

        filtroIniciado = false;

        if (brancoTotal) {
          // Branco total: resgate confirmado → vai para a entrada
          Serial.println(F("[RESGATE] Branco total confirmado. Sala de resgate validada! Preparando entrada..."));
          estadoResgate = RES_ENTRADA_SALA;
        } else {
          // Ainda tem linha: cruzamento tardio ou ruído
          ultimoErro     = 0;
          contadorFalhas = 0;
          modoLinha      = SEGUINDO;
          estadoResgate  = RES_AGUARDANDO_INICIO;
          estadoAtual    = ESTADO_LINHA;
          Serial.println(F("[RESGATE] Linha presente apos filtro. Abortando. Voltando para linha."));
        }
      }
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_ENTRADA_SALA  (novo)
    // Executa UMA VEZ ao entrar na sala:
    //   1. Salva o Yaw atual como referência de "linha reta" (0° relativo)
    //   2. Inicializa os buffers de filtro dos sonares
    //   3. Reseta variáveis de debounce da saída
    //   4. Transita imediatamente para RES_NAVEGANDO
    //
    // Este estado isolado garante que a referência angular seja sempre
    // capturada com o robô parado e estável, nunca no meio de um movimento.
    // --------------------------------------------------------------------------
    case RES_ENTRADA_SALA: {
      pararMotores();

      // Captura o Yaw atual como referência de "andar reto" na sala
      tcaselect(CANAL_GY521);
      mpu.update();
      anguloReferenciaReto = mpu.getAngleZ();
      anguloBaseGiroRes    = anguloReferenciaReto;

      // Inicializa os buffers circulares do filtro de sonar
      inicializarBuffersSonar();

      // Reseta o debounce da saída
      saidaEmDebounce    = false;
      tempoDetectouSaida = 0;

      tempoUltimoSonarRes = millis();

      Serial.print(F("[RESGATE] Entrada na sala. Yaw de referencia: "));
      Serial.print(anguloReferenciaReto);
      Serial.println(F(" graus. Iniciando navegacao..."));

      estadoResgate = RES_NAVEGANDO;
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_NAVEGANDO
    // Navegação autônoma dentro da sala de resgate usando:
    //   • Malha fechada de Yaw (giroscópio) para manter linha reta
    //   • Três ultrassônicos filtrados por mediana para mapeamento de paredes
    //   • Varredura contínua da barra IR para detectar a saída
    //
    // ESTRATÉGIA DE NAVEGAÇÃO POR PAREDE (wall-following simplificado):
    //   O robô avança em linha reta corrigida pelo Yaw. Ao detectar parede
    //   frontal, para e gira 90° para o lado com mais espaço lateral.
    //   Não há seguimento de parede lateral — o Yaw mantém a reta após cada giro.
    //   Isso é suficiente para labirintos ortogonais (ladrilhos 30×30 cm).
    // --------------------------------------------------------------------------
    case RES_NAVEGANDO: {

      // -----------------------------------------------------------------------
      // PASSO 1: Varredura contínua da saída (prioridade máxima — verifica ANTES
      //          de qualquer decisão de movimentação)
      // -----------------------------------------------------------------------
      if (verificarSaidaResgate()) {
        pararMotores();

        // Limpa todo o estado interno da navegação do resgate
        resetarEstadoNavegacaoResgate();

        // Limpa variáveis do PID para retomada limpa pelo ESTADO_LINHA
        ultimoErro     = 0;
        contadorFalhas = 0;
        modoLinha      = SEGUINDO;

        estadoAtual = ESTADO_LINHA;
        Serial.println(F("[RESGATE] SAIDA DETECTADA! Portal encontrado. Retornando ao PID."));
        return;
      }

      // -----------------------------------------------------------------------
      // PASSO 2: Throttle do sonar — executa o ciclo de navegação em ~18 Hz
      // -----------------------------------------------------------------------
      if (millis() - tempoUltimoSonarRes <= TIMEOUT_SONAR_RESGATE_MS) break;
      tempoUltimoSonarRes = millis();

      // -----------------------------------------------------------------------
      // PASSO 3: Leitura dos três sonares com filtro de mediana
      // -----------------------------------------------------------------------
      int distFrente, distEsq, distDir;
      lerSonarFiltrado(distFrente, distEsq, distDir);

      Serial.print(F("[RESGATE-NAV] F:")); Serial.print(distFrente);
      Serial.print(F(" E:"));              Serial.print(distEsq);
      Serial.print(F(" D:"));              Serial.println(distDir);

      // -----------------------------------------------------------------------
      // PASSO 4: Decisão — parede frontal?
      // -----------------------------------------------------------------------
      if (distFrente <= (int)DIST_PAREDE_FRENTE_CM) {
        // Para o robô e escolhe o lado com mais espaço livre
        controlarRodas(0, 0);

        // Captura Yaw para o giro de 90°
        tcaselect(CANAL_GY521);
        mpu.update();
        anguloBaseGiroRes = mpu.getAngleZ();

        if (distDir >= distEsq) {
          ladoGiroRes   =  1;
          estadoResgate = RES_GIRANDO_DIREITA;
          Serial.println(F("[RESGATE-NAV] Parede frontal! Girando 90 graus para DIREITA."));
        } else {
          ladoGiroRes   = -1;
          estadoResgate = RES_GIRANDO_ESQUERDA;
          Serial.println(F("[RESGATE-NAV] Parede frontal! Girando 90 graus para ESQUERDA."));
        }
        tempoInicioGiroRes = millis();
        return;
      }

      // -----------------------------------------------------------------------
      // PASSO 5: Avanço em linha reta com correção de Yaw (malha fechada)
      //
      // CONTROLE PROPORCIONAL DO YAW:
      //   Erro = anguloReferenciaReto - yawAtual
      //   Positivo → robô deriva para a esquerda → aumenta velocidade da esquerda
      //   Negativo → robô deriva para a direita  → aumenta velocidade da direita
      //
      //   Correção = KP_YAW_RESGATE × erro, limitada a ±CORRECAO_YAW_MAX PWM
      //
      // Esta abordagem compensa assimetrias mecânicas (rodas com diâmetros
      // ligeiramente diferentes, atrito diferencial, etc.) de forma contínua
      // sem acumular erro ao longo do percurso nos ladrilhos brancos.
      // -----------------------------------------------------------------------
      tcaselect(CANAL_GY521);
      mpu.update();
      float yawAtual  = mpu.getAngleZ();
      float erroYaw   = anguloReferenciaReto - yawAtual;

      // Só corrige se o desvio ultrapassar a tolerância (evita micro-oscilações)
      int correcao = 0;
      if (abs(erroYaw) > TOLERANCIA_YAW_RESGATE) {
        correcao = constrain((int)(KP_YAW_RESGATE * erroYaw), -CORRECAO_YAW_MAX, CORRECAO_YAW_MAX);
      }

      // Aplica a velocidade de resgate com a correção diferencial
      // Lado esquerdo: +correcao aumenta quando deriva para esq (erro > 0)
      // Lado direito:  -correcao diminui quando deriva para esq (freia lado externo)
      controlarRodas(
        VELOCIDADE_RESGATE_RETO - correcao,  // Direita
        VELOCIDADE_RESGATE_RETO + correcao   // Esquerda
      );
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_GIRANDO_DIREITA
    // Gira 90° para a direita usando o Yaw do MPU6050 como referência absoluta.
    // Convenção do MPU neste projeto: giro à DIREITA → Yaw DIMINUI (negativo).
    //
    // Após o giro: salva novo Yaw como referência de linha reta para o próximo
    // corredor — essencial para que a malha de Yaw funcione corretamente.
    // --------------------------------------------------------------------------
    case RES_GIRANDO_DIREITA: {

      // Varredura contínua da saída mesmo durante o giro
      if (verificarSaidaResgate()) {
        pararMotores();
        resetarEstadoNavegacaoResgate();
        ultimoErro     = 0;
        contadorFalhas = 0;
        modoLinha      = SEGUINDO;
        estadoAtual    = ESTADO_LINHA;
        Serial.println(F("[RESGATE] SAIDA detectada durante giro! Retornando ao PID."));
        return;
      }

      tcaselect(CANAL_GY521);
      mpu.update();
      float yawAtual   = mpu.getAngleZ();
      float anguloAlvo = anguloBaseGiroRes - 90.0; // Direita → subtrai 90°
      float erro       = anguloAlvo - yawAtual;

      if (abs(erro) > 3.0) {
        // Velocidade proporcional ao erro: mais rápido longe do alvo, freia ao chegar
        int velGiro = constrain(
          VELOCIDADE_RESGATE_GIRO + (int)(abs(erro) * 1.2f),
          VELOCIDADE_RESGATE_GIRO,
          VELOCIDADE_RESGATE_GIRO + 50
        );
        // Girar à DIREITA: esquerda para frente, direita para trás
        controlarRodas(-velGiro, velGiro);

        // Timeout de segurança contra travamento mecânico
        if (millis() - tempoInicioGiroRes > TIMEOUT_GIRO_RESGATE_MS) {
          Serial.println(F("[RESGATE-GIRO] TIMEOUT no giro direita! Forcando retomada."));
          controlarRodas(0, 0);
          // Atualiza referência com o ângulo real atingido (mesmo incompleto)
          anguloReferenciaReto = mpu.getAngleZ();
          tempoUltimoSonarRes  = millis();
          estadoResgate        = RES_NAVEGANDO;
        }
      } else {
        // Giro concluído com precisão
        controlarRodas(0, 0);

        // CRÍTICO: salva o Yaw atual como nova referência de "linha reta"
        // para o corredor perpendicular que o robô vai percorrer a seguir
        anguloReferenciaReto = mpu.getAngleZ();

        Serial.print(F("[RESGATE-GIRO] Giro 90 DIREITA concluido. Nova ref. Yaw: "));
        Serial.println(anguloReferenciaReto);

        tempoUltimoSonarRes = millis();
        estadoResgate       = RES_NAVEGANDO;
      }
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_GIRANDO_ESQUERDA
    // Gira 90° para a esquerda usando o Yaw do MPU6050 como referência absoluta.
    // Convenção do MPU neste projeto: giro à ESQUERDA → Yaw AUMENTA (positivo).
    // --------------------------------------------------------------------------
    case RES_GIRANDO_ESQUERDA: {

      // Varredura contínua da saída mesmo durante o giro
      if (verificarSaidaResgate()) {
        pararMotores();
        resetarEstadoNavegacaoResgate();
        ultimoErro     = 0;
        contadorFalhas = 0;
        modoLinha      = SEGUINDO;
        estadoAtual    = ESTADO_LINHA;
        Serial.println(F("[RESGATE] SAIDA detectada durante giro! Retornando ao PID."));
        return;
      }

      tcaselect(CANAL_GY521);
      mpu.update();
      float yawAtual   = mpu.getAngleZ();
      float anguloAlvo = anguloBaseGiroRes + 90.0; // Esquerda → soma 90°
      float erro       = anguloAlvo - yawAtual;

      if (abs(erro) > 3.0) {
        int velGiro = constrain(
          VELOCIDADE_RESGATE_GIRO + (int)(abs(erro) * 1.2f),
          VELOCIDADE_RESGATE_GIRO,
          VELOCIDADE_RESGATE_GIRO + 50
        );
        // Girar à ESQUERDA: direita para frente, esquerda para trás
        controlarRodas(velGiro, -velGiro);

        if (millis() - tempoInicioGiroRes > TIMEOUT_GIRO_RESGATE_MS) {
          Serial.println(F("[RESGATE-GIRO] TIMEOUT no giro esquerda! Forcando retomada."));
          controlarRodas(0, 0);
          anguloReferenciaReto = mpu.getAngleZ();
          tempoUltimoSonarRes  = millis();
          estadoResgate        = RES_NAVEGANDO;
        }
      } else {
        controlarRodas(0, 0);

        // Salva nova referência de linha reta após o giro
        anguloReferenciaReto = mpu.getAngleZ();

        Serial.print(F("[RESGATE-GIRO] Giro 90 ESQUERDA concluido. Nova ref. Yaw: "));
        Serial.println(anguloReferenciaReto);

        tempoUltimoSonarRes = millis();
        estadoResgate       = RES_NAVEGANDO;
      }
      break;
    }

    // --------------------------------------------------------------------------
    // SUB-ESTADO: RES_CONCLUIDO
    // Reservado para expansão futura (depositar vítimas, aguardar sinal, etc.)
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
