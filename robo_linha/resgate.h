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
const int           VELOCIDADE_TESTE_SILVER   = 110;   // PWM de avanço durante os 250ms de validação
const unsigned long TEMPO_VALIDACAO_SILVER     = 250;  // Janela de confirmação em ms

// --- Filtro do Cruzamento em Cruz ---
// Ao entrar em ZONA_RESGATE, o robô avança um pouco para sair completamente
// da fita prata e checar se havia uma linha preta por baixo (cruzamento em +)
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
// VARIÁVEIS DE ESTADO DO MÓDULO (escopo de arquivo — não poluem o global)
// ==============================================================================

static EstadoResgate estadoResgate         = RES_AGUARDANDO_INICIO;

// Validação da Silver Tape
static unsigned long tempoInicioValidacao  = 0;

// -----------------------------------------------------------------------
// Aborto por Verde: variáveis da sub-FSM não-bloqueante
// -----------------------------------------------------------------------
static EstadoAbortoPorVerde estadoAborto   = ABORTO_INATIVO;

// Duração da ré de reposicionamento.
// O robô avança a VELOCIDADE_TESTE_SILVER durante TEMPO_VALIDACAO_SILVER ms
// no máximo. Usamos o tempo real que passou para calcular quanto recuar,
// garantindo que ele volte aproximadamente para onde estava na borda do verde.
static unsigned long tempoAvancadoMs       = 0; // ms efetivamente avançados antes do verde
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
 * Salva o timestamp e transita para ESTADO_VALIDANDO_SILVER_TAPE.
 */
inline void iniciarValidacaoSilverTape() {
  tempoInicioValidacao = millis();
  estadoAtual = ESTADO_VALIDANDO_SILVER_TAPE;
  Serial.println(F("[RESGATE] Possivel Silver Tape detectada. Iniciando validacao..."));
}

/*
 * executarValidacaoSilverTape()
 * Chamada a cada ciclo do loop() enquanto o robô está em ESTADO_VALIDANDO_SILVER_TAPE.
 *
 * LÓGICA:
 *   1. Mantém o robô avançando em linha reta (VELOCIDADE_TESTE_SILVER).
 *      Registra o tempo real de avanço para poder compensá-lo se houver aborto.
 *   2. A cada ciclo, monitora os sensores de cor RGB — sem bloquear o loop.
 *      Se VERDE for detectado: entra na sub-FSM de aborto (EstadoAbortoPorVerde)
 *      que executa de forma não-bloqueante:
 *        Fase ABORTO_RECUANDO       → ré proporcional ao avanço feito
 *        Fase ABORTO_LENDO_COR      → pausa de estabilização óptica (60ms)
 *        Fase ABORTO_PREPARANDO_GIRO → mini-avanço de alinhamento + captura do
 *                                       Yaw + seta tipoGiro → ESTADO_VERDE
 *   3. Após TEMPO_VALIDACAO_SILVER ms sem verde:
 *      - Todos os 8 IR cravados em 1000 → ESTADO_ZONA_RESGATE
 *      - Caso contrário → falso positivo → ESTADO_LINHA
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
          // Recua na mesma velocidade que avançou para espelhar o deslocamento
          controlarRodas(-VELOCIDADE_TESTE_SILVER, -VELOCIDADE_TESTE_SILVER);
        } else {
          // Ré concluída — para e parte para a leitura
          controlarRodas(0, 0);
          tempoInicioPausaCor = millis();
          estadoAborto        = ABORTO_LENDO_COR;
          Serial.println(F("[RESGATE-ABORTO] Re concluida. Aguardando estabilizacao optica..."));
        }
        break;

      // ----------------------------------------------------------
      // Fase 2: Pausa para estabilização óptica (chassis parado)
      // Equivalente ao delay(80) de avaliarInterseccao().
      // ----------------------------------------------------------
      case ABORTO_LENDO_COR: {
        controlarRodas(0, 0); // Garante que está parado

        if (millis() - tempoInicioPausaCor >= PAUSA_ESTABILIZACAO_COR_MS) {
          // Lê os dois sensores TCS
          uint16_t rD, gD, bD, cD;
          uint16_t rE, gE, bE, cE;

          tcaselect(CANAL_TCS_DIR);
          tcsDir.getRawData(&rD, &gD, &bD, &cD);
          tcaselect(CANAL_TCS_ESQ);
          tcsEsq.getRawData(&rE, &gE, &bE, &cE);

          bool verdeDir = ehVerde(rD, gD, bD, cD, limiarLuminosidadeDir);
          bool verdeEsq = ehVerde(rE, gE, bE, cE, limiarLuminosidadeEsq);

          if (verdeDir || verdeEsq) {
            // Verde confirmado após recuo: determina o tipo de giro
            if      (verdeDir && verdeEsq) tipoGiro =  180;
            else if (verdeDir)             tipoGiro =   65;
            else                           tipoGiro =  -65;

            Serial.print(F("[RESGATE-ABORTO] Verde confirmado. Tipo de giro: "));
            Serial.println(tipoGiro);

            // Inicia o mini-avanço de alinhamento do eixo de rodas com o cruzamento
            // (equivalente ao controlarRodas(100,100) + delay(150) de avaliarInterseccao)
            controlarRodas(VEL_MINI_AVANCO, VEL_MINI_AVANCO);
            tempoInicioMiniAvanco = millis();
            estadoAborto          = ABORTO_PREPARANDO_GIRO;

          } else {
            // Cor sumiu após a ré (reflexo ou ruído): retorna para linha normalmente
            Serial.println(F("[RESGATE-ABORTO] Cor nao confirmada apos recuo. Retornando para linha."));
            tempoInicioValidacao = 0; // Reseta timestamp para não re-disparar
            estadoAborto         = ABORTO_INATIVO;
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
      // Equivalente ao delay(150) + pararMotores() + delay(50)
      // + mpu.update() + anguloInicial = ... de avaliarInterseccao().
      // ----------------------------------------------------------
      case ABORTO_PREPARANDO_GIRO:
        if (millis() - tempoInicioMiniAvanco < TEMPO_MINI_AVANCO_MS) {
          controlarRodas(VEL_MINI_AVANCO, VEL_MINI_AVANCO);
        } else {
          // Alinhado — para, captura o Yaw e dispara o giro
          controlarRodas(0, 0);

          // Pequena pausa passiva (não-bloqueante: o próximo ciclo já lê o Yaw)
          // O MPU já é atualizado no topo do loop(), mas forçamos aqui para precisão
          tcaselect(CANAL_GY521);
          mpu.update();
          anguloInicial = mpu.getAngleZ();

          // Bloqueia re-detecção de cor por 1.5 s (igual ao ESTADO_VERDE do .ino)
          ultimaLeituraCor = millis() + 1500;

          // Reseta variáveis da validação antes de sair
          tempoInicioValidacao = 0;
          estadoAborto         = ABORTO_INATIVO;

          // Transita para o giro — mesmo estado que avaliarInterseccao() usaria
          estadoAtual = ESTADO_VERDE;
          Serial.println(F("[RESGATE-ABORTO] Alinhado. Disparando ESTADO_VERDE."));
        }
        break;

      default:
        estadoAborto = ABORTO_INATIVO;
        break;
    }

    return; // Enquanto o aborto está em andamento, não executa o restante da função
  }

  // ============================================================
  // CAMINHO A: avanço normal de validação (aborto não ativo)
  // ============================================================

  // Passo 1: registra quanto tempo o robô efetivamente avançou
  // (usado para calcular a ré proporcional caso verde seja detectado)
  tempoAvancadoMs = millis() - tempoInicioValidacao;

  // Mantém avanço suave durante a janela de validação
  controlarRodas(VELOCIDADE_TESTE_SILVER, VELOCIDADE_TESTE_SILVER);

  // Passo 2: FILTRO CRÍTICO — monitora cor a cada ciclo
  if (lerSensoresCor_verdeDetectado()) {
    Serial.print(F("[RESGATE-ABORTO] Verde detectado apos "));
    Serial.print(tempoAvancadoMs);
    Serial.println(F("ms de avanco. Iniciando re proporcional..."));

    // Para imediatamente
    controlarRodas(0, 0);

    // Calcula quanto tempo de ré é necessário para compensar o avanço.
    // Usa 90% do tempo avançado (margem de segurança: motores têm inércia).
    duracaoReAborto    = (tempoAvancadoMs * 90UL) / 100UL;

    // Garante um mínimo de ré para não ficar estático se o avanço foi mínimo
    if (duracaoReAborto < 40) duracaoReAborto = 40;

    tempoInicioAbortoRe  = millis();
    estadoAborto         = ABORTO_RECUANDO;
    // Não muda estadoAtual aqui: permanece em ESTADO_VALIDANDO_SILVER_TAPE
    // até o aborto ser concluído e disparar ESTADO_VERDE ou ESTADO_LINHA.
    return;
  }

  // Passo 3: Checagem final após a janela de 250ms
  if (millis() - tempoInicioValidacao >= TEMPO_VALIDACAO_SILVER) {

    // Força nova leitura dos sensores IR para decisão precisa
    qtr.readLineBlack(sensorValues);
    int cravados = contarSensoresCravados1000();

    if (cravados == NUM_SENSORES_IR) {
      // TODOS os 8 sensores cravados: Silver Tape confirmada!
      pararMotores();
      tempoInicioValidacao = 0; // Reseta para não re-disparar
      estadoResgate = RES_FILTRO_CRUZ;
      estadoAtual   = ESTADO_ZONA_RESGATE;
      Serial.println(F("[RESGATE] Silver Tape CONFIRMADA! Entrando na Zona de Resgate."));
    } else {
      // Nem todos cravados: era intersecção comum ou linha preta grossa
      tempoInicioValidacao = 0;
      estadoAtual = ESTADO_LINHA;
      Serial.print(F("[RESGATE] Falso positivo (apenas "));
      Serial.print(cravados);
      Serial.println(F(" de 8 cravados). Retornando para linha."));
    }
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
