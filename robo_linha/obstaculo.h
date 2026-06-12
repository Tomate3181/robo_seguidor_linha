#ifndef OBSTACULO_H
#define OBSTACULO_H

/*
 * ==============================================================================
 * MÓDULO: obstaculo.h
 * DESCRIÇÃO: Implementa a lógica completa de desvio de obstáculo por contorno
 *            circular/suave, integrada à FSM principal via ModoObstaculo.
 *
 * FÍSICA DO CONTORNO CIRCULAR:
 * O algoritmo segue a estratégia "wall-following" adaptada:
 *   1. RÉ: Cria espaço entre o robô e o objeto antes de girar, evitando colisão
 *          durante a rotação (o arco descrito pela traseira pode tocar o objeto).
 *   2. GIRO DE 65°: Afasta a proa do robô da trajetória do obstáculo. O ângulo
 *          de 65° foi escolhido para que, ao iniciar o arco, o robô já esteja
 *          tangenciando o contorno lateral do objeto sem precisar de correção
 *          adicional. O Yaw do MPU6050 (eixo Z) é usado como encoder angular
 *          absoluto — imune a derrapagem de rodas.
 *   3. ARCO DE CONTORNO: O robô descreve uma curva com raio diferencial
 *          (roda interna mais lenta). O sensor ultrassônico lateral mede a
 *          distância ao objeto e serve como malha de feedback para manter o
 *          robô a uma distância segura (~15 cm) durante o contorno.
 *   4. RECUPERAÇÃO: Quando os sensores IR centrais voltam a enxergar a faixa
 *          preta, o objeto já foi contornado e o robô pode retomar o PID.
 *
 * REGRAS DE OURO:
 *   - Proibido delay(). Toda temporização usa millis().
 *   - Não redeclara pinos nem objetos de hardware.
 *   - Reutiliza controlarRodas(), obterDistanciaFiltrada(), qtr, sonarDir,
 *     sonarEsq e mpu definidos em motores.h e sensores.h.
 * ==============================================================================
 */

#include <Arduino.h>
#include "config.h"

// -----------------------------------------------------------------------
// Referências a objetos e variáveis declarados nos outros módulos.
// Não declaramos nada de hardware aqui — apenas apontamos para o que existe.
// -----------------------------------------------------------------------
extern void   controlarRodas(int vDir, int vEsq);
extern void   pararMotores();
extern void   tcaselect(uint8_t i);

// Objetos de hardware (declarados em sensores.h)
#include "sensores.h"   // traz qtr, mpu, sonarDir, sonarEsq, obterDistanciaFiltrada()

// Variável global da FSM (declarada em robo_linha.ino)
extern EstadoRobo estadoAtual;
extern ModoLinha  modoLinha;
extern int        ultimoErro;
extern int        contadorFalhas;

// -----------------------------------------------------------------------
// CONSTANTES DO DESVIO — ajuste conforme o comportamento real na pista
// -----------------------------------------------------------------------

// Ré inicial: velocidade e duração máxima em ms (segurança de timeout)
const int     VEL_RE_OBSTACULO          = 100;  // PWM da ré (positivo; a direção é invertida em controlarRodas)
const unsigned long DURACAO_RE_MS       = 350;  // Tempo de ré (~5 cm a baixa velocidade)

// Giro de 65° — validado pelo Yaw do MPU6050
const float   ANGULO_GIRO_DESVIO        = 65.0; // Graus a girar após confirmar obstáculo
const int     VEL_GIRO_OBSTACULO        = 120;  // PWM do giro no eixo
const unsigned long TIMEOUT_GIRO_MS     = 3000; // Timeout de segurança: 3 s

// Arco de contorno circular
const int     VEL_ARCO_EXTERNO          = 130;  // Roda do lado externo ao objeto (mais rápida)
const int     VEL_ARCO_INTERNO          = 55;   // Roda do lado interno — cria a curvatura do arco
const float   DIST_LATERAL_ALVO_CM      = 12.0; // Distância de seguimento lateral ao objeto (cm)
const float   DIST_LATERAL_TOLERANCIA   = 4.0;  // Banda morta: ±4 cm antes de corrigir
const unsigned long TIMEOUT_CONTORNO_MS = 8000; // Timeout de segurança do contorno: 8 s

// Detecção de linha na recuperação
const int     LIMIAR_IR_LINHA           = 600;  // Valor mínimo do QTR para considerar "linha preta"
const int     SENSORES_MIN_LINHA        = 2;    // Quantos sensores precisam ver a linha para confirmar
const unsigned long TIMEOUT_BUSCA_MS    = 6000; // Timeout de segurança na busca da linha: 6 s

// Antiruído: quantas leituras consecutivas abaixo de 10 cm para confirmar obstáculo
const int     LEITURAS_CONSECUTIVAS_OBS = 3;

// -----------------------------------------------------------------------
// VARIÁVEIS DE ESTADO DO MÓDULO (escopo de arquivo — não poluem o global)
// -----------------------------------------------------------------------

// Lado para o qual o robô vai girar/contornar
// -1 = Esquerda, +1 = Direita
// A função iniciarDesvioObstaculo() escolhe o lado desimpedido.
static int    ladoDesvio                = 1;  // Padrão: direita

// Ângulo inicial do Yaw no momento em que o giro começa
static float  anguloBaseGiro            = 0.0;

// Marcadores de tempo para cada fase
static unsigned long tempoInicioRe      = 0;
static unsigned long tempoInicioGiro    = 0;
static unsigned long tempoInicioArco    = 0;
static unsigned long tempoBuscaLinha    = 0;

// Último instante em que o sonar lateral foi lido (throttle de 50 ms)
static unsigned long ultimaLeituraSonar = 0;

// -----------------------------------------------------------------------
// FUNÇÕES PÚBLICAS DO MÓDULO
// -----------------------------------------------------------------------

/*
 * iniciarDesvioObstaculo()
 * Chamada UMA VEZ pelo loop() ao confirmar o obstáculo.
 * Determina o lado desimpedido, registra o ângulo atual do Yaw e
 * transita para OBSTACULO_RE — a primeira fase do desvio.
 *
 * Convenção do MPU6050 neste projeto (confirmada no ESTADO_VERDE do .ino):
 *   Giro para a ESQUERDA  → Yaw AUMENTA (positivo)
 *   Giro para a DIREITA   → Yaw DIMINUI (negativo)
 */
void iniciarDesvioObstaculo() {
    // Para imediatamente antes de qualquer manobra
    controlarRodas(0, 0);

    // Lê os sonares laterais para escolher o lado com mais espaço
    int distEsq = obterDistanciaFiltrada(sonarEsq);
    int distDir = obterDistanciaFiltrada(sonarDir);

    // Escolhe o lado desimpedido
    ladoDesvio = (distDir >= distEsq) ? 1 : -1;

    Serial.print(F("[OBSTACULO] Lado escolhido: "));
    Serial.println(ladoDesvio == 1 ? F("DIREITA") : F("ESQUERDA"));
    Serial.print(F("[OBSTACULO] Dist. Esq: ")); Serial.print(distEsq);
    Serial.print(F(" cm | Dist. Dir: ")); Serial.print(distDir); Serial.println(F(" cm"));

    // Captura o Yaw atual como base para o giro de 65°
    tcaselect(CANAL_GY521);
    mpu.update();
    anguloBaseGiro = mpu.getAngleZ();

    Serial.print(F("[OBSTACULO] Yaw base de giro: "));
    Serial.println(anguloBaseGiro);

    // Registra o instante de início da ré
    tempoInicioRe = millis();

    // Transita para a primeira fase do desvio
    estadoAtual = ESTADO_OBSTACULO_RE;

    Serial.println(F("[OBSTACULO] Iniciando RE..."));
}

/*
 * executarRe()
 * Fase 1 do desvio: recua o robô por DURACAO_RE_MS milissegundos.
 * Propósito físico: abrir espaço entre o casco do robô e o objeto para
 * que o giro de 65° não cause colisão com a traseira durante a rotação.
 */
void executarRe() {
    // Mantém a ré ativa enquanto o tempo não esgotou
    if (millis() - tempoInicioRe < DURACAO_RE_MS) {
        controlarRodas(-VEL_RE_OBSTACULO, -VEL_RE_OBSTACULO);
    } else {
        // Ré concluída — para os motores e prepara o giro
        controlarRodas(0, 0);

        // Registra Yaw novamente após a ré (o recuo pode gerar leve desvio angular)
        tcaselect(CANAL_GY521);
        mpu.update();
        anguloBaseGiro = mpu.getAngleZ();

        tempoInicioGiro = millis();

        Serial.println(F("[OBSTACULO] RE concluida. Iniciando GIRO de 65 graus..."));
        estadoAtual = ESTADO_OBSTACULO_GIRANDO;
    }
}

/*
 * executarGiro65()
 * Fase 2 do desvio: gira o robô exatamente 65° usando o Yaw do MPU6050.
 *
 * MATEMÁTICA DO GIRO:
 *   - ladoDesvio == +1 (Direita): o Yaw DIMINUI → anguloAlvo = base - 65°
 *   - ladoDesvio == -1 (Esquerda): o Yaw AUMENTA → anguloAlvo = base + 65°
 *
 * O controle é do tipo "bang-bang" com velocidade proporcional ao erro
 * angular (idêntico ao ESTADO_VERDE do projeto), garantindo frenagem suave
 * ao se aproximar do alvo e evitando ultrapassagem por inércia.
 */
void executarGiro65() {
    // Atualiza o Yaw — já foi feito no topo do loop(), mas chamamos novamente
    // para máxima precisão nesta fase crítica.
    tcaselect(CANAL_GY521);
    mpu.update();
    float yawAtual = mpu.getAngleZ();

    // Calcula o ângulo alvo conforme o lado escolhido
    float anguloAlvo = anguloBaseGiro + (ladoDesvio * ANGULO_GIRO_DESVIO);

    // Erro angular (positivo = precisa girar para a esquerda, negativo = direita)
    float erroAngulo = anguloAlvo - yawAtual;

    // Tolerância de ±3° (igual ao ESTADO_VERDE — garante parada precisa)
    if (abs(erroAngulo) > 3.0) {
        // Velocidade proporcional ao erro (mesmo padrão do ESTADO_VERDE)
        int velGiro = 80 + (int)(abs(erroAngulo) * 1.5);
        if (velGiro > 160) velGiro = 160; // Limita inércia excessiva

        if (erroAngulo > 0) {
            // Girar para a ESQUERDA: roda esquerda para trás, direita para frente
            controlarRodas(velGiro, -velGiro);
        } else {
            // Girar para a DIREITA: roda direita para trás, esquerda para frente
            controlarRodas(-velGiro, velGiro);
        }

        // Timeout de segurança
        if (millis() - tempoInicioGiro > TIMEOUT_GIRO_MS) {
            Serial.println(F("[OBSTACULO] TIMEOUT no giro! Forçando contorno..."));
            controlarRodas(0, 0);
            tempoInicioArco = millis();
            estadoAtual = ESTADO_OBSTACULO_CONTORNO;
        }
    } else {
        // Atingiu os 65° com precisão
        controlarRodas(0, 0);

        Serial.print(F("[OBSTACULO] Giro concluido! Yaw final: "));
        Serial.println(yawAtual);

        tempoInicioArco = millis();
        ultimaLeituraSonar = millis();

        Serial.println(F("[OBSTACULO] Iniciando CONTORNO em arco..."));
        estadoAtual = ESTADO_OBSTACULO_CONTORNO;
    }
}

/*
 * executarContornoArco()
 * Fase 3 do desvio: contorno circular com malha fechada usando sonar lateral.
 *
 * FÍSICA DO ARCO DIFERENCIAL:
 *   O robô descreve uma curva ao aplicar velocidades diferentes nas rodas:
 *   - Roda externa (lado oposto ao objeto): VEL_ARCO_EXTERNO (mais rápida)
 *   - Roda interna (lado do objeto): VEL_ARCO_INTERNO (mais lenta)
 *   O raio do arco é proporcional à diferença de velocidades.
 *
 * MALHA DE FEEDBACK LATERAL:
 *   O sonar do lado do objeto (ladoDesvio) mede a distância ao objeto.
 *   - Se distância > alvo + tolerância: objeto sumindo → curva mais fechada
 *     (diminui roda interna para girar mais em direção ao objeto)
 *   - Se distância < alvo - tolerância: muito perto → curva mais aberta
 *     (aumenta roda interna para se afastar)
 *   - Se dentro da tolerância: mantém arco nominal
 *
 * CONDIÇÃO DE SAÍDA:
 *   Os sensores IR centrais (posições 3, 4, 5, 6 da barra QTR-8) detectam
 *   a linha preta novamente → o obstáculo foi contornado.
 */
void executarContornoArco() {
    // Throttle do sonar: não ultrapassa a frequência máxima do HC-SR04 (~20 Hz)
    if (millis() - ultimaLeituraSonar > 50) {
        ultimaLeituraSonar = millis();

        // Lê o sonar do lado onde o objeto está
        int distLateral;
        if (ladoDesvio == 1) {
            distLateral = obterDistanciaFiltrada(sonarDir);
        } else {
            distLateral = obterDistanciaFiltrada(sonarEsq);
        }

        // Correção da curvatura baseada na distância lateral ao objeto
        int velInterno = VEL_ARCO_INTERNO; // valor base da roda interna

        float erroLateral = distLateral - DIST_LATERAL_ALVO_CM;

        if (erroLateral > DIST_LATERAL_TOLERANCIA) {
            // Muito longe do objeto: fecha mais o arco (diminui roda interna)
            // Isso faz o robô curvar mais em direção ao objeto
            velInterno = VEL_ARCO_INTERNO - 30;
            if (velInterno < 0) velInterno = 0;
        } else if (erroLateral < -DIST_LATERAL_TOLERANCIA) {
            // Muito perto do objeto: abre o arco (aumenta roda interna)
            // Isso faz o robô se afastar levemente do objeto
            velInterno = VEL_ARCO_INTERNO + 30;
            if (velInterno > VEL_ARCO_EXTERNO) velInterno = VEL_ARCO_EXTERNO;
        }

        // Aplica velocidades diferenciais conforme o lado do contorno
        if (ladoDesvio == 1) {
            // Contornando pela DIREITA: objeto à direita
            // Roda esquerda (externa) mais rápida; roda direita (interna) mais lenta
            controlarRodas(velInterno, VEL_ARCO_EXTERNO);
        } else {
            // Contornando pela ESQUERDA: objeto à esquerda
            // Roda direita (externa) mais rápida; roda esquerda (interna) mais lenta
            controlarRodas(VEL_ARCO_EXTERNO, velInterno);
        }
    }

    // -----------------------------------------------------------------------
    // VERIFICAÇÃO DE RECUPERAÇÃO: sensores IR centrais vendo linha preta
    // -----------------------------------------------------------------------
    qtr.readLineBlack(sensorValues);

    int sensoresVendoLinha = 0;
    // Avalia apenas os 4 sensores centrais (índices 2, 3, 4, 5 na barra de 8)
    // para evitar falsos positivos nas bordas durante a curva
    for (uint8_t i = 2; i <= 5; i++) {
        if (sensorValues[i] >= LIMIAR_IR_LINHA) {
            sensoresVendoLinha++;
        }
    }

    if (sensoresVendoLinha >= SENSORES_MIN_LINHA) {
        // Linha encontrada! Obstáculo contornado com sucesso.
        controlarRodas(0, 0);

        // Zera variáveis do PID para não acumular erro da fase de desvio
        ultimoErro    = 0;
        contadorFalhas = 0;
        modoLinha     = SEGUINDO;

        Serial.println(F("[OBSTACULO] Linha reencontrada! Retornando ao PID."));
        estadoAtual = ESTADO_LINHA;
        return;
    }

    // -----------------------------------------------------------------------
    // TIMEOUT DE SEGURANÇA do contorno
    // -----------------------------------------------------------------------
    if (millis() - tempoInicioArco > TIMEOUT_CONTORNO_MS) {
        Serial.println(F("[OBSTACULO] TIMEOUT no contorno! Iniciando busca de linha..."));
        controlarRodas(0, 0);
        tempoBuscaLinha = millis();
        estadoAtual = ESTADO_OBSTACULO_BUSCA;
    }
}

/*
 * executarBuscaLinha()
 * Fase 4 (contingência): se o timeout do contorno expirou sem encontrar a
 * linha, o robô faz uma curva suave em direção ao centro da pista até o
 * QTR-8A encontrar a faixa preta novamente.
 */
void executarBuscaLinha() {
    // Curva suave em direção ao lado oposto ao objeto (busca a linha)
    if (ladoDesvio == 1) {
        // Estava contornando pela direita → curva suave para a esquerda
        controlarRodas(VEL_ARCO_EXTERNO, VEL_ARCO_INTERNO);
    } else {
        // Estava contornando pela esquerda → curva suave para a direita
        controlarRodas(VEL_ARCO_INTERNO, VEL_ARCO_EXTERNO);
    }

    // Verifica se qualquer sensor encontrou a linha
    qtr.readLineBlack(sensorValues);
    bool encontrouLinha = false;
    for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
        if (sensorValues[i] >= LIMIAR_IR_LINHA) {
            encontrouLinha = true;
            break;
        }
    }

    if (encontrouLinha) {
        controlarRodas(0, 0);
        ultimoErro    = 0;
        contadorFalhas = 0;
        modoLinha     = SEGUINDO;

        Serial.println(F("[OBSTACULO] Linha reencontrada na busca. Retornando ao PID."));
        estadoAtual = ESTADO_LINHA;
        return;
    }

    // Timeout final de segurança — retorna ao ESTADO_LINHA de qualquer forma
    if (millis() - tempoBuscaLinha > TIMEOUT_BUSCA_MS) {
        Serial.println(F("[OBSTACULO] TIMEOUT na busca. Forçando retorno ao ESTADO_LINHA."));
        controlarRodas(0, 0);
        ultimoErro    = 0;
        contadorFalhas = 0;
        modoLinha     = SEGUINDO;
        estadoAtual = ESTADO_LINHA;
    }
}

#endif // OBSTACULO_H
