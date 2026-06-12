#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==============================================================================
// MAPEAMENTO DE HARDWARE (Arduino Mega 2560)
// ==============================================================================

// --- Sensores IR (Barra QRE-8D) ---
#define NUM_SENSORES_IR 8
const uint8_t PINOS_IR[NUM_SENSORES_IR] = {30, 31, 32, 33, 34, 35, 36, 37};

// --- Endereço do Multiplexador I2C (TCA9548A) ---
#define TCA_ADDR 0x70

// Canais do Multiplexador

#define CANAL_GY521    1
#define CANAL_TCS_DIR  2
#define CANAL_TCS_ESQ  3

// --- Motores (Shield - M1, M2, M3, M4) ---
// Mapeamento extraído da programação antiga (seguirLinha_v2.0.ino)
#define MOTOR_TR_DIR 2  // Antigo m_tras_dir(1)
#define MOTOR_TR_ESQ 1  // Antigo m_tras_esq(2)
#define MOTOR_FR_DIR 3  // Antigo m_frente_dir(3)
#define MOTOR_FR_ESQ 4  // Antigo m_frente_esq(4)

// --- Sensores Ultrassônicos (HC-SR04) ---
#define PINO_TRIG_FRENTE 48
#define PINO_ECHO_FRENTE 49

#define PINO_TRIG_ESQ 50
#define PINO_ECHO_ESQ 51

#define PINO_TRIG_DIR 52
#define PINO_ECHO_DIR 53

#define MAX_DISTANCE 60 // Distância máxima para ping (em cm). 60cm ~ 3.5ms de timeout.

// Botão para avançar a calibração
#define PINO_BOTAO 23

// Botão para resetar a programação (voltar ao estado original)
#define PINO_BOTAO_RESET 42

// ==============================================================================
// DEFINIÇÕES DA MÁQUINA DE ESTADOS (FSM)
// ==============================================================================

enum EstadoRobo {
  ESTADO_CALIBRACAO,
  ESTADO_LINHA,
  ESTADO_VALIDANDO_SILVER_TAPE,
  ESTADO_ZONA_RESGATE,
  ESTADO_VERDE,
  ESTADO_VERMELHO,

  // -----------------------------------------------------------------------
  // Estados do desvio de obstáculo (contorno circular)
  // Cada estado representa uma fase distinta da manobra — veja obstaculo.h
  // -----------------------------------------------------------------------
  ESTADO_OBSTACULO_RE,       // Fase 1: Ré para abrir espaço antes do giro
  ESTADO_OBSTACULO_GIRANDO,  // Fase 2: Giro de 65° validado pelo Yaw do MPU6050
  ESTADO_OBSTACULO_CONTORNO, // Fase 3: Arco de contorno com malha fechada (sonar lateral)
  ESTADO_OBSTACULO_BUSCA     // Fase 4: Busca da linha após timeout do contorno (contingência)
};

// ==============================================================================
// PARÂMETROS E CONSTANTES DE CONTROLE (PID E MOVIMENTO)
// ==============================================================================
const float KP = 0.15; // Constante Proporcional (Suavizado para evitar viradas bruscas)
const float KD = 2.5;  // Constante Derivativa (Reduzido para não dar solavancos na leitura)
const float KI = 0.0;  // Constante Integral (Geralmente 0 para seguidor de linha)

const int VELOCIDADE_BASE = 120;
const int VELOCIDADE_MAX = 255;
const int VELOCIDADE_GAP = 130;
const int TEMPO_PARA_12CM = 1200; // Tempo em ms para andar 12cm

// ==============================================================================
// CALIBRAÇÃO DE LUMINOSIDADE DA SILVER TAPE (Sensores RGB TCS34725)
// ==============================================================================
// Configuração de hardware: GAIN_4X, INTEGRATIONTIME_24MS, sensor a ~3mm do chão.
//
// Valores medidos (referência para ajuste dos thresholds abaixo):
//   Preto:        ESQ C~282,  DIR C~211
//   Silver Tape:  ESQ C~1297, DIR C~829
//   Verde:        ESQ C~721,  DIR C~501
//   Vermelho:     ESQ C~715,  DIR C~487
//   Branco:       ESQ C~4118, DIR C~2605
//
// Definições concretas ficam em sensores.h (lumCinzaEsqCalibrado / lumCinzaDirCalibrado).
//
// LÓGICA DE FUSÃO DA SILVER TAPE (detalhada em resgate.h):
//   Aceita quando AMBOS os sensores satisfazem:
//     Clear >= lumCalibrado * FATOR_MIN_SILVER      (não é preto: prata ~4.6x acima do preto)
//     Clear <= lumCalibrado * FATOR_MAX_SILVER      (não é branco: prata ~3.2x abaixo do branco)
// ==============================================================================

// Fração mínima do Clear calibrado para aceitar como prata (exclui preto e sujeira)
// Prata ~1297, preto ~282 → razão 4.6x. Threshold 0.45 → aceita a partir de ~584.
// Margem generosa para variação de luz ambiente (±30%).
const float FATOR_MIN_SILVER = 0.45f;

// Fração máxima do Clear calibrado para aceitar como prata (exclui branco puro)
// Prata ~1297, branco ~4118 → razão 3.2x. Threshold 2.2 → rejeita acima de ~2853.
// Protege contra sensor sobre o branco do piso da arena.
const float FATOR_MAX_SILVER = 2.2f;

// Tempo (ms) de avanço para posicionar os sensores RGB sobre a fita antes da leitura
const unsigned long TEMPO_POSICIONA_RGB = 200;

// Mantido por compatibilidade (não usado na lógica nova de fusão)
const int TOLERANCIA_CINZA = 60;



// ==============================================================================
// MODOS DE OPERAÇÃO DA LINHA (Sub-estados Não-Bloqueantes)
// ==============================================================================
enum ModoLinha {
  SEGUINDO,
  INSISTINDO,
  GAP_AVANCA,
  GAP_RE_AJUSTE
};

// ModoObstaculo removido: a lógica de desvio foi promovida a estados
// independentes da FSM principal (ESTADO_OBSTACULO_RE, _GIRANDO, etc.)
// Toda a implementação está encapsulada em obstaculo.h.

#endif // CONFIG_H
