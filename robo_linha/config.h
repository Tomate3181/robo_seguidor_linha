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

// ==============================================================================
// DEFINIÇÕES DA MÁQUINA DE ESTADOS (FSM)
// ==============================================================================

enum EstadoRobo {
  ESTADO_CALIBRACAO,
  ESTADO_LINHA,
  ESTADO_VERDE,
  ESTADO_VERMELHO,
  ESTADO_OBSTACULO,
  ESTADO_RESGATE // Novo estado para a Zona de Resgate
};

// ==============================================================================
// PARÂMETROS E CONSTANTES DE CONTROLE (PID E MOVIMENTO)
// ==============================================================================
const float KP = 0.06; // Constante Proporcional (Suavizado para evitar viradas bruscas)
const float KD = 0.8;  // Constante Derivativa (Reduzido para não dar solavancos na leitura)
const float KI = 0.0;  // Constante Integral (Geralmente 0 para seguidor de linha)

const int VELOCIDADE_BASE = 120;
const int VELOCIDADE_MAX = 255;  
const int VELOCIDADE_GAP = 130;
const int TEMPO_PARA_12CM = 1000; // Tempo em ms para andar 12cm

// --- Parâmetros da Zona de Resgate (Wall-Following) ---
const int VELOCIDADE_RESGATE = 110;          // Velocidade base na zona de resgate
const int DISTANCIA_ALVO_PAREDE = 15;        // Distância alvo para a parede lateral (em cm)
const int DISTANCIA_OBSTACULO_FRENTE = 15;   // Distância limite para detectar parede frontal (em cm)
const int DISTANCIA_QUINA_PAREDE = 32;       // Distância a partir da qual a parede lateral sumiu (quina) (em cm)
const float KP_PAREDE = 4.5;                 // Ganho proporcional do seguidor de parede
const unsigned long TEMPO_AVANCO_QUINA = 400;  // Tempo para avançar após perder a parede para contornar a quina (em ms)
const unsigned long TEMPO_MINIMO_RESGATE = 5000; // Tempo mínimo na zona de resgate para evitar re-gatilho na entrada (em ms)

// ==============================================================================
// MODOS DE OPERAÇÃO DA LINHA (Sub-estados Não-Bloqueantes)
// ==============================================================================
enum ModoLinha {
  SEGUINDO,
  INSISTINDO,
  GAP_AVANCA,
  GAP_RE_AJUSTE
};

enum ModoObstaculo {
  GIRO_INICIAL,
  CONTORNO_LATERAL,
  BUSCA_LINHA
};

enum ModoResgate {
  RESGATE_ENTRANDO,         // Avanço inicial cego para cruzar a silver tape
  RESGATE_SEGUINDO_PAREDE,  // Seguidor de parede proporcional
  RESGATE_GIRANDO_ESQUERDA, // Giro de 90° à esquerda para desviar de parede frontal
  RESGATE_GIRANDO_DIREITA   // Giro de 90° à direita para contornar quina externa
};

#endif // CONFIG_H
