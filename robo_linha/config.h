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
#define CANAL_OLED     0
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

// ==============================================================================
// DEFINIÇÕES DA MÁQUINA DE ESTADOS (FSM)
// ==============================================================================

enum EstadoRobo {
  ESTADO_CALIBRACAO,
  ESTADO_LINHA,
  ESTADO_VERDE,
  ESTADO_VERMELHO,
  ESTADO_OBSTACULO
};

// ==============================================================================
// PARÂMETROS E CONSTANTES DE CONTROLE (PID E MOVIMENTO)
// ==============================================================================
const float KP = 0.08; // Constante Proporcional (Suavizado para evitar viradas bruscas)
const float KD = 0.5;  // Constante Derivativa (Reduzido para não dar solavancos na leitura)
const float KI = 0.0;  // Constante Integral (Geralmente 0 para seguidor de linha)

const int VELOCIDADE_BASE = 150; 
const int VELOCIDADE_MAX = 255;  
const int VELOCIDADE_GAP = 130;
const int TEMPO_PARA_12CM = 1250; // Tempo em ms para andar 12cm

// ==============================================================================
// MODOS DE OPERAÇÃO DA LINHA (Sub-estados Não-Bloqueantes)
// ==============================================================================
enum ModoLinha {
  SEGUINDO,
  INSISTINDO,
  GAP_AVANCA,
  GAP_RE_AJUSTE
};

#endif // CONFIG_H
