#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==============================================================================
// MAPEAMENTO DE HARDWARE (Arduino Mega 2560)
// ==============================================================================

// --- Sensores IR (Barra QTR-8) ---
#define NUM_SENSORES_IR 8
const uint8_t PINOS_IR[NUM_SENSORES_IR] = {24, 25, 26, 27, 28, 29, 30, 31};

// --- Endereço do Multiplexador I2C (TCA9548A) ---
#define TCA_ADDR 0x70

// Canais do Multiplexador
#define CANAL_GY521    0
#define CANAL_TCS_DIR  1
#define CANAL_TCS_ESQ  2
#define CANAL_OLED     3

// --- Motores (Shield - M1, M2, M3, M4) ---
// Identificação padrão para a maioria das bibliotecas de Shield (como AFMotor)
#define MOTOR_FR_ESQ 1
#define MOTOR_FR_DIR 2
#define MOTOR_TR_ESQ 3
#define MOTOR_TR_DIR 4

// --- Sensor Ultrassônico ---
// Pinos sugeridos, a definir/alterar conforme o circuito físico
#define PINO_TRIG 22
#define PINO_ECHO 23

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
// PARÂMETROS E CONSTANTES DE CONTROLE (PID)
// ==============================================================================
const float KP = 1.0;  // Constante Proporcional (Ajustar em testes)
const float KI = 0.0;  // Constante Integral (Ajustar em testes)
const float KD = 0.0;  // Constante Derivativa (Ajustar em testes)

const int VELOCIDADE_BASE = 150; // Velocidade nominal do robô (0-255)
const int VELOCIDADE_MAX = 255;  // Velocidade máxima permitida

#endif // CONFIG_H
