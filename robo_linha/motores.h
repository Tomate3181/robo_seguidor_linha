#ifndef MOTORES_H
#define MOTORES_H

#include "AFMotor_R4.h" // Usando a biblioteca do shield original do projeto
#include "config.h"

// Instanciação dos motores conforme os canais definidos no config.h
AF_DCMotor motorFL(MOTOR_FR_ESQ); // Frontal Esquerdo
AF_DCMotor motorFR(MOTOR_FR_DIR); // Frontal Direito
AF_DCMotor motorBL(MOTOR_TR_ESQ); // Traseiro Esquerdo
AF_DCMotor motorBR(MOTOR_TR_DIR); // Traseiro Direito

// ==============================================================================
// FUNÇÕES DE CONTROLE DOS MOTORES
// ==============================================================================

// Função unificada para controlar a velocidade e sentido de ambos os lados (como na programação antiga)
void controlarRodas(int vDir, int vEsq) {
  // --- LADO DIREITO (FR e BR) ---
  if (vDir >= 0) {
    int v = constrain(vDir, 0, VELOCIDADE_MAX);
    motorFR.setSpeed(v); motorBR.setSpeed(v);
    motorFR.run(FORWARD); motorBR.run(FORWARD);
  } else {
    int v = constrain(abs(vDir), 0, VELOCIDADE_MAX);
    motorFR.setSpeed(v); motorBR.setSpeed(v);
    motorFR.run(BACKWARD); motorBR.run(BACKWARD);
  }
  
  // --- LADO ESQUERDO (FL e BL) ---
  if (vEsq >= 0) {
    int v = constrain(vEsq, 0, VELOCIDADE_MAX);
    motorFL.setSpeed(v); motorBL.setSpeed(v);
    motorFL.run(FORWARD); motorBL.run(FORWARD);
  } else {
    int v = constrain(abs(vEsq), 0, VELOCIDADE_MAX);
    motorFL.setSpeed(v); motorBL.setSpeed(v);
    motorFL.run(BACKWARD); motorBL.run(BACKWARD);
  }
}


void initMotores() {
  motorFL.setSpeed(VELOCIDADE_BASE);
  motorFR.setSpeed(VELOCIDADE_BASE);
  motorBL.setSpeed(VELOCIDADE_BASE);
  motorBR.setSpeed(VELOCIDADE_BASE);
  
  motorFL.run(RELEASE);
  motorFR.run(RELEASE);
  motorBL.run(RELEASE);
  motorBR.run(RELEASE);
}

void pararMotores() {
  motorFL.run(RELEASE);
  motorFR.run(RELEASE);
  motorBL.run(RELEASE);
  motorBR.run(RELEASE);
}

// Função para rotacionar o robô no próprio eixo
void rotacionarEixo(bool sentidoHorario, int velocidade) {
  motorFL.setSpeed(velocidade);
  motorFR.setSpeed(velocidade);
  motorBL.setSpeed(velocidade);
  motorBR.setSpeed(velocidade);

  if (sentidoHorario) {
    // Para girar horário: Esquerda pra frente, Direita pra trás
    motorFL.run(FORWARD);
    motorBL.run(FORWARD);
    motorFR.run(BACKWARD);
    motorBR.run(BACKWARD);
  } else {
    // Para girar anti-horário: Esquerda pra trás, Direita pra frente
    motorFL.run(BACKWARD);
    motorBL.run(BACKWARD);
    motorFR.run(FORWARD);
    motorBR.run(FORWARD);
  }
}

#endif // MOTORES_H
