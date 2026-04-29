#ifndef MOTORES_H
#define MOTORES_H

#include <AFMotor.h> // Usando a biblioteca padrão de Motor Shield (L293D) como base
#include "config.h"

// Instanciação dos motores conforme os canais definidos no config.h
AF_DCMotor motorFL(MOTOR_FR_ESQ); // Frontal Esquerdo
AF_DCMotor motorFR(MOTOR_FR_DIR); // Frontal Direito
AF_DCMotor motorBL(MOTOR_TR_ESQ); // Traseiro Esquerdo
AF_DCMotor motorBR(MOTOR_TR_DIR); // Traseiro Direito

// ==============================================================================
// FUNÇÕES DE CONTROLE DOS MOTORES
// ==============================================================================

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
