#ifndef SENSORES_H
#define SENSORES_H

#include <QTRSensors.h>
#include "config.h"
#include "display_utils.h"
#include "motores.h"

// Objeto da biblioteca QTRSensors
QTRSensors qtr;

// Referência ao estado atual definido no robo_linha.ino
extern EstadoRobo estadoAtual;

// ==============================================================================
// FUNÇÕES DOS SENSORES
// ==============================================================================

void initSensores() {
  // Configurando como RC, pois os pinos 24-31 no Mega são portas digitais
  qtr.setTypeRC();
  qtr.setSensorPins(PINOS_IR, NUM_SENSORES_IR);
}

void executarCalibracao() {
  // Feedback visual no OLED
  atualizarStatus("Sistema", "CALIBRANDO...");
  
  unsigned long tempoInicio = millis();
  unsigned long ultimoInversao = millis();
  bool sentidoGiro = true; // true = Horário, false = Anti-horário
  
  // Aciona os motores para começar o giro sobre o próprio eixo
  rotacionarEixo(sentidoGiro, VELOCIDADE_BASE);
  
  // Loop de calibração que dura exatamente 5 segundos (5000 ms)
  while (millis() - tempoInicio < 5000) {
    qtr.calibrate();
    
    // Lógica de oscilação: Inverte o sentido dos motores a cada 500ms
    if (millis() - ultimoInversao >= 500) {
      sentidoGiro = !sentidoGiro; // Inverte o valor booleano
      rotacionarEixo(sentidoGiro, VELOCIDADE_BASE); // Aplica a nova direção
      ultimoInversao = millis();  // Reseta o temporizador de inversão
    }
  }
  
  // Fim da calibração
  pararMotores();
  
  // Feedback de conclusão no OLED
  atualizarStatus("CALIBRACAO", "OK");
  
  // Apenas nesta rotina específica do Setup inicial o uso de delay() é aceitável
  delay(1000); 
  
  // Transição de Estado
  estadoAtual = ESTADO_LINHA;
}

#endif // SENSORES_H
