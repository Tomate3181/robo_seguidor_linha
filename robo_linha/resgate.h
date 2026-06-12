#ifndef RESGATE_H
#define RESGATE_H

#include <Arduino.h>
#include "config.h"

// Variáveis e Constantes Locais do Resgate
static unsigned long tempoInicioValidacaoResgate = 0;
const unsigned long TEMPO_VALIDACAO_SILVER_TAPE = 250; // Tempo de avanço na fita (ms)
const int VELOCIDADE_TESTE_SILVER_TAPE = 120; // Ajuste conforme necessário

// Variável externa da FSM principal definida em robo_linha.ino
extern EstadoRobo estadoAtual; 

// Funções externas de controle (vindas de motores.h e sensores.h)
extern void controlarRodas(int velEsq, int velDir);
extern void pararMotores();

// ==============================================================================
// FUNÇÕES DE TRANSIÇÃO E VALIDAÇÃO
// ==============================================================================

inline void iniciarValidacaoSilverTape() {
    tempoInicioValidacaoResgate = millis();
    estadoAtual = ESTADO_VALIDANDO_SILVER_TAPE;
    Serial.println(F("[RESGATE] Possivel Silver Tape. Iniciando validacao..."));
}

inline void executarValidacaoSilverTape() {
    // 1. Mantém os motores movendo-se para frente (velocidade constante de teste)
    controlarRodas(VELOCIDADE_TESTE_SILVER_TAPE, VELOCIDADE_TESTE_SILVER_TAPE);

    // 2. Verifica se o tempo estipulado terminou (Lógica Estritamente Não-Bloqueante)
    if (millis() - tempoInicioValidacaoResgate >= TEMPO_VALIDACAO_SILVER_TAPE) {
        
        // Cria um array local para forçar uma nova leitura da barra QTR
        uint16_t sensorValuesValidacao[NUM_SENSORES_IR];
        qtr.readLineBlack(sensorValuesValidacao); 

        int sensoresCravados1000 = 0;
        for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
            if (sensorValuesValidacao[i] == 1000) {
                sensoresCravados1000++;
            }
        }

        // 3. Checagem final da condição
        if (sensoresCravados1000 == NUM_SENSORES_IR) {
            // Condição 1: TODOS os 8 sensores cravados em 1000
            pararMotores();
            estadoAtual = ESTADO_ZONA_RESGATE;
            Serial.println(F("[RESGATE] Silver Tape CONFIRMADA! Entrando na Zona de Resgate."));
        } else {
            // Condição 2: Falso positivo (apenas intersecção ou cruzamento em T)
            estadoAtual = ESTADO_LINHA;
            Serial.println(F("[RESGATE] Falso Positivo detectado. Abortando validacao e retornando para linha."));
        }
    }
}

// ==============================================================================
// LÓGICA PRINCIPAL DA ZONA DE RESGATE
// ==============================================================================

// Assinatura da função que gerenciará o resgate (ultrassons, captura das bolinhas, etc.)
inline void executarRotinaResgate() {
    // TODO: Implementar a FSM interna da Zona de Resgate.
    // Dicas para implementação futura:
    // - Crie um "enum EstadoResgate" para as sub-tarefas (buscando triângulo, varrendo, depositando).
    // - Utilize variáveis de controle não-bloqueantes com millis() para leituras do HC-SR04.
    
    // Por enquanto, apenas garante que o robô fique parado ao confirmar a fita.
    controlarRodas(0, 0);
}

#endif // RESGATE_H
