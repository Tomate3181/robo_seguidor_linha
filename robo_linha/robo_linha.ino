#include <Wire.h>
#include "config.h"

#include "motores.h"
#include "sensores.h"
#include "resgate.h"

// Variável global para armazenar o estado atual da Máquina de Estados Finitos (FSM)
EstadoRobo estadoAtual = ESTADO_CALIBRACAO;

// Variáveis de controle de linha
ModoLinha modoLinha = SEGUINDO;
int ultimoLado = 0;
int ultimoErro = 0;
int contadorFalhas = 0;
unsigned long tempoInicioInsistir = 0;
unsigned long tempoInicioGap = 0;

// Variável para armazenar o tipo de giro determinado pelo sensor RGB
int tipoGiro = 0;
float anguloInicial = 0;

// Variáveis de desvio de obstáculo
ModoObstaculo modoObstaculo = GIRO_INICIAL;
float anguloInicialObstaculo = 0;
unsigned long tempoInicioObstaculo = 0;
int contadorContorno = 0;
unsigned long tempoUltimoSonar = 0;



// ==============================================================================
// VARIÁVEIS E FUNÇÕES DE DEBUG
// ==============================================================================
EstadoRobo ultimoEstadoDebug = ESTADO_CALIBRACAO;
ModoLinha ultimoModoLinhaDebug = SEGUINDO;

String getNomeEstado(EstadoRobo e) {
  switch(e) {
    case ESTADO_CALIBRACAO: return "CALIBRACAO";
    case ESTADO_LINHA: return "LINHA";
    case ESTADO_VERDE: return "VERDE";
    case ESTADO_VERMELHO: return "VERMELHO";
    case ESTADO_OBSTACULO: return "OBSTACULO";

    default: return "DESCONHECIDO";
  }
}

String getNomeModoLinha(ModoLinha m) {
  switch(m) {
    case SEGUINDO: return "SEGUINDO";
    case INSISTINDO: return "INSISTINDO";
    case GAP_AVANCA: return "GAP_AVANCA";
    case GAP_RE_AJUSTE: return "GAP_RE_AJUSTE";
    default: return "DESCONHECIDO";
  }
}

// ==============================================================================
// FUNÇÃO DO MULTIPLEXADOR I2C (TCA9548A)
// ==============================================================================
// Seleciona o canal I2C (0 a 7) para comunicar com um dispositivo específico
// Esta função DEVE ser chamada antes de qualquer comunicação I2C (OLED, GY-521, TCS)
void tcaselect(uint8_t i) {
  if (i > 7) return; // Segurança: Evita selecionar um canal inexistente
  
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  // Aguarda 200ms para estabilização da energia de todos os módulos I2C (OLED, Sensores)
  // Isso previne que o Arduino tente se comunicar antes do OLED estar 'acordado'
  delay(200);
  
  Serial.begin(115200);
  
  pinMode(PINO_BOTAO, INPUT_PULLUP); // Habilita o resistor interno do Arduino para o botão
  pinMode(PINO_BOTAO_RESET, INPUT_PULLUP); // Habilita o resistor interno para o botão de reset
  
  // Inicializa o barramento I2C
  Wire.begin(); 
  
  // APLICAÇÃO: Ativação do Timeout nativo para evitar congelamento por ruído elétrico
  Wire.setWireTimeout(3000, true); // Tempo limite de 3ms. 'true' ativa o auto-reset do barramento
  Wire.clearWireTimeoutFlag();     // Limpa erros residuais iniciais
  
  Serial.println(F("======================================="));
  Serial.println(F("   Iniciando Robo Seguidor de Linha    "));
  Serial.println(F("======================================="));
  


  
  // Inicialização dos módulos
  initMotores();
  initSensores();
}

// ==============================================================================
// LOOP PRINCIPAL (Máquina de Estados)
// ==============================================================================
void loop() {
  // REGRA DE OURO: Código não-bloqueante. Não utilize delay() no loop principal!
  
  // RASTREADOR DE MUDANÇA DE ESTADO (DEBUG FSM)
  if (estadoAtual != ultimoEstadoDebug) {
    Serial.print(F("[DEBUG-FSM] Mudanca de Estado: "));
    Serial.print(getNomeEstado(ultimoEstadoDebug));
    Serial.print(F(" -> "));
    Serial.println(getNomeEstado(estadoAtual));
    ultimoEstadoDebug = estadoAtual;
  }
  
  // APLICAÇÃO: Verificação ativa contra travamento físico do barramento I2C
  if (Wire.getWireTimeoutFlag()) {
    Serial.println(F("[ALERTA] I2C travou por ruido! Forcando recuperacao..."));
    Wire.clearWireTimeoutFlag(); // Destrava limpando o erro interno
    tcaselect(CANAL_GY521);      // Força o reestabelecimento do canal do giroscópio no TCA
  }
  
  // Atualiza o giroscópio a cada ciclo para o rastreio do Yaw(Z) não perder precisão
  tcaselect(CANAL_GY521);
  mpu.update();

  // --- Botão de Reset (Porta 42) ---
  static unsigned long tempoFimCooldownReset = 0;

  // Se o botão for pressionado (LOW)
  if (digitalRead(PINO_BOTAO_RESET) == LOW) {
    if (tempoFimCooldownReset == 0) {
      Serial.println(F("[RESET] Botao de Reset acionado! Voltando ao estado original..."));
      controlarRodas(0, 0);
      pararMotores();
      
      // Reseta o estado para seguimento de linha (mantendo a calibração prévia)
      estadoAtual = ESTADO_LINHA;
      modoLinha = SEGUINDO;
      ultimoLado = 0;
      ultimoErro = 0;
      contadorFalhas = 0;
      
      // Define o fim do cooldown para daqui a 5 segundos (5000ms)
      tempoFimCooldownReset = millis() + 5000;
    }
  }

  // Se estiver sob cooldown do reset, mantém o robô parado e exibe contagem regressiva
  if (tempoFimCooldownReset > 0) {
    if (millis() < tempoFimCooldownReset) {
      controlarRodas(0, 0);
      pararMotores();
      
      static unsigned long ultimoPrintReset = 0;
      if (millis() - ultimoPrintReset > 1000) {
        ultimoPrintReset = millis();
        unsigned long segundosRestantes = (tempoFimCooldownReset - millis()) / 1000 + 1;
        Serial.print(F("[RESET] Retomando em "));
        Serial.print(segundosRestantes);
        Serial.println(F("s..."));
      }
      return; // Interrompe o loop principal para manter o robô parado no cooldown
    } else {
      tempoFimCooldownReset = 0;
      Serial.println(F("[RESET] Cooldown finalizado! Iniciando movimento."));
    }
  }

  switch (estadoAtual) {
    case ESTADO_CALIBRACAO:
      // Executa a calibração dos sensores e do giroscópio
      executarCalibracao();
      break;

    case ESTADO_LINHA: {
      uint16_t position = qtr.readLineBlack(sensorValues);
      
      // Conta os sensores para tomar decisões lógicas ANTES do PID
      int sensoresNoPreto = 0;
      int sensoresNoCinza = 0;
      int sensoresCravados1000 = 0;

      for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
        if (sensorValues[i] == 1000) {
          sensoresCravados1000++;
        }
        
        if (sensorValues[i] > 650) {
          sensoresNoPreto++; // Linha preta absoluta
        } else if (sensorValues[i] > 150) {
          sensoresNoCinza++; // Faixa reflexiva (cinza da silver tape ou sujeira)
        }
      }

      // =====================================================================
      // GATILHO 1: SUSPEITA DE SILVER TAPE (ZONA DE RESGATE)
      // =====================================================================
      if (sensoresCravados1000 >= 5) {
        iniciarValidacaoSilverTape();
        break; // Quebra a execução atual e transiciona imediatamente
      }

      bool vendoLinha = (sensoresNoPreto > 0);

      // =====================================================================
      // GATILHO 2: CRUZAMENTOS VERDE/VERMELHO E SUSPEITA DE RESGATE
      // =====================================================================
      static unsigned long tempoUltimoCruzamento = 0;
      if (sensoresNoPreto >= 4 && (millis() - tempoUltimoCruzamento > 1000)) {
        tempoUltimoCruzamento = millis();
        bool mudouEstado = avaliarInterseccao(); 
        
        if (mudouEstado) {
          break; // Achou verde/vermelho, sai do case ESTADO_LINHA
        }
      }

      // =====================================================================
      // GATILHO 3: OBSTÁCULO FRONTAL (SONAR)
      // =====================================================================
      if (millis() - tempoUltimoSonar > 50) {
        tempoUltimoSonar = millis();
        if (obterDistanciaFiltrada(sonarFrente) <= 10) {
          controlarRodas(0, 0); 
          modoObstaculo = GIRO_INICIAL;
          tcaselect(CANAL_GY521); mpu.update();
          anguloInicialObstaculo = mpu.getAngleZ();
          tempoInicioObstaculo = millis();
          estadoAtual = ESTADO_OBSTACULO;
          break;
        }
      }

      // =====================================================================
      // MÁQUINA DE ESTADOS DO PID (O Seguidor de Linha em si)
      // =====================================================================
      
      // DEBUG DO MODO LINHA
      if (modoLinha != ultimoModoLinhaDebug) {
        Serial.print(F("[DEBUG-LINHA] Mudanca de Modo: "));
        Serial.print(getNomeModoLinha(ultimoModoLinhaDebug));
        Serial.print(F(" -> "));
        Serial.println(getNomeModoLinha(modoLinha));
        ultimoModoLinhaDebug = modoLinha;
      }

      static unsigned long ultimoPrintPID = 0;
      if (millis() - ultimoPrintPID > 500) {
        ultimoPrintPID = millis();
        Serial.print(F("[DEBUG-PID] Mod: ")); Serial.print(getNomeModoLinha(modoLinha));
        Serial.print(F(" | Erro: ")); Serial.print(3500 - position);
        Serial.print(F(" | Pretos: ")); Serial.println(sensoresNoPreto);
      }

      switch (modoLinha) {
        case SEGUINDO:
          if (!vendoLinha) { // O preto sumiu (Gap ou Quina)
            modoLinha = INSISTINDO;
            tempoInicioInsistir = millis();
          } else {
            contadorFalhas = 0; 
            int erro = 3500 - position;
            
            if (erro > 500) ultimoLado = 1;       
            else if (erro < -500) ultimoLado = -1; 
            else if (abs(erro) < 300) ultimoLado = 0; 

            int P = erro * KP;
            int D = (erro - ultimoErro) * KD;
            int ajuste = P + D;
            ultimoErro = erro;

            controlarRodas(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste);
          }
          break;

        case INSISTINDO:
          if (millis() - tempoInicioInsistir < 300) {
            if (ultimoLado == 1) controlarRodas(220, -180); 
            else if (ultimoLado == -1) controlarRodas(-180, 220);
            else controlarRodas(VELOCIDADE_BASE, VELOCIDADE_BASE); 

            if (vendoLinha) modoLinha = SEGUINDO; 
          } else {
            contadorFalhas++;
            if (contadorFalhas >= 3) {
              modoLinha = GAP_RE_AJUSTE;
              tempoInicioGap = millis();
            } else {
              modoLinha = GAP_AVANCA;
              tempoInicioGap = millis();
            }
          }
          break;

        case GAP_AVANCA:
          if (millis() - tempoInicioGap < TEMPO_PARA_12CM) {
            controlarRodas(VELOCIDADE_GAP, VELOCIDADE_GAP);
            if (vendoLinha) modoLinha = SEGUINDO; 
          } else {
            controlarRodas(-VELOCIDADE_GAP, -VELOCIDADE_GAP);
            if (vendoLinha) modoLinha = SEGUINDO;
          }
          break;

        case GAP_RE_AJUSTE:
          if (millis() - tempoInicioGap < 2000) {
            controlarRodas(-100, -100);
            if (vendoLinha) {
              contadorFalhas = 0;
              modoLinha = SEGUINDO;
            }
          } else {
            contadorFalhas = 0;
            modoLinha = SEGUINDO;
          }
          break;
      }
      break;
    }

    case ESTADO_VERDE: {
      // 1. Define o ângulo alvo com base no tipo de giro
      // Convenção MPU: Giro pra Esquerda (+) / Giro pra Direita (-)
      float anguloAlvo = anguloInicial - tipoGiro; 
      
      // 2. Obtém o ângulo atual do MPU (já atualizado no topo do loop)
      float anguloAtual = mpu.getAngleZ();
      float erroAngulo = anguloAlvo - anguloAtual;

      // 3. Controla o giro até atingir o alvo (+/- 3 graus de tolerância)
      if (abs(erroAngulo) > 3.0) {
        int velGiro = 80 + abs(erroAngulo) * 1.5; 
        if (velGiro > 150) velGiro = 150; // Limite máximo para evitar inércia excessiva

        if (erroAngulo > 0) {
          controlarRodas(velGiro, -velGiro); 
        } else {
          controlarRodas(-velGiro, velGiro);
        }
      } else {
        // Atingiu o ângulo!
        controlarRodas(0, 0);
        
        // Zera as variáveis do PID para não acumular erro da perda da linha anterior
        ultimoErro = 0;
        contadorFalhas = 0;
        modoLinha = SEGUINDO; // Garante que vai voltar caçando a linha

        // Bloqueia a leitura do sensor de cor por 1.5s após o giro
        ultimaLeituraCor = millis() + 1500;

        // Volta para a linha
        estadoAtual = ESTADO_LINHA;

      }
      break;
    }

    case ESTADO_VERMELHO:
      // Parada Total Imediata
      controlarRodas(0, 0);
      break;

    case ESTADO_OBSTACULO: {
      switch (modoObstaculo) {
        case GIRO_INICIAL: {
          // Gira para a esquerda (convenção: Esquerda é positivo no MPU)
          controlarRodas(-100, 100); 
          float anguloAlvo = anguloInicialObstaculo + 90.0;
          if (mpu.getAngleZ() >= anguloAlvo) {
            controlarRodas(0, 0);
            contadorContorno = 0;
            modoObstaculo = CONTORNO_LATERAL;
            tempoInicioObstaculo = millis();
          }
          // Timeout de segurança
          if (millis() - tempoInicioObstaculo > 2500) {
             modoObstaculo = CONTORNO_LATERAL; // Força avanço
          }
          break;
        }
        case CONTORNO_LATERAL: {
          controlarRodas(100, 100); // Avança contornando
          if (millis() - tempoUltimoSonar > 50) {
            tempoUltimoSonar = millis();
            int distD = obterDistanciaFiltrada(sonarDir);
            if (distD > 30) {
              contadorContorno++;
              if (contadorContorno >= 3) { // Passou da quina do objeto
                modoObstaculo = BUSCA_LINHA;
                tempoInicioObstaculo = millis();
              }
            } else {
              contadorContorno = 0; // Se voltou a ver a parede, zera
            }
          }
          break;
        }
        case BUSCA_LINHA: {
          // Curva reversa para a direita buscando a linha original
          controlarRodas(120, 20); 
          
          uint16_t position = qtr.readLineBlack(sensorValues);
          bool vendoLinha = false;
          for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
            if (sensorValues[i] > 200) {
              vendoLinha = true;
              break;
            }
          }

          if (vendoLinha) {
            controlarRodas(0, 0);
            ultimoErro = 0;
            contadorFalhas = 0;
            modoLinha = SEGUINDO;
            estadoAtual = ESTADO_LINHA;
          } else if (millis() - tempoInicioObstaculo > 6000) { 
            // Perdeu totalmente a linha, segurança.
            controlarRodas(0, 0);
            estadoAtual = ESTADO_LINHA; // Retorna para tentar se achar
          }
          break;
        }
      }
      break;
    }

    case ESTADO_VALIDANDO_SILVER_TAPE:
      executarValidacaoSilverTape();
      break;

    case ESTADO_ZONA_RESGATE:
      executarRotinaResgate();
      break;

    default:
      estadoAtual = ESTADO_LINHA;
      break;
  }
}