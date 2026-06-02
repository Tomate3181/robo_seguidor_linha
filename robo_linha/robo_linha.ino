#include <Wire.h>
#include "config.h"

#include "motores.h"
#include "sensores.h"

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
  
  // APLICAÇÃO: Verificação ativa contra travamento físico do barramento I2C
  if (Wire.getWireTimeoutFlag()) {
    Serial.println(F("[ALERTA] I2C travou por ruido! Forcando recuperacao..."));
    Wire.clearWireTimeoutFlag(); // Destrava limpando o erro interno
    tcaselect(CANAL_GY521);      // Força o reestabelecimento do canal do giroscópio no TCA
  }
  
  // Atualiza o giroscópio a cada ciclo para o rastreio do Yaw(Z) não perder precisão
  tcaselect(CANAL_GY521);
  mpu.update();

  switch (estadoAtual) {
    case ESTADO_CALIBRACAO:
      // Executa a calibração dos sensores e do giroscópio
      executarCalibracao();
      break;

    case ESTADO_LINHA: {
      // Leitura da posição da linha e dos sensores
      uint16_t position = qtr.readLineBlack(sensorValues);
      
      // Varre TODOS os 8 sensores procurando qualquer indício de preto (> 200)
      bool vendoLinha = false;
      int sensoresNoPreto = 0;

      for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
        if (sensorValues[i] > 500) {
          vendoLinha = true;
          sensoresNoPreto++;
        }
      }

      // =====================================================================
      // GATILHO INTELIGENTE: LEITURA DE COR SOB DEMANDA
      // Se 4 ou mais sensores detectam preto, assumimos que é uma linha horizontal (Cruzamento ou T)
      // O temporizador evita que ele leia o mesmo cruzamento várias vezes seguidas
      // =====================================================================
      static unsigned long tempoUltimoCruzamento = 0;
      if (sensoresNoPreto >= 4 && (millis() - tempoUltimoCruzamento > 1500)) {
        tempoUltimoCruzamento = millis();
        
        bool mudouEstado = avaliarInterseccao(); // A mágica acontece aqui
        
        // Se a função detectou verde ou vermelho, ela já alterou o estadoAtual.
        if (mudouEstado) {
          break; // Sai do case ESTADO_LINHA e vai processar a cor no loop
        }
      }
      // =====================================================================


      // Verifica sonar frontal a cada 50ms para não travar o loop
      if (millis() - tempoUltimoSonar > 50) {
        tempoUltimoSonar = millis();
        if (obterDistanciaFiltrada(sonarFrente) <= 10) {
          controlarRodas(0, 0); // Para imediatamente
          modoObstaculo = GIRO_INICIAL;
          tcaselect(CANAL_GY521);
          mpu.update();
          anguloInicialObstaculo = mpu.getAngleZ();
          tempoInicioObstaculo = millis();
          estadoAtual = ESTADO_OBSTACULO;
          break;
        }
      }

      // Máquina de estados interna para controle da linha (Não-bloqueante)
      switch (modoLinha) {
        case SEGUINDO:
          if (!vendoLinha) {
            modoLinha = INSISTINDO;
            tempoInicioInsistir = millis();
          } else {
            contadorFalhas = 0; 
            int erro = 3500 - position;
            
            // Memoriza o lado para curvas fechadas, mas zera se estiver andando reto
            if (erro > 500) ultimoLado = 1;       
            else if (erro < -500) ultimoLado = -1; 
            else if (abs(erro) < 300) ultimoLado = 0; // Se perder a linha no gap, não vai girar loucamente!

            // Cálculo do PID
            int P = erro * KP;
            int D = (erro - ultimoErro) * KD;
            int ajuste = P + D;
            ultimoErro = erro;

            controlarRodas(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste);
          }
          break;

        case INSISTINDO:
          // Tenta insistir na curva por 300ms (dá mais tempo para virar os 90 graus)
          if (millis() - tempoInicioInsistir < 300) {
            // Mais força na virada se estava em curva. Se estava reto (ultimoLado == 0), apenas vai reto
            if (ultimoLado == 1) controlarRodas(220, -180); 
            else if (ultimoLado == -1) controlarRodas(-180, 220);
            else controlarRodas(VELOCIDADE_BASE, VELOCIDADE_BASE); // Gap! Vai reto.

            // CORREÇÃO: Aceita a linha em QUALQUER uma das abas dos 8 sensores para se recuperar
            if (vendoLinha) {
              modoLinha = SEGUINDO; 
            }
          } else {
            // Falhou em encontrar a linha, passa para o tratamento de Gap
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
          // Avança pelo tempo determinado para buscar a linha após um gap
          if (millis() - tempoInicioGap < TEMPO_PARA_12CM) {
            controlarRodas(VELOCIDADE_GAP, VELOCIDADE_GAP);
            
            // CORREÇÃO: Monitora os 8 sensores. Se o robô estiver torto no meio do Gap 
            // e a linha bater em uma das pontas, ele captura instantaneamente.
            if (vendoLinha) {
              modoLinha = SEGUINDO; 
            }
          } else {
            // Não encontrou andando para frente, recua procurando nos 8 sensores
            controlarRodas(-VELOCIDADE_GAP, -VELOCIDADE_GAP);
            if (vendoLinha) {
              modoLinha = SEGUINDO;
            }
          }
          break;

        case GAP_RE_AJUSTE:
          // Dá ré por 2 segundos após falhar várias vezes no gap
          if (millis() - tempoInicioGap < 2000) {
            controlarRodas(-100, -100);
            
            // Se durante a marcha ré algum dos 8 sensores encostar na linha, aborta o re-ajuste
            if (vendoLinha) {
              contadorFalhas = 0;
              modoLinha = SEGUINDO;
            }
          } else {
            contadorFalhas = 0;
            modoLinha = SEGUINDO; // Retoma a tentativa de seguir
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

    default:
      estadoAtual = ESTADO_LINHA;
      break;
  }
}