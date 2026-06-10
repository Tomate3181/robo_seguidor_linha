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

// Variáveis do estado de resgate (Wall-Following e Navegação)
ModoResgate modoResgate = RESGATE_ENTRANDO;
unsigned long tempoInicioResgate = 0;
float anguloInicialResgate = 0;
unsigned long tempoEntradaResgate = 0;
bool registrouAnguloDireita = false;
float anguloGiroDireita = 0;
// BUG #3b FIX: Extraída do escopo 'static' para poder ser resetada entre sub-estados
int votosSemParede = 0;


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
      
      // --- DETECÇÃO DE SILVER TAPE (ENTRADA DA ZONA DE RESGATE) ---
      static int votosSilverTape = 0;
      bool suspeitaSilverTape = false;

      if (detectouSilverTape(sensorValues)) {
        votosSilverTape++;
        suspeitaSilverTape = true;
      } else {
        votosSilverTape = 0;
      }
      
      // SE ELE DESCONFIAR DA SILVER TAPE, VAI RETO E IGNORA O RESTO!
      if (suspeitaSilverTape && votosSilverTape < 4) {
        controlarRodas(VELOCIDADE_BASE, VELOCIDADE_BASE);
        break; // Sai do switch para NÃO executar o PID de linha abaixo
      }
      
      if (votosSilverTape >= 4) { // Exige 4 confirmações consecutivas (~10ms) para filtrar ruídos
        // Confirmação TCS34725: 3 critérios simultâneos via ehCinzaRGB()
        //   1. JANELA c (150–500): exclui branco difuso (>500) e escuro (<150)
        //   2. NEUTRALIDADE R≈G≈B: cinza/prata não tem matiz dominante
        //   3. SATURAÇÃO BAIXA (<15%): elimina cores do chão
        uint16_t rD, gD, bD, cD;
        uint16_t rE, gE, bE, cE;
        
        tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&rD, &gD, &bD, &cD);
        tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&rE, &gE, &bE, &cE);
        
        bool dirConfirma = ehCinzaRGB(rD, gD, bD, cD, cinzaCalibradoDir);
        bool esqConfirma = ehCinzaRGB(rE, gE, bE, cE, cinzaCalibradoEsq);
        
        // LOG DIAGNÓSTICO COMPLETO: use esses valores para ajustar limiares em pista
        // Silver tape real deve ter: c=150~500, R≈G≈B (diferença < ~30 entre canais)
        // Branco deve ser bloqueado por: c > 500
        Serial.print(F("[SILVER_DIR] R=")); Serial.print(rD);
        Serial.print(F(" G=")); Serial.print(gD);
        Serial.print(F(" B=")); Serial.print(bD);
        Serial.print(F(" C=")); Serial.print(cD);
        Serial.print(F(" -> ")); Serial.println(dirConfirma ? F("OK") : F("FALSO"));
        
        Serial.print(F("[SILVER_ESQ] R=")); Serial.print(rE);
        Serial.print(F(" G=")); Serial.print(gE);
        Serial.print(F(" B=")); Serial.print(bE);
        Serial.print(F(" C=")); Serial.print(cE);
        Serial.print(F(" -> ")); Serial.println(esqConfirma ? F("OK") : F("FALSO"));
        
        // Se pelo menos um dos sensores confirmar (reflexividade + cromatica)
        if (dirConfirma || esqConfirma) {
          votosSilverTape = 0; // Reseta o contador
          controlarRodas(0, 0);
          delay(150); // Breve pausa para estabilização física do chassi
          
          estadoAtual = ESTADO_RESGATE;
          modoResgate = RESGATE_ENTRANDO;
          tempoInicioResgate = millis();
          tempoEntradaResgate = millis();
          votosSemParede = 0; // Reseta o contador global do wall-following
          
          tcaselect(CANAL_GY521);
          mpu.update();
          anguloInicialResgate = mpu.getAngleZ();
          
          Serial.println(F("[ALERTA] Silver tape confirmada via RGB! Transitando para ESTADO_RESGATE."));
          break; // Sai do case ESTADO_LINHA
        } else {
          votosSilverTape = 0; // Falso positivo, reseta
          Serial.println(F("[INFO] Silver tape descartada pelo validador RGB (Falso Positivo)."));
        }
      }
      
      // Varre TODOS os 8 sensores procurando qualquer indício de preto (> 500)
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

    case ESTADO_RESGATE: {
      // 1. Leitura periódica e não-bloqueante dos sonares (a cada 50ms)
      static unsigned long tempoUltimoSonarResgate = 0;
      static int distFrente = MAX_DISTANCE;
      static int distDir = MAX_DISTANCE;
      
      if (millis() - tempoUltimoSonarResgate > 50) {
        tempoUltimoSonarResgate = millis();
        distFrente = obterDistanciaFiltrada(sonarFrente);
        distDir = obterDistanciaFiltrada(sonarDir);
      }
      
      // 2. Registro do cronômetro de início da zona de resgate
      if (tempoEntradaResgate == 0) {
        tempoEntradaResgate = millis();
      }
      
      // 3. Verificação de saída de emergência: caso detecte a linha preta após tempo mínimo
      if (millis() - tempoEntradaResgate > TEMPO_MINIMO_RESGATE) {
        uint16_t vIR[NUM_SENSORES_IR];
        qtr.readLineBlack(vIR);
        int sensoresNaLinha = 0;
        for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
          if (vIR[i] > 650) { // Linha preta sólida
            sensoresNaLinha++;
          }
        }
        
        static int votosSaida = 0;
        if (sensoresNaLinha >= 2) { // Exige pelo menos 2 sensores na fita preta
          votosSaida++;
        } else {
          votosSaida = 0;
        }
        
        if (votosSaida >= 4) { // Exige 4 confirmações consecutivas (~10ms)
          controlarRodas(0, 0);
          ultimoErro = 0;
          contadorFalhas = 0;
          modoLinha = SEGUINDO;
          estadoAtual = ESTADO_LINHA;
          tempoEntradaResgate = 0; // Reseta cronômetro do resgate
          votosSaida = 0;
          Serial.println(F("[RESGATE] Saida confirmada! Retornando ao ESTADO_LINHA."));
          break;
        }
      }
      
      // 4. Execução dos sub-estados da navegação do resgate
      switch (modoResgate) {
case RESGATE_ENTRANDO: {
          controlarRodas(VELOCIDADE_RESGATE, VELOCIDADE_RESGATE);
          
          unsigned long tempoDecorrido = millis() - tempoInicioResgate;
          // Ignora sensores nos primeiros 800ms para cruzar a silver tape totalmente
          if (tempoDecorrido > 800) {
            if (distFrente <= 18) {
              controlarRodas(-90, -90); // Dá uma pequena ré de 100ms para não raspar o bico girando
              delay(100); 
              controlarRodas(0, 0);
              tcaselect(CANAL_GY521); mpu.update();
              anguloInicialResgate = mpu.getAngleZ();
              tempoInicioResgate = millis();
              modoResgate = RESGATE_GIRANDO_ESQUERDA;
            }
            else if (distDir <= 25) { // Tolerância para "pegar" a parede lateral
              votosSemParede = 0; // BUG #3b FIX: Garante contador limpo ao entrar
              modoResgate = RESGATE_SEGUINDO_PAREDE;
            }
            // BUG #3 FIX: Timeout reduzido de 2500ms para 1500ms.
            // 2500ms era tempo demais parado sem encontrar parede → robô andava muito longe.
            else if (tempoDecorrido > 1500) {
              votosSemParede = 0; // BUG #3b FIX: Garante contador limpo ao entrar
              modoResgate = RESGATE_SEGUINDO_PAREDE;
            }
          }
          break;
        }
        
        case RESGATE_SEGUINDO_PAREDE: {
          // A: Parede frontal à vista -> parar e iniciar giro de 90° à esquerda
          if (distFrente <= DISTANCIA_OBSTACULO_FRENTE) {
            controlarRodas(0, 0);
            tcaselect(CANAL_GY521); mpu.update();
            anguloInicialResgate = mpu.getAngleZ();
            tempoInicioResgate = millis();
            modoResgate = RESGATE_GIRANDO_ESQUERDA;
            break;
          }
          
          // B: Parede lateral direita sumiu -> iniciar contorno de quina externa
          // BUG #3b FIX: votosSemParede agora é variável global (linha ~37)
          // para poder ser resetada ao transitar de RESGATE_ENTRANDO para cá.
          if (distDir > DISTANCIA_QUINA_PAREDE) {
            votosSemParede++; // Filtro de ruído do Sonar
          } else {
            votosSemParede = 0;
          }

          if (votosSemParede >= 3) { // Só gira se confirmar 3 vezes que a parede sumiu!
            votosSemParede = 0;
            tempoInicioResgate = millis();
            registrouAnguloDireita = false; 
            modoResgate = RESGATE_GIRANDO_DIREITA;
            break;
          }
          
          // C: Controle Proporcional (Wall-Following)
          int erroParede = distDir - DISTANCIA_ALVO_PAREDE;
          
          // TRAVA DE SEGURANÇA: Limita o erro máximo. 
          // Impede solavancos violentos caso o sonar falhe e leia 60cm do nada.
          erroParede = constrain(erroParede, -10, 10); 
          
          int ajuste = erroParede * KP_PAREDE;
          
          controlarRodas(VELOCIDADE_RESGATE - ajuste, VELOCIDADE_RESGATE + ajuste);
          break;
        }
        
        case RESGATE_GIRANDO_ESQUERDA: {
          // BUG #4 FIX: Giro com controle PROPORCIONAL (igual ao ESTADO_VERDE).
          // A velocidade fixa 90 causava overshoot físico — o chassi continuava
          // girando por inércia mesmo depois do código parar os motores.
          // Com velocidade proporcional, o robô desacelera ao se aproximar do alvo.
          float anguloAlvo  = anguloInicialResgate + 88.0; // 88° para compensar inércia
          float anguloAtual = mpu.getAngleZ();
          float erroAngulo  = anguloAlvo - anguloAtual;
          
          if (erroAngulo > 2.0) {
            // Velocidade proporcional: rápido longe, lento perto
            // Mínimo 60 para ter torque suficiente; máximo 130 para evitar inércia
            int velGiro = constrain(60 + (int)(erroAngulo * 2.0), 60, 130);
            controlarRodas(velGiro, -velGiro); // Anti-horário = esquerda
          } else {
            // Atingiu o ângulo alvo!
            controlarRodas(0, 0);
            votosSemParede = 0; // Reseta contador ao entrar no seguidor de parede
            modoResgate = RESGATE_SEGUINDO_PAREDE;
            Serial.println(F("[RESGATE] Giro a esquerda finalizado."));
          }
          
          // Timeout de emergência (mantido para segurança)
          if (millis() - tempoInicioResgate > 2500) {
            controlarRodas(0, 0);
            votosSemParede = 0;
            modoResgate = RESGATE_SEGUINDO_PAREDE;
            Serial.println(F("[RESGATE] Timeout no giro a esquerda!"));
          }
          break;
        }
        
        case RESGATE_GIRANDO_DIREITA: {
          unsigned long tempoDecorrido = millis() - tempoInicioResgate;
          
          if (tempoDecorrido < TEMPO_AVANCO_QUINA) {
            // Fase 1: Avança reto para ultrapassar a quina fisicamente
            controlarRodas(VELOCIDADE_RESGATE, VELOCIDADE_RESGATE);
          } else {
            // Fase 2: Gira no próprio eixo para a direita (convenção: horário é negativo no Yaw)
            if (!registrouAnguloDireita) {
              tcaselect(CANAL_GY521);
              mpu.update();
              anguloGiroDireita = mpu.getAngleZ();
              registrouAnguloDireita = true;
            }
            
            controlarRodas(-95, 95);
            float anguloAlvo = anguloGiroDireita - 88.0;
            
            float anguloAtual = mpu.getAngleZ();
            if (anguloAtual <= anguloAlvo) {
              controlarRodas(0, 0);
              registrouAnguloDireita = false;
              modoResgate = RESGATE_SEGUINDO_PAREDE;
              Serial.println(F("[RESGATE] Giro a direita finalizado."));
            }
            
            // Timeout de emergência na Fase 2 (3 segundos extras)
            if (tempoDecorrido > (TEMPO_AVANCO_QUINA + 3000)) {
              controlarRodas(0, 0);
              registrouAnguloDireita = false;
              modoResgate = RESGATE_SEGUINDO_PAREDE;
              Serial.println(F("[RESGATE] Timeout no giro a direita!"));
            }
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