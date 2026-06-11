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

// Variáveis de Validação da Zona de Resgate
bool suspeitaResgateAtiva = false;
unsigned long tempoSuspeitaResgate = 0;
ModoValidacao modoValidacao = VALIDACAO_RE;
unsigned long tempoInicioValidacao = 0;

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
    case ESTADO_VALIDACAO_RESGATE: return "VALIDACAO_RESGATE";
    case ESTADO_RESGATE: return "RESGATE";
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

      for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
        if (sensorValues[i] > 650) {
          sensoresNoPreto++; // Linha preta absoluta
        } else if (sensorValues[i] > 150) {
          sensoresNoCinza++; // Faixa reflexiva (cinza da silver tape ou sujeira)
        }
      }

      bool vendoLinha = (sensoresNoPreto > 0);

      // =====================================================================
      // GATILHO 1: AVALIAÇÃO DA SUSPEITA DE RESGATE (SILVER TAPE)
      // =====================================================================
      static unsigned long tempoBrancoTotal = 0; // Filtro de bounce

      if (suspeitaResgateAtiva) {
        if (sensoresNoPreto >= 2) {
          // Regra 2: Se após a intersecção o QTR detectar linha preta, a suspeita é descartada imediatamente.
          suspeitaResgateAtiva = false;
          tempoBrancoTotal = 0;
          Serial.println(F("[DEBUG-RESGATE] Suspeita descartada! Linha preta encontrada."));
        } else if (sensoresNoPreto == 0 && (millis() - tempoSuspeitaResgate < 1500)) {
          // Regra 3: Branco total detectado.
          // CORREÇÃO DE BUG: Adicionado um debounce de 150ms. Se for apenas o robô
          // saindo da linha de raspão por causa da inércia da curva de 90 graus, ele não aborta.
          if (tempoBrancoTotal == 0) tempoBrancoTotal = millis();

          if (millis() - tempoBrancoTotal > 150) {
            estadoAtual = ESTADO_VALIDACAO_RESGATE;
            modoValidacao = VALIDACAO_RE;
            tempoInicioValidacao = millis();
            suspeitaResgateAtiva = false; // Consome a suspeita
            tempoBrancoTotal = 0;
            Serial.println(F("[RESGATE] Suspeita confirmada por debounce! Iniciando validacao fisica."));
            break; 
          }
        } else {
          tempoBrancoTotal = 0; // Zera se viu algo que não é 0 preto mas também não é >=2 (ex: 1 sensor apenas)
          if (millis() - tempoSuspeitaResgate >= 1500) {
            // Timeout de segurança se demorar demais
            suspeitaResgateAtiva = false;
            Serial.println(F("[DEBUG-RESGATE] Suspeita de resgate expirou por timeout."));
          }
        }
      }

      // =====================================================================
      // GATILHO 2: CRUZAMENTOS VERDE/VERMELHO E SUSPEITA DE RESGATE
      // =====================================================================
      static unsigned long tempoUltimoCruzamento = 0;
      if (sensoresNoPreto >= 4 && (millis() - tempoUltimoCruzamento > 1000)) {
        tempoUltimoCruzamento = millis();
        bool mudouEstado = avaliarInterseccao(); 
        
        if (mudouEstado) {
          break; // Achou verde/vermelho, sai do case ESTADO_LINHA
        } else {
          // Regra 1: Passou por intersecção, NÃO leu verde, e seguiu reto.
          // Inicia a suspeita de cinza (Silver Tape).
          suspeitaResgateAtiva = true;
          tempoSuspeitaResgate = millis();
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
        Serial.print(F(" | Pretos: ")); Serial.print(sensoresNoPreto);
        Serial.print(F(" | SuspeitaResgate: ")); Serial.println(suspeitaResgateAtiva ? "ATIVA" : "NAO");
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

    case ESTADO_VALIDACAO_RESGATE: {
      // Regra 5: Se durante qualquer momento da validação os sensores QTR detectarem linha preta, a validação é abortada.
      uint16_t vIR[NUM_SENSORES_IR];
      qtr.readLineBlack(vIR);
      int sensoresNoPreto = 0;
      for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
        if (vIR[i] > 650) sensoresNoPreto++;
      }
      
      if (sensoresNoPreto >= 2) {
        controlarRodas(0, 0);
        estadoAtual = ESTADO_LINHA;
        modoLinha = SEGUINDO; // Retorna instantaneamente para o estado de seguir linha
        Serial.println(F("[VALIDACAO] Linha preta detectada! Falso positivo descartado."));
        break;
      }

      switch (modoValidacao) {
        case VALIDACAO_RE:
          // Dá ré para alinhar o sensor RGB com a marca do cinza
          if (millis() - tempoInicioValidacao < 250) { // Tempo estimado para a ré (~250ms)
            controlarRodas(-100, -100); 
          } else {
            controlarRodas(0, 0); 
            modoValidacao = VALIDACAO_RGB;
            tempoInicioValidacao = millis();
          }
          break;

        case VALIDACAO_RGB:
          // Aguarda 100ms para estabilizar a inércia da parada antes de ler a cor
          if (millis() - tempoInicioValidacao > 100) { 
            uint16_t rD, gD, bD, cD, rE, gE, bE, cE;
            tcaselect(CANAL_TCS_DIR); tcsDir.getRawData(&rD, &gD, &bD, &cD);
            tcaselect(CANAL_TCS_ESQ); tcsEsq.getRawData(&rE, &gE, &bE, &cE);

            // Regra 4: Validação rigorosa do cinza usando a calibração com margem apertada (+/- 100)
            bool dirOk = ehCinzaRigoroso(rD, gD, bD, cD, cinzaCalibradoDir);
            bool esqOk = ehCinzaRigoroso(rE, gE, bE, cE, cinzaCalibradoEsq);

            if (dirOk || esqOk) {
              Serial.println(F("[RESGATE] Silver tape VALIDADA rigorosamente!"));
              estadoAtual = ESTADO_RESGATE;
              modoResgate = RESGATE_ENTRANDO;
              tempoInicioResgate = millis();
              tempoEntradaResgate = millis();
              votosSemParede = 0;
              tcaselect(CANAL_GY521); mpu.update();
              anguloInicialResgate = mpu.getAngleZ();
            } else {
              Serial.println(F("[VALIDACAO] Falso positivo RGB. Voltando para a linha."));
              estadoAtual = ESTADO_LINHA;
              modoLinha = GAP_AVANCA; // Retoma avançando levemente para sair da zona branca
              tempoInicioGap = millis();
            }
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
      static unsigned long tempoUltimoSonarResgate = 0;
      static int distFrente = MAX_DISTANCE;
      static int distDir = MAX_DISTANCE;
      
      if (millis() - tempoUltimoSonarResgate > 50) {
        tempoUltimoSonarResgate = millis();
        distFrente = obterDistanciaFiltrada(sonarFrente);
        distDir = obterDistanciaFiltrada(sonarDir);
      }
      
      if (tempoEntradaResgate == 0) {
        tempoEntradaResgate = millis();
      }
      
      // =====================================================================
      // VALIDAÇÃO DA SAÍDA DO RESGATE (EXCLUSIVA VIA IR - MAIS VELOZ)
      // =====================================================================
      if (millis() - tempoEntradaResgate > TEMPO_MINIMO_RESGATE) {
        uint16_t vIR[NUM_SENSORES_IR];
        qtr.readLineBlack(vIR);
        int sensoresNaLinha = 0;
        for (uint8_t i = 0; i < NUM_SENSORES_IR; i++) {
          if (vIR[i] > 650) { 
            sensoresNaLinha++;
          }
        }
        
        // Se o IR identificou uma linha preta sólida (pelo menos 2 sensores na linha)
        if (sensoresNaLinha >= 2) { 
          ultimoErro = 0;
          contadorFalhas = 0;
          modoLinha = SEGUINDO;
          estadoAtual = ESTADO_LINHA;
          tempoEntradaResgate = 0; 
          Serial.println(F("[RESGATE] Saida detectada via IR! Voltando pra linha."));
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