# Requirements Document

## Introduction

Este documento descreve os requisitos para a refatoração completa da lógica de desvio de obstáculo do robô seguidor de linha Arduino (OBR). A abordagem atual (`ESTADO_OBSTACULO` com `ModoObstaculo`) será substituída por uma estratégia de **contorno suave/circular** encapsulada no módulo `obstaculo.h`. A nova lógica adiciona três estados dedicados à FSM (`OBSTACULO_RE`, `OBSTACULO_GIRANDO`, `OBSTACULO_CONTORNO`), utiliza o giroscópio MPU6050 para giro preciso de 65°, malha fechada com sonar lateral para o arco de contorno, e recuperação automática de pista via barra QTR-8A. Toda a temporização é não-bloqueante, baseada em `millis()`.

---

## Glossary

- **FSM**: Máquina de Estados Finitos — estrutura de controle principal do robô em `robo_linha.ino`.
- **Sistema_Obstaculo**: O módulo de software `obstaculo.h` responsável por toda a lógica de contorno.
- **Controlador_FSM**: O arquivo `robo_linha.ino` que hospeda o `loop()` e o `switch(estadoAtual)`.
- **Sensor_Sonar_Frente**: Sensor ultrassônico HC-SR04 frontal (TRIG 48 / ECHO 49), acessado via `sonarFrente` e `obterDistanciaFiltrada()`.
- **Sensor_Sonar_Lateral**: Sensor ultrassônico HC-SR04 lateral (esquerdo ou direito), acessado via `sonarEsq` / `sonarDir` e `obterDistanciaFiltrada()`.
- **Sensor_QTR**: Barra de 8 sensores infravermelhos QTR-8A (pinos 30–37), lida via `qtr.readLineBlack(sensorValues)`.
- **Giroscopio_MPU**: Objeto `mpu` (MPU6050 via TCA9548A canal 1), lido via `mpu.getAngleZ()` para validação do ângulo de yaw (eixo Z).
- **Controlador_Rodas**: Função `controlarRodas(vDir, vEsq)` de `motores.h` — única interface de acionamento dos motores.
- **Filtro_Antiruido**: Mecanismo de validação por N leituras consecutivas para rejeitar falsos positivos do sonar.
- **Re_Inicial**: Movimento de ré breve executado imediatamente após confirmação de obstáculo para criar distância de manobra.
- **Giro_65**: Rotação de 65° no próprio eixo, validada matematicamente pelo Yaw do Giroscopio_MPU.
- **Arco_Contorno**: Trajetória curvilínea mantida pelo controle de distância lateral em malha fechada com o Sensor_Sonar_Lateral.
- **Distancia_Lateral_Alvo**: Distância de referência em cm que o robô deve manter do obstáculo durante o Arco_Contorno.
- **Lado_Desimpedido**: O lado (esquerdo ou direito) sem obstáculo lateral, determinado durante a validação de obstáculo.
- **Linha_Preta**: Faixa reflexiva detectada pelos sensores centrais (#3 e #4, índices 2 e 3) do Sensor_QTR com valor > 650.
- **OBSTACULO_RE**: Novo estado FSM — execução da ré inicial não-bloqueante.
- **OBSTACULO_GIRANDO**: Novo estado FSM — execução do giro de 65° validado pelo Giroscopio_MPU.
- **OBSTACULO_CONTORNO**: Novo estado FSM — execução do arco de contorno em malha fechada.
- **ESTADO_LINHA**: Estado FSM existente de seguimento de linha com controle PID.
- **config.h**: Arquivo de configuração central contendo `enum EstadoRobo`, constantes de hardware e parâmetros de controle.
- **millis()**: Função Arduino que retorna o tempo decorrido em milissegundos desde o boot — base de toda temporização não-bloqueante.

---

## Requirements

### Requisito 1: Modularização da Lógica de Obstáculo

**User Story:** Como desenvolvedor do robô OBR, quero toda a lógica de desvio de obstáculo isolada em `obstaculo.h`, para que o arquivo `robo_linha.ino` permaneça limpo e os módulos sejam mantidos de forma independente.

#### Critérios de Aceitação

1. THE Sistema_Obstaculo SHALL encapsular todas as variáveis de estado, funções de transição e lógica de controle de contorno em um único arquivo `obstaculo.h` protegido por guard macro `#ifndef OBSTACULO_H`.
2. THE Sistema_Obstaculo SHALL utilizar exclusivamente `controlarRodas()` de `motores.h` e `obterDistanciaFiltrada()`, `qtr`, `mpu`, `sonarFrente`, `sonarEsq`, `sonarDir` de `sensores.h` — sem redeclarar pinos, reinstanciar objetos de hardware ou incluir bibliotecas de hardware diretamente.
3. THE Controlador_FSM SHALL incluir `obstaculo.h` após `sensores.h` e delegar a execução dos estados `OBSTACULO_RE`, `OBSTACULO_GIRANDO` e `OBSTACULO_CONTORNO` às funções públicas de `obstaculo.h`.
4. THE Sistema_Obstaculo SHALL manter nomenclatura de variáveis e funções em português, com comentários explicando a física do contorno circular.
5. THE Sistema_Obstaculo SHALL seguir a indentação, guard macros e estilo das funções dos demais arquivos do projeto (`config.h`, `motores.h`, `sensores.h`).

---

### Requisito 2: Adição dos Novos Estados na FSM

**User Story:** Como desenvolvedor do robô OBR, quero três novos estados dedicados ao contorno de obstáculo no `enum EstadoRobo` de `config.h`, para que a FSM principal os trate em `cases` separados com responsabilidade única.

#### Critérios de Aceitação

1. THE config.h SHALL declarar os estados `OBSTACULO_RE`, `OBSTACULO_GIRANDO` e `OBSTACULO_CONTORNO` dentro do `enum EstadoRobo`, posicionados após `ESTADO_OBSTACULO` existente.
2. THE config.h SHALL declarar as constantes de obstáculo: `OBSTACULO_DIST_DETECCAO` (distância de ativação em cm), `OBSTACULO_LEITURAS_CONFIRMACAO` (número de leituras consecutivas para confirmar), `OBSTACULO_TEMPO_RE_MS` (duração da ré em ms), `OBSTACULO_ANGULO_GIRO` (ângulo de giro em graus), `OBSTACULO_DIST_LATERAL_ALVO` (distância lateral de contorno em cm) e `OBSTACULO_TOLERANCIA_ANGULO` (tolerância de giro em graus).
3. THE Controlador_FSM SHALL adicionar `cases` para `OBSTACULO_RE`, `OBSTACULO_GIRANDO` e `OBSTACULO_CONTORNO` no `switch(estadoAtual)` do `loop()`.
4. THE Controlador_FSM SHALL remover o `enum ModoObstaculo` e todas as variáveis de controle do `ESTADO_OBSTACULO` legado (`anguloInicialObstaculo`, `tempoInicioObstaculo`, `contadorContorno`) do escopo global de `robo_linha.ino`, transferindo o controle interno para `obstaculo.h`.

---

### Requisito 3: Validação Antirruído por Leituras Consecutivas

**User Story:** Como engenheiro de confiabilidade do robô OBR, quero que o sistema de obstáculo seja ativado somente após 3 leituras consecutivas do sonar frontal abaixo de 10 cm, para que reflexos espúrios e ruído elétrico não interrompam o seguimento de linha.

#### Critérios de Aceitação

1. WHEN o Controlador_FSM estiver no `ESTADO_LINHA` e o intervalo desde a última leitura de sonar for ≥ 50 ms, THE Sensor_Sonar_Frente SHALL ser consultado via `obterDistanciaFiltrada(sonarFrente)`.
2. WHEN `obterDistanciaFiltrada(sonarFrente)` retornar um valor ≤ `OBSTACULO_DIST_DETECCAO`, THE Sistema_Obstaculo SHALL incrementar um contador interno de leituras consecutivas.
3. WHEN `obterDistanciaFiltrada(sonarFrente)` retornar um valor > `OBSTACULO_DIST_DETECCAO`, THE Sistema_Obstaculo SHALL zerar o contador de leituras consecutivas.
4. WHEN o contador de leituras consecutivas atingir `OBSTACULO_LEITURAS_CONFIRMACAO` (valor padrão: 3), THE Sistema_Obstaculo SHALL sinalizar obstáculo confirmado, zerando o contador e preparando a transição de estado.
5. IF o obstáculo for confirmado e o `estadoAtual` for `ESTADO_LINHA`, THEN THE Controlador_FSM SHALL chamar `iniciarObstaculo()` de `obstaculo.h`, que registrará o Yaw inicial do Giroscopio_MPU, determinará o Lado_Desimpedido e transicionará `estadoAtual` para `OBSTACULO_RE`.

---

### Requisito 4: Ré Inicial Não-Bloqueante

**User Story:** Como engenheiro de controle do robô OBR, quero que o robô execute uma ré breve ao confirmar obstáculo usando `millis()`, para que o chassi ganhe distância de manobra antes do giro sem travar o loop principal.

#### Critérios de Aceitação

1. WHEN `estadoAtual` for `OBSTACULO_RE`, THE Sistema_Obstaculo SHALL acionar `controlarRodas(-VELOCIDADE_BASE, -VELOCIDADE_BASE)` para mover o robô em ré.
2. WHILE o tempo decorrido desde o início da ré for < `OBSTACULO_TEMPO_RE_MS`, THE Sistema_Obstaculo SHALL manter o comando de ré sem usar `delay()`.
3. WHEN o tempo decorrido atingir `OBSTACULO_TEMPO_RE_MS`, THE Sistema_Obstaculo SHALL chamar `controlarRodas(0, 0)`, registrar o Yaw atual do Giroscopio_MPU como referência do giro e transicionar `estadoAtual` para `OBSTACULO_GIRANDO`.
4. THE Sistema_Obstaculo SHALL registrar via `Serial.println()` as mensagens `[OBS] Re iniciada` e `[OBS] Re concluida. Iniciando giro.` nas respectivas transições.

---

### Requisito 5: Giro de 65° Validado pelo Giroscópio

**User Story:** Como engenheiro de controle do robô OBR, quero que o robô gire exatamente 65° para o Lado_Desimpedido usando o Yaw do MPU6050 como referência matemática, para que o posicionamento inicial do arco de contorno seja preciso e repetível.

#### Critérios de Aceitação

1. WHEN `estadoAtual` for `OBSTACULO_GIRANDO`, THE Sistema_Obstaculo SHALL calcular o ângulo-alvo como `anguloReferenciaGiro ± OBSTACULO_ANGULO_GIRO` (sinal definido pelo Lado_Desimpedido, seguindo a convenção do MPU: anti-horário positivo).
2. WHILE `abs(mpu.getAngleZ() - anguloAlvoGiro)` for > `OBSTACULO_TOLERANCIA_ANGULO`, THE Sistema_Obstaculo SHALL acionar `controlarRodas()` em sentidos opostos para girar no próprio eixo, com velocidade proporcional ao erro angular (velocidade mínima 60, máxima 150).
3. WHEN `abs(mpu.getAngleZ() - anguloAlvoGiro)` for ≤ `OBSTACULO_TOLERANCIA_ANGULO`, THE Sistema_Obstaculo SHALL chamar `controlarRodas(0, 0)` e transicionar `estadoAtual` para `OBSTACULO_CONTORNO`.
4. IF o tempo no estado `OBSTACULO_GIRANDO` ultrapassar 3000 ms, THEN THE Sistema_Obstaculo SHALL forçar a transição para `OBSTACULO_CONTORNO` como mecanismo de segurança, registrando `[OBS] Timeout giro. Forcando contorno.` via `Serial.println()`.
5. THE Sistema_Obstaculo SHALL registrar o ângulo atual via `Serial.print()` a cada 200 ms durante o estado `OBSTACULO_GIRANDO` para facilitar diagnóstico.

---

### Requisito 6: Arco de Contorno em Malha Fechada

**User Story:** Como engenheiro de controle do robô OBR, quero que o robô execute um arco de contorno mantendo distância lateral constante do obstáculo via sonar lateral, para que o contorno seja suave e o robô não colida nem se afaste demais do objeto.

#### Critérios de Aceitação

1. WHEN `estadoAtual` for `OBSTACULO_CONTORNO`, THE Sistema_Obstaculo SHALL ler periodicamente (intervalo ≥ 50 ms) a distância lateral via `obterDistanciaFiltrada()` no sonar correspondente ao Lado_Desimpedido oposto (sonar voltado para o obstáculo).
2. WHILE o Sensor_QTR não detectar Linha_Preta, THE Sistema_Obstaculo SHALL calcular um ajuste de curva baseado no erro entre a distância lateral lida e `OBSTACULO_DIST_LATERAL_ALVO`, aplicando `controlarRodas(velBase + ajuste, velBase - ajuste)` para manter o arco.
3. THE Sistema_Obstaculo SHALL limitar o ajuste de curva lateral a um valor máximo de ±80, para que nenhuma roda receba velocidade negativa durante o contorno normal.
4. WHEN a distância lateral lida for > `OBSTACULO_DIST_LATERAL_ALVO` (robô se afastando do obstáculo), THE Sistema_Obstaculo SHALL aumentar a velocidade da roda interna para curvar em direção ao obstáculo.
5. WHEN a distância lateral lida for < `OBSTACULO_DIST_LATERAL_ALVO` (robô se aproximando demais), THE Sistema_Obstaculo SHALL reduzir a velocidade da roda interna para abrir a curva.
6. IF o tempo no estado `OBSTACULO_CONTORNO` ultrapassar 10000 ms sem detectar Linha_Preta, THEN THE Sistema_Obstaculo SHALL parar os motores, registrar `[OBS] Timeout contorno. Retornando a ESTADO_LINHA.` e transicionar `estadoAtual` para `ESTADO_LINHA`.

---

### Requisito 7: Recuperação de Pista via Sensor QTR

**User Story:** Como operador do robô OBR em competição, quero que o robô detecte automaticamente a linha preta pelos sensores centrais do QTR-8A ao final do contorno e retome o seguimento PID, para que a missão continue sem intervenção humana.

#### Critérios de Aceitação

1. WHILE `estadoAtual` for `OBSTACULO_CONTORNO`, THE Sistema_Obstaculo SHALL verificar os valores dos sensores centrais do Sensor_QTR (sensores de índice 2, 3, 4 e 5 de `sensorValues[8]`) a cada ciclo de controle.
2. WHEN pelo menos 2 dos sensores centrais do Sensor_QTR retornarem valor > 650 (indicando Linha_Preta), THE Sistema_Obstaculo SHALL considerar a linha recuperada.
3. WHEN a linha for recuperada, THE Sistema_Obstaculo SHALL chamar `controlarRodas(0, 0)`, zerar `ultimoErro` e `contadorFalhas` no escopo de `robo_linha.ino` via variáveis `extern`, e transicionar `estadoAtual` para `ESTADO_LINHA`.
4. THE Sistema_Obstaculo SHALL registrar `[OBS] Linha recuperada! Retornando ao PID.` via `Serial.println()` ao detectar a recuperação.

---

### Requisito 8: Determinação do Lado Desimpedido

**User Story:** Como engenheiro de decisão do robô OBR, quero que o sistema determine automaticamente para qual lado girar com base na leitura dos sonares laterais no momento da confirmação do obstáculo, para que o robô sempre contorne pelo lado com mais espaço disponível.

#### Critérios de Aceitação

1. WHEN `iniciarObstaculo()` for chamado, THE Sistema_Obstaculo SHALL ler `obterDistanciaFiltrada(sonarEsq)` e `obterDistanciaFiltrada(sonarDir)` para comparar os espaços laterais.
2. WHEN `distanciaEsq` for > `distanciaDir`, THE Sistema_Obstaculo SHALL definir `Lado_Desimpedido` como esquerda (giro anti-horário: valor positivo no MPU).
3. WHEN `distanciaDir` for ≥ `distanciaEsq`, THE Sistema_Obstaculo SHALL definir `Lado_Desimpedido` como direita (giro horário: valor negativo no MPU).
4. THE Sistema_Obstaculo SHALL registrar via `Serial.print()` as distâncias laterais medidas e o Lado_Desimpedido escolhido para fins de diagnóstico.

---

### Requisito 9: Proibição de Delay Bloqueante

**User Story:** Como arquiteto de software do robô OBR, quero que toda temporização em `obstaculo.h` use exclusivamente `millis()`, para que o loop principal nunca seja bloqueado e os demais sensores e estados da FSM continuem respondendo.

#### Critérios de Aceitação

1. THE Sistema_Obstaculo SHALL implementar todas as temporizações (ré, timeout de giro, timeout de contorno, intervalo de leitura de sonar) usando comparações com `millis()` e variáveis do tipo `unsigned long`.
2. THE Sistema_Obstaculo SHALL não conter nenhuma chamada a `delay()` em qualquer função.
3. IF uma temporização em `obstaculo.h` necessitar esperar um evento, THEN THE Sistema_Obstaculo SHALL retornar o controle ao `loop()` imediatamente após salvar o instante de início em uma variável `unsigned long`, verificando a condição na próxima execução do `case` correspondente.
