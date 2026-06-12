# Implementation Plan: obstaculo-contorno-circular

## Overview

Refatoração completa da lógica de desvio de obstáculo do robô seguidor de linha OBR.
A abordagem legada (`ESTADO_OBSTACULO` + `enum ModoObstaculo`) é substituída por três estados dedicados na FSM (`OBSTACULO_RE`, `OBSTACULO_GIRANDO`, `OBSTACULO_CONTORNO`) e um novo módulo encapsulado em `obstaculo.h`.

A implementação segue a ordem de dependências: `config.h` → estrutura de `obstaculo.h` → cada função pública → integração em `robo_linha.ino`.

---

## Tasks

- [ ] 1. Atualizar `config.h` — novos estados, constantes e remoção de legado
  - [ ] 1.1 Substituir os estados de obstáculo no `enum EstadoRobo`
    - Remover `ESTADO_OBSTACULO_RE`, `ESTADO_OBSTACULO_GIRANDO`, `ESTADO_OBSTACULO_CONTORNO` e `ESTADO_OBSTACULO_BUSCA` do enum (nomes com prefixo `ESTADO_`)
    - Adicionar `OBSTACULO_RE`, `OBSTACULO_GIRANDO` e `OBSTACULO_CONTORNO` (sem prefixo `ESTADO_`) posicionados após o stub `ESTADO_OBSTACULO`
    - Manter `ESTADO_OBSTACULO` como stub de segurança comentado
    - _Arquivo: `config.h`_
    - _Requirements: 2.1_

  - [ ] 1.2 Adicionar as seis constantes do módulo de obstáculo
    - Adicionar bloco `// PARÂMETROS DO MÓDULO DE OBSTÁCULO` com as constantes:
      `OBSTACULO_DIST_DETECCAO 10`, `OBSTACULO_LEITURAS_CONFIRMACAO 3`,
      `OBSTACULO_TEMPO_RE_MS 300`, `OBSTACULO_ANGULO_GIRO 65.0f`,
      `OBSTACULO_DIST_LATERAL_ALVO 15`, `OBSTACULO_TOLERANCIA_ANGULO 3.0f`
    - Também garantir que `LEITURAS_CONSECUTIVAS_OBS` (já em uso no `.ino`) seja mantida ou substituída por `OBSTACULO_LEITURAS_CONFIRMACAO`
    - _Arquivo: `config.h`_
    - _Requirements: 2.2_

- [ ] 2. Criar estrutura base de `obstaculo.h`
  - [ ] 2.1 Criar o arquivo `obstaculo.h` do zero com guard, includes e externs
    - Escrever guard macro `#ifndef OBSTACULO_H / #define OBSTACULO_H / #endif`
    - Adicionar `#include <Arduino.h>` e `#include "config.h"` e `#include "sensores.h"`
    - Declarar `extern void controlarRodas(int vDir, int vEsq)`, `extern void pararMotores()`, `extern void tcaselect(uint8_t i)`
    - Declarar externs das variáveis de `robo_linha.ino`: `extern EstadoRobo estadoAtual`, `extern ModoLinha modoLinha`, `extern int ultimoErro`, `extern int contadorFalhas`
    - Adicionar comentário de bloco explicando a física do contorno circular (ré → giro → arco → recuperação)
    - _Arquivo: `obstaculo.h`_
    - _Requirements: 1.1, 1.2, 1.5_

  - [ ] 2.2 Declarar todas as variáveis estáticas internas do módulo
    - Declarar grupo do filtro antirruído: `static int contadorLeituras = 0`, `static unsigned long tempoUltimaLeituraFrente = 0`
    - Declarar grupo da ré: `static unsigned long tempoInicioRe = 0`
    - Declarar grupo do giro: `static float anguloAlvoGiro = 0.0f`, `static unsigned long tempoInicioGiro = 0`, `static unsigned long ultimoPrintGiro = 0`
    - Declarar grupo de decisão de lado: `static int8_t ladoDesimpedido = -1`, `static NewPing* sonarParaObstaculo = nullptr`
    - Declarar grupo do contorno: `static unsigned long tempoInicioContorno = 0`, `static unsigned long tempoUltimaLeituraLateral = 0`, `static int contadorLinhaDetectada = 0`
    - _Arquivo: `obstaculo.h`_
    - _Requirements: 1.1, 9.1_

- [ ] 3. Implementar `verificarObstaculo()` em `obstaculo.h`
  - [ ] 3.1 Escrever o corpo da função `bool verificarObstaculo()`
    - Se `(millis() - tempoUltimaLeituraFrente) < 50` → retornar `false` imediatamente (responde à Property 1)
    - Atualizar `tempoUltimaLeituraFrente = millis()`
    - Ler `d = obterDistanciaFiltrada(sonarFrente)`
    - Se `d == 0 || d >= MAX_DISTANCE` → zerar `contadorLeituras`, retornar `false`
    - Se `d <= OBSTACULO_DIST_DETECCAO` → incrementar `contadorLeituras`; se `contadorLeituras >= OBSTACULO_LEITURAS_CONFIRMACAO` → zerar contador e retornar `true`
    - Caso contrário → zerar `contadorLeituras`, retornar `false`
    - _Arquivo: `obstaculo.h`_
    - _Requirements: 3.1, 3.2, 3.3, 3.4_

  - [ ]* 3.2 Escrever testes de propriedade para `verificarObstaculo()`
    - **Property 1: Filtro antirruído respeita intervalo mínimo** — gerar `dt < 50`, confirmar retorno `false` e contador inalterado
    - **Validates: Requirements 3.1**
    - **Property 2: Leituras próximas incrementam o contador** — gerar `d ∈ [1, OBSTACULO_DIST_DETECCAO]`, confirmar incremento de 1
    - **Validates: Requirements 3.2**
    - **Property 3: Leituras distantes zeram o contador** — gerar `d > OBSTACULO_DIST_DETECCAO`, confirmar `contadorLeituras == 0`
    - **Validates: Requirements 3.3**
    - **Property 4: Confirmação após N leituras consecutivas** — sequência de N chamadas válidas, última retorna `true`
    - **Validates: Requirements 3.4**

- [ ] 4. Implementar `iniciarObstaculo()` em `obstaculo.h`
  - [ ] 4.1 Escrever o corpo da função `void iniciarObstaculo()`
    - Chamar `controlarRodas(0, 0)` para parada imediata
    - Ler `dEsq = obterDistanciaFiltrada(sonarEsq)` e `dDir = obterDistanciaFiltrada(sonarDir)`
    - Se `dEsq > dDir` → `ladoDesimpedido = +1`, `sonarParaObstaculo = &sonarDir`; senão → `ladoDesimpedido = -1`, `sonarParaObstaculo = &sonarEsq`
    - Imprimir distâncias e lado escolhido via `Serial.print()` (diagnóstico)
    - Calcular `anguloAlvoGiro = mpu.getAngleZ() + (ladoDesimpedido * OBSTACULO_ANGULO_GIRO)` usando convenção MPU (anti-horário positivo)
    - Setar `tempoInicioRe = millis()`, `estadoAtual = OBSTACULO_RE`
    - Imprimir `[OBS] Re iniciada` via `Serial.println()`
    - _Arquivo: `obstaculo.h`_
    - _Requirements: 3.5, 4.4, 8.1, 8.2, 8.3, 8.4_

  - [ ]* 4.2 Escrever teste de propriedade para escolha de lado
    - **Property 14: Escolha de lado é determinística e cobre todos os casos** — gerar `(dEsq, dDir)` arbitrários; verificar que `dEsq > dDir → ladoDesimpedido == +1 && sonarParaObstaculo == &sonarDir`; e `dDir >= dEsq → ladoDesimpedido == -1 && sonarParaObstaculo == &sonarEsq`
    - **Validates: Requirements 8.2, 8.3**

  - [ ]* 4.3 Escrever teste de propriedade para cálculo do ângulo-alvo
    - **Property 7: Ângulo-alvo calculado corretamente conforme lado e convenção MPU** — gerar `(yawRef: float, ladoDesimpedido: {-1,+1})`; verificar `anguloAlvoGiro == yawRef + (ladoDesimpedido * OBSTACULO_ANGULO_GIRO)`
    - **Validates: Requirements 5.1**

- [ ] 5. Implementar `executarRe()` em `obstaculo.h`
  - [ ] 5.1 Escrever o corpo da função `void executarRe()`
    - Chamar `controlarRodas(-VELOCIDADE_BASE, -VELOCIDADE_BASE)` para acionar ré
    - Se `(millis() - tempoInicioRe) < OBSTACULO_TEMPO_RE_MS` → retornar imediatamente (não-bloqueante)
    - Ao atingir o timeout: chamar `controlarRodas(0, 0)`
    - Recalcular `anguloAlvoGiro = mpu.getAngleZ() + (ladoDesimpedido * OBSTACULO_ANGULO_GIRO)` pós-ré (absorve drift)
    - Setar `tempoInicioGiro = millis()`, `ultimoPrintGiro = 0`, `estadoAtual = OBSTACULO_GIRANDO`
    - Imprimir `[OBS] Re concluida. Iniciando giro.` via `Serial.println()`
    - _Arquivo: `obstaculo.h`_
    - _Requirements: 4.1, 4.2, 4.3, 4.4, 9.1, 9.2, 9.3_

  - [ ]* 5.2 Escrever testes de propriedade para `executarRe()`
    - **Property 5: Ré mantém estado enquanto timeout não expira** — gerar `dt ∈ [0, OBSTACULO_TEMPO_RE_MS - 1]`; verificar `estadoAtual == OBSTACULO_RE` e `controlarRodas` chamada com ambos os parâmetros `== -VELOCIDADE_BASE`
    - **Validates: Requirements 4.1, 4.2**
    - **Property 6: Ré transiciona ao atingir o timeout** — gerar `dt >= OBSTACULO_TEMPO_RE_MS`; verificar `estadoAtual == OBSTACULO_GIRANDO` e `controlarRodas(0, 0)` chamada
    - **Validates: Requirements 4.3**

- [ ] 6. Implementar `executarGiro()` em `obstaculo.h`
  - [ ] 6.1 Escrever o corpo da função `void executarGiro()`
    - Ler `anguloAtual = mpu.getAngleZ()` (MPU já atualizado no topo do `loop()`)
    - Calcular `erroAngulo = anguloAlvoGiro - anguloAtual`
    - Se `abs(erroAngulo) > OBSTACULO_TOLERANCIA_ANGULO`:
      - Calcular `velGiro = constrain(60 + abs(erroAngulo) * 1.2, 60, 150)`
      - Se `ladoDesimpedido == +1` (anti-horário): `controlarRodas(-velGiro, +velGiro)`
      - Senão (horário): `controlarRodas(+velGiro, -velGiro)`
      - A cada 200 ms imprimir ângulo atual, alvo e erro via `Serial.print()`
    - Senão (alvo atingido): `controlarRodas(0, 0)`, inicializar contorno e setar `estadoAtual = OBSTACULO_CONTORNO`
    - Timeout independente: se `(millis() - tempoInicioGiro) > 3000` → forçar transição para `OBSTACULO_CONTORNO` com mensagem `[OBS] Timeout giro. Forcando contorno.`
    - Sub-rotina de inicialização do contorno: setar `tempoInicioContorno = millis()`, `tempoUltimaLeituraLateral = 0`, `contadorLinhaDetectada = 0`
    - _Arquivo: `obstaculo.h`_
    - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 9.1, 9.2, 9.3_

  - [ ]* 6.2 Escrever testes de propriedade para `executarGiro()`
    - **Property 8: Velocidade de giro proporcional ao erro angular** — gerar `E: float > OBSTACULO_TOLERANCIA_ANGULO`; verificar `velGiro == constrain(60 + abs(E) * 1.2, 60, 150)`
    - **Validates: Requirements 5.2**
    - **Property 9: Giro transiciona para contorno ao atingir tolerância angular** — gerar `(anguloAtual, anguloAlvoGiro)` com `abs(diff) <= OBSTACULO_TOLERANCIA_ANGULO`; verificar `estadoAtual == OBSTACULO_CONTORNO` e `controlarRodas(0, 0)` chamada
    - **Validates: Requirements 5.3**
    - **Property 10: Timeout de giro força transição para contorno** — gerar `dt > 3000`; verificar `estadoAtual == OBSTACULO_CONTORNO` independente do ângulo
    - **Validates: Requirements 5.4**

- [ ] 7. Implementar `executarContorno()` em `obstaculo.h`
  - [ ] 7.1 Escrever o corpo da função `void executarContorno()`
    - **Fase A — Verificação de linha (todo ciclo):** chamar `qtr.readLineBlack(sensorValues)`; contar sensores com `sensorValues[i] > 650` nos índices 2, 3, 4 e 5; se `count >= 2` → incrementar `contadorLinhaDetectada`; se `contadorLinhaDetectada >= 2` → `controlarRodas(0, 0)`, zerar `ultimoErro` e `contadorFalhas`, setar `modoLinha = SEGUINDO`, `estadoAtual = ESTADO_LINHA`, imprimir `[OBS] Linha recuperada! Retornando ao PID.` e retornar; senão → zerar `contadorLinhaDetectada`
    - **Fase B — Controle lateral (intervalo 50 ms):** verificar throttle `tempoUltimaLeituraLateral`; ler `distanciaLateral = obterDistanciaFiltrada(*sonarParaObstaculo)`; calcular `erroLateral = distanciaLateral - OBSTACULO_DIST_LATERAL_ALVO`; `ajuste = constrain(erroLateral * 2, -80, 80)`; chamar `controlarRodas(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste)`
    - **Fase C — Timeout de segurança:** se `(millis() - tempoInicioContorno) > 10000` → `controlarRodas(0, 0)`, `estadoAtual = ESTADO_LINHA`, imprimir `[OBS] Timeout contorno. Retornando a ESTADO_LINHA.`
    - _Arquivo: `obstaculo.h`_
    - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 7.1, 7.2, 7.3, 7.4, 9.1, 9.2, 9.3_

  - [ ]* 7.2 Escrever testes de propriedade para `executarContorno()`
    - **Property 11: Ajuste lateral segue fórmula com saturação** — gerar `dL: int ∈ [0, MAX_DISTANCE]`; verificar `ajuste == constrain((dL - OBSTACULO_DIST_LATERAL_ALVO) * 2, -80, 80)` e `controlarRodas(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste)`
    - **Validates: Requirements 6.2, 6.3, 6.4, 6.5**
    - **Property 12: Timeout de contorno retorna para ESTADO_LINHA** — gerar `dt > 10000`; verificar `estadoAtual == ESTADO_LINHA` e `controlarRodas(0, 0)`
    - **Validates: Requirements 6.6**
    - **Property 13: Detecção de linha usa limiar e índices corretos** — gerar `sensorValues[8]` aleatório; verificar que condição de recuperação ↔ `count(indices 2-5 > 650) >= 2`; índices 0, 1, 6 e 7 não influenciam
    - **Validates: Requirements 7.1, 7.2**

- [ ] 8. Integrar `obstaculo.h` em `robo_linha.ino` — include, remoção de legado, gatilho e cases
  - [ ] 8.1 Adicionar `#include "obstaculo.h"` e remover variáveis globais legadas
    - Adicionar `#include "obstaculo.h"` após `#include "resgate.h"` no topo do arquivo
    - Remover as variáveis globais de obstáculo legadas que foram internalizadas no módulo: comentário de bloco indicando a remoção e qualquer variável declarada como `ModoObstaculo`, `anguloInicialObstaculo`, `tempoInicioObstaculo`, `contadorContorno`
    - Remover (ou comentar com nota) `contadorLeiturasFrontal` e `tempoUltimoSonar` — substituídos pelos homólogos internos de `obstaculo.h`
    - _Arquivo: `robo_linha.ino`_
    - _Requirements: 2.4, 1.3_

  - [ ] 8.2 Substituir o Gatilho 3 em `ESTADO_LINHA` pela chamada delegada
    - Localizar o bloco `// GATILHO 3: OBSTÁCULO FRONTAL (SONAR)` no `case ESTADO_LINHA`
    - Substituir toda a lógica inline de leitura de sonar, incremento de `contadorLeiturasFrontal` e chamada a `iniciarDesvioObstaculo()` por:
      ```cpp
      if (verificarObstaculo()) {
        iniciarObstaculo();
        break;
      }
      ```
    - _Arquivo: `robo_linha.ino`_
    - _Requirements: 1.3, 3.1, 3.2, 3.3, 3.4, 3.5_

  - [ ] 8.3 Substituir os cases de obstáculo no `switch(estadoAtual)`
    - Remover os cases `ESTADO_OBSTACULO_RE`, `ESTADO_OBSTACULO_GIRANDO`, `ESTADO_OBSTACULO_CONTORNO` e `ESTADO_OBSTACULO_BUSCA` (com prefixo `ESTADO_`) que chamavam `executarRe()`, `executarGiro65()`, `executarContornoArco()` e `executarBuscaLinha()`
    - Adicionar os três novos cases sem prefixo `ESTADO_`:
      ```cpp
      case OBSTACULO_RE:
        executarRe();
        break;
      case OBSTACULO_GIRANDO:
        executarGiro();
        break;
      case OBSTACULO_CONTORNO:
        executarContorno();
        break;
      ```
    - _Arquivo: `robo_linha.ino`_
    - _Requirements: 2.3_

  - [ ] 8.4 Atualizar `getNomeEstado()` com os novos nomes de estado
    - Remover os cases com nomes antigos (`ESTADO_OBSTACULO_RE`, etc.) da função `getNomeEstado()`
    - Adicionar cases para `OBSTACULO_RE` → `"OBSTACULO_RE"`, `OBSTACULO_GIRANDO` → `"OBSTACULO_GIRANDO"`, `OBSTACULO_CONTORNO` → `"OBSTACULO_CONTORNO"`
    - _Arquivo: `robo_linha.ino`_
    - _Requirements: 1.3_

- [ ] 9. Checkpoint — Verificar compilação e consistência geral
  - Garantir que o projeto compila sem erros no Arduino IDE para o target Arduino Mega 2560
  - Confirmar que não há referências pendentes aos nomes de estados legados (`ESTADO_OBSTACULO_RE`, `ModoObstaculo`, `iniciarDesvioObstaculo`, `executarGiro65`, `executarContornoArco`, `executarBuscaLinha`)
  - Verificar via Serial Monitor que os logs aparecem na ordem correta: `[OBS] Re iniciada` → `[OBS] Re concluida. Iniciando giro.` → prints de `[OBS-GIRO]` → `[OBS] Linha recuperada! Retornando ao PID.`
  - Perguntar ao usuário se há ajustes necessários antes de fechar a feature.

---

## Notes

- Tarefas marcadas com `*` são opcionais e podem ser puladas para um MVP mais rápido
- Cada tarefa referencia os requisitos específicos para rastreabilidade
- Os checkpoints garantem validação incremental
- As propriedades PBT devem ser implementadas em um harness de teste no host (não no Arduino) usando mocks de `millis()`, `controlarRodas()`, `obterDistanciaFiltrada()`, `mpu.getAngleZ()` e `qtr.readLineBlack()` — conforme descrito na seção Testing Strategy do design
- As tarefas 1 e 2 são pré-requisitos absolutos de todas as demais
- A tarefa 8 só pode ser executada após todas as funções de `obstaculo.h` estarem implementadas (tarefas 3–7)

---

## Task Dependency Graph

```json
{
  "waves": [
    { "id": 0, "tasks": ["1.1", "1.2"] },
    { "id": 1, "tasks": ["2.1", "2.2"] },
    { "id": 2, "tasks": ["3.1", "4.1"] },
    { "id": 3, "tasks": ["3.2", "4.2", "4.3", "5.1"] },
    { "id": 4, "tasks": ["5.2", "6.1"] },
    { "id": 5, "tasks": ["6.2", "7.1"] },
    { "id": 6, "tasks": ["7.2", "8.1"] },
    { "id": 7, "tasks": ["8.2", "8.3", "8.4"] }
  ]
}
```
