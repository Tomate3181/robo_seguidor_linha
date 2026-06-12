# Design Document — obstaculo-contorno-circular

## Overview

A feature refatora completamente a lógica de desvio de obstáculo do robô seguidor de linha OBR. A abordagem legada (`ESTADO_OBSTACULO` + `enum ModoObstaculo`) é substituída por três estados dedicados na FSM (`OBSTACULO_RE`, `OBSTACULO_GIRANDO`, `OBSTACULO_CONTORNO`) e um novo módulo encapsulado em `obstaculo.h`.

A estratégia de contorno é composta por quatro etapas sequenciais:

1. **Confirmação** — filtro antirruído de 3 leituras consecutivas do sonar frontal (≤ 10 cm).
2. **Ré** — recuo breve de 300 ms para criar distância de manobra.
3. **Giro** — rotação de 65° no próprio eixo, validada matematicamente pelo yaw do MPU6050.
4. **Contorno** — arco curvilíneo com malha fechada de distância lateral via sonar, até detecção da linha preta pelo QTR-8A.

Toda temporização é não-bloqueante, baseada em `millis()`. Nenhuma chamada a `delay()` é permitida em `obstaculo.h`.

---

## Architecture

### Diagrama de Dependências

```
robo_linha.ino
  ├── config.h        (enum EstadoRobo, constantes)
  ├── motores.h       (controlarRodas, pararMotores)
  ├── sensores.h      (qtr, mpu, sonarFrente/Esq/Dir, obterDistanciaFiltrada)
  ├── resgate.h       (lógica zona de resgate — inalterado)
  └── obstaculo.h     (NOVO — toda a lógica de contorno circular)
```

`obstaculo.h` não inclui nenhuma biblioteca de hardware diretamente. Ele depende das instâncias globais declaradas em `sensores.h` e das funções de `motores.h`, acessadas através das inclusões já feitas em `robo_linha.ino` (que atua como unidade de compilação única no paradigma Arduino `.ino`).

### Diagrama de Transição de Estados (FSM)

```
ESTADO_LINHA
  │  [verificarObstaculo() == true]
  │  → iniciarObstaculo()
  ▼
OBSTACULO_RE ─── (millis() - tempoInicioRe >= OBSTACULO_TEMPO_RE_MS) ──▶ OBSTACULO_GIRANDO
                                                                               │
                                               (abs(erroAngulo) ≤ OBSTACULO_TOLERANCIA_ANGULO)
                                               OU (millis() - tempoInicioGiro > 3000)
                                                                               │
                                                                               ▼
                                                                     OBSTACULO_CONTORNO
                                                                               │
                                         ┌─────────────────────────────────────┤
                                         │                                     │
                          (QTR: ≥2 sensores centrais > 650)    (millis() - tempoInicioContorno > 10000)
                                         │                                     │
                                         ▼                                     ▼
                                   ESTADO_LINHA                          ESTADO_LINHA
                                (PID retomado)                         (timeout — segurança)
```

---

## Components and Interfaces

### Modificações em `config.h`

#### 1. Novos valores no `enum EstadoRobo`

Adicionar após `ESTADO_OBSTACULO`:

```cpp
enum EstadoRobo {
  ESTADO_CALIBRACAO,
  ESTADO_LINHA,
  ESTADO_VALIDANDO_SILVER_TAPE,
  ESTADO_ZONA_RESGATE,
  ESTADO_VERDE,
  ESTADO_VERMELHO,
  ESTADO_OBSTACULO,      // mantido como stub de segurança
  OBSTACULO_RE,          // NOVO: executa ré não-bloqueante
  OBSTACULO_GIRANDO,     // NOVO: giro de 65° validado pelo MPU
  OBSTACULO_CONTORNO     // NOVO: arco de contorno em malha fechada
};
```

#### 2. Remoção do `enum ModoObstaculo`

O bloco abaixo é **removido** de `config.h`:

```cpp
// REMOVER:
enum ModoObstaculo {
  GIRO_INICIAL,
  CONTORNO_LATERAL,
  BUSCA_LINHA
};
```

#### 3. Seis novas constantes de obstáculo

Adicionar na seção de parâmetros de controle:

```cpp
// ==============================================================================
// PARÂMETROS DO MÓDULO DE OBSTÁCULO
// ==============================================================================
#define OBSTACULO_DIST_DETECCAO        10      // cm — limiar de detecção do sonar frontal
#define OBSTACULO_LEITURAS_CONFIRMACAO  3      // nº de leituras consecutivas para confirmar
#define OBSTACULO_TEMPO_RE_MS         300      // ms — duração da ré inicial
#define OBSTACULO_ANGULO_GIRO         65.0f    // graus — ângulo do giro inicial
#define OBSTACULO_DIST_LATERAL_ALVO    15      // cm — distância alvo durante o contorno
#define OBSTACULO_TOLERANCIA_ANGULO    3.0f    // graus — tolerância do controlador de giro
```

---

### Novo arquivo `obstaculo.h`

#### API Pública (5 funções inline)

```cpp
// Chamada no ESTADO_LINHA (gatilho antirruído) — retorna true se obstáculo confirmado
bool verificarObstaculo();

// Chamada quando verificarObstaculo() retorna true — inicia a sequência de contorno
void iniciarObstaculo();

// Chamada no case OBSTACULO_RE do switch(estadoAtual)
void executarRe();

// Chamada no case OBSTACULO_GIRANDO do switch(estadoAtual)
void executarGiro();

// Chamada no case OBSTACULO_CONTORNO do switch(estadoAtual)
void executarContorno();
```

#### Variáveis Internas (estáticas/arquivo-escopo)

```cpp
// --- Filtro antirruído (verificarObstaculo) ---
static int            contadorLeituras         = 0;
static unsigned long  tempoUltimaLeituraFrente  = 0;

// --- Estado OBSTACULO_RE ---
static unsigned long  tempoInicioRe             = 0;

// --- Estado OBSTACULO_GIRANDO ---
static float          anguloAlvoGiro            = 0.0f;
static unsigned long  tempoInicioGiro           = 0;
static unsigned long  ultimoPrintGiro           = 0;

// --- Decisão de lado ---
static int8_t         ladoDesimpedido           = -1;  // -1=direita, +1=esquerda
static NewPing*       sonarParaObstaculo        = nullptr; // aponta para sonarDir ou sonarEsq

// --- Estado OBSTACULO_CONTORNO ---
static unsigned long  tempoInicioContorno       = 0;
static unsigned long  tempoUltimaLeituraLateral = 0;
static int            contadorLinhaDetectada    = 0;
```

#### Declarações `extern` necessárias

```cpp
// Variáveis de robo_linha.ino
extern EstadoRobo     estadoAtual;
extern int            ultimoErro;
extern int            contadorFalhas;
extern ModoLinha      modoLinha;

// Objetos de sensores.h (instanciados lá, referenciados aqui)
// qtr, sensorValues, mpu, sonarFrente, sonarEsq, sonarDir
// → acessíveis por ser a mesma unidade de compilação (.ino inclui tudo)
```

> **Nota sobre `tempoUltimoSonar`:** A variável `tempoUltimoSonar` de `robo_linha.ino` é **substituída** por `tempoUltimaLeituraFrente` interno a `obstaculo.h`. O gatilho 3 em `ESTADO_LINHA` não fará mais leitura direta do sonar — delegará para `verificarObstaculo()`.

---

## Data Models

### Estrutura de Estado Interno do Módulo

O módulo não usa `struct` explícita; o estado é composto pelas variáveis estáticas descritas acima. O ciclo de vida é:

```
[boot] → variáveis inicializadas com zeros/nullptr
       → verificarObstaculo() acumula contadorLeituras
       → iniciarObstaculo() seta tempoInicioRe, anguloAlvoGiro, ladoDesimpedido, sonarParaObstaculo
       → executarRe() usa tempoInicioRe
       → executarGiro() usa anguloAlvoGiro, tempoInicioGiro
       → executarContorno() usa sonarParaObstaculo, tempoInicioContorno, contadorLinhaDetectada
       → transição para ESTADO_LINHA reseta contadorLinhaDetectada
```

### Convenção do MPU6050 (crítico para o giro)

- Giro **anti-horário** (vista de cima) → `mpu.getAngleZ()` **aumenta** (positivo).
- Giro **horário** → `mpu.getAngleZ()` **diminui** (negativo).
- `ladoDesimpedido = +1` → esquerda → giro anti-horário → `anguloAlvoGiro = yawAtual + 65.0`.
- `ladoDesimpedido = -1` → direita → giro horário → `anguloAlvoGiro = yawAtual - 65.0`.
- Fórmula unificada: `anguloAlvoGiro = mpu.getAngleZ() + (ladoDesimpedido * OBSTACULO_ANGULO_GIRO)`.

### Mapeamento Sonar × Lado

| `ladoDesimpedido` | Giro para | Obstáculo fica à | Sonar de contorno |
|---|---|---|---|
| `+1` (esquerda) | anti-horário | direita | `sonarDir` |
| `-1` (direita) | horário | esquerda | `sonarEsq` |

---

## Fluxo de Controle Detalhado

### `verificarObstaculo()` — gatilho no ESTADO_LINHA

```
1. Se (millis() - tempoUltimaLeituraFrente) < 50 → retorna false (aguarda intervalo)
2. tempoUltimaLeituraFrente = millis()
3. d = obterDistanciaFiltrada(sonarFrente)
4. Se d == 0 ou d >= MAX_DISTANCE → trata como livre → goto 6
5. Se d <= OBSTACULO_DIST_DETECCAO:
       contadorLeituras++
       Se contadorLeituras >= OBSTACULO_LEITURAS_CONFIRMACAO:
           contadorLeituras = 0
           retorna true  ← OBSTÁCULO CONFIRMADO
       retorna false
6. contadorLeituras = 0
   retorna false
```

### `iniciarObstaculo()` — chamada quando verificarObstaculo() retorna true

```
1. controlarRodas(0, 0)  — para imediatamente
2. dEsq = obterDistanciaFiltrada(sonarEsq)
   dDir = obterDistanciaFiltrada(sonarDir)
3. Se dEsq > dDir:
       ladoDesimpedido = +1   (esquerda — mais espaço)
       sonarParaObstaculo = &sonarDir  (obstáculo ficará à direita)
   Senão:
       ladoDesimpedido = -1   (direita)
       sonarParaObstaculo = &sonarEsq
4. Serial.print/println das distâncias e lado escolhido (diagnóstico)
5. anguloAlvoGiro = mpu.getAngleZ() + (ladoDesimpedido * OBSTACULO_ANGULO_GIRO)
6. tempoInicioRe = millis()
7. estadoAtual = OBSTACULO_RE
8. Serial.println("[OBS] Re iniciada")
```

### `executarRe()` — case OBSTACULO_RE

```
1. controlarRodas(-VELOCIDADE_BASE, -VELOCIDADE_BASE)  — aciona ré
2. Se (millis() - tempoInicioRe) < OBSTACULO_TEMPO_RE_MS → retorna (não-bloqueante)
3. Tempo atingido:
   a. controlarRodas(0, 0)
   b. Recalcula alvo com yaw pós-ré:
      anguloAlvoGiro = mpu.getAngleZ() + (ladoDesimpedido * OBSTACULO_ANGULO_GIRO)
      — necessário pois a ré pode causar drift de alguns graus no yaw
   c. tempoInicioGiro = millis()
   d. ultimoPrintGiro = 0
   e. estadoAtual = OBSTACULO_GIRANDO
   f. Serial.println("[OBS] Re concluida. Iniciando giro.")
```

### `executarGiro()` — case OBSTACULO_GIRANDO

```
1. anguloAtual = mpu.getAngleZ()  — MPU já foi atualizado no topo do loop()
2. erroAngulo = anguloAlvoGiro - anguloAtual

3. Se abs(erroAngulo) > OBSTACULO_TOLERANCIA_ANGULO:
   a. velGiro = constrain(60 + abs(erroAngulo) * 1.2, 60, 150)
   b. Se ladoDesimpedido == +1 (anti-horário → esq frente, dir ré):
          controlarRodas(-velGiro, +velGiro)
      Senão (horário → esq ré, dir frente):
          controlarRodas(+velGiro, -velGiro)
   c. Se (millis() - ultimoPrintGiro) >= 200:
          ultimoPrintGiro = millis()
          Serial.print("[OBS-GIRO] Atual: "); Serial.print(anguloAtual);
          Serial.print(" Alvo: "); Serial.print(anguloAlvoGiro);
          Serial.print(" Erro: "); Serial.println(erroAngulo)

4. Senão (alvo atingido):
   a. controlarRodas(0, 0)
   b. → inicializar_contorno()  [ver abaixo]
   c. estadoAtual = OBSTACULO_CONTORNO

5. TIMEOUT (independente do ângulo):
   Se (millis() - tempoInicioGiro) > 3000:
       controlarRodas(0, 0)
       → inicializar_contorno()
       Serial.println("[OBS] Timeout giro. Forcando contorno.")
       estadoAtual = OBSTACULO_CONTORNO

— Sub-rotina inicializar_contorno():
   tempoInicioContorno = millis()
   tempoUltimaLeituraLateral = 0
   contadorLinhaDetectada = 0
```

> **Direção do giro vs. `controlarRodas`:**
> - Anti-horário (`ladoDesimpedido == +1`): roda direita para frente, esquerda para trás → `controlarRodas(-velGiro, +velGiro)` (vDir=esquerda, vEsq=direita — verificar assinatura: `controlarRodas(vDir, vEsq)` onde vDir controla FR/BR e vEsq controla FL/BL).
> - Horário (`ladoDesimpedido == -1`): roda esquerda para frente, direita para trás → `controlarRodas(+velGiro, -velGiro)`.

### `executarContorno()` — case OBSTACULO_CONTORNO

```
FASE A — Verificação de recuperação de linha (todo ciclo):
1. qtr.readLineBlack(sensorValues)
2. Conta sensores centrais com valor > 650: índices 2, 3, 4, 5
3. Se count >= 2:
       contadorLinhaDetectada++
       Se contadorLinhaDetectada >= 2:   — anti-noise adicional
           controlarRodas(0, 0)
           ultimoErro = 0
           contadorFalhas = 0
           modoLinha = SEGUINDO
           estadoAtual = ESTADO_LINHA
           Serial.println("[OBS] Linha recuperada! Retornando ao PID.")
           retorna
   Senão:
       contadorLinhaDetectada = 0

FASE B — Controle lateral (intervalo 50ms):
4. Se (millis() - tempoUltimaLeituraLateral) < 50 → pula leitura lateral
5. tempoUltimaLeituraLateral = millis()
6. distanciaLateral = obterDistanciaFiltrada(*sonarParaObstaculo)
7. erroLateral = distanciaLateral - OBSTACULO_DIST_LATERAL_ALVO
8. ajuste = constrain(erroLateral * 2, -80, 80)

   Física do ajuste (sonarParaObstaculo aponta para o lado do obstáculo):
   — erroLateral > 0 (robô afastou): ajuste > 0
       → controlarRodas(VELBASE + ajuste, VELBASE - ajuste)
       → roda do lado do obstáculo (vDir se obstáculo à direita) mais rápida
       → curva em direção ao obstáculo ✓
   — erroLateral < 0 (robô aproximou): ajuste < 0
       → roda do lado do obstáculo mais lenta
       → abre a curva, afasta do obstáculo ✓

9. controlarRodas(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste)

FASE C — Timeout de segurança:
10. Se (millis() - tempoInicioContorno) > 10000:
        controlarRodas(0, 0)
        estadoAtual = ESTADO_LINHA
        Serial.println("[OBS] Timeout contorno. Retornando a ESTADO_LINHA.")
```

> **Nota sobre a física do ajuste:** `sonarParaObstaculo` aponta para o sonar voltado para o obstáculo. Quando o obstáculo está à direita (`ladoDesimpedido = -1`, usou `sonarEsq` ← errado, ver tabela), revisar: se foi para esquerda (`ladoDesimpedido = +1`), obstáculo está à direita, usa `sonarDir`. O ajuste positivo (robô afastou) resulta em `controlarRodas(VELBASE + ajuste, VELBASE - ajuste)` onde `vDir > vEsq`, fazendo o robô curvar para a esquerda — em direção ao obstáculo que está à direita. ✓

---

## Modificações em `robo_linha.ino`

### 1. Include adicional

```cpp
#include "resgate.h"
#include "obstaculo.h"   // ← ADICIONAR após resgate.h
```

### 2. Remoção de variáveis globais legadas

Remover do escopo global:

```cpp
// REMOVER estas 4 linhas:
ModoObstaculo modoObstaculo = GIRO_INICIAL;
float anguloInicialObstaculo = 0;
unsigned long tempoInicioObstaculo = 0;
int contadorContorno = 0;
// A variável tempoUltimoSonar também pode ser removida — substituída por
// tempoUltimaLeituraFrente dentro de obstaculo.h
```

### 3. Substituição do Gatilho 3 em `ESTADO_LINHA`

**Antes (código legado):**
```cpp
// GATILHO 3: OBSTÁCULO FRONTAL (SONAR)
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
```

**Depois (novo código):**
```cpp
// GATILHO 3: OBSTÁCULO FRONTAL (SONAR)
if (verificarObstaculo()) {
  iniciarObstaculo();
  break;
}
```

### 4. Substituição do `case ESTADO_OBSTACULO` pelos 3 novos cases

**Remover** o bloco `case ESTADO_OBSTACULO: { switch(modoObstaculo) { ... } }` inteiro.

**Adicionar** no `switch(estadoAtual)`:

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

// Stub de segurança — caso o estado legado seja atingido por algum motivo
case ESTADO_OBSTACULO:
  estadoAtual = ESTADO_LINHA;
  break;
```

### 5. Atualização da função `getNomeEstado()` (debug)

```cpp
String getNomeEstado(EstadoRobo e) {
  switch(e) {
    case ESTADO_CALIBRACAO:            return "CALIBRACAO";
    case ESTADO_LINHA:                 return "LINHA";
    case ESTADO_VERDE:                 return "VERDE";
    case ESTADO_VERMELHO:              return "VERMELHO";
    case ESTADO_OBSTACULO:             return "OBSTACULO_LEGADO";
    case OBSTACULO_RE:                 return "OBSTACULO_RE";       // NOVO
    case OBSTACULO_GIRANDO:            return "OBSTACULO_GIRANDO";  // NOVO
    case OBSTACULO_CONTORNO:           return "OBSTACULO_CONTORNO"; // NOVO
    default:                           return "DESCONHECIDO";
  }
}
```

---

## Correctness Properties

*Uma propriedade é uma característica ou comportamento que deve ser verdadeiro em todas as execuções válidas de um sistema — essencialmente, uma declaração formal sobre o que o sistema deve fazer. Propriedades servem como ponte entre especificações legíveis por humanos e garantias de corretude verificáveis automaticamente.*

As funções de `obstaculo.h` operam sobre dados numéricos (distâncias, ângulos, tempo) com lógica de controle pura, tornando Property-Based Testing (PBT) aplicável para validar as invariantes de controle.

### Reflexão sobre redundâncias

Após análise do prework:

- **3.2 e 3.3** (incrementar/zerar contador) são propriedades complementares, não redundantes — cobrem os dois ramos do mesmo `if`.
- **3.4** (confirmação após N leituras) implica o comportamento acumulado de 3.2, mas testa uma propriedade de sequência diferente — mantida separada.
- **4.2 e 4.3** (manter ré / transicionar) são complementares — cobrem `<` e `>=` do mesmo limiar de tempo.
- **5.1** (cálculo do ângulo-alvo) e **5.2** (velocidade proporcional) são independentes — mantidas.
- **6.2, 6.3, 6.4, 6.5** — 6.3, 6.4 e 6.5 são **subsuministradas** por 6.2 (a fórmula completa com `constrain` já cobre os casos extremos e a direção). Consolidadas em uma única propriedade.
- **8.2 e 8.3** (escolha de lado) — complementares e simétricas, consolidadas em uma propriedade.

---

### Property 1: Filtro antirruído respeita intervalo mínimo

*Para qualquer* instante de chamada onde o delta desde `tempoUltimaLeituraFrente` seja inferior a 50 ms, `verificarObstaculo()` deve retornar `false` sem incrementar `contadorLeituras` e sem consultar o sonar.

**Validates: Requirements 3.1**

---

### Property 2: Leituras próximas incrementam o contador

*Para qualquer* distância `d` onde `0 < d <= OBSTACULO_DIST_DETECCAO` retornada por `obterDistanciaFiltrada(sonarFrente)` (com intervalo de 50 ms já decorrido), `verificarObstaculo()` deve incrementar `contadorLeituras` em exatamente 1.

**Validates: Requirements 3.2**

---

### Property 3: Leituras distantes zeram o contador

*Para qualquer* distância `d` onde `d > OBSTACULO_DIST_DETECCAO` (e `d < MAX_DISTANCE`), `verificarObstaculo()` deve zerar `contadorLeituras`, independentemente do valor anterior do contador.

**Validates: Requirements 3.3**

---

### Property 4: Confirmação após N leituras consecutivas

*Para qualquer* sequência de exatamente `OBSTACULO_LEITURAS_CONFIRMACAO` chamadas consecutivas a `verificarObstaculo()` com intervalo ≥ 50 ms e distância ≤ `OBSTACULO_DIST_DETECCAO`, a última chamada da sequência deve retornar `true` e zerar `contadorLeituras`.

**Validates: Requirements 3.4**

---

### Property 5: Ré mantém estado enquanto timeout não expira

*Para qualquer* par `(tempoInicioRe, tempoAtual)` onde `tempoAtual - tempoInicioRe < OBSTACULO_TEMPO_RE_MS`, `executarRe()` não deve alterar `estadoAtual` (permanece `OBSTACULO_RE`) e deve acionar `controlarRodas` com ambos os parâmetros negativos e iguais a `-VELOCIDADE_BASE`.

**Validates: Requirements 4.1, 4.2**

---

### Property 6: Ré transiciona ao atingir o timeout

*Para qualquer* par `(tempoInicioRe, tempoAtual)` onde `tempoAtual - tempoInicioRe >= OBSTACULO_TEMPO_RE_MS`, `executarRe()` deve setar `estadoAtual = OBSTACULO_GIRANDO` e chamar `controlarRodas(0, 0)`.

**Validates: Requirements 4.3**

---

### Property 7: Ângulo-alvo calculado corretamente conforme lado e convenção MPU

*Para qualquer* ângulo de referência `yawRef` (float arbitrário) e qualquer `ladoDesimpedido` ∈ {-1, +1}, o ângulo-alvo calculado deve ser exatamente `yawRef + (ladoDesimpedido * OBSTACULO_ANGULO_GIRO)`.

**Validates: Requirements 5.1**

---

### Property 8: Velocidade de giro proporcional ao erro angular

*Para qualquer* erro angular `E` onde `abs(E) > OBSTACULO_TOLERANCIA_ANGULO`, a velocidade de giro calculada em `executarGiro()` deve ser `constrain(60 + abs(E) * 1.2, 60, 150)`.

**Validates: Requirements 5.2**

---

### Property 9: Giro transiciona para contorno ao atingir tolerância angular

*Para qualquer* par `(anguloAtual, anguloAlvoGiro)` onde `abs(anguloAlvoGiro - anguloAtual) <= OBSTACULO_TOLERANCIA_ANGULO`, `executarGiro()` deve setar `estadoAtual = OBSTACULO_CONTORNO` e chamar `controlarRodas(0, 0)`.

**Validates: Requirements 5.3**

---

### Property 10: Timeout de giro força transição para contorno

*Para qualquer* par `(tempoInicioGiro, tempoAtual)` onde `tempoAtual - tempoInicioGiro > 3000`, `executarGiro()` deve setar `estadoAtual = OBSTACULO_CONTORNO` independente do ângulo atual.

**Validates: Requirements 5.4**

---

### Property 11: Ajuste lateral segue fórmula com saturação

*Para qualquer* distância lateral `dL` retornada pelo sonar de contorno, o ajuste aplicado deve ser `constrain((dL - OBSTACULO_DIST_LATERAL_ALVO) * 2, -80, 80)`, com `controlarRodas` recebendo `(VELOCIDADE_BASE + ajuste, VELOCIDADE_BASE - ajuste)`. Isso garante:
- `dL > ALVO` → ajuste > 0 → roda vDir mais rápida → curva em direção ao obstáculo.
- `dL < ALVO` → ajuste < 0 → roda vDir mais lenta → abre a curva.
- `|erroLateral| > 40` → ajuste saturado em ±80 → nenhuma roda recebe velocidade negativa durante contorno normal.

**Validates: Requirements 6.2, 6.3, 6.4, 6.5**

---

### Property 12: Timeout de contorno retorna para ESTADO_LINHA

*Para qualquer* par `(tempoInicioContorno, tempoAtual)` onde `tempoAtual - tempoInicioContorno > 10000`, `executarContorno()` deve setar `estadoAtual = ESTADO_LINHA` e chamar `controlarRodas(0, 0)`.

**Validates: Requirements 6.6**

---

### Property 13: Detecção de linha usa limiar e índices corretos

*Para qualquer* array `sensorValues[8]` gerado aleatoriamente, a condição de "linha recuperada" deve ser verdadeira se e somente se pelo menos 2 dos valores nos índices 2, 3, 4 e 5 forem > 650. Índices 0, 1, 6 e 7 não devem influenciar a condição.

**Validates: Requirements 7.1, 7.2**

---

### Property 14: Escolha de lado é determinística e cobre todos os casos

*Para qualquer* par de distâncias `(dEsq, dDir)` medidas pelos sonares laterais:
- Se `dEsq > dDir` → `ladoDesimpedido == +1` (esquerda) e `sonarParaObstaculo == &sonarDir`.
- Se `dDir >= dEsq` → `ladoDesimpedido == -1` (direita) e `sonarParaObstaculo == &sonarEsq`.

Esta propriedade cobre o domínio completo (não há par sem resposta definida).

**Validates: Requirements 8.2, 8.3**

---

## Error Handling

### Leituras de sonar inválidas

`obterDistanciaFiltrada()` já retorna `MAX_DISTANCE` quando `ping_cm()` retorna 0 (sem eco). Em `verificarObstaculo()`, leituras `== 0` ou `>= MAX_DISTANCE` são tratadas como "livre" (caminho feliz — sem obstáculo).

Em `executarContorno()`, se o sonar lateral retornar `MAX_DISTANCE` (obstáculo saiu do alcance), `erroLateral = MAX_DISTANCE - ALVO` é muito grande, mas o `constrain(±80)` impede ajuste excessivo. O robô curva suavemente em direção à última posição conhecida do obstáculo.

### Drift do MPU6050

O MPU é atualizado no topo de cada `loop()` via `tcaselect(CANAL_GY521); mpu.update()`. O ângulo-alvo é recalculado após a ré (pós drift da ré) para absorver erros acumulados.

### Timeout em cascata

Se o giro falhar (timeout 3 s) e o contorno também falhar (timeout 10 s), o robô retorna a `ESTADO_LINHA` pelo mecanismo de timeout de contorno. O PID retoma o seguimento de linha a partir do ponto atual.

### I2C timeout (tratado em `robo_linha.ino`)

O mecanismo existente de `Wire.getWireTimeoutFlag()` no topo do `loop()` já protege contra travamento do barramento. `obstaculo.h` não precisa tratar isso diretamente.

---

## Testing Strategy

### Abordagem Dual

A feature combina testes **por propriedade** (para lógica numérica pura) com testes **por exemplo** (para comportamentos específicos de integração).

### Property-Based Testing

**Biblioteca recomendada para C++ no contexto desktop/host (fora do Arduino):** [rapidcheck](https://github.com/emil-e/rapidcheck) ou uma implementação minimalista baseada em `std::mt19937` + asserts para rodar no computador host (não no Arduino diretamente).

**Estratégia de isolamento:** As funções de `obstaculo.h` dependem de variáveis globais e objetos de hardware. Para PBT no host, criar um `obstaculo_test_harness.h` que:
1. Substitui `mpu.getAngleZ()`, `obterDistanciaFiltrada()`, `controlarRodas()`, `millis()` e `qtr.readLineBlack()` por funções mock com injeção de parâmetros.
2. Expõe as variáveis estáticas internas via funções `get`/`set` para setup e verificação de estado.

**Configuração mínima:** 100 iterações por propriedade.

**Tag de rastreabilidade nos testes:**
```cpp
// Feature: obstaculo-contorno-circular, Property 4: Confirmacao apos N leituras consecutivas
RC_GTEST_PROP(ObstaculoTest, ConfirmacaoAposNLeituras, ()) { ... }
```

### Propriedades a implementar como testes PBT

| Property | Tipo de gerador | Verificação |
|---|---|---|
| 1 (intervalo 50ms) | `(t0, dt)` onde `dt < 50` | `verificarObstaculo()` retorna false, contador inalterado |
| 2 (incremento contador) | `d ∈ [1, DIST_DETECCAO]` | `contadorLeituras` aumentou em 1 |
| 3 (zera contador) | `d ∈ [DIST_DETECCAO+1, MAX_DISTANCE-1]` | `contadorLeituras == 0` |
| 4 (confirmação N leituras) | sequência de N distâncias válidas | última chamada retorna true |
| 5 (ré mantém estado) | `dt ∈ [0, TEMPO_RE_MS - 1]` | `estadoAtual == OBSTACULO_RE`, rodas negativas |
| 6 (ré transiciona) | `dt >= TEMPO_RE_MS` | `estadoAtual == OBSTACULO_GIRANDO` |
| 7 (ângulo-alvo) | `(yaw: float, lado: {-1,+1})` | `anguloAlvo == yaw + lado * 65.0` |
| 8 (velocidade proporcional) | `E: float > TOLERANCIA` | `vel == constrain(60 + abs(E)*1.2, 60, 150)` |
| 9 (giro → contorno) | `(atual, alvo)` com `abs(diff) <= TOL` | `estadoAtual == OBSTACULO_CONTORNO` |
| 10 (timeout giro) | `dt > 3000` | `estadoAtual == OBSTACULO_CONTORNO` |
| 11 (ajuste lateral) | `dL: int ∈ [0, MAX_DISTANCE]` | `ajuste == constrain((dL-ALVO)*2, -80, 80)` |
| 12 (timeout contorno) | `dt > 10000` | `estadoAtual == ESTADO_LINHA`, motores parados |
| 13 (detecção linha) | `sensorValues[8]` aleatório | condição ↔ count(indices 2-5 > 650) >= 2 |
| 14 (escolha de lado) | `(dEsq, dDir): int ∈ [0, MAX]` | lado e sonarPtr corretos |

### Testes Unitários por Exemplo

Cobrir os cenários concretos:

1. **Obstáculo com leituras mistas** — 2 leituras válidas + 1 ruído → contador não dispara.
2. **Iniciar com lados iguais** — `dEsq == dDir` → deve escolher direita (`-1`).
3. **Contorno com linha imediata** — `sensorValues` já com linha desde o início → deve retornar ao PID no primeiro ciclo (após anti-noise de 2).
4. **Contorno com sonar saturado** — `sonarParaObstaculo` retorna `MAX_DISTANCE` → ajuste deve ser constrained a +80.

### Testes de Integração (no hardware)

1. Executar percurso completo com obstáculo cilíndrico de ~20 cm de diâmetro — verificar que o robô contorna e retorna à linha em < 8 s.
2. Verificar via Serial Monitor que os logs aparecem na ordem correta: `[OBS] Re iniciada` → `[OBS] Re concluida. Iniciando giro.` → prints de `[OBS-GIRO]` → `[OBS] Linha recuperada!`.
3. Testar com obstáculo à esquerda e à direita para verificar simetria do algoritmo de escolha de lado.
