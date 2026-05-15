# 🛠️ README TÉCNICO - Próximos Passos e Melhorias Futuras

Este documento serve como um guia de desenvolvimento e backlog técnico, contendo os próximos desafios a serem implementados no Robô Seguidor de Linha, bem como ideias estruturais e melhorias de código para garantir estabilidade e sucesso nas pistas e na Zona de Resgate.

## 🚀 1. Funcionalidades Prioritárias (Para Amanhã)

### A. Desvio de Obstáculos Inteligente (FSM: `ESTADO_OBSTACULO`)
**Objetivo:** Contornar de forma fluida uma caixa/obstáculo na pista e retornar com precisão à linha preta.
**Plano de Implementação:**
- **Gatilho:** Utilizar o sensor ultrassônico `FRENTE` na lógica principal. Se a distância cair abaixo de um limite seguro (ex: 10cm), acionar `estadoAtual = ESTADO_OBSTACULO`.
- **Lógica de Contorno (Ultrassons Laterais + Giroscópio):**
  1. O robô detecta o bloqueio e freia imediatamente.
  2. Lê o MPU6050 e rotaciona **90° exatos para o lado** (esq ou dir) usando a mesma lógica suave de aproximação proporcional implementada para as marcações verdes.
  3. Começa a avançar reto, utilizando o **ultrassônico lateral correspondente** (`ESQUERDA` ou `DIREITA`) como um "novo sensor de linha". É só criar um controle Proporcional simples onde a distância do sensor lateral mantém o robô a exatos "X cm" da lateral do obstáculo.
  4. Quando o sensor lateral disparar para o "infinito", o robô sabe que a quina do obstáculo passou e pode girar novamente para retornar e caçar a linha preta com o QRE-8D.

### B. A Temida Zona de Resgate (FSM: Novo `ESTADO_RESGATE`)
**Objetivo:** Navegar em ambiente aberto e contido, sem trilhas, orientando-se pelas paredes para vasculhar a sala ou encontrar a porta de saída.
**Plano de Implementação:**
- **A Entrada (Silver Tape):** A fita cinza geralmente causa um comportamento peculiar nos sensores QRE-8D ou até mesmo no Sensor RGB (pelo altíssimo reflexo difuso). Calibre ou estabeleça uma condição especial (ex: todos os IRs captam valores medianos idênticos simultaneamente) para disparar a transição para `ESTADO_RESGATE`.
- **Comportamento (Lógica Wall-Follower / Rato de Labirinto):**
  1. No resgate, ignore completamente o PID da linha preta.
  2. Assuma a **Regra da Mão Direita (ou Esquerda):** Use o sensor da frente para não bater de cara na parede e mantenha o robô colado à parede lateral, fazendo correções de motor (usando os valores do ECHO lateral).
  3. Em qualquer "quina" ou bloqueio frontal, o Giroscópio toma as rédeas para garantir que as curvas de navegação sejam geometricamente perfeitas, impedindo que o robô ande em diagonal na sala.
  4. Para sair, uma câmera ou sensor de cor/linha pode estar ativo em background verificando o momento exato de retornar ao `ESTADO_LINHA`.

---

## 🧠 2. Ideias e Dicas Práticas de Código e Hardware

Após analisar todos os arquivos criados e refatorados, nossa Arquitetura Não-Bloqueante (que não usa `delay()`) está excelente. Contudo, na hora de implementar essas novidades amanhã, atente-se a isto:

1. **Cuidado Fatal com o `pulseIn()` (Ultrassônico):**
   - A função nativa `pulseIn()` para ler o HC-SR04 é **bloqueante**. Se um sensor não encontrar a parede (retornar 0), ele pode congelar o Arduino por dezenas de milissegundos esperando resposta. Isso **destruiria** o nosso PID suave na linha.
   - **Solução de Ouro:** Baixe e utilize a biblioteca **`NewPing`**. Ela permite ler o ECHO por temporizadores/interrupções (ping timer), o que vai manter a rodagem lisa enquanto os sensores olham pros lados em background.

2. **Limpeza e Estabilidade do I2C:**
   - Estamos usando O MPU6050, dois TCS34725 e um OLED pendurados no TCA9548A. Quanto mais cabos voando, maior a capacitância parasita e o ruído, podendo travar o canal I2C (`Wire`).
   - Mantenha esses fios curtinhos e evite passar cabos de I2C encostados nos fios de força dos motores.

3. **Evoluindo a Velocidade e o PID:**
   - Graças à suavização com `KD=0.5` que implementamos, seu robô agora é "estável". Assim que a mecânica provar robustez amanhã, comece a aumentar a `VELOCIDADE_BASE` gradativamente (ex: 180, 200). 
   - Ao aumentar a base, o robô vai ficar mais "agressivo" na entrada das curvas; se ele balançar muito, suba o `KD` para `0.8` ou `1.0` para que ele "freie" antecipadamente a roda interna da curva, sem depender só do P.

4. **Blindagem do Sensor de Linha:**
   - Faça uma pequena saia lateral no QRE-8D de fita isolante ou plástico preto opaco para fazer sombra. Reflexos do sol ou luzes da sala interferem absurdamente em competições de seguidor de linha, especialmente perto de marcações verdes.

---
**Resumo:** O esqueleto que criamos não só está profissional, como é escalável para qualquer tamanho de pista. O diferencial competitivo será a navegação com o **Wall-Follower Assíncrono** que você fará amanhã. Sucesso e rumo ao primeiro lugar! 🏆🤖