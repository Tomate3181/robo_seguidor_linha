# Contexto Técnico: Otimização de Sensores TCS34725

## 1. Objetivo do Sistema
Robô seguidor de linha de alta performance com detecção de marcas de cor (Verde/Vermelho) utilizando sensores TCS34725 multiplexados (TCA9548A). O foco é garantir detecção precisa em alta velocidade, minimizando falsos positivos e latência.

## 2. Problemas Identificados
- **Latência de Leitura:** O `millis()` de 50ms é muito lento, resultando em perda da marca de cor (robô "passa varando").
- **Ruído (Verde Fantasma):** Leituras falsas de cor em zonas pretas ou brancas devido à baixa luminosidade ou reflexo.
- **Saturação/Brilho:** O sensor sofre com variações de iluminação ambiente, tornando o sistema RGB instável.

## 3. Estratégias de Otimização (Roadmap)

### A. Ajustes de Hardware e Sincronização
- **Integration Time:** Redução do `TCS34725_INTEGRATIONTIME` para 2.4ms ou 24ms para aumentar a taxa de amostragem.
- **Ciclo de Leitura:** Redução do *delay* de processamento para <= 10ms.

### B. Processamento de Sinais
- **Espaço de Cor HSV:** Utilização do valor **Hue (Matiz)** como principal métrica, por ser invariante à luminosidade total.
- **Debouncing (Votação):** Implementação de um contador de confirmação. A detecção só é validada após N leituras consecutivas (ex: 3 leituras positivas).
- **Thresholds Dinâmicos:** Calibração inicial (setup) do sensor para definir o `limiarLuminosidade` baseada no ambiente real.

### C. Lógica de Filtragem (Pseudo-código)
- **Filtro de Ruído:** Se `C < limiar_minimo`, ignorar leitura (luz insuficiente/fora da pista).
- **Filtro de Estabilidade:**
  ```cpp
  if (ehVerde()) {
     contadorVerde++;
  } else {
     contadorVerde = 0;
  }
  if (contadorVerde >= 3) { acaoVerde(); }
  ```

## 4. Parâmetros de Ajuste (Tuning)
- **Range Hue Verde:**[95.0 - 165.0]
- **Saturação Mínima:** 0.35 (Fita central) / 0.18 (Borda)
- **Tempo de Loop:** 10ms (meta)