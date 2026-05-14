#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

// Definições da tela
#define SCREEN_WIDTH 128 // Largura do display OLED, em pixels
#define SCREEN_HEIGHT 64 // Altura do display OLED, em pixels

// Pino de reset (-1 se compartilhar o pino de reset do Arduino)
#define OLED_RESET -1 

// Cria a instância do display I2C (Endereço comum é 0x3C)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Assinatura da função do multiplexador que está no robo_linha.ino
extern void tcaselect(uint8_t i);

// ==============================================================================
// INICIALIZAÇÃO DO DISPLAY OLED
// ==============================================================================
void initDisplay() {
  // 1. Abre a comunicação I2C no canal 0 (CANAL_OLED definido no config.h)
  tcaselect(CANAL_OLED);
  
  // 2. Inicializa o display no endereço I2C 0x3C
  // SSD1306_SWITCHCAPVCC = gerar a alta voltagem a partir do 3.3v interno
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao inicializar o SSD1306 (OLED)"));
    // Apenas avisa via serial, mas não trava o robô para evitar que pare na pista
  } else {
    // Configuração inicial visual
    display.clearDisplay();
    display.setTextSize(1);              // Tamanho normal do texto
    display.setTextColor(SSD1306_WHITE); // Texto em "branco" (pixels acesos)
    display.setCursor(0, 0);             // Canto superior esquerdo
    display.println(F("OLED OK!"));
    display.display();                   // Envia o buffer para a tela
  }
}

// ==============================================================================
// ATUALIZAÇÃO DO DISPLAY OLED
// ==============================================================================
void atualizarStatus(String linha1, String linha2) {
  // Sempre garantir que o multiplexador está apontando para o OLED
  // antes de enviar qualquer comando via barramento I2C
  tcaselect(CANAL_OLED);
  
  // Limpa o buffer de vídeo
  display.clearDisplay();
  
  // Configuração padrão
  display.setTextSize(2);              // Tamanho um pouco maior para fácil leitura na pista
  display.setTextColor(SSD1306_WHITE);
  
  // Escreve a Primeira Linha
  display.setCursor(0, 0); 
  display.println(linha1);
  
  // Escreve a Segunda Linha
  display.setCursor(0, 32); 
  display.println(linha2);
  
  // Atualiza efetivamente o visor
  display.display();
}

#endif // DISPLAY_UTILS_H
