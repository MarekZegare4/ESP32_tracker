#pragma once
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

// #define SHARP_SCK  14
// #define SHARP_MOSI 13
// #define SHARP_SS   12

#define SHARP_SCK  18
#define SHARP_MOSI 23
#define SHARP_SS   5

#define BLACK 0
#define WHITE 1

Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 400, 240);

const unsigned char crosshair [] PROGMEM = {
	0xff, 0xf0, 0x03, 0xff, 0xc0, 0x80, 0x10, 0x02, 0x00, 0x40, 0x80, 0x10, 0x02, 0x00, 0x40, 0xff, 
	0x9f, 0xfe, 0x7f, 0xc0, 0x00, 0x80, 0x00, 0x40, 0x00, 0x00, 0x80, 0x00, 0x40, 0x00, 0x00, 0xff, 
	0xff, 0xc0, 0x00
};

void menu() {
	// Linie dzielące ekran na 3 części
    display.drawLine(display.width()/2, 0, display.width()/2, display.width()/2, BLACK);
	display.drawLine(0, display.width()/2, display.width(), display.width()/2, BLACK);
    display.refresh();
	display.clearDisplayBuffer();
}

void artificialHorizon() {
	// Rysowanie sztucznego horyzontu
	int srodekX = display.width()/2 + display.width()/4;
	int srodekY = display.width()/4;
	int szer = 2;
	display.drawLine(0, display.width()/4, display.width(), display.width()/4, BLACK);
	display.fillRect(srodekX - 20, srodekY - szer, srodekX - 8, srodekY + szer, WHITE);
	display.drawLine(display.width()/2, srodekY, display.width(), srodekY, BLACK);
	display.drawLine(srodekX - 20, srodekY + szer, srodekX - 8, srodekY + szer, BLACK);
	display.drawLine(srodekX + 8, srodekY + szer, srodekX + 20, srodekY + szer, BLACK);
	display.drawLine(srodekX - 20, srodekY - szer, srodekX - 8, srodekY - szer, BLACK);
	display.drawLine(srodekX + 8, srodekY - szer, srodekX + 20, srodekY - szer, BLACK);
	display.drawLine(srodekX - 20, srodekY + szer, srodekX - 20, srodekY - szer, BLACK);
	display.drawLine(srodekX + 20, srodekY + szer, srodekX + 20, srodekY - szer, BLACK);
	//display.drawBitmap(srodekX - 17, srodekY - 3, crosshair, 34, 7, BLACK);
	display.refresh();
}

void ScreenTask (void * parameters) {
  for(;;){
   //artificialHorizon();
	menu();
    //vTaskDelay(1/FPS*portTICK_PERIOD_MS);
  }
}

void DisplayInitialize(){
	display.begin();
	display.clearDisplay();
}