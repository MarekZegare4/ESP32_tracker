#pragma once
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <mat.h>
#include <SPI.h>

// #define SHARP_SCK  14
// #define SHARP_MOSI 13
// #define SHARP_SS   15

// #define SHARP_SCK  18
// #define SHARP_MOSI 23
#define SHARP_SS   5

SPIClass vspi = SPIClass(VSPI);


#define BLACK 0
#define WHITE 1
uint32_t freq = 2000000;
Adafruit_SharpMem display(&vspi, SHARP_SS, 400, 240, freq);

GFXcanvas1 AH(display.width()/2, display.width()/2); // Artificial Horizon

double deg = 0;

void menu() {
	// Linie dzielące ekran na 3 części
    display.drawLine(display.width()/2, 0, display.width()/2, display.width()/2, BLACK);
	display.drawLine(0, display.width()/2, display.width(), display.width()/2, BLACK);
}

void artificialHorizon() {
	// Rysowanie sztucznego horyzontu
	AH.fillScreen(WHITE);
	int srodekX = AH.width()/2;
	int srodekY = AH.width()/2;
	int szer = 2;
	int y1;
	int y2;
	y1 = (srodekY - AH.width()/2*(tan(radians(deg))));
	y2 = (srodekY + AH.width()/2*(tan(radians(deg))));
	//canvas.drawLine(display.width()/2, y1, display.width(), y2, BLACK);

	AH.drawLine(0, y1, AH.width(), y2, BLACK);

	//display.drawBitmap(0, 0, canvas.getBuffer(), 400, 240, WHITE);
	// display.fillRect(srodekX - 20, srodekY - szer, srodekX - 8, srodekY + szer, WHITE);
	// // display.drawLine(display.width()/2, srodekY, display.width(), srodekY, BLACK);
	// display.drawLine(srodekX - 20, srodekY + szer, srodekX - 8, srodekY + szer, BLACK);
	// display.drawLine(srodekX + 8, srodekY + szer, srodekX + 20, srodekY + szer, BLACK);
	// display.drawLine(srodekX - 20, srodekY - szer, srodekX - 8, srodekY - szer, BLACK);
	// display.drawLine(srodekX + 8, srodekY - szer, srodekX + 20, srodekY - szer, BLACK);
	// display.drawLine(srodekX - 20, srodekY + szer, srodekX - 20, srodekY - szer, BLACK);
	// display.drawLine(srodekX + 20, srodekY + szer, srodekX + 20, srodekY - szer, BLACK);
	//display.drawBitmap(srodekX - 17, srodekY - 3, crosshair, 34, 7, BLACK);
	display.drawBitmap(display.width()/2+1, 0, AH.getBuffer(), AH.width(), AH.height(), WHITE, BLACK);
}

void ScreenTask (void * parameters) {
	for(;;){
		int FPS = 60;
   		menu();
		artificialHorizon();
		display.refresh();
    	vTaskDelay((1/FPS)/portTICK_PERIOD_MS);
  	}
}

void DegTask(void * parameters) {
	for(;;) {
		deg += 0.1;
		vTaskDelay(10 / portTICK_PERIOD_MS);
		if (deg > 90) {
			deg = 0;
		}
	}
}

void DisplayInitialize(){
	//hspi.begin();
	display.begin();
	display.clearDisplay();
}