#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <mat.h>
#include <SPI.h>
#include "display.h"

// 'Interface-Essential-Alert-Circle-1--Streamline-Pixel', 24x24px
const unsigned char epd_bitmap_Interface_Essential_Alert_Circle_1__Streamline_Pixel [] PROGMEM = {
		0xff, 0x0f, 0xf0, 0xfc, 0x03, 0xf0, 0xf0, 0x00, 0xf0, 0xe1, 0xf8, 0x70, 0xc1, 0xf8, 0x30, 0xc1, 
	0xe8, 0x30, 0x81, 0xe8, 0x10, 0x81, 0xf8, 0x10, 0x01, 0xf8, 0x00, 0x00, 0xf0, 0x00, 0x00, 0xf0, 
	0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x10, 0x80, 0xf0, 0x10, 0xc0, 0xf0, 0x30, 0xc0, 0xf0, 0x30, 
	0xe0, 0xf0, 0x70, 0xf0, 0x00, 0xf0, 0xfc, 0x03, 0xf0, 0xff, 0x0f, 0xf0
};

extern displayElements dispElem;
// #define SHARP_SCK  14
// #define SHARP_MOSI 13
// #define SHARP_SS   15

// #define SHARP_SCK  18
// #define SHARP_MOSI 23
#define SHARP_SS   5

SPIClass vspi = SPIClass(VSPI);

#define BLACK 0
#define WHITE 1
#define FREQ_2MHZ 2000000
#define FPS 30

Adafruit_SharpMem display(&vspi, SHARP_SS, 400, 240, FREQ_2MHZ);

GFXcanvas1 AH(display.width()/2, display.width()/2); // Artificial Horizon
GFXcanvas1 TXT(display.width()/2, display.width()/2); // text part of the screen

double deg = 0;

void Menu() {
	// Linie dzielące ekran na 3 części
    display.drawLine(display.width()/2, 0, display.width()/2, display.width()/2, BLACK);
	display.drawLine(0, display.width()/2, display.width(), display.width()/2, BLACK);
    TXT.fillScreen(WHITE);
    TXT.setTextSize(1);
    TXT.setTextColor(BLACK);
    TXT.setCursor(5, 10);
	TXT.println("Mag heading: " + dispElem.gcsCompass);
	TXT.drawBitmap(20,20, epd_bitmap_Interface_Essential_Alert_Circle_1__Streamline_Pixel, 20, 20, WHITE, BLACK);
    display.drawBitmap(0, 0, TXT.getBuffer(), TXT.width(), TXT.height(), WHITE, BLACK);
}

void ArtificialHorizon() {
	// Rysowanie sztucznego horyzontu
	deg = degrees(dispElem.attitudeRoll);
	AH.fillScreen(WHITE);
	int srodekX = AH.width()/2;
	int srodekY = AH.width()/2;
	int szer = 2;
	int y1;
	int y2;
	y1 = (srodekY + AH.width()/2*(tan(radians(deg))));
	y2 = (srodekY - AH.width()/2*(tan(radians(deg))));
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

void DisplayTask (void * parameters) {
	for(;;){
   		Menu();
		ArtificialHorizon();
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