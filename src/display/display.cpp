#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <mat.h>
#include <SPI.h>
#include "display.h"
#include "gfx.h"

// https://javl.github.io/image2cpp/
// https://www.streamlinehq.com/icons/pixel

#define SHARP_SS 5
#define BLACK 0
#define WHITE 1
#define FREQ_2MHZ 2000000
#define FPS 30

displayElements dispElem;
SPIClass vspi = SPIClass(VSPI);
Adafruit_SharpMem display(&vspi, SHARP_SS, 400, 240, FREQ_2MHZ);

int width = display.width();
int height = display.height();

GFXcanvas1 AH(150, 150); // Artificial Horizon
GFXcanvas1 TXT(width/2, width/2); // text part of the screen

double deg = 0;
int srodekX = AH.width()/2;
int srodekY = AH.width()/2;
int szer = 2;

void MainScreen() {
    TXT.fillScreen(WHITE);
    TXT.setTextSize(1);
    TXT.setTextColor(BLACK);
    TXT.setCursor(5, 10);
	TXT.println("Mag heading: " + dispElem.gcsCompass);
	TXT.setCursor(5, 20);
	TXT.println(dispElem.mavStatusMsg);
	TXT.drawBitmap(20,20, epd_bitmap_Interface_Essential_Alert_Circle_1__Streamline_Pixel, 20, 20, WHITE, BLACK);
    display.drawBitmap(0, 0, TXT.getBuffer(), TXT.width(), TXT.height(), WHITE, BLACK);
}

void ArtificialHorizon() {
	// Rysowanie sztucznego horyzontu
	deg = degrees(dispElem.attitudeRoll);
	AH.fillScreen(WHITE);
	int y1 = (srodekY + AH.width()/2*(tan(radians(deg))));
	int y2 = (srodekY - AH.width()/2*(tan(radians(deg))));
	//canvas.drawLine(display.width()/2, y1, display.width(), y2, BLACK);
	AH.drawLine(0, 0, 0, AH.height(), BLACK);
	AH.drawLine(0, AH.height() - 1, AH.width(), AH.height() - 1, BLACK);
	AH.drawLine(0, y1, AH.width(), y2, BLACK);

	// //display.drawBitmap(0, 0, canvas.getBuffer(), 400, 240, WHITE);
	// AH.fillRect(srodekX - 20, srodekY - szer, srodekX - 8, srodekY + szer, WHITE);
	// // display.drawLine(display.width()/2, srodekY, display.width(), srodekY, BLACK);
	// AH.drawLine(srodekX - 20, srodekY + szer, srodekX - 8, srodekY + szer, BLACK);
	// AH.drawLine(srodekX + 8, srodekY + szer, srodekX + 20, srodekY + szer, BLACK);
	// AH.drawLine(srodekX - 20, srodekY - szer, srodekX - 8, srodekY - szer, BLACK);
	// AH.drawLine(srodekX + 8, srodekY - szer, srodekX + 20, srodekY - szer, BLACK);
	// AH.drawLine(srodekX - 20, srodekY + szer, srodekX - 20, srodekY - szer, BLACK);
	// AH.drawLine(srodekX + 20, srodekY + szer, srodekX + 20, srodekY - szer, BLACK);
	//display.drawBitmap(srodekX - 17, srodekY - 3, crosshair, 34, 7, BLACK);
	AH.drawBitmap(srodekX - 17, srodekY - 3, crosshair, 34, 7, BLACK);
	display.drawBitmap(width - 150, 0, AH.getBuffer(), AH.width(), AH.height(), WHITE, BLACK);
}

void DisplayTask (void * parameters) {
	for(;;){
   		MainScreen();
		ArtificialHorizon();
		display.refresh();
    	vTaskDelay((1/FPS)/portTICK_PERIOD_MS);
  	}
}

void DisplayInitialize(){
	display.begin();
	display.clearDisplay();
}