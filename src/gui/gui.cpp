#include "gui.h"
#include "mavlink/mav.h"
#include "tracking/tracking.h"
// https://javl.github.io/image2cpp/
// https://www.streamlinehq.com/icons/pixel

#define DISPLAY_CS 5
#define BLACK 0
#define WHITE 1
#define FREQ_2MHZ 2000000
#define FPS 30

SPIClass vspi = SPIClass(VSPI);
Adafruit_SharpMem display(&vspi, DISPLAY_CS, 400, 240, FREQ_2MHZ);

int width = display.width();
int height = display.height();

GFXcanvas1 AH(150, 150); // Artificial Horizon
GFXcanvas1 TXT(width/2, width/2); // text part of the screen

void displayInitialize(){
	display.begin();
	display.clearDisplay();
}

void mainScreen() {
    TXT.fillScreen(WHITE);
    TXT.setTextSize(1);
    TXT.setTextColor(BLACK);
    TXT.setCursor(5, 10);
	TXT.println("Mag heading: " + String(getCompassDegree()));
    display.drawBitmap(0, 0, TXT.getBuffer(), TXT.width(), TXT.height(), WHITE, BLACK);
}

void artificialHorizon() {
	// Rysowanie sztucznego horyzontu
	AH.fillScreen(WHITE);
	int srodekX = AH.width()/2;
	int srodekY = AH.width()/2;
	int szer = 3;
	int y1 = (srodekY + AH.width()/2*(tan((getUavAttitude().roll))));
	int y2 = (srodekY - AH.width()/2*(tan((getUavAttitude().roll))));
	//canvas.drawLine(display.width()/2, y1, display.width(), y2, BLACK);
	AH.drawLine(0, 0, 0, AH.height(), BLACK);
	AH.drawLine(0, AH.height() - 1, AH.width(), AH.height() - 1, BLACK);
	AH.drawLine(0, y1, AH.width(), y2, BLACK);

	// Lewa strona
	AH.fillRect(srodekX - srodekX*0.7, srodekY, (srodekX - srodekX*0.25) - (srodekX - srodekX*0.7) + 1, (srodekY + szer) - srodekY, WHITE);
	AH.fillRect(srodekX - srodekX*0.25 - szer, srodekY, (srodekX - srodekX*0.25) - (srodekX - srodekX*0.25 - szer), (srodekY + 10) - srodekY, WHITE); 
	AH.drawLine(srodekX - srodekX*0.7, srodekY, srodekX - srodekX*0.25, srodekY, BLACK); // górna linia pozioma
	AH.drawLine(srodekX - srodekX*0.7, srodekY + szer, srodekX - srodekX*0.25 - szer, srodekY + szer, BLACK); // dolna linia pozioma
	AH.drawLine(srodekX - srodekX*0.7, srodekY, srodekX - srodekX*0.7, srodekY + szer, BLACK); // lewa linia pionowa 
	AH.drawLine(srodekX - srodekX*0.25 - szer, srodekY + szer, srodekX - srodekX*0.25 - szer, srodekY + 10, BLACK); // lewa linia pionowa mniejsza
	AH.drawLine(srodekX - srodekX*0.25, srodekY, srodekX - srodekX*0.25, srodekY + 10, BLACK); // prawa linia pionowa mniejsza
	AH.drawLine(srodekX - srodekX*0.25 - szer, srodekY + 10, srodekX - srodekX*0.25, srodekY + 10, BLACK); // linia pozioma mniejsza
	
	// Prawa strona
	AH.fillRect(srodekX + srodekX*0.25, srodekY, (srodekX + srodekX*0.7) - (srodekX + srodekX*0.25) + 1, (srodekY + szer) - srodekY, WHITE);
	AH.fillRect(srodekX + srodekX*0.25, srodekY, (srodekX + srodekX*0.25 + szer) - (srodekX + srodekX*0.25), (srodekY + 10) - srodekY, WHITE);
	AH.drawLine(srodekX + srodekX*0.7, srodekY, srodekX + srodekX*0.25, srodekY, BLACK); // górna linia pozioma
	AH.drawLine(srodekX + srodekX*0.7, srodekY + szer, srodekX + srodekX*0.25 + szer, srodekY + szer, BLACK); // dolna linia pozioma
	AH.drawLine(srodekX + srodekX*0.7, srodekY, srodekX + srodekX*0.7, srodekY + szer, BLACK); // lewa linia pionowa
	AH.drawLine(srodekX + srodekX*0.25 + szer, srodekY + szer, srodekX + srodekX*0.25 + szer, srodekY + 10, BLACK); // prawa linia pionowa mniejsza
	AH.drawLine(srodekX + srodekX*0.25, srodekY, srodekX + srodekX*0.25, srodekY + 10, BLACK); // lewa linia pionowa mniejsza
	AH.drawLine(srodekX + srodekX*0.25 + szer, srodekY + 10, srodekX + srodekX*0.25, srodekY + 10, BLACK); // linia pozioma mniejsza

	// Środek
	AH.fillRect(srodekX - szer, srodekY, 2*szer, 2*szer, WHITE);
	AH.drawRect(srodekX - szer, srodekY, 2*szer, 2*szer, BLACK);
	display.drawBitmap(width - 150, 0, AH.getBuffer(), AH.width(), AH.height(), WHITE, BLACK);
}

void displayTask (void * parameters) {
	for(;;){
   		mainScreen();
		artificialHorizon();
		display.refresh();
    	vTaskDelay((1/FPS)/portTICK_PERIOD_MS);
  	}
}