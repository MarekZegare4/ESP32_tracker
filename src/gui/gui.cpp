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

GFXcanvas1 a_h(150, 150); // Artificial Horizon
GFXcanvas1 text(width/2, width/2); // text part of the screen

void displayInitialize(){
	display.begin();
	display.clearDisplay();
}

void mainScreen() {
    text.fillScreen(WHITE);
    text.setTextSize(1);
    text.setTextColor(BLACK);
    text.setCursor(5, 10);
	text.println("Mag heading: " + String(getCompassDegree()));
    display.drawBitmap(0, 0, text.getBuffer(), text.width(), text.height(), WHITE, BLACK);
}

void artificialHorizon() {
	// Rysowanie sztucznego horyzontu
	a_h.fillScreen(WHITE);
	int srodekX = a_h.width()/2;
	int srodekY = a_h.width()/2;
	int szer = 3;
	int y1 = (srodekY + a_h.width()/2*(tan((getUavAttitude().roll))));
	int y2 = (srodekY - a_h.width()/2*(tan((getUavAttitude().roll))));
	//canvas.drawLine(display.width()/2, y1, display.width(), y2, BLACK);
	a_h.drawLine(0, 0, 0, a_h.height(), BLACK);
	a_h.drawLine(0, a_h.height() - 1, a_h.width(), a_h.height() - 1, BLACK);
	a_h.drawLine(0, y1, a_h.width(), y2, BLACK);

	// Lewa strona
	a_h.fillRect(srodekX - srodekX*0.7, srodekY, (srodekX - srodekX*0.25) - (srodekX - srodekX*0.7) + 1, (srodekY + szer) - srodekY, WHITE);
	a_h.fillRect(srodekX - srodekX*0.25 - szer, srodekY, (srodekX - srodekX*0.25) - (srodekX - srodekX*0.25 - szer), (srodekY + 10) - srodekY, WHITE); 
	a_h.drawLine(srodekX - srodekX*0.7, srodekY, srodekX - srodekX*0.25, srodekY, BLACK); // górna linia pozioma
	a_h.drawLine(srodekX - srodekX*0.7, srodekY + szer, srodekX - srodekX*0.25 - szer, srodekY + szer, BLACK); // dolna linia pozioma
	a_h.drawLine(srodekX - srodekX*0.7, srodekY, srodekX - srodekX*0.7, srodekY + szer, BLACK); // lewa linia pionowa 
	a_h.drawLine(srodekX - srodekX*0.25 - szer, srodekY + szer, srodekX - srodekX*0.25 - szer, srodekY + 10, BLACK); // lewa linia pionowa mniejsza
	a_h.drawLine(srodekX - srodekX*0.25, srodekY, srodekX - srodekX*0.25, srodekY + 10, BLACK); // prawa linia pionowa mniejsza
	a_h.drawLine(srodekX - srodekX*0.25 - szer, srodekY + 10, srodekX - srodekX*0.25, srodekY + 10, BLACK); // linia pozioma mniejsza
	
	// Prawa strona
	a_h.fillRect(srodekX + srodekX*0.25, srodekY, (srodekX + srodekX*0.7) - (srodekX + srodekX*0.25) + 1, (srodekY + szer) - srodekY, WHITE);
	a_h.fillRect(srodekX + srodekX*0.25, srodekY, (srodekX + srodekX*0.25 + szer) - (srodekX + srodekX*0.25), (srodekY + 10) - srodekY, WHITE);
	a_h.drawLine(srodekX + srodekX*0.7, srodekY, srodekX + srodekX*0.25, srodekY, BLACK); // górna linia pozioma
	a_h.drawLine(srodekX + srodekX*0.7, srodekY + szer, srodekX + srodekX*0.25 + szer, srodekY + szer, BLACK); // dolna linia pozioma
	a_h.drawLine(srodekX + srodekX*0.7, srodekY, srodekX + srodekX*0.7, srodekY + szer, BLACK); // lewa linia pionowa
	a_h.drawLine(srodekX + srodekX*0.25 + szer, srodekY + szer, srodekX + srodekX*0.25 + szer, srodekY + 10, BLACK); // prawa linia pionowa mniejsza
	a_h.drawLine(srodekX + srodekX*0.25, srodekY, srodekX + srodekX*0.25, srodekY + 10, BLACK); // lewa linia pionowa mniejsza
	a_h.drawLine(srodekX + srodekX*0.25 + szer, srodekY + 10, srodekX + srodekX*0.25, srodekY + 10, BLACK); // linia pozioma mniejsza

	// Środek
	a_h.fillRect(srodekX - szer, srodekY, 2*szer, 2*szer, WHITE);
	a_h.drawRect(srodekX - szer, srodekY, 2*szer, 2*szer, BLACK);
	display.drawBitmap(width - 150, 0, a_h.getBuffer(), a_h.width(), a_h.height(), WHITE, BLACK);
}

void displayTask (void * parameters) {
	for(;;){
   		mainScreen();
		artificialHorizon();
		display.refresh();
    	vTaskDelay((1/FPS)/portTICK_PERIOD_MS);
  	}
}