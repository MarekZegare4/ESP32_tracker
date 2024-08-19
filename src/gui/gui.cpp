#include <Arduino.h>
#include "gui.h"
#include "mavlink/mav.h"
#include "tracking/tracking.h"
#include "gps/gps.h"
// https://javl.github.io/image2cpp/
// https://www.streamlinehq.com/icons/pixel

#define DISPLAY_CS 5
#define BLACK 0
#define WHITE 1
#define FREQ_2MHZ 2000000
#define FPS 30
#define BUTTON_1_PIN 26
#define BUTTON_2_PIN 25
#define BUTTON_3_PIN 35
#define BUTTON_4_PIN 34

bool button_flag = false;

SPIClass vspi = SPIClass(VSPI);
Adafruit_SharpMem display(&vspi, DISPLAY_CS, 400, 240, FREQ_2MHZ);

int width = display.width();
int height = display.height();

GFXcanvas1 a_h(150, 150); // Artificial Horizon
GFXcanvas1 text(width/2, width/2); // text part of the screen

String button_1 = "waiting for press";
String button_2 = "waiting for press";
String button_3 = "waiting for press";
String button_4 = "waiting for press";

enum eScreen {
	GPS,
	MAP,
	TEST,
	SETTINGS
};

eScreen screen = GPS;

void IRAM_ATTR button_1ISR() {
	int current_level = gpio_get_level(GPIO_NUM_25);
	if (current_level == 0) {
		button_1 = "pressed";
	} else {
		button_1 = "released";
	}
	button_flag = true;
	screen = GPS;
}

void IRAM_ATTR button_2ISR() {
	int current_level = gpio_get_level(GPIO_NUM_26);
	if (current_level == 0) {
		button_2 = "pressed";
	} else {
		button_2 = "released";
	}
	button_flag = true;
	screen = MAP;
}

void IRAM_ATTR button_3ISR() {
	int current_level = gpio_get_level(GPIO_NUM_34);
	if (current_level == 0) {
		button_3 = "pressed";
	} else {
		button_3 = "released";
	}
	button_flag = true;
	screen = TEST;
}

void IRAM_ATTR button_4ISR() {
	int current_level = gpio_get_level(GPIO_NUM_35);
	if (current_level == 0) {
		button_4 = "pressed";
	} else {
		button_4 = "released";
	}
	button_flag = true;
	screen = SETTINGS;
}

void guiInitialize(){
	pinMode(BUTTON_1_PIN, INPUT);
	pinMode(BUTTON_2_PIN, INPUT);
	pinMode(BUTTON_3_PIN, INPUT);
	pinMode(BUTTON_4_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), button_1ISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(BUTTON_2_PIN), button_2ISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(BUTTON_3_PIN), button_3ISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(BUTTON_4_PIN), button_4ISR, FALLING);
	display.begin();
	display.clearDisplay();
}

void mainScreen() {
    text.fillScreen(WHITE);
    text.setTextSize(1);
    text.setTextColor(BLACK);
    text.setCursor(5, 10);
	//text.println("Mag heading: " + String(getCompassDegree()));
	text.println("UAV");
	text.setCursor(5, 20);
	text.println("GPS: " + String(getUavGPS().global_lat) + " " + String(getUavGPS().global_lon) + " " + String(getUavGPS().global_alt));
	text.setCursor(5, 40);
	text.println("Tracker");
	text.setCursor(5, 50);
	text.println("GPS: " + String(getTrackerGPS().latitude) + " " + String(getTrackerGPS().longitude) + " " + String(getTrackerGPS().altitude));
	text.setCursor(5, 60);
	text.println("Fix: " + String(getTrackerGPS().fixType) + " Sat: " + String(getTrackerGPS().satCount));
	text.setCursor(5, 80);
	text.println("SYS TEXT");
	text.setCursor(5, 90);
	text.println(String(getUavSysText().text));
	display.drawBitmap(0, 0, text.getBuffer(), text.width(), text.height(), WHITE, BLACK);
}

void screenTest(String motive = "light") {
	static const unsigned char PROGMEM image_paint_0_bits[] = {0x07,0xff,0xc0,0x0f,0xff,0xe0,0x18,0x00,0x30,0x33,0xff,0x98,0x67,0xff,0xcc,0xcc,0x00,0x66,0x99,0xff,0x32,0xb3,0xff,0x9a,0xa6,0x00,0xca,0xac,0xfe,0x6a,0xa9,0xff,0x2a,0x2b,0x01,0xa8,0x0a,0x7c,0xa0,0x02,0xfe,0x80,0x00,0xfe,0x00,0x00,0xfe,0x00};
	int background = WHITE;
	int foreground = BLACK;

	if (motive == "light") {
		background = WHITE;
		foreground = BLACK;
	}
	else if (motive == "dark") {
		background = BLACK;
		foreground = WHITE;
	}

	display.fillScreen(background);
	display.drawLine(1, 209, 399, 209, foreground);
	display.drawRect(10, 216, 70, 20, foreground);
	display.setTextColor(foreground);
	display.setTextSize(1);
	display.setTextWrap(false);
	display.setCursor(36, 222);
	display.print(button_1);
	display.drawLine(11, 215, 80, 215, foreground);
	display.drawRect(110, 215, 70, 20, foreground);
	display.drawLine(80, 234, 80, 216, foreground);
	display.setCursor(126, 222);
	display.print(button_2);
	display.drawLine(111, 214, 180, 214, foreground);
	display.drawLine(180, 233, 180, 215, foreground);
	display.drawRect(210, 215, 70, 20, foreground);
	display.setCursor(219, 222);
	display.print(button_3);
	display.drawLine(211, 214, 280, 214, foreground);
	display.drawLine(280, 233, 280, 215, foreground);
	display.drawRect(309, 215, 70, 20, foreground);
	display.setCursor(333, 221);
	display.print(button_4);
	display.drawLine(310, 214, 379, 214, foreground);
	display.drawLine(379, 233, 379, 215, foreground);
	display.drawLine(1, 207, 399, 207, foreground);
	display.drawLine(0, 23, 399, 23, foreground);
	display.drawBitmap(5, 4, image_paint_0_bits, 23, 16, foreground);
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

void guiTask (void * parameters) {
	for(;;){

		if (button_flag) {
			button_flag = false;
			display.clearDisplay();
		}

		switch (screen)
		{
		case GPS:
			mainScreen();
			break;
		
		case MAP:
			break;
		
		case TEST:
			screenTest("light");
			break;
		
		case SETTINGS:
			break;
		
		default:
			break;
		}
   		//mainScreen();
		//screenTest("light");
		//artificialHorizon();
		display.refresh();
    	vTaskDelay((1/FPS)/portTICK_PERIOD_MS);
  	}
}