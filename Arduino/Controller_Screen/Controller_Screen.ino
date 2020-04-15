
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(8, 9); //  TX,RX

#define _sclk 13
#define _miso 12
#define _mosi 11
#define _cs 7
#define _dc 5
#define _rst 6

#define XCENTRE 506
#define YCENTRE 528

Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);

const int LeftButton = A2;     // the number of the pushbutton pin
const int RightButton = A5;     // the number of the pushbutton pin

const int LeftXin = A1;  // Analog input pin for left joystick X
const int LeftYin = A0;  // Analog input pin for left joystick Y
const int RightXin = A6;  // Analog input pin for right joystick X
const int RightYin = A7;  // Analog input pin for right joystick Y

int prevLXDisplay = 0;
int prevLYDisplay = 0;
int prevRXDisplay = 0;
int prevRYDisplay = 0;

char data_packet[30];

void setup() {

  tft.begin();
  delay(300);
  tft.setRotation(3);
  tft.fillScreen(ILI9340_BLACK);
  delay(300);
  tft.setCursor(20, 60);
  tft.setTextColor(ILI9340_BLUE);  tft.setTextSize(6);
  tft.println("BIG FACE");
  tft.setCursor(20, 120);
  tft.println("ROBOTICS");
  delay(500);

  Serial.begin(9600);
  BTSerial.begin(9600); //Bluetooth software serial
  
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(RightButton, INPUT_PULLUP);
 

  while(digitalRead(RightButton) == HIGH){ //Wait right here until right joystick button is pressed
  }
  tft.fillScreen(ILI9340_BLACK);
}

void loop(void) {
  
  int LXValue = analogRead(LeftXin);
  int LXDisplay = map(LXValue, 1023, 0, 20, 140);
  int LYValue = analogRead(LeftYin);
  int LYDisplay = map(LYValue, 0, 1023, 60, 180);
  
  int RXValue = analogRead(RightXin);
  int RXDisplay = map(RXValue, 1023, 0, 180, 300);
  int RYValue = analogRead(RightYin);
  int RYDisplay = map(RYValue, 0, 1023, 60, 180);

    
  tft.fillCircle(prevRXDisplay, prevRYDisplay, 10, ILI9340_BLACK);
  tft.fillCircle(prevLXDisplay, prevLYDisplay, 10, ILI9340_BLACK);
  drawGuides();

  tft.fillCircle(LXDisplay, LYDisplay, 10, ILI9340_RED);
  tft.fillCircle(RXDisplay, RYDisplay, 10, ILI9340_RED);

  prevLXDisplay = LXDisplay;
  prevLYDisplay = LYDisplay;
  prevRXDisplay = RXDisplay;
  prevRYDisplay = RYDisplay;

  int XRValue = (XCENTRE-RXValue)/2;
  XRValue = constrain(XRValue, -255, 255);
  
  int YRValue = (YCENTRE-RYValue)/2;
  YRValue = constrain(YRValue, -255, 255);

  int XLValue = (XCENTRE-LXValue)/2;
  XLValue = constrain(XLValue, -255, 255);
  
  int YLValue = (YCENTRE-LYValue)/2;
  YLValue = constrain(YLValue, -255, 255);

  int leftbut = digitalRead(LeftButton);
  int rightbut = digitalRead(RightButton);
 
  sprintf(data_packet, "%d,%d,%d,%d,%d,%d",XRValue, YRValue, XLValue, YLValue, leftbut, rightbut);
  BTSerial.println(data_packet);
  Serial.println(data_packet);

  delay(100);

}

void drawGuides(){
  //tft.drawLine(x1, y1, x2, y2, color);

  int LeftCentX = 80;
  int LeftCentY = 120;
  int RightCentX = 240;
  int RightCentY = 120;
  
  tft.drawLine(LeftCentX, LeftCentY, LeftCentX-60, LeftCentY, ILI9340_WHITE);
  tft.drawLine(LeftCentX, LeftCentY, LeftCentX+60, LeftCentY, ILI9340_WHITE);
  tft.drawLine(LeftCentX, LeftCentY, LeftCentX, LeftCentY-60, ILI9340_WHITE);
  tft.drawLine(LeftCentX, LeftCentY, LeftCentX, LeftCentY+60, ILI9340_WHITE);

  tft.drawLine(RightCentX, RightCentY, RightCentX-60, RightCentY, ILI9340_WHITE);
  tft.drawLine(RightCentX, RightCentY, RightCentX+60, RightCentY, ILI9340_WHITE);
  tft.drawLine(RightCentX, RightCentY, RightCentX, RightCentY-60, ILI9340_WHITE);
  tft.drawLine(RightCentX, RightCentY, RightCentX, RightCentY+60, ILI9340_WHITE);
  
}


