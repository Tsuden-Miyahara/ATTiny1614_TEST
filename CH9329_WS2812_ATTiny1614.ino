#include <CH9329_Keyboard.h>

// #include <Adafruit_NeoPixel.h>
#include <tinyNeoPixel.h>
#include <avr/power.h>

#include <RotaryEncoder.h>
// #include <SoftwareSerial.h>

//#include "CH9329.h"


// 74HC165 (Keys)
#define HC165_SL     PIN_PA1 // SHLD (1)
#define HC165_CLK    PIN_PA2 // CLK  (2)
#define HC165_SER    PIN_PA3 // Q7   (9)

// WS2812C-2020 (LEDs)
#define LED_PIN      PIN_PA4
#define NUM_LEDS     8

// Rotary-Encoder
#define ROT_LED_1    PIN_PA5
#define ROT_LED_2    PIN_PA6
#define ROT_SW       PIN_PA7
#define ROT_A        PIN_PB0
#define ROT_B        PIN_PB1

// CH9329 (HID)
#define CH9329_TX    PIN_PB2
#define CH9329_RX    PIN_PB3

#define STAT         24
#define TIMER_PERIOD 10

volatile uint16_t _incomingData;
volatile uint16_t incomingData = 0xFFFF;
volatile uint16_t incomingDataOld = 0xFFFF;

volatile long oldPosition = -999;
volatile long newPosition = 0;
volatile uint8_t stat = 0;

uint8_t reportData[KEY_REPORT_DATA_LENGTH] = {};

RotaryEncoder encoder(ROT_B, ROT_A, RotaryEncoder::LatchMode::FOUR3);

//SoftwareSerial CH9329_Serial(CH9329_TX, CH9329_RX);
//Keyboard kb = Keyboard(&Serial);

// Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ400);
tinyNeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile uint32_t timer_count = 0;

byte myShiftIn(int dataPin, int clockPin, int loadPin);
uint32_t getColor(bool pushed, uint8_t lv);
void Press(byte ckey, byte ukey);

void setup() {
  Serial.begin(9600);
  CH9329_Keyboard.begin(Serial);

  pixels.begin();
  
  pinMode(HC165_SL, OUTPUT);
  pinMode(HC165_CLK, OUTPUT);
  pinMode(HC165_SER, INPUT);
  
  //pinMode(ROT_A, INPUT);
  //pinMode(ROT_B, INPUT);
  
  pinMode(ROT_LED_1, OUTPUT);
  pinMode(ROT_LED_2, OUTPUT);

  pinMode(ROT_SW, INPUT_PULLUP);
  
  digitalWrite(HC165_SL, HIGH);
  digitalWrite(HC165_CLK, LOW);

  //prev_ROT_A_VAL = digitalRead(ROT_A);
  //prev_ROT_B_VAL = digitalRead(ROT_B);
  
  // タイマー設定
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
  TCB0.CCMP = 1;// F_CPU / TIMER_PERIOD - 1;
  TCB0.INTCTRL = TCB_CAPT_bm;

  // 割り込み設定
  sei();
}

void loop() {

  if (fastDigitalRead(ROT_SW) == LOW) {
    stat = 0;
    encoder.setPosition(0);
    fastDigitalWrite(ROT_LED_1, HIGH);
    fastDigitalWrite(ROT_LED_2, HIGH);
    while (fastDigitalRead(ROT_SW) == LOW);
    fastDigitalWrite(ROT_LED_1, LOW);
    fastDigitalWrite(ROT_LED_2, LOW);
  }

  pixels.clear();
  _incomingData = incomingData;
  for(uint8_t i = 0; i < NUM_LEDS; i++) {// rgb(90, 60, 80): Cherry
    pixels.setPixelColor(
      i,
      getColor(
        isPressed(i),
        getLv(i)
      )
    );
  }
  pixels.show();

  if (isPressed(0)) {
    CH9329_Keyboard.write(KEY_KP_ENTER);
  }
  else if (isPressed(1)) {
    switch(getLv(1)) {
      case 0:
        CH9329_Keyboard.print("Hello World!\n");
        break;
      default:
        CH9329_Keyboard.print("Hi there!\n");
        break;
    }
  }
}

volatile uint8_t _temp;
ISR(TCB0_INT_vect) {
  timer_count++;

  // setRotValue();
  encoder.tick();
  newPosition = encoder.getPosition();
  if (newPosition != oldPosition) {
    if (newPosition < 0) {
      encoder.setPosition(0);
      _temp = 0;
    }
    else if (STAT < newPosition) {
      encoder.setPosition(STAT);
      _temp = STAT;
    }
    else {
      _temp = newPosition;
    }

    if (_temp < stat) {
      fastDigitalWrite(ROT_LED_1, LOW);
      fastDigitalWrite(ROT_LED_2, HIGH);
    }
    else if (_temp > stat) {
      fastDigitalWrite(ROT_LED_1, HIGH);
      fastDigitalWrite(ROT_LED_2, LOW);
    }
    stat = _temp;
    oldPosition = newPosition;
  }

  incomingData = myShiftIn(HC165_SER, HC165_CLK, HC165_SL);
  
  if (incomingData != incomingDataOld){
    incomingDataOld = incomingData;
  }

  if (999 < timer_count) {
    timer_count = 0;
  }
}







byte myShiftIn(int dataPin, int clockPin, int loadPin){
  byte data;

  fastDigitalWrite(loadPin, LOW); //A-Hを格納
  fastDigitalWrite(loadPin, HIGH); //確定
  
  data = fastDigitalRead(dataPin); //Hの値を読む
  for (uint8_t i = 1; i < 8; i++){
    fastDigitalWrite(clockPin, HIGH);
    data = data << 1 | (fastDigitalRead(dataPin)); //G,F,E...Aの値を読む
    fastDigitalWrite(clockPin, LOW);
  }
  return data;
}

void fastDigitalWrite(uint8_t pin, uint8_t val) {
  uint8_t bit = digitalPinToBitMask(pin);
  volatile uint8_t *out;
  out = portOutputRegister( digitalPinToPort(pin) );
  if (val == LOW) *out &= ~bit;
  else *out |= bit;
}
uint8_t fastDigitalRead(uint8_t pin)
{
  if (*portInputRegister( digitalPinToPort(pin) ) & digitalPinToBitMask(pin)) return HIGH;
  return LOW;
}

bool isPressed(byte i) {
  return (_incomingData >> i & 1) == 0;
}
byte getLv(byte i) {
  return stat / 8 + (i < (stat % 8) ? 1 : 0);
}
uint32_t getColor(bool pressed, uint8_t lv) {
  if (pressed) return pixels.Color(255, 255, 255);
  switch(lv) {
    case 1: return pixels.Color(30, 50, 90);
    case 2: return pixels.Color(60, 90, 20);
    case 3: return pixels.Color(90, 50, 10);
    default: return pixels.Color(90, 60, 80);
  }
}

/*
byte keyRelease[14] = {0x57,0xAB,0x00,0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C};
void Press(byte ckey, byte ukey) {
  byte sum = 0x10C + ckey + ukey;
  byte keyPress[14] = {
    0x57,0xAB,0x00,0x02,0x08,
    ckey,
    0x00,
    ukey,0x00,0x00,0x00,0x00,0x00,
    sum
  };
  fastDigitalWrite(ROT_LED_1, HIGH);
  fastDigitalWrite(ROT_LED_2, LOW);
  Serial.write(keyPress, 14);
  //while(Serial.available() > 0) {Serial.read();}
  Serial.write(keyRelease, 14);
  //while(Serial.available() > 0) {Serial.read();}
  fastDigitalWrite(ROT_LED_1, LOW);
  fastDigitalWrite(ROT_LED_2, HIGH);
  //delay(5);
}
*/




