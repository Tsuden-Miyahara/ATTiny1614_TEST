#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define LED_PIN PIN_PA4
#define NUM_LEDS 8

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ400);

void setup() {
  pixels.begin();
}

void loop() {
  pixels.clear();

  for(int i = 0; i < NUM_LEDS; i++) {// rgb(90, 60, 80): Cherry
    pixels.setPixelColor(i, pixels.Color(20 + random(108), 20 + random(108), 20 + random(108)));
  }
  pixels.show();
  delay(100);
}
