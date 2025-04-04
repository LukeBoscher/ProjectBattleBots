#include <Adafruit_NeoPixel.h>
#define DEBUG

#define NEO_PIXEL_PIN 8 //Attached to digital pin 8
#define NUM_PIXELS 4 // Number of NeoPixels

Adafruit_NeoPixel pixels(NUM_PIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // put your setup code here, to run once:
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  // put your main code here, to run repeatedly:
  lightShow();
}


void lightShow() 
{
    for (int i = 0; i < 5; i++) { // Repeat the effect 5 times
        // Rainbow cycle effect
        for (int j = 0; j < 256; j += 5) 
        {
            for (int k = 0; k < NUM_PIXELS; k++) {
                pixels.setPixelColor(k, pixels.Color(j, 255 - j, (j * 2) % 255));
            }
            pixels.show();
            delay(50);
        }

        // Strobe effect (all pixels flash white)
        for (int j = 0; j < 5; j++) {
            pixels.fill(pixels.Color(255, 255, 255)); // White
            pixels.show();
            delay(100);
            pixels.clear();
            pixels.show();
            delay(100);
        }

        // Chasing lights effect
        for (int j = 0; j < NUM_PIXELS; j++) 
        {
            pixels.clear();
            pixels.setPixelColor(j, pixels.Color(0, 255, 0)); // Green
            pixels.show();
            delay(200);
        }

        // Random colors flashing
        for (int j = 0; j < 10; j++) {
            for (int k = 0; k < NUM_PIXELS; k++) 
            {
                pixels.setPixelColor(k, pixels.Color(random(255), random(255), random(255)));
            }
            pixels.show();
            delay(200);
        }
    }

    pixels.clear();
    pixels.show();
}
