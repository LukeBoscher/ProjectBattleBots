#include <Adafruit_NeoPixel.h>

#define NEO_PIXEL_PIN 8 //Attached to digital pin 8
#define NUMPIXELS 4 // Number of NeoPixels


// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  brakeLight();
}


void brakeLight()
{
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 150, 0)); //Set left rear color to orange (G,R,B)
  pixels.setPixelColor(1, pixels.Color(0, 150, 0)); //Set right rear color to orange (G,R,B)
  pixels.show();   // Send the updated pixel colors to the hardware.
}

void rightSignal()
{
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(1, pixels.Color(70, 255, 0)); //Set right rear color to orange (G,R,B)
    pixels.setPixelColor(2, pixels.Color(70, 255, 0)); //Set right front color to orange (G,R,B)
    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(500); // Wait for the specified time

    // Turn the pixels off
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(500); // Wait for the specified time
}

void leftSignal()
{
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.setPixelColor(0, pixels.Color(70, 255, 0)); //Set left rear color to orange (G,R,B)
    pixels.setPixelColor(3, pixels.Color(70, 255, 0)); //Set left front color to orange (G,R,B)
    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(500); // Wait for the specified time

    // Turn the pixels off
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(500); // Wait for the specified time
}