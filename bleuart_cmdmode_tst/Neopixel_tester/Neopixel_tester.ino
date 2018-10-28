#include <Adafruit_NeoPixel.h>

// set to pin connected to data input of WS8212 (NeoPixel) strip
#define PIN         0

// any pin with analog input (used to initialize random number generator)
#define RNDPIN      2

// number of LEDs (NeoPixels) in your strip
// (please note that you need 3 bytes of RAM available for each pixel)
#define NUMPIXELS   20

// max LED brightness (1 to 255) – start with low values!
// (please note that high brightness requires a LOT of power)
#define BRIGHTNESS  64

// increase to get narrow spots, decrease to get wider spots
#define FOCUS       65

// decrease to speed up, increase to slow down (it's not a delay actually)
#define DELAY       4000

// set to 1 to display FPS rate
#define DEBUG       0

uint32_t color  = 0x00ffbb; // aqua

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


void setup() {
 
  // initialize LED strip
  strip.begin();
  strip.show();
}

void loop() {
    //colorLoop(10);
    strip.setPixelColor(0, color);
    

#if DEBUG
  // keep track of FPS rate
  fps++;
  if (ms>nextms) {
    // 1 second passed – reset counter
    nextms = ms + 1000;
    pfps = fps;
    fps = 0;
  }
  // show FPS rate by setting one pixel to white
  strip.setPixelColor(pfps,BRIGHTNESS,BRIGHTNESS,BRIGHTNESS);
#endif

  // send data to LED strip
  strip.show();
}

int rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
      //checkButton(buttonPin);
    }
    strip.show();
    delay(wait);
  }
}

int colorLoop(uint8_t wait) {
  uint16_t i, j;

//  int prevMode = mode;
  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((j) & 255));
      //checkButton(buttonPin);
      //if(prevMode != mode)
      //{
      //  return 0;
      //}
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

