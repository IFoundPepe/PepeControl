#include <Adafruit_NeoPixel.h>

#define PIN 13

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(54, PIN);

uint8_t  mode   = 7, // Current animation effect
         offset = 0; // Position of spinny eyes
uint32_t color  = 0x00ffbb; // aqua
uint32_t other_color = 0x00ff77; //aqua green
uint32_t red_alert = 0xff0000; // red
uint32_t prevTime;
const int buttonPin = 2;    // the number of the pushbutton pin
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 20;    // the debounce time; increase if the output flickers

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  pixels.begin();
  pixels.setBrightness(80); // 1/3 brightness
  prevTime = millis();
}

void loop() {
  uint8_t  i;
  uint8_t  j;
  uint8_t  k;
  uint32_t t;

  switch(mode) {

   case 0: // Random sparks - just one LED on at a time!
    mode++;
    break;
 
   case 1: // Spinny wheels (8 LEDs on at a time)
    for(i=0; i<16; i++) {
      uint32_t c = other_color;
      if(((offset + i) & 7) < 4) c = color; // 4 pixels on...
      pixels.setPixelColor(   i, c); // First eye
    }
    pixels.show();
    offset++;
    delay(90);
    break;

    case 2: //Red Alert!
        for(i=0; i<16; i++) {
      uint32_t c = 0;
      if(((offset + i) & 7) < 1) c = red_alert; // 4 pixels on...
      pixels.setPixelColor(   i, c); // First eye
    }
    pixels.show();
    offset++;
    delay(90);
    break;

  case 3:  //Color select mode
    colorLoop(10);
    break;

  case 4:  //Hold on the current color
    break;


  case 5:  //Party Mode!
    i = random(16);
    j = random(16);
    k = random(16);
    pixels.setPixelColor(i, Wheel((random(255)) & 255));
    pixels.setPixelColor(j, Wheel((random(255)) & 255));
    pixels.setPixelColor(k, Wheel((random(255)) & 255));
    pixels.show();
    delay(100);
    pixels.setPixelColor(i, 0);
    pixels.setPixelColor(j, 0);
    pixels.setPixelColor(k, 0);
    break; 
 

  case 6:  //rave Mode!
    i = random(16);
    j = random(16);
    k = random(16);
    pixels.setPixelColor(i, Wheel((random(255)) & 255));
    pixels.setPixelColor(j, Wheel((random(255)) & 255));
    pixels.setPixelColor(k, Wheel((random(255)) & 255));
    pixels.show();
    delay(20);
    pixels.setPixelColor(i, 0);
    pixels.setPixelColor(j, 0);
    pixels.setPixelColor(k, 0);
    break; 

  case 7:
    pixels.setPixelColor(i, 0);
    pixels.setPixelColor(j, 0);
    pixels.setPixelColor(k, 0);
    pixels.show();
    break;
  }
     
  checkButton(buttonPin);
  
}

int rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel((i+j) & 255));
      checkButton(buttonPin);
    }
    pixels.show();
    delay(wait);
  }
}

int colorLoop(uint8_t wait) {
  uint16_t i, j;

  int prevMode = mode;
  for(j=0; j<256; j++) {
    for(i=0; i<pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel((j) & 255));
      checkButton(buttonPin);
      if(prevMode != mode)
      {
        return 0;
      }
    }
    pixels.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void checkButton(uint32_t button)
{
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
            //Increment mode or reset if we're over the limit
            mode++;
            if(mode > 7)
               mode = 1;
      }
    }
  }
  
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
  
}

