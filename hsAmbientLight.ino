#include <avr/sleep.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define NEOPIXEL_CONTROL_PIN 3 
#define NUMPIXELS 144 
#define STATE_CONTROL_BUTTON 12
#define LIGHT_SENSOR A3
#define LED_BASIC 13
static uint8_t BRIGHTNESS_MAX = 64;
static uint8_t BRIGHTNESS_MIN = 16;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL_CONTROL_PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 20 

void setup()
{
  Serial.begin(115200);
// This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined(__AVR_ATtiny85__)
  if (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pinMode(LED_BASIC, OUTPUT);  
  digitalWrite(LED_BASIC, HIGH);
  
  strip.setBrightness(BRIGHTNESS_MAX);
  strip.begin();
  strip.clear();
  strip.show(); // Initialize all pixels to 'off'

  pinMode(LIGHT_SENSOR, INPUT);  
  float light_value = 0;
  for(int i = 0; i < 10; i++) {
    int val = analogRead(LIGHT_SENSOR);
    light_value += val / 10.0;
    delay(10);
  }

  if(light_value > 900) {
    BRIGHTNESS_MAX = 64;
    BRIGHTNESS_MIN = 16;
    blink(1, 100);
  }
  else {
    BRIGHTNESS_MAX = 128;
    BRIGHTNESS_MIN = 32;
    blink(2, 100);
  }

  // Serial.print("avg value : ");
  // Serial.println(light_value);

  pinMode(STATE_CONTROL_BUTTON, INPUT);
  digitalWrite(STATE_CONTROL_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STATE_CONTROL_BUTTON), click_control_button, LOW);

  // WELCOME START //
  colorStack(strip.Color(0, 0, BRIGHTNESS_MAX), 0, 7); 
  strip.clear();
  strip.show();
  colorWipe(strip.Color(0, 0, BRIGHTNESS_MAX), 1); 
  
  for(int n = 0; n < 2; n++) {
    for(int brightness = BRIGHTNESS_MAX; brightness >= BRIGHTNESS_MIN; brightness--) {
      strip.setBrightness(brightness);
      for (int i = 0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, strip.Color(0, 0, brightness));
      }
      strip.show();
      delay(10);
    }
    for(int brightness = BRIGHTNESS_MIN; brightness <= BRIGHTNESS_MAX; brightness++) {
      strip.setBrightness(brightness);
      for (int i = 0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, strip.Color(0, 0, brightness));
      }
      strip.show();
      delay(10);
    }
  }

  delay(5*1000);

  for(int brightness = BRIGHTNESS_MAX; brightness >= 0; brightness--) {
    strip.setBrightness(brightness);
    for (int i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, strip.Color(0, 0, brightness));
    }
    strip.show();
    delay(10);
  }

  strip.clear();
  strip.setBrightness(BRIGHTNESS_MAX);
  strip.show();
}

int state = 0;

void loop()
{
  sleep_cpu();
  
  // Serial.print("state : ");
  // Serial.println(state);

  int button_state = digitalRead(STATE_CONTROL_BUTTON);
  if(button_state == HIGH) {
    delay(100);
    return;
  }
  else {
    state++;
  }
  
  if(state == 1) {
      colorStack(strip.Color(BRIGHTNESS_MAX, 0, 0), 0, 7); 
  }
  else if(state == 2) {
      colorStack(strip.Color(0, BRIGHTNESS_MAX, 0), 0, 7); 
  }
  else if(state == 3) {
      colorStack(strip.Color(0, 0, BRIGHTNESS_MAX), 0, 7); 
  }
  else if(state == 4) {
    rainbowCycle(10);
    state = 0;
  }
  else {
    state = 0;
  }

  delay(1000);
  colorWipe(strip.Color(0, 0, 0), 1);
  
  // colorWipe(strip.Color(0, 0, 0, 255), 50); // White RGBW
  //  Send a theater pixel chase in...

  //  rainbow(20);
  //  rainbowCycle(20);
}
void blink(int count, int interval_ms) {
  for(int i = 0; i < count; i++) {
    digitalWrite(LED_BASIC, HIGH);
    delay(interval_ms);
    digitalWrite(LED_BASIC, LOW);
    delay(interval_ms);
  }
}
void click_control_button(void) 
{
  state++;
  Serial.println(state);

}

void colorWipe(uint32_t c, uint8_t wait)
{
  for (uint16_t i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void colorStack(uint32_t c, uint8_t wait, uint8_t block_size)
{
  for (uint16_t i = strip.numPixels() - 1; i >= block_size; i -= block_size) {

    for(uint16_t i = 0; i < block_size; i++) { // init state
      strip.setPixelColor(i, c);
    }
    strip.show();

    for(uint16_t j = block_size; j < i; j += 1) {
        strip.setPixelColor(j - block_size, strip.Color(0, 0, 0));
        strip.setPixelColor(j + block_size-1, c);
        strip.show();
        delay(wait);
    }
  }
}

void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256; j++)
  {
    for (i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++)
  { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++)
    {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return strip.Color((255 - WheelPos * 3)/2, 0, (WheelPos * 3)/2);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color(0, (WheelPos * 3)/2, (255 - WheelPos * 3)/2);
  }
  WheelPos -= 170;
  return strip.Color((WheelPos * 3)/2, (255 - WheelPos * 3)/2, 0);
}
