#include <SPI.h>
#include <Audio.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>

#include "AudioSampleIdle.h"
#include "AudioSampleHit.h"
#include "AudioSampleOn.h"
#include "AudioSampleOff.h"
#include "AudioSampleSwing.h"

#define FLASH_TYPE                  SPIFLASHTYPE_W25Q16BV
#define NUM_PIXELS                  84
#define STRIP_NEOPIXEL_PIN          5
#define BOARD_NEOPIXEL_PIN          8
#define POWER_PIN                   10
#define SWITCH_PIN                  9
#define ERROR_PIN                   13

Adafruit_QSPI_GD25Q flash;
Adafruit_NeoPixel strip(NUM_PIXELS, STRIP_NEOPIXEL_PIN, NEO_GRB);
Adafruit_NeoPixel neo_board(1, BOARD_NEOPIXEL_PIN, NEO_GRB);
Adafruit_M0_Express_CircuitPython pythonfs(flash);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

AudioPlayMemory sound0;
AudioOutputAnalogStereo audioOutput;
AudioConnection patchCord1(sound0, 0, audioOutput, 0);

volatile int sw_state = 0;
volatile int sw_flag = 0;
volatile long sw_time = 0;
volatile long sw_press = 0;

int delay_lut[NUM_PIXELS];

int mode = 0;
int mode_sel = 0;
int last_mode = 0;
int color_mode = 0;

#define NUM_COLORS 3
uint32_t saber_colors[] = { 0xFF0000, 0x007FFF, 0x00FF00 };

void sw_isr(void)
{
  long t = millis();
  if((t - sw_press) > 10)
  {
    if(!sw_state)
    {
      // Switch is down
      sw_state = 1;
      sw_time = t;
    } else {
      // Switch was released
      sw_state = 0;
      sw_flag = 1;
      sw_time = t - sw_time;
    }
  }
  sw_press = t;
}

void error(String error) {
  Serial.print("******* ERROR *******  ");
  Serial.println(error);
  while(1)
  {
    neo_board.setPixelColor(0, neo_board.Color(255, 0, 0));
    neo_board.show();
    delay(500);

    neo_board.setPixelColor(0, neo_board.Color(0, 0, 255));
    neo_board.show();
    delay(500);
  }
}

void setup() {
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  //while (!Serial) {
  //  delay(100);
  //}

  Serial.println("Starting lightsaber ..................");

  mode = 0;
  last_mode = 0;
  color_mode = 0;
  
  Serial.println("Setup Digital pins ...................");
  pinMode(POWER_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(ERROR_PIN, OUTPUT);
  sw_press = millis();
  digitalWrite(ERROR_PIN, 0);
  digitalWrite(POWER_PIN, 0);

  // Setup switch interrupt
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), sw_isr, CHANGE);

  Serial.println("Setup NeoPixels ......................");
  neo_board.begin();
  neo_board.setBrightness(127);
  neo_board.setPixelColor(0, neo_board.Color(255, 0, 0));
  neo_board.show();

  strip.begin();
  strip.setBrightness(32);
  strip.show();

  // Setup Audio
  Serial.println("Setup Audio ..........................");
  AudioMemory(10);

  Serial.println("Setup Accelerometer ..................");
  int _c = 5;
  while(_c >= 0)
  {
    _c--;
    Serial.println(_c);
    if(lis.begin(0x18)) 
    {
      break;
    }
    Serial.println("Accelerometer Error ... trying again ...");
  }
  if(_c <= 0)
  {
    error("Unable to find Accelerometer ................");
  }
  lis.setRange(LIS3DH_RANGE_4_G);


  digitalWrite(POWER_PIN, 1);
  delay(1000);

  strip.setBrightness(0);
  strip.fill(strip.Color(0, 0, 0));
  strip.show();

  Serial.println("Setup Lookup Tables ..................");
  int n;
  for(n=0;n<NUM_PIXELS;n++)
  {
    delay_lut[n] = 20 * cos(n * M_PI / (2 * NUM_PIXELS));
  }

  Serial.println("Setup Done.");

  neo_board.setPixelColor(0, neo_board.Color(0, 0, 0));
  neo_board.show();
}

void saber_power_on(void)
{
  // To so some form 
  strip.setBrightness(64);
  int n=0;
  for(n = 0;n<NUM_PIXELS;n++)
  {
    strip.setPixelColor(n, saber_colors[color_mode]);
    strip.show();
    delay(delay_lut[n]);
  }
}

void saber_power_off(void)
{
  // To so some form 
  int n=0;
  for(n = 0;n<NUM_PIXELS;n++)
  {
    strip.setPixelColor(NUM_PIXELS - n, strip.Color(0, 0, 0));
    strip.show();
    delay(delay_lut[n]);
  }
}

void saber_swing_on(void)
{
  int n;

  strip.fill(saber_colors[color_mode]);
  for(n=64; n<=255; n++)
  {
    strip.setBrightness(n);
    strip.show();
  }
}

void saber_idle(void)
{
  int n;

  strip.fill(saber_colors[color_mode]);
  for(n=255; n>=64; n--)
  {
    strip.setBrightness(n);
    strip.show();
  }
}

void saber_hit_on(void)
{
  int n;

  strip.fill(strip.Color(255, 255, 255));
  for(n=64; n<=255; n++)
  {
    strip.setBrightness(n);
    strip.show();
  }
}

void saber_flash(int times)
{
  int n, m;

  for(m=0;m<times;m++)
  {
    strip.setBrightness(255);
    strip.fill(saber_colors[color_mode]);
    strip.show();
    delay(250);
    strip.setBrightness(0);
    strip.show();
    delay(250);
  }
}

void loop() { 
  if(sw_flag)
  {
    // The switch was pressed, sw_time is the press time
    sw_flag = 0;
    Serial.print(sw_time);
    Serial.println("");

    if(sw_time < 1000)
    {
      if(mode)
      {
        sound0.play(AudioSampleOff);
        saber_power_off();
        digitalWrite(POWER_PIN, 0);
        mode = 0;
      } else {
        digitalWrite(POWER_PIN, 1);
        sound0.play(AudioSampleOn);
        saber_power_on();
        mode = 1;
      }
    }
  }

  if(sw_state && mode) 
  {
    if((millis() - sw_time) < 4000)
    {
      mode_sel = 0;
    } else if(((millis() - sw_time) >= 4000) && (mode_sel < 1)) {
      saber_flash(2);
      saber_idle();
      mode_sel = 1;
    } else if(((millis() - sw_time) >= 8000) && (mode_sel < 2)) {
      saber_flash(4);
      saber_idle();
      mode_sel = 2;
    }
  }

  if(!sw_state && (mode_sel != 0))
  {
    if(mode_sel == 1)
    {
      color_mode++;
      if(color_mode >= NUM_COLORS)
      {
        color_mode = 0;
      }
      saber_idle();
    }

    mode_sel = 0;
  }

  if(mode)
  {
    lis.read();

    long int acc = (int)pow(lis.x, 2);
    acc += (int)pow(lis.y, 2);
    acc += (int)pow(lis.z, 2);
    acc /= 100000;

    if(acc > 1000 && acc <= 1300)
    {
      mode = 2;
    } else if(acc > 1300){
      mode = 3;
    } else {
      mode = 1;
    }

    if(mode > 1)
    {
      Serial.print(acc);
      Serial.print(" \t");
      Serial.print(mode);
      Serial.print(" \t");
      Serial.print(last_mode);
      Serial.print(" \tX: "); Serial.print(lis.x);
      Serial.print(" \tY: "); Serial.print(lis.y);
      Serial.print(" \tZ: "); Serial.print(lis.z);
      Serial.println(" m/s^2 ");
    }

    if((mode == 1) && (last_mode == 1))
    {
      if(!sound0.isPlaying())
      {
        sound0.play(AudioSampleIdle);
      }
    }

    if((mode == 1) && (last_mode != 1))
    {
      sound0.play(AudioSampleIdle);
      saber_idle();
    }

    if((mode == 2) && (last_mode == 1))
    {
      // swing
      sound0.play(AudioSampleSwing);
      saber_swing_on();
    }
    
    if((mode == 3) && (last_mode == 1))
    {
      sound0.play(AudioSampleHit);
      saber_hit_on();
    }

    if((mode == 2) && (last_mode == 3))
    {
      sound0.play(AudioSampleSwing);
      saber_swing_on();
    }

  } else {
    digitalWrite(ERROR_PIN, 0);
  }

  last_mode = mode;

}
