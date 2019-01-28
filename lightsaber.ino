/*
 * =====================================================================================
 *
 *       Filename:  lightsaber.ino
 *
 *    Description:  Arduino (Adafruit M4 Feather) lightsaber code
 *
 *        Version:  1.0
 *        Created:  01/27/2019 09:10:23
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stuart B. Wilkins (sbw), stuart@stuwilkins.org
 *
 * =====================================================================================
 */

#include <Audio.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_SleepyDog.h>

// Sound files

#include "AudioSampleIdle.h"
#include "AudioSampleHit.h"
#include "AudioSampleOn.h"
#include "AudioSampleOff.h"
#include "AudioSampleSwing.h"
#include "AudioSampleDont_underestimate.h"
#include "AudioSampleDarthfaith.h"

// Definitions

#define NUM_PIXELS                  83
#define STRIP_NEOPIXEL_PIN          5
#define BOARD_NEOPIXEL_PIN          8
#define POWER_PIN                   10
#define SWITCH_PIN                  9
#define RED_LED_PIN					11 
#define GREEN_LED_PIN				12 
#define BLUE_LED_PIN				13 
#define VBAT_PIN                    A6
#define SW_DEBOUNCE_TIME            100
#define NEOPIXEL_RED                0xFF0000
#define NEOPIXEL_GREEN              0x00FF00
#define NEOPIXEL_BLUE               0x0000FF
#define NEOPIXEL_OFF                0x000000
#define WAKEUP_THRESHOLD            1500
#define HIT_THRESHOLD               1000
#define SWING_THRESHOLD             850
#define LED_TIMEOUT                 10000
#define WATCHDOG_TIME               10000
#define LED_LUT_N                   650

#define DEBUG                       1

#define debug_print(fmt) \
            do { if (DEBUG) Serial.print(fmt); } while (0)
#define debug_println(fmt) \
            do { if (DEBUG) Serial.println(fmt); } while (0)

// Hardware definitions

Adafruit_NeoPixel strip(NUM_PIXELS * 2, STRIP_NEOPIXEL_PIN, NEO_GRB);
Adafruit_NeoPixel neo_board(1, BOARD_NEOPIXEL_PIN, NEO_GRB);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
AudioPlayMemory sound0;
AudioOutputAnalogStereo audioOutput;
AudioConnection patchCord1(sound0, 0, audioOutput, 0);

// Global Variables

volatile int sw_state = 0;
volatile int sw_flag = 0;
volatile long sw_time = 0;
volatile long sw_press_rising = 0;
volatile long sw_press_falling = 0;

int delay_lut[NUM_PIXELS];
int led_lut[LED_LUT_N];

int mode = 0;
int mode_sel = 0;
int last_mode = 0;
int color_mode = 0;

unsigned long loop_timer = 0;
unsigned long led_timer = 0;
unsigned long led_count = 0;

#define NUM_COLORS 3
uint32_t saber_colors[] = { 0x007FFF, 0xFF0000, 0x00FF00 };

void switch_isr_rising(void);
void switch_isr_falling(void);

void switch_isr_falling(void)
{
    long t = millis();
    if((t - sw_press_falling) > SW_DEBOUNCE_TIME)
    {
        sw_time = t;
        sw_state = 1;
        attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switch_isr_rising, RISING);
        sw_press_falling = t;
    }
}

void switch_isr_rising(void)
{
    long t = millis();
    if((t - sw_press_falling) > SW_DEBOUNCE_TIME)
    {
        sw_state = 0;
        sw_flag = 1;
        sw_time = t - sw_time;
        attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switch_isr_falling, FALLING);
        sw_press_falling = t;
    }
}

void set_rgb_led(int led, int val)
{
    // Ensure Power is on
    if(led)
    {
        digitalWrite(POWER_PIN, HIGH);
    }
    int neo = 0;

    analogWrite(GREEN_LED_PIN, 0);
    analogWrite(BLUE_LED_PIN, 0);
    analogWrite(RED_LED_PIN, 0);

    if(led & 0x4)
    {
        // Red LED
        neo |= ((val & 0xFF) << 16);
        analogWrite(RED_LED_PIN, val);
    }

    if(led & 0x2)
    {
        // Green LED
        neo |= ((val & 0xFF) << 8);
        analogWrite(GREEN_LED_PIN, val);
    }

    if(led & 0x1)
    {
        // Blue LED
        neo |= (val & 0xFF);
        analogWrite(BLUE_LED_PIN, val);
    }

    neo_board.setPixelColor(0, neo); 
    neo_board.show();
}

void led_flash(int num_flash)
{
    for(int n=0;n<num_flash;n++)
    {
        set_rgb_led(0x2, 255);
        delay(100);
        set_rgb_led(0x0, 255);
        delay(100);
    }
}

void error(String error) {
    debug_print("******* ERROR *******  ");
    debug_println(error);
    while(1)
    {
        set_rgb_led(0x4, 255); // RED
        delay(100);
        set_rgb_led(0x0, 255); // RED
        delay(100);
    }
}

float get_battery_voltage(void)
{
	float measuredvbat = analogRead(VBAT_PIN);
	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	return measuredvbat;
}

void Timer4Callback0()
{
    debug_println("Tick tock...");
}

void setup() {
    // Initialize serial port and wait for it to open before continuing.
    pinMode(POWER_PIN,     OUTPUT);
    pinMode(SWITCH_PIN,    INPUT_PULLUP);
    pinMode(RED_LED_PIN,   OUTPUT);
    pinMode(BLUE_LED_PIN,  OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    // Setup onboard neopixel
    neo_board.begin();
    neo_board.setBrightness(255);
    set_rgb_led(0x4, 255);

    Serial.begin(115200);
    unsigned long _start = millis();
    while (!Serial && ((millis() - _start) < 15000)) 
    {
        delay(100);
    }

    debug_println("Starting lightsaber ..................");
    set_rgb_led(0x6, 255);

    // Setup watchdog
    int countdownMS = Watchdog.enable(WATCHDOG_TIME);
    debug_print("Set watchdog time to ");
    debug_println(countdownMS);

    mode = 0;
    last_mode = 0;
    color_mode = 0;

    sw_press_falling = millis();
    sw_press_rising = millis();

    // Setup switch interrupt
    attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), switch_isr_falling, FALLING);

    debug_println("Setup NeoPixels ......................");

    strip.begin();
    strip.setBrightness(32);
    strip.show();

    // Setup Audio
    debug_println("Setup Audio ..........................");
    AudioMemory(10);

    debug_println("Setup Accelerometer ..................");
    if(!lis.begin(0x18)) 
    {
        error("Unable to find Accelerometer ................");
    }

    debug_println("Setup Accelerometer Range ............");
    lis.setRange(LIS3DH_RANGE_4_G);

    strip.setBrightness(0);
    strip.fill(strip.Color(0, 0, 0));
    strip.show();

    debug_println("Setup Lookup Tables ..................");
    for(int n=0;n<NUM_PIXELS;n++)
    {
        delay_lut[n] = 20 * cos(n * M_PI / (2 * NUM_PIXELS));
#ifdef DISPLAY_LUT
        debug_print("\t");
        debug_print(n);
        debug_print("\t");
        debug_print(delay_lut[n]);
        debug_println("");
#endif
    }

    for(int n=0;n<LED_LUT_N;n++)
    {
        led_lut[n] = 255 * sin(n * M_PI / LED_LUT_N);
    }

    debug_println("Setup Done.");
    Watchdog.reset();

    // Indicate we are ready by turning on LED
    set_rgb_led(0x1, 255);
    sound0.play(AudioSampleDont_underestimate);

    led_timer = millis();
    led_count = 0;
}

void saber_power_on(void)
{
    // To so some form 
    strip.setBrightness(64);
    for(int n = 0;n<NUM_PIXELS;n++)
    {
        strip.setPixelColor(n, saber_colors[color_mode]);
        strip.setPixelColor((NUM_PIXELS * 2) - n - 1, saber_colors[color_mode]);
        strip.show();
        delay(delay_lut[n]);
        Watchdog.reset();
    }
}

void saber_power_off(void)
{
    // To so some form 
    for(int n = 0;n<NUM_PIXELS;n++)
    {
        strip.setPixelColor(NUM_PIXELS - n - 1, strip.Color(0, 0, 0));
        strip.setPixelColor(NUM_PIXELS + n, strip.Color(0, 0, 0));
        strip.show();
        delay(delay_lut[n]);
        Watchdog.reset();
    }
}

void saber_swing_on(void)
{
    strip.fill(saber_colors[color_mode]);
    for(int n=64; n<=255; n+=2)
    {
        strip.setBrightness(n);
        strip.show();
        Watchdog.reset();
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
        Watchdog.reset();
    }
}

void saber_hit_on(void)
{
    strip.fill(strip.Color(255, 255, 255));
    for(int n=64; n<=255; n+=2)
    {
        strip.setBrightness(n);
        strip.show();
        Watchdog.reset();
    }
}

void saber_flash(int times)
{
    for(int m=0;m<times;m++)
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
    Watchdog.reset(); // Pet the dog!

    // Periodically read battery voltage
    if((millis() - loop_timer) > 10000)
    {
        debug_print("Battery voltage = ");
        debug_println(get_battery_voltage());

        loop_timer = millis();
    }
    
    // Read accelerometer
    lis.read();
    long int acc = (int)pow(lis.x, 2);
    acc += (int)pow(lis.y, 2);
    acc += (int)pow(lis.z, 2);
    acc /= 100000;

    if(sw_flag)
    {
        // The switch was pressed, sw_time is the press time
        sw_flag = 0;
        debug_print("Switch pressed for :");
        debug_print(sw_time);
        debug_println("");

        if(sw_time < 1000)
        {
            if(mode)
            {
                sound0.play(AudioSampleOff);
                set_rgb_led(0x4, 255);
                saber_power_off();
                mode = 0;
                led_timer = millis();
                led_count = 0;
            } else {
                digitalWrite(POWER_PIN, HIGH);
                set_rgb_led(0x2, 255);
                sound0.play(AudioSampleOn);
                saber_power_on();
                mode = 1;
            }
        }
    }

    // Turn off signaling LED after predefined time
    if(!mode)
    {
        if((millis() - led_timer) > LED_TIMEOUT)
        {
            set_rgb_led(0, 0);
            digitalWrite(POWER_PIN, 0);
            led_count++;
        } else {
            // Pulse the LED
            set_rgb_led(0x1, led_lut[led_count % LED_LUT_N]);
            led_count++;
        }
    }

    // Check for multimode
    if(sw_state && mode) 
    {
        if((millis() - sw_time) < 1000)
        {
            mode_sel = 0;
        } else if(((millis() - sw_time) >= 2000) && (mode_sel < 1)) {
            led_flash(2);
            mode_sel = 1;
        } else if(((millis() - sw_time) >= 3000) && (mode_sel < 2)) {
            led_flash(3);
            mode_sel = 2;
        } else if(((millis() - sw_time) >= 4000) && (mode_sel < 3)) {
            led_flash(4);
            mode_sel = 3;
        }
    }

    if(!sw_state && (mode_sel != 0))
    {
        if(mode_sel == 3)
        {
            color_mode++;
            if(color_mode >= NUM_COLORS)
            {
                color_mode = 0;
            }
            saber_idle();
        } else if(mode_sel == 2) {
            //sound0.play(AudioSampleMaster);
        } else if(mode_sel == 1) {
            sound0.play(AudioSampleDarthfaith);
        }

        mode_sel = 0;
        sw_state = 0;
        set_rgb_led(0x2, 255);
    }

    // If we hold the saber, pulse the LEDs
    if(!mode && (acc > WAKEUP_THRESHOLD))
    {
        led_timer = millis();
    }

    if(mode)
    {

        if(acc > HIT_THRESHOLD){
            mode = 3;
        } else if(acc > SWING_THRESHOLD) {
            mode = 2;
        } else {
            mode = 1;
        }

        if(mode > 1)
        {
            debug_print(acc);
            debug_print(" \t");
            debug_print(mode);
            debug_print(" \t");
            debug_print(last_mode);
            debug_print(" \tX: "); debug_print(lis.x);
            debug_print(" \tY: "); debug_print(lis.y);
            debug_print(" \tZ: "); debug_print(lis.z);
            debug_println(" m/s^2 ");
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
            sound0.stop();
            sound0.play(AudioSampleSwing);
            saber_swing_on();
        }

        if((mode == 3) && (last_mode == 1))
        {
            sound0.stop();
            sound0.play(AudioSampleHit);
            saber_hit_on();
        }

        if((mode == 2) && (last_mode == 3))
        {
            sound0.play(AudioSampleSwing);
            saber_swing_on();
        }

    }

    last_mode = mode;
}
