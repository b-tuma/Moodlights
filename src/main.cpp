// ##### BEGIN GPL LICENSE BLOCK #####
//
//  This program is free software: you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation, either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// ##### END GPL LICENSE BLOCK #####
//
// Copyright (C) 2021  Bruno Tuma <bruno.tuma@outlook.com>

#include <NeoPixelBus.h>
#include <OneButton.h>
#include <BlynkSimpleStream.h>
#include <ESP8266WiFi.h>
#include <math.h>
#include <main.h>
#include <EEPROM.h>

const uint8_t EEPROM_size = 0x1C + 1;
const uint16_t RGBW_PixelCount = 44;
const uint16_t RGB_PixelCount = 150;
const uint8_t RGB_Rows = 3;
const uint8_t button_left_pin = 5;
const uint8_t button_right_pin = 15;
const uint16_t RGB_floor_size = RGB_PixelCount / RGB_Rows;

// Sensitive Fields
const int blynk_custom_port = 8080;
const char* blynk_custom_domain = "192.168.0.100";
const char* blynk_auth = "your_blynk_project_authentication_key";
const char* wifi_ssid = "wifi_SSID";
const char* wifi_pass = "wifi_Password";

WiFiClient wifiClient;

bool connectBlynk()
{
  wifiClient.stop();
  return wifiClient.connect(blynk_custom_domain, blynk_custom_port);
}

void connectWiFi()
{
  if (wifi_pass && strlen(wifi_pass)) {
    WiFi.begin((char*)wifi_ssid, (char*)wifi_pass);
  } else {
    WiFi.begin((char*)wifi_ssid);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// Tweening Functions

int easeInOutSine(int start, int finish, float position)
{
    float newPos = position > 1.0 ? 2.0 - position : position;
    return floor((finish - start) * (-(cos(PI * newPos) - 1) / 2)) + start;
}

int linearEasing(int position, int startValue, int change, int duration)
{
    return startValue + (position*change)/duration;
}

int linearEasingf(int position, int startValue, int change)
{
    return (startValue + (position*change)/255.) + 0.5;
}

uint8_t inOutSine8(int position,float duration)
{
    return (-127.5 * (cos(PI*position/duration)-1)) + 0.5;
}

// Color space conversions

// Based on https://www.alanzucconi.com/2016/01/06/colour-interpolation/
HsvColor lerpHSVW(HsvColor startColor, HsvColor endColor, int position, bool noRotate = false)
{
    // Hue interpolation
    int h = 0;
    int startHue = startColor.H;
    int endHue = endColor.H;
    int distance = endHue - startHue;
    if (startHue > endHue)
    {
        // Swap (startColor Hue, endColor Hue)
        endHue = startColor.H;
        startHue = endColor.H;

        distance = -distance;
        position = 255 - position;
    }

    if(distance > 127 && !noRotate) // 180deg
    {
        startHue = startHue + 255; // 360deg
        h = linearEasingf(position, startHue, (endHue - startHue)) % 255;
    }
    if(distance <= 127 || noRotate) // 180deg
    {
        h = linearEasingf(position, startHue, distance);
    }
    return HsvColor(h,
                    linearEasingf(position,startColor.S,(endColor.S - startColor.S)),
                    linearEasingf(position,startColor.V,(endColor.V - startColor.V)),
                    linearEasingf(position,startColor.W,(endColor.W - startColor.W))
                    );
}

RgbwColor HsvToRgbw(HsvColor hsvwColor)
{
    unsigned char region, remainder, p, q, t;

    if (hsvwColor.S == 0)
    {
        return RgbwColor(hsvwColor.V);
    }

    region = hsvwColor.H / 43;
    remainder = (hsvwColor.H - (region * 43)) * 6; 

    p = (hsvwColor.V * (255 - hsvwColor.S)) >> 8;
    q = (hsvwColor.V * (255 - ((hsvwColor.S * remainder) >> 8))) >> 8;
    t = (hsvwColor.V * (255 - ((hsvwColor.S * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            return RgbwColor(hsvwColor.V, t, p, hsvwColor.W);
        case 1:
            return RgbwColor(q, hsvwColor.V, p, hsvwColor.W);
        case 2:
            return RgbwColor(p, hsvwColor.V, t, hsvwColor.W);
        case 3:
            return RgbwColor(p, q, hsvwColor.V, hsvwColor.W);
        case 4:
            return RgbwColor(t, p, hsvwColor.V, hsvwColor.W);
        default:
            return RgbwColor(hsvwColor.V, p, q, hsvwColor.W);
    }
}

RgbColor HsvToRgb(HsvColor hsvColor)
{
    RgbwColor rgbwColor = HsvToRgbw(hsvColor);
    return RgbColor(rgbwColor.R, rgbwColor.G, rgbwColor.B);
}

// Upper Strip functions

NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart0800KbpsMethod> strip_upper(RGB_PixelCount);
HsvColor upper_HSV_1; // H: 0x00, S: 0x01, V: 0x02
HsvColor upper_HSV_2; // H: 0x03, S: 0x04, V: 0x05
HsvColor upper_HSV_3; // H: 0x06, S: 0x07, V: 0x08
HsvColor upper_current_HSV_1;
HsvColor upper_current_HSV_2;
HsvColor upper_current_HSV_3;
bool upper_on = false;
bool upper_update = false;
bool upper_colorshift = false; // 0x09
bool three_color_mode = false; // 0x0A
unsigned int upper_global_intensity = 100; // 0x0B
unsigned int upper_division_factor = 2; // 0x0C
unsigned int upper_intensity[RGB_Rows]; // 0: 0x0D, 1: 0x0E, 2: 0x0F
unsigned long upper_colorshift_duration = 3000; // 0x10 (divide by 1000)
uint8_t upper_transition_array[RGB_PixelCount];

void toggle_upper()
{
    if(upper_on)
    {
        upper_on = false;
        strip_upper.ClearTo(RgbColor(0));
        strip_upper.Show();
    }
    else
    {
        upper_on = true;
        upper_update = true;
    }
}

void doubleClick_upper()
{
    if(upper_on)
    {
        upper_update = true;
        upper_global_intensity = BlynkMathClamp(upper_global_intensity + 10, 0, 100);
    }
    
}


bool hold_upper;

void longClick_upper()
{
    if(upper_on)
    {
        hold_upper = true;
    }
}

void longEnd_upper()
{
    hold_upper = false;
}

void generate_upper_transition()
{
    float duration = RGB_floor_size/upper_division_factor;
    for (size_t i = 0; i < RGB_floor_size; i++)
    {
        uint8_t value = inOutSine8(i, duration);
        upper_transition_array[i] = upper_transition_array[RGB_floor_size + i] = upper_transition_array[RGB_floor_size + RGB_floor_size + i] = constrain(value, 0, 255);
    }
}


int upper_transition_step = 0;
unsigned long previousMillis_upper = 0;
bool upper_reverse = false;

void upper_strip_loop()
{
    if(upper_colorshift)
    {
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis_upper > 20)
        {
            int counter = constrain(map(currentMillis,previousMillis_upper,previousMillis_upper + upper_colorshift_duration,0,255),0 ,255);
            if(!three_color_mode) // Two Color Mode
            {
                if(upper_reverse) counter = 255 - counter;
                upper_current_HSV_1 = lerpHSVW(upper_HSV_1, upper_HSV_2, counter, true);
                upper_current_HSV_2 = lerpHSVW(upper_HSV_2, upper_HSV_1, counter, true);
                if(counter >= 255)
                {
                    previousMillis_upper = currentMillis;
                    upper_reverse = true;
                } 
                if(counter <= 0)
                {
                    previousMillis_upper = currentMillis;
                    upper_reverse = false;
                }
            }
            else // Three Color Mode
            {
                switch (upper_transition_step)
                {
                case 0:
                    upper_current_HSV_1 = lerpHSVW(upper_HSV_1, upper_HSV_2, counter, true);
                    upper_current_HSV_2 = lerpHSVW(upper_HSV_2, upper_HSV_3, counter, true);
                    upper_current_HSV_3 = lerpHSVW(upper_HSV_3, upper_HSV_1, counter, true);
                    break;
                case 1:
                    upper_current_HSV_1 = lerpHSVW(upper_HSV_2, upper_HSV_3, counter, true);
                    upper_current_HSV_2 = lerpHSVW(upper_HSV_3, upper_HSV_1, counter, true);
                    upper_current_HSV_3 = lerpHSVW(upper_HSV_1, upper_HSV_2, counter, true);
                    break;
                case 2:
                    upper_current_HSV_1 = lerpHSVW(upper_HSV_3, upper_HSV_1, counter, true);
                    upper_current_HSV_2 = lerpHSVW(upper_HSV_1, upper_HSV_2, counter, true);
                    upper_current_HSV_3 = lerpHSVW(upper_HSV_2, upper_HSV_3, counter, true);
                    break;
                }
                if(counter >= 255)
                {
                    previousMillis_upper = currentMillis;
                    upper_transition_step += 1;
                    if(upper_transition_step >= 3)
                    {
                        upper_transition_step = 0;
                    }
                }
            }
        }
    }
    else
    {
        upper_current_HSV_1 = upper_HSV_1;
        upper_current_HSV_2 = upper_HSV_2;
        upper_current_HSV_3 = upper_HSV_3;
    }
    for (size_t i = 0; i < RGB_PixelCount; i++)
    {
        // First lerp
        HsvColor hsvColor = lerpHSVW(upper_current_HSV_1, upper_current_HSV_2, upper_transition_array[i]);
        
        // Second lerp
        int terciary_influence = 0;
        switch (i)
        {
        case 0 ... RGB_floor_size - 1:
            terciary_influence = upper_intensity[0];
            break;
        case RGB_floor_size ... (RGB_floor_size * 2) - 1:
            terciary_influence = upper_intensity[1];
            break;
        default:
            terciary_influence = upper_intensity[2];
            break;
        }

        hsvColor = lerpHSVW(hsvColor, upper_current_HSV_3, terciary_influence);

        // Brightness correction
        hsvColor.V = hsvColor.V * upper_global_intensity/100;

        RgbColor color = HsvToRgb(hsvColor);
        strip_upper.SetPixelColor(i, color);
    }
    strip_upper.Show();
}

// Lower strip functions

NeoPixelBus<NeoGrbwFeature, NeoEsp8266Uart1800KbpsMethod> strip_lower(RGBW_PixelCount);
HsvColor lower_HSVW_1; // H: 0x11, S: 0x12, V: 0x13, W: 0x14
HsvColor lower_HSVW_2; // H: 0x15, S: 0x16, V: 0x17, W: 0x18
HsvColor lower_current_HSVW_1;
HsvColor lower_current_HSVW_2;
bool lower_on = false;
bool lower_update = false;
bool lower_colorshift = false; // 0x19
unsigned int lower_global_intensity = 100; // 0x1A
unsigned int lower_division_factor = 2; // 0x1B
unsigned long lower_colorshift_duration = 3000; // 0x1C (divide by 1000)
uint8_t lower_transition_array[RGBW_PixelCount];

void toggle_lower()
{
    if(lower_on)
    {
        lower_on = false;
        strip_lower.ClearTo(RgbwColor(0));
        strip_lower.Show();
    }
    else
    {
        lower_on = true;
        lower_update = true;
    }
}

void doubleClick_lower()
{
    if(lower_on)
    {
        lower_update = true;
        lower_global_intensity = BlynkMathClamp(lower_global_intensity + 10, 0, 100);
    }
}

bool hold_lower;

void longClick_lower()
{
    if(lower_on)
    {
        hold_lower = true;
    }
}

void longEnd_lower()
{
    hold_lower = false;
}

void generate_lower_transition()
{
    float duration = RGBW_PixelCount/lower_division_factor;
    for (size_t i = 0; i < RGBW_PixelCount; i++)
    {
        uint8_t value = inOutSine8(i, duration);
        lower_transition_array[i] = constrain(value,0,255);
    }   
}


unsigned long previousMillis_lower = 0;
bool lower_reverse = false;

void lower_strip_loop()
{
    if(lower_colorshift)
    {
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis_lower > 20)
        {
            int counter = constrain(map(currentMillis,previousMillis_lower,previousMillis_lower + lower_colorshift_duration,0,255),0 ,255);
            if(lower_reverse) counter = 255 - counter;
            lower_current_HSVW_1 = lerpHSVW(lower_HSVW_1, lower_HSVW_2, counter);
            lower_current_HSVW_2 = lerpHSVW(lower_HSVW_2, lower_HSVW_1, counter);
            if(counter >= 255)
            {
                previousMillis_lower = currentMillis;
                lower_reverse = true;
            } 
            if(counter <= 0)
            {
                previousMillis_lower = currentMillis;
                lower_reverse = false;
            }
        }
    }
    else
    {
        lower_current_HSVW_1 = lower_HSVW_1;
        lower_current_HSVW_2 = lower_HSVW_2;
    }
    for (size_t i = 0; i < RGBW_PixelCount; i++)
      {
          HsvColor hsvwColor = lerpHSVW(lower_current_HSVW_1, lower_current_HSVW_2, lower_transition_array[i]);
          hsvwColor.V = hsvwColor.V * lower_global_intensity/100;
          hsvwColor.W = hsvwColor.W * lower_global_intensity/100;
          RgbwColor color = HsvToRgbw(hsvwColor);
          strip_lower.SetPixelColor(i, color);
      }
      strip_lower.Show();
}

void load_params()
{
    EEPROM.begin(EEPROM_size);
    upper_HSV_1 = HsvColor(EEPROM.read(0x00),EEPROM.read(0x01),EEPROM.read(0x02),0);
    upper_HSV_2 = HsvColor(EEPROM.read(0x03),EEPROM.read(0x04),EEPROM.read(0x05),0);
    upper_HSV_3 = HsvColor(EEPROM.read(0x06),EEPROM.read(0x07),EEPROM.read(0x08),0);
    upper_colorshift = EEPROM.read(0x09);
    three_color_mode = EEPROM.read(0x0A);
    upper_global_intensity = EEPROM.read(0x0B);
    upper_division_factor = EEPROM.read(0x0C);
    upper_intensity[0] = EEPROM.read(0x0D);
    upper_intensity[1] = EEPROM.read(0x0E);
    upper_intensity[2] = EEPROM.read(0x0F);
    upper_colorshift_duration = EEPROM.read(0x10) * 1000;

    lower_HSVW_1 = HsvColor(EEPROM.read(0x11),EEPROM.read(0x12),EEPROM.read(0x13),EEPROM.read(0x14));
    lower_HSVW_2 = HsvColor(EEPROM.read(0x15),EEPROM.read(0x16),EEPROM.read(0x17),EEPROM.read(0x18));
    lower_colorshift = EEPROM.read(0x19);
    lower_global_intensity = EEPROM.read(0x1A);
    lower_division_factor = EEPROM.read(0x1B);
    lower_colorshift_duration = EEPROM.read(0x1C) * 1000;
    EEPROM.end();
}

void eeprom_write(int value, int address)
{
    if(value != EEPROM.read(address))
    {
        EEPROM.write(address, value);
    }
}

void save_params()
{
    EEPROM.begin(EEPROM_size);
    eeprom_write(upper_HSV_1.H, 0x00);
    eeprom_write(upper_HSV_1.S, 0x01);
    eeprom_write(upper_HSV_1.V, 0x02);
    eeprom_write(upper_HSV_2.H, 0x03);
    eeprom_write(upper_HSV_2.S, 0x04);
    eeprom_write(upper_HSV_2.V, 0x05);
    eeprom_write(upper_HSV_3.H, 0x06);
    eeprom_write(upper_HSV_3.S, 0x07);
    eeprom_write(upper_HSV_3.V, 0x08);
    eeprom_write(upper_colorshift, 0x09);
    eeprom_write(three_color_mode, 0x0A);
    eeprom_write(upper_global_intensity, 0x0B);
    eeprom_write(upper_division_factor, 0x0C);
    eeprom_write(upper_intensity[0], 0x0D);
    eeprom_write(upper_intensity[1], 0x0E);
    eeprom_write(upper_intensity[2], 0x0F);
    eeprom_write(upper_colorshift_duration/1000,0x10);
    eeprom_write(lower_HSVW_1.H, 0x11);
    eeprom_write(lower_HSVW_1.S, 0x12);
    eeprom_write(lower_HSVW_1.V, 0x13);
    eeprom_write(lower_HSVW_1.W, 0x14);
    eeprom_write(lower_HSVW_2.H, 0x15);
    eeprom_write(lower_HSVW_2.S, 0x16);
    eeprom_write(lower_HSVW_2.V, 0x17);
    eeprom_write(lower_HSVW_2.W, 0x18);
    eeprom_write(lower_colorshift, 0x19);
    eeprom_write(lower_global_intensity, 0x1A);
    eeprom_write(lower_division_factor, 0x1B);
    eeprom_write(lower_colorshift_duration/1000,0x1C);
    EEPROM.end();
}


OneButton button_upper(button_left_pin, false);
OneButton button_lower(button_right_pin, false);

void setup()
{
    connectWiFi();

    connectBlynk();

    Blynk.begin(wifiClient, blynk_auth);
    // this resets all the neopixels to an off state
    strip_upper.Begin();
    strip_upper.Show();
    strip_lower.Begin();
    strip_lower.Show();
    button_upper.attachClick(toggle_upper);
    button_upper.attachDoubleClick(doubleClick_upper);
    button_upper.attachLongPressStart(longClick_upper);
    button_upper.attachLongPressStop(longEnd_upper);
    button_lower.attachClick(toggle_lower);
    button_lower.attachDoubleClick(doubleClick_lower);
    button_lower.attachLongPressStart(longClick_lower);
    button_lower.attachLongPressStop(longEnd_lower);

    load_params();

    // First refresh to strips intensity array
    generate_lower_transition();
    generate_upper_transition();
}


unsigned long previousMillis_button_upper = 0;
unsigned long previousMillis_button_lower = 0;
const long interval = 500;  

void loop()
{
  // Reconnect WiFi
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    return;
  }

  // Reconnect to Blynk Cloud
  if (!wifiClient.connected()) {
    connectBlynk();
    return;
  }

  // Run Blynk
  Blynk.run();

  // Update Lower Strip
  if((lower_update || lower_colorshift) && lower_on)
  {
      lower_update = false;
      lower_strip_loop();
  }

  // Update Upper Strip
  if((upper_update || upper_colorshift) && upper_on)
  {
      upper_update = false;
      upper_strip_loop();
  }

  // Update buttons
  button_upper.tick();
  button_lower.tick();

  if(hold_upper)
  {
      unsigned long currentMillis_button_upper = millis();

      if(currentMillis_button_upper - previousMillis_button_upper >= interval)
      {
          previousMillis_button_upper = currentMillis_button_upper;

          upper_update = true;
          upper_global_intensity = BlynkMathClamp(upper_global_intensity - 10, 0, 100);
      }
  }

  if(hold_lower)
  {
      unsigned long currentMillis_button_lower = millis();

      if(currentMillis_button_lower - previousMillis_button_lower >= interval)
      {
          previousMillis_button_lower = currentMillis_button_lower;

          lower_update = true;
          lower_global_intensity = BlynkMathClamp(lower_global_intensity - 10, 0, 100);
      }
  }

}


// BLYNK INPUTS
#pragma region Blynk Inputs

BLYNK_WRITE(V0) // Save Settings
{
    if(param.asInt() == 1)
    {
        save_params();
    }
}

BLYNK_WRITE(V1) // Toggle On/Off - Lower
{
    if(param.asInt() == 1)
    {
        toggle_lower();
    }
}

BLYNK_WRITE(V2) // Global Multiplier - Lower
{
    if(!lower_on) toggle_lower();
    lower_global_intensity = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V3) // Color 1 Hue - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_1.H = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V4) // Color 1 Saturation - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_1.S = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V5) // Color 1 Value - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_1.V = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V6) // Color 1 Whiteness - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_1.W = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V7) // Color 2 Hue - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_2.H = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V8) // Color 2 Saturation - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_2.S = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V9) // Color 2 Value - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_2.V = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V10) // Color 2 Whiteness - Lower
{
    if(!lower_on) toggle_lower();
    lower_HSVW_2.W = param.asInt();
    lower_update = true;
}

BLYNK_WRITE(V11) // ColorShift Toggle - Lower
{
    if(!lower_on) toggle_lower();
    if(param.asInt() == 1)
    {
        lower_colorshift = !lower_colorshift;
    }
    lower_update = true;
}

BLYNK_WRITE(V12) // ColorShift Duration - Lower
{
    if(!lower_on) toggle_lower();
    lower_colorshift_duration = param.asInt() * 1000;
    lower_update = true;
}

BLYNK_WRITE(V13) // Division Factor - Lower
{
    if(!lower_on) toggle_lower();
    lower_division_factor = pow(2,param.asInt());
    generate_lower_transition();
    lower_update = true;
}

BLYNK_WRITE(V20) // Toggle On/Off - Upper
{
    if(param.asInt() == 1)
    {
        toggle_upper();
    }
}

BLYNK_WRITE(V21) // Global Multiplier - Upper
{
    if(!upper_on) toggle_upper();
    upper_global_intensity = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V22) // Color 1 Hue - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_1.H = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V23) // Color 1 Saturation - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_1.S = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V24) // Color 1 Value - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_1.V = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V25) // Color 2 Hue - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_2.H = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V26) // Color 2 Saturation - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_2.S = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V27) // Color 2 Value - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_2.V = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V28) // Color 3 Hue - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_3.H = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V29) // Color 3 Saturation - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_3.S = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V30) // Color 3 Value - Upper
{
    if(!upper_on) toggle_upper();
    upper_HSV_3.V = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V31) // ColorShift Toggle - Upper
{
    if(!upper_on) toggle_upper();
    if(param.asInt() == 1)
    {
        upper_colorshift = !upper_colorshift;
    }
    upper_update = true;
}

BLYNK_WRITE(V32) // ColorShift Duration - Upper
{
    if(!upper_on) toggle_upper();
    upper_colorshift_duration = param.asInt() * 1000;
    upper_update = true;
}

BLYNK_WRITE(V33) // Division Factor - Upper
{
    if(!upper_on) toggle_upper();
    upper_division_factor = pow(2,param.asInt());
    generate_upper_transition();
    upper_update = true;
}

BLYNK_WRITE(V34) // ColorShift Mode - Upper
{
    if(!upper_on) toggle_upper();
    switch (param.asInt())
    {
    case 1: // Two Color
        three_color_mode = false;
        break;
    case 2: // Three Color
        three_color_mode = true;
        break;
    }
    upper_update = true;
}

BLYNK_WRITE(V35) // Intensity 1 - Upper
{
    if(!upper_on) toggle_upper();
    upper_intensity[0] = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V36) // Intensity 2 - Upper
{
    if(!upper_on) toggle_upper();
    upper_intensity[1] = param.asInt();
    upper_update = true;
}

BLYNK_WRITE(V37) // Intensity 3 - Upper
{
    if(!upper_on) toggle_upper();
    upper_intensity[2] = param.asInt();
    upper_update = true;
}

#pragma endregion Blynk Inputs
// END OF INPUTS