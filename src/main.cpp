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

const uint16_t RGBW_PixelCount = 44;
const uint16_t RGB_PixelCount = 150;
const uint8_t button_left_pin = 5;
const uint8_t button_right_pin = 15;

OneButton button_left(button_left_pin, false);
OneButton button_right(button_right_pin, false);

NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart0800KbpsMethod> strip_upper(RGB_PixelCount);
NeoPixelBus<NeoGrbwFeature, NeoEsp8266Uart1800KbpsMethod> strip_lower(RGBW_PixelCount);

// Sensitive Fields
const int blynk_custom_port = 8080;
const char* blynk_custom_domain = "192.168.0.100";
const char* blynk_auth = "your_blynk_project_authentication_key";
const char* wifi_ssid = "wifi_SSID";
const char* wifi_pass = "wifi_Password";


float old_globalIntensity_upper;

bool isOn_upper = true;


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

HsvwColor lower_HSVW_2;
HsvwColor lower_HSVW_1;
int lower_global_intensity = 100;
bool lower_colorshift = false;
unsigned long lower_colorshift_duration = 3000;
bool lower_on = true;
int lower_global_intensity_old;
int lower_division_factor = 2;
uint8_t lower_transition_array[RGBW_PixelCount];
HsvwColor lower_current_1;
HsvwColor lower_current_2;
int lower_current_time = 0;

int upper_HSV_1[3];
int upper_HSV_2[3];
int upper_HSV_3[3];
float intensity1;
float intensity2;
float intensity3;
float globalIntensity_upper = 1.0;
bool updateUpper = false;

int middle_RGBW[4];
int sides_RGBW[4];

bool updateLower = false;


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

// Based on https://www.alanzucconi.com/2016/01/06/colour-interpolation/
HsvwColor lerpHSVW(HsvwColor startColor, HsvwColor endColor, int position)
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

    if(distance > 127) // 180deg
    {
        startHue = startHue + 255; // 360deg
        h = linearEasingf(position, startHue, (endHue - startHue)) % 255;
    }
    if(distance <= 127) // 180deg
    {
        h = linearEasingf(position, startHue, distance);
    }
    return HsvwColor(h,
                    linearEasingf(position,startColor.S,(endColor.S - startColor.S)),
                    linearEasingf(position,startColor.V,(endColor.V - startColor.V)),
                    linearEasingf(position,startColor.W,(endColor.W - startColor.W))
                    );
}

RgbwColor HsvToRgbw(HsvwColor hsvwColor)
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

void click_left()
{
    if(isOn_upper)
    {
        isOn_upper = false;
        old_globalIntensity_upper = globalIntensity_upper;
        globalIntensity_upper = 0.0;
        updateUpper = true;
    }
    else
    {
        isOn_upper = true;
        globalIntensity_upper = old_globalIntensity_upper;
        updateUpper = true;
    }
}

void doubleClick_left()
{
    if(isOn_upper)
    {
        updateUpper = true;
        globalIntensity_upper = BlynkMathClamp(globalIntensity_upper + 0.1, 0.0, 1.0);
    }
    
}

bool long_left;

void longClick_left()
{
    if(isOn_upper)
    {
        long_left = true;
    }
    
}

void longEnd_left()
{
    long_left = false;
}



void doubleClick_right()
{
    if(lower_on)
    {
        updateLower = true;
        lower_global_intensity = BlynkMathClamp(lower_global_intensity + 10, 0, 100);
        
    }
    
}

bool long_right;

void longClick_right()
{
    if(lower_on)
    {
        long_right = true;
    }
}

void longEnd_right()
{
    long_right = false;
}

void click_right()
{
    if(lower_on)
    {
        lower_on = false;
        lower_global_intensity_old = lower_global_intensity;
        lower_global_intensity = 0;
        updateLower = true;
    }
    else
    {
        lower_on = true;
        lower_global_intensity = lower_global_intensity_old;
        updateLower = true;
    }
}

void update_lower_transition()
{
    float duration = RGBW_PixelCount/lower_division_factor;
    for (size_t i = 0; i < RGBW_PixelCount; i++)
    {
        uint8_t value = inOutSine8(i, duration);
        lower_transition_array[i] = constrain(value,0,255);
    }
    
}

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
    button_left.attachClick(click_left);
    button_left.attachDoubleClick(doubleClick_left);
    button_left.attachLongPressStart(longClick_left);
    button_left.attachLongPressStop(longEnd_left);
    button_right.attachClick(click_right);
    button_right.attachDoubleClick(doubleClick_right);
    button_right.attachLongPressStart(longClick_right);
    button_right.attachLongPressStop(longEnd_right);
    update_lower_transition();
}

unsigned long previousMillis_left = 0;
unsigned long previousMillis_right = 0;
unsigned long previousMillis_lower = 0;
bool lower_reverse = false;
const long interval = 500;  



void lower_strip_loop()
{
    if(lower_colorshift)
    {
        unsigned long currentMillis = millis();
        if(currentMillis - previousMillis_lower > 20)
        {
            int counter = constrain(map(currentMillis,previousMillis_lower,previousMillis_lower + lower_colorshift_duration,0,255),0 ,255);
            if(lower_reverse) counter = 255 - counter;
            lower_current_1 = lerpHSVW(lower_HSVW_1, lower_HSVW_2, counter);
            lower_current_2 = lerpHSVW(lower_HSVW_2, lower_HSVW_1, counter);
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
        lower_current_1 = lower_HSVW_1;
        lower_current_2 = lower_HSVW_2;
    }
    for (size_t i = 0; i < RGBW_PixelCount; i++)
      {
          HsvwColor hsvwColor = lerpHSVW(lower_current_1, lower_current_2, lower_transition_array[i]);
          hsvwColor.V = hsvwColor.V * lower_global_intensity/100;
          hsvwColor.W = hsvwColor.W * lower_global_intensity/100;
          RgbwColor color = HsvToRgbw(hsvwColor);
          strip_lower.SetPixelColor(i, color);
      }
      strip_lower.Show();
}

void upper_strip_loop()
{
    for (size_t i = 0; i < RGB_PixelCount; i++)
      {
          if(i > 100)
          {
              float currentCompletion = ((i-100)*2.0)/50.0;
              RgbColor newColor(
                BlynkMathClamp(easeInOutSine(upper_HSV_1[0], upper_HSV_2[0], currentCompletion) + (upper_HSV_3[0] * intensity3),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(upper_HSV_1[1], upper_HSV_2[1], currentCompletion) + (upper_HSV_3[1] * intensity3),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(upper_HSV_1[2], upper_HSV_2[2], currentCompletion) + (upper_HSV_3[2] * intensity3),0,255) * globalIntensity_upper
              );
              strip_upper.SetPixelColor(i, newColor);
          }
          else if (i > 50)
          {
              float currentCompletion = ((i-50)*2.0)/50.0;
              RgbColor newColor(
                BlynkMathClamp(easeInOutSine(upper_HSV_1[0], upper_HSV_2[0], currentCompletion) + (upper_HSV_3[0] * intensity2),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(upper_HSV_1[1], upper_HSV_2[1], currentCompletion) + (upper_HSV_3[1] * intensity2),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(upper_HSV_1[2], upper_HSV_2[2], currentCompletion) + (upper_HSV_3[2] * intensity2),0,255) * globalIntensity_upper
              );
              strip_upper.SetPixelColor(i, newColor);
          }
          else
          {
              float currentCompletion = ((i)*2.0)/50.0;
              RgbColor newColor(
                BlynkMathClamp(easeInOutSine(upper_HSV_1[0], upper_HSV_2[0], currentCompletion) + (upper_HSV_3[0] * intensity1),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(upper_HSV_1[1], upper_HSV_2[1], currentCompletion) + (upper_HSV_3[1] * intensity1),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(upper_HSV_1[2], upper_HSV_2[2], currentCompletion) + (upper_HSV_3[2] * intensity1),0,255) * globalIntensity_upper
              );
              strip_upper.SetPixelColor(i, newColor);
          }
      }
      strip_upper.Show();
}

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

  Blynk.run();

  if(updateLower || lower_colorshift)
  {
      updateLower = false;
      lower_strip_loop();
  }

  if(updateUpper)
  {
      updateUpper = false;
      upper_strip_loop();
  }
  
  button_left.tick();
  button_right.tick();

  if(long_left)
  {
      unsigned long currentMillis_left = millis();

      if(currentMillis_left - previousMillis_left >= interval)
      {
          previousMillis_left = currentMillis_left;

          updateUpper = true;
          globalIntensity_upper = BlynkMathClamp(globalIntensity_upper - 0.1, 0.0, 1.0);
      }
  }

  if(long_right)
  {
      unsigned long currentMillis_right = millis();

      if(currentMillis_right - previousMillis_right >= interval)
      {
          previousMillis_right = currentMillis_right;

          updateLower = true;
          lower_global_intensity = BlynkMathClamp(lower_global_intensity - 10, 0, 100);
      }
  }

}

// BLYNK INPUTS

BLYNK_WRITE(V0) // Toggle On/Off - Lower
{
    if(param.asInt() == 1)
    {
        click_right();
    }
}

BLYNK_WRITE(V1) // Save Settings - Lower
{
    if(param.asInt() == 1)
    {

    }
}

BLYNK_WRITE(V2) // Global Multiplier - Lower
{
    if(!lower_on) click_right();
    lower_global_intensity = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V3) // Color 1 Hue - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_1.H = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V4) // Color 1 Saturation - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_1.S = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V5) // Color 1 Value - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_1.V = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V6) // Color 1 Whiteness - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_1.W = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V7) // Color 2 Hue - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_2.H = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V8) // Color 2 Saturation - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_2.S = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V9) // Color 2 Value - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_2.V = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V10) // Color 2 Whiteness - Lower
{
    if(!lower_on) click_right();
    lower_HSVW_2.W = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V11) // ColorShift Toggle - Lower
{
    if(!lower_on) click_right();
    if(param.asInt() == 1)
    {
        lower_colorshift = !lower_colorshift;
    }
    updateLower = true;
}

BLYNK_WRITE(V12) // ColorShift Duration - Lower
{
    if(!lower_on) click_right();
    lower_colorshift_duration = param.asInt() * 1000;
    updateLower = true;
}

BLYNK_WRITE(V13) // Division Factor - Lower
{
    if(!lower_on) click_right();
    lower_division_factor = pow(2,param.asInt());
    update_lower_transition();
    updateLower = true;
}

// END OF INPUTS

