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
float old_globalIntensity_lower;
bool isOn_upper = true;
bool isOn_lower = true;

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

int lower_middle_HSVW[4];
int lower_sides_HSVW[4];
int middle_RGB[3];
int sides_RGB[3];
int upper_RGB[3];
float intensity1;
float intensity2;
float intensity3;
float globalIntensity_upper = 1.0;
bool updateUpper = false;

int middle_RGBW[4];
int sides_RGBW[4];
int globalIntensity_lower = 100;
bool updateLower = false;

BLYNK_WRITE(V0) // Middle_RGB_Lower
{
    isOn_lower = true;
    middle_RGBW[0] = param[0].asInt();
    middle_RGBW[1] = param[1].asInt();
    middle_RGBW[2] = param[2].asInt();
    updateLower = true;
}

BLYNK_WRITE(V1) // Middle_W_Lower
{
    isOn_lower = true;
    lower_middle_HSVW[3] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V2) // Middle_RGB_Lower
{
    isOn_lower = true;
    sides_RGBW[0] = param[0].asInt();
    sides_RGBW[1] = param[1].asInt();
    sides_RGBW[2] = param[2].asInt();
    updateLower = true;
}

BLYNK_WRITE(V3) // Middle_W_Lower
{
    isOn_lower = true;
    lower_sides_HSVW[3] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V20)
{
    isOn_lower = true;
    lower_middle_HSVW[0] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V21)
{
    isOn_lower = true;
    lower_middle_HSVW[1] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V22)
{
    isOn_lower = true;
    lower_middle_HSVW[2] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V23)
{
    isOn_lower = true;
    lower_sides_HSVW[0] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V24)
{
    isOn_lower = true;
    lower_sides_HSVW[1] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V25)
{
    isOn_lower = true;
    lower_sides_HSVW[2] = param.asInt();
    updateLower = true;
}

BLYNK_WRITE(V4) // Middle_RGB_Upper
{
    isOn_upper = true;
    middle_RGB[0] = param[0].asInt();
    middle_RGB[1] = param[1].asInt();
    middle_RGB[2] = param[2].asInt();
    updateUpper = true;
}

BLYNK_WRITE(V5) // Sides_RGB_Upper
{
    isOn_upper = true;
    sides_RGB[0] = param[0].asInt();
    sides_RGB[1] = param[1].asInt();
    sides_RGB[2] = param[2].asInt();
    updateUpper = true;
}

BLYNK_WRITE(V6) // Middle_RGB_Upper
{
    isOn_upper = true;
    upper_RGB[0] = param[0].asInt();
    upper_RGB[1] = param[1].asInt();
    upper_RGB[2] = param[2].asInt();
    updateUpper = true;
}

BLYNK_WRITE(V7) // Middle_RGB_Upper
{
    isOn_upper = true;
    intensity1 = param.asFloat();
    updateUpper = true;
}

BLYNK_WRITE(V8) // Middle_RGB_Upper
{
    isOn_upper = true;
    intensity2 = param.asFloat();
    updateUpper = true;
}

BLYNK_WRITE(V9) // Middle_RGB_Upper
{
    isOn_upper = true;
    intensity3 = param.asFloat();
    updateUpper = true;
}

BLYNK_WRITE(V10) // Middle_RGB_Upper
{
    isOn_upper = true;
    globalIntensity_upper = param.asFloat();
    old_globalIntensity_upper = globalIntensity_upper;
    updateUpper = true;
}

BLYNK_WRITE(V11) // Middle_RGB_Upper
{
    isOn_lower = true;
    globalIntensity_lower = param.asInt();
    old_globalIntensity_lower = globalIntensity_lower;
    updateLower = true;
}



int easeInOutSine(int start, int finish, float position)
{
    float newPos = position > 1.0 ? 2.0 - position : position;
    return floor((finish - start) * (-(cos(PI * newPos) - 1) / 2)) + start;
}


// from 0 to 100
// Based on https://www.alanzucconi.com/2016/01/06/colour-interpolation/
HsvwColor lerpHSVW(int *startColor, int *endColor, int position)
{
    // Hue interpolation
    int h = 0;
    int startHue = startColor[0];
    int endHue = endColor[0];
    int distance = endHue - startHue;
    if (startHue > endHue)
    {
        // Swap (startColor Hue, endColor Hue)
        endHue = startColor[0];
        startHue = endColor[0];

        distance = -distance;
        position = 100 - position;
    }

    if(distance > 127) // 180deg
    {
        startHue = startHue + 255; // 360deg
        h = (startHue + (position * (endHue - startHue))/100) % 255; // 360deg
    }
    if(distance <= 127) // 180deg
    {
        h = startHue + (position * distance)/100;
    }
    return HsvwColor(h,
                    startColor[1] + (position * (endColor[1] - startColor[1]))/100,
                    (startColor[2] + (position * (endColor[2] - startColor[2]))/100)*globalIntensity_lower/100,
                    (startColor[3] + (position * (endColor[3] - startColor[3]))/100)*globalIntensity_lower/100
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
    if(isOn_lower)
    {
        updateLower = true;
        globalIntensity_lower = BlynkMathClamp(globalIntensity_lower + 10, 0, 100);
        
    }
    
}

bool long_right;

void longClick_right()
{
    if(isOn_lower)
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
    if(isOn_lower)
    {
        isOn_lower = false;
        old_globalIntensity_lower = globalIntensity_lower;
        globalIntensity_lower = 0;
        updateLower = true;
    }
    else
    {
        isOn_lower = true;
        globalIntensity_lower = old_globalIntensity_lower;
        updateLower = true;
    }
}


BLYNK_WRITE(V12) // Middle_RGB_Upper
{
    if(param.asInt() == 1)
    {
        click_left();
    }
    
}

BLYNK_WRITE(V13) // Middle_RGB_Upper
{
    if(param.asInt() == 1)
    {
        click_right();
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
}

unsigned long previousMillis_left = 0;
unsigned long previousMillis_right = 0;
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

  Blynk.run();

  if(updateLower)
  {
      updateLower = false;

      for (size_t i = 0; i < RGBW_PixelCount; i++)
      {
          int currentCompletion = (i/(float)RGBW_PixelCount)*200;
          if (currentCompletion > 100) currentCompletion = 200 - currentCompletion;
          HsvwColor hsvwColor = lerpHSVW(lower_middle_HSVW, lower_sides_HSVW, currentCompletion);
          RgbwColor color = HsvToRgbw(hsvwColor);
          //RgbwColor convertedColor(color);
          /* RgbwColor newColor(
            easeInOutSine(middle_RGBW[0], sides_RGBW[0], currentCompletion) * globalIntensity_lower,
            easeInOutSine(middle_RGBW[1], sides_RGBW[1], currentCompletion) * globalIntensity_lower,
            easeInOutSine(middle_RGBW[2], sides_RGBW[2], currentCompletion) * globalIntensity_lower,
            easeInOutSine(middle_RGBW[3], sides_RGBW[3], currentCompletion) * globalIntensity_lower);
          strip_lower.SetPixelColor(i,newColor); */
          strip_lower.SetPixelColor(i, color);
      }
      strip_lower.Show();
  }

  if(updateUpper)
  {
      updateUpper = false;
      for (size_t i = 0; i < RGB_PixelCount; i++)
      {
          if(i > 100)
          {
              float currentCompletion = ((i-100)*2.0)/50.0;
              RgbColor newColor(
                BlynkMathClamp(easeInOutSine(middle_RGB[0], sides_RGB[0], currentCompletion) + (upper_RGB[0] * intensity3),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(middle_RGB[1], sides_RGB[1], currentCompletion) + (upper_RGB[1] * intensity3),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(middle_RGB[2], sides_RGB[2], currentCompletion) + (upper_RGB[2] * intensity3),0,255) * globalIntensity_upper
              );
              strip_upper.SetPixelColor(i, newColor);
          }
          else if (i > 50)
          {
              float currentCompletion = ((i-50)*2.0)/50.0;
              RgbColor newColor(
                BlynkMathClamp(easeInOutSine(middle_RGB[0], sides_RGB[0], currentCompletion) + (upper_RGB[0] * intensity2),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(middle_RGB[1], sides_RGB[1], currentCompletion) + (upper_RGB[1] * intensity2),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(middle_RGB[2], sides_RGB[2], currentCompletion) + (upper_RGB[2] * intensity2),0,255) * globalIntensity_upper
              );
              strip_upper.SetPixelColor(i, newColor);
          }
          else
          {
              float currentCompletion = ((i)*2.0)/50.0;
              RgbColor newColor(
                BlynkMathClamp(easeInOutSine(middle_RGB[0], sides_RGB[0], currentCompletion) + (upper_RGB[0] * intensity1),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(middle_RGB[1], sides_RGB[1], currentCompletion) + (upper_RGB[1] * intensity1),0,255) * globalIntensity_upper,
                BlynkMathClamp(easeInOutSine(middle_RGB[2], sides_RGB[2], currentCompletion) + (upper_RGB[2] * intensity1),0,255) * globalIntensity_upper
              );
              strip_upper.SetPixelColor(i, newColor);
          }
      }
      strip_upper.Show();
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
          globalIntensity_lower = BlynkMathClamp(globalIntensity_lower - 10, 0, 100);
      }
  }

}

