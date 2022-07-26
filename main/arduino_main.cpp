/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Bluepad32.h>

#include <math_functions.h>
#include "defines.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//LiquidCrystal_I2C lcd(0x27, 20, 4);  //Hier wird das Display benannt (Adresse/Zeichen pro Zeile/Anzahl Zeilen). In unserem Fall „lcd“. Die Adresse des I²C Displays kann je nach Modul variieren.
SoftwareSerial HoverSerial_front(RX0, TX0); // RX, TX
SoftwareSerial HoverSerial_rear(RX1, TX1);  // RX, TX
// BluetoothSerial ESP_BT; //Object for Bluetooth


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...




bool dsp_connected;
typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed_per_wheel;
  uint16_t checksum;
} SerialCommand;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;




Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

void scan_i2c(){
  byte error, address;
  int nDevices;
  printf("Scanning...\n");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      printf("I2C device found at address 0x%02hhX",address);
      nDevices++;
    }
    else if (error==4) {
      printf("Unknow error at address 0x%02hhX",address);
    }    
  }
  if (nDevices == 0) {
    printf("No I2C devices found\n");
  }
  else {
    printf("%i I2C devices found\n",nDevices);
  }  
}

void draw_line(const char* in, int y) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, y);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; in[i]; i++)
    display.write(in[i]);
  
  display.display();
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

/*
uint8_t val_len[20][4];

void init_debug_screen()
{
  for (int x = 0; x < 20; x++)
    for (int y = 0; y < 4; y++)
      val_len[x][y] = 0;
  lcd.clear();
  //Display_show_string(0, 0, "Phase:");
  //Display_show_string(0, 1, "Pos:");
  //Display_show_string(0, 2, "blockcur:");
  //Display_show_string(0, 3, "Pwr:               V");
}

void update_num(uint8_t x, uint8_t y, int value)
{
  char buff[] = "                    ";
  int8_t tmp_len;// = Display_show_int(x, y, value);
  int8_t old_len = val_len[x][y];
  if (tmp_len < old_len)
  {
    buff[old_len - tmp_len] = '\0';
    //Display_show_string(x - tmp_len, y, buff);
  }
  for (int i = 0; x < MAX(tmp_len, old_len); i++)
    val_len[x - i][y] = MAX(tmp_len - i, 0);
}

void update_debug_screen()
{
  //update_num(19, 0, phase_period[0] + phase_period[1] / 2);
  //update_num(17, 1, last_pos[0]);
  //update_num(19, 1, last_pos[1]);
  //update_num(19, 2, blockcurlr[0] + blockcurlr[1]);
  //Display_show_float(18, 3, ADC122BATTERY_VOLTAGE(battery_voltage), 5);
}
*/

void display_init(){
    Wire.begin(I2C_SDA, I2C_SCL);
    scan_i2c();
    if(!(dsp_connected = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)))
        printf("SSD1306 allocation failed");
    else
        display.display();
}



//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        printf("CALLBACK: Gamepad connected, but could not found empty slot\n");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        printf("CALLBACK: Gamepad disconnected, but not found in myGamepads\n");
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  printf("Hoverboard Serial v1.0");
  // ESP_BT.begin("ESP32_BobbyCon"); //Name of your Bluetooth Signal
  pinMode(THROTTLE0_PIN,INPUT);
  pinMode(STEERING_PIN,INPUT);
  //lcd.init(); //Im Setup wird der LCD gestartet
  //lcd.backlight(); //Hintergrundbeleuchtung einschalten (0 schaltet die Beleuchtung aus).


  HoverSerial_front.begin(HOVER_SERIAL_BAUD);

  HoverSerial_rear.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  //init_debug_screen();


    printf("Firmware: %s\n", BP32.firmwareVersion());
    init_buffer();
    display_init();
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    //BP32.forgetBluetoothKeys();
    BP32.enableNewBluetoothConnections(true);
}


unsigned long iTimeSend = 0;

void Send(SoftwareSerial *board, int16_t speed0, int16_t speed1)
{
  SerialCommand Command;
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)speed0;
  Command.speed_per_wheel = (int16_t)speed1;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed_per_wheel);

  // Write to Serial
  board->write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
// Global variables
uint8_t idx = 0; // index_buff_vals for new data pointer
byte *p;         // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
SerialFeedback NewFeedback;
bool Receive(SoftwareSerial *board, SerialFeedback *out)
{

  uint16_t bufStartFrame; // Buffer Start Frame
  // byte buffer[sizeof(SerialFeedback)];
  //  Check for new data availability in the Serial buffer
  if (board->available())
  {
    incomingByte = board-> read();                            // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return false;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.println(incomingByte, HEX);
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }
  // Update previous states
  incomingBytePrev = incomingByte;
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
    idx = 0; // Reset the index_buff_vals (it prevents to enter in this if condition in the next cycle)

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&out, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(out->cmd1);
      Serial.print(" 2: ");
      Serial.print(out->cmd2);
      Serial.print(" 3: ");
      Serial.print(out->speedR_meas);
      Serial.print(" 4: ");
      Serial.print(out->speedL_meas);
      Serial.print(" 5: ");
      Serial.print(out->batVoltage);
      Serial.print(" 6: ");
      Serial.print(out->boardTemp);
      Serial.print(" 7: ");
      Serial.println(out->cmdLed);
      return true;
    }
    else
    {
      Serial.println("Non-valid data skipped");
      return false;
    }
  }
  else{
    return false;
  }
}

SerialFeedback SerialFeedback_front;
SerialFeedback SerialFeedback_rear;

int torgue[4];
int speed_per_wheel[4];
int speed;
int send_cnt = 0;
// Arduino loop function. Runs in CPU 1
char sprint_buffer[256];
void loop() {
  int a0;
  unsigned long timeNow = millis();
  int throttle = throttle_calc(clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN),0)));
  float steering = calc_steering_eagle(clean_adc_full(a0 = value_buffer(analogRead(STEERING_PIN),1)));
  // Check for new received data
  //if(Receive(&HoverSerial_front, &SerialFeedback_front) || Receive(&HoverSerial_rear, &SerialFeedback_rear)){
  //  speed_per_wheel[0] = SerialFeedback_front.speedL_meas;
  //  speed_per_wheel[1] = SerialFeedback_front.speedR_meas;
  //  speed_per_wheel[2] = SerialFeedback_rear.speedL_meas;
  //  speed_per_wheel[3] = SerialFeedback_rear.speedR_meas;
  //  speed = calc_median(speed_per_wheel,4);
  //}  // Send commands
  if (iTimeSend > timeNow)
    return;
  iTimeSend = timeNow + TIME_SEND;
  calc_torque_per_wheel(throttle, steering, torgue);
  Send(&HoverSerial_front, torgue[0], torgue[1]);
  Send(&HoverSerial_rear, torgue[2], torgue[3]);
  if (!((send_cnt++) % 20)){
    sprintf(sprint_buffer, "Throttle: %i\nSteering: %f\n%i  \t  %i\n%i  \t  %i\n",throttle,steering*45/M_PI_4,torgue[0],torgue[1],torgue[2],torgue[3]);
    display.clearDisplay();
    draw_line(sprint_buffer, 0);
  }
  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
  if ((send_cnt++) % 7)
    return;
  BP32.update();
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      GamepadPtr myGamepad = myGamepads[i];

      if (myGamepad && myGamepad->isConnected()) {
          // There are different ways to query whether a button is pressed.
          // By query each button individually:
          //  a(), b(), x(), y(), l1(), etc...
          if (myGamepad->a()) {
              static int colorIdx = 0;
              // Some gamepads like DS4 and DualSense support changing the color LED.
              // It is possible to change it by calling:
              switch (colorIdx % 3) {
                  case 0:
                      // Red
                      myGamepad->setColorLED(255, 0, 0);
                      break;
                  case 1:
                      // Green
                      myGamepad->setColorLED(0, 255, 0);
                      break;
                  case 2:
                      // Blue
                      myGamepad->setColorLED(0, 0, 255);
                      break;
              }
              colorIdx++;
          }

          if (myGamepad->b()) {
              // Turn on the 4 LED. Each bit represents one LED.
              static int led = 0;
              led++;
              // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
              // support changing the "Player LEDs": those 4 LEDs that usually indicate
              // the "gamepad seat".
              // It is possible to change them by calling:
              myGamepad->setPlayerLEDs(led & 0x0f);
          }

          if (myGamepad->x()) {
              // Duration: 255 is ~2 seconds
              // force: intensity
              // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
              // rumble.
              // It is possible to set it by calling:
              myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
          }

          // Another way to query the buttons, is by calling buttons(), or
          // miscButtons() which return a bitmask.
          // Some gamepads also have DPAD, axis and more.
          printf(
              "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
              "%4d, brake: %4d, throttle: %4d, misc: 0x%02x\n",
              i,                        // Gamepad Index
              myGamepad->dpad(),        // DPAD
              myGamepad->buttons(),     // bitmask of pressed buttons
              myGamepad->axisX(),       // (-511 - 512) left X Axis
              myGamepad->axisY(),       // (-511 - 512) left Y axis
              myGamepad->axisRX(),      // (-511 - 512) right X axis
              myGamepad->axisRY(),      // (-511 - 512) right Y axis
              myGamepad->brake(),       // (0 - 1023): brake button
              myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
              myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
          );

          // You can query the axis and other properties as well. See Gamepad.h
          // For all the available functions.
      }
  }
  

  delay(150);
}
