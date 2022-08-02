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
#include <Bluepad32.h>

#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <Arduino.h>
#include <SoftwareSerial.h>

#include <math_functions.h>
#include "config.h"
#include "defines.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  //Hier wird das Display benannt (Adresse/Zeichen pro Zeile/Anzahl Zeilen). In
// unserem Fall „lcd“. Die Adresse des I²C Displays kann je nach Modul variieren.
SoftwareSerial HoverSerial_front(RX0, TX0);  // RX, TX
SoftwareSerial HoverSerial_rear(RX1, TX1);   // RX, TX
// BluetoothSerial ESP_BT; //Object for Bluetooth

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...

bool dsp_connected;
typedef struct {
    uint16_t start;
    int16_t steer;
    int16_t speed_per_wheel;
    uint16_t checksum;
} SerialCommand;

typedef struct {
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

void scan_i2c() {
    byte error, address;
    int nDevices;
    printf("Scanning...\n");
    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            printf("I2C device found at address 0x%02hhX\n", address);
            nDevices++;
        } else if (error == 4) {
            printf("Unknow error at address 0x%02hhX\n", address);
        }
    }
    printf("%i I2C device%s found\n", nDevices, nDevices > 1 ? "s" : "");
}

void draw_line(const char* in, int y) {
    display.clearDisplay();

    display.setTextSize(1);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, y);              // Start at top-left corner
    display.cp437(true);                  // Use full 256 char 'Code Page 437' font

    // Not all the characters will fit on the display. This is normal.
    // Library will draw what it can and the rest will be clipped.
    for (int16_t i = 0; in[i]; i++)
        display.write(in[i]);

    display.display();
}

void draw_lcd(const char* in, int y) {

    lcd.setCursor(0, y);              // Start at top-left corner
    // Not all the characters will fit on the display. This is normal.
    // Library will draw what it can and the rest will be clipped.
    for (int16_t i = 0; in[i]; i++)
        lcd.write(in[i]);

    lcd.display();
}

void testscrolltext(void) {
    display.clearDisplay();

    display.setTextSize(2);  // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 0);
    display.println(F("scroll"));
    display.display();  // Show initial text
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

void display_init() {
    Wire.begin(I2C_SDA, I2C_SCL);
    scan_i2c();
    lcd.begin(20,4);
    lcd.clear();
    lcd.noBlink();
    lcd.noCursor();
    lcd.backlight();
    if (!(dsp_connected = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)))
        printf("SSD1306 allocation failed");
    else
        display.display();
}

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
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
//    TaskHandle_t tasks[task_count] = {NULL, NULL, NULL};
//    xTaskCreate(&init_ota, "init_ota", 2048 * 3, NULL, 5, &tasks[0]);
//    xTaskCreate(&init_user_default, "init_user_default", 2048 * 2, NULL, 5, &tasks[1]);
//    xTaskCreate(&init_servo, "init_servo", 2048 * 2, (void*)12, 5, &tasks[2]);
//    for(int y = task_count; y != 0; y = task_count)
//    for(int x = 0; x < task_count; x++)
//        if(tasks[x] == NULL)
//          y--;
    printf("Hoverboard Serial v1.0");
    // ESP_BT.begin("ESP32_BobbyCon"); //Name of your Bluetooth Signal
    pinMode(THROTTLE0_PIN, INPUT);
    pinMode(STEERING_PIN, INPUT);
    // lcd.init(); //Im Setup wird der LCD gestartet
    // lcd.backlight(); //Hintergrundbeleuchtung einschalten (0 schaltet die Beleuchtung aus).

    HoverSerial_front.begin(HOVER_SERIAL_BAUD);

    HoverSerial_rear.begin(HOVER_SERIAL_BAUD);
    pinMode(LED_BUILTIN, OUTPUT);
    // init_debug_screen();

    printf("Firmware: %s\n", BP32.firmwareVersion());
    init_buffer();
    display_init();
    // Setup the Bluepad32 callbacks
    // BP32.forgetBluetoothKeys();
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    // BP32.forgetBluetoothKeys();
    BP32.enableNewBluetoothConnections(true);
}

inline float rad2deg(float rad){
    return rad * 45.0 / M_PI_4;
}

void display_state(int throttle, float steering, float steering_desired, int *torgue, int* torgue_regulated, int speed, int input_src){
    char line_buffer[512];
    snprintf(line_buffer, 512, "T%i S%.1f SD%.1f", throttle,rad2deg(steering),rad2deg(steering_desired));
    lcd.setCursor(0, 0);              // Start at top-left corner
    lcd.printf(line_buffer);

    snprintf(line_buffer, 512, "%i%c%i %i%c%i", torgue[0], torgue_regulated[0]<0 ? '-' : '+' , ABS(torgue_regulated[0]), torgue[1], torgue_regulated[1]<0 ? '-' : '+' , ABS(torgue_regulated[1]));
    lcd.setCursor(0, 1);              // Start at top-left corner
    lcd.printf(line_buffer);

    snprintf(line_buffer, 512, "%i %i", torgue[2], torgue[3]);
    lcd.setCursor(0, 2);              // Start at top-left corner
    lcd.printf(line_buffer);

    snprintf(line_buffer, 512, "Input:%i  S:%i", input_src, speed);
    lcd.setCursor(0, 3);              // Start at top-left corner
    lcd.printf(line_buffer);
    lcd.display();
}

unsigned long iTimeSend = 0;

void Send(SoftwareSerial* board, int16_t speed0, int16_t speed1) {
    SerialCommand Command;
    // Create command
    Command.start = (uint16_t)START_FRAME;
    Command.steer = (int16_t)speed0;
    Command.speed_per_wheel = (int16_t)speed1;
    Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed_per_wheel);

    // Write to Serial
    board->write((uint8_t*)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
// Global variables
uint8_t idx = 0;  // index_buff_vals for new data pointer
byte* p;          // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
SerialFeedback NewFeedback;
bool Receive(SoftwareSerial* board, SerialFeedback* out) {
    uint16_t bufStartFrame;  // Buffer Start Frame
    // byte buffer[sizeof(SerialFeedback)];
    //  Check for new data availability in the Serial buffer
    if (board->available()) {
        incomingByte = board->read();                                        // Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
    } else {
        return false;
    }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
    Serial.println(incomingByte, HEX);
#endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
        p = (byte*)&NewFeedback;
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++ = incomingByte;
        idx++;
    }
    // Update previous states
    incomingBytePrev = incomingByte;
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum =
            (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^
                       NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
        idx = 0;  // Reset the index_buff_vals (it prevents to enter in this if condition in the next cycle)

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
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
        } else {
            Serial.println("Non-valid data skipped");
            return false;
        }
    } else {
        return false;
    }
}

SerialFeedback SerialFeedback_front;
SerialFeedback SerialFeedback_rear;
bool controller = false;
int torgue[4];
int speed_per_wheel[4];
int speed;
int send_cnt = 0;
// Arduino loop function. Runs in CPU 1
char sprint_buffer[256];
int16_t pad_steering;
uint16_t pad_throttle, pad_brake;
void loop() {
    int speed = 0;
    int a0;
    int torgue_regulated[2] = {0,0};
    unsigned long timeNow = millis();
    int throttle = throttle_calc(clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN), 0)));
    float steering = calc_steering_eagle(clean_adc_steering(a0 = value_buffer(analogRead(STEERING_PIN), 1)));
    // Check for new received data
    // if(Receive(&HoverSerial_front, &SerialFeedback_front) || Receive(&HoverSerial_rear, &SerialFeedback_rear)){
    //  speed_per_wheel[0] = SerialFeedback_front.speedL_meas;
    //  speed_per_wheel[1] = SerialFeedback_front.speedR_meas;
    //  speed_per_wheel[2] = SerialFeedback_rear.speedL_meas;
    //  speed_per_wheel[3] = SerialFeedback_rear.speedR_meas;
    //  speed = calc_median(speed_per_wheel,4);
    //}  // Send commands
    if (iTimeSend > timeNow)
        return;
    iTimeSend = timeNow + TIME_SEND;
    if (!controller)
        calc_torque_per_wheel(throttle, steering, torgue);
    else
        calc_torque_per_wheel(pad_throttle - pad_brake, pad_steering * 2, torgue);
    Send(&HoverSerial_front, torgue[0], torgue[1]);
    Send(&HoverSerial_rear, torgue[2], torgue[3]);
    if (!((send_cnt++) % 20)) {
        sprintf(sprint_buffer, "Throttle: %i\nSteering: %f\n%i  \t  %i\n%i  \t  %i\n%i: S%i B%i T%i", throttle,
                steering * 45 / M_PI_4, torgue[0], torgue[1], torgue[2], torgue[3], controller, pad_steering, pad_brake,
                pad_throttle);
        display.clearDisplay();
        draw_line(sprint_buffer, 0);
        display_state(throttle, steering, pad_steering, torgue, torgue_regulated, speed, controller);
    }
    // Blink the LED
    digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
    if ((send_cnt++) % 7)
        return;
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];
        if (myGamepad && myGamepad->isConnected()) {
            if (myGamepad->x())
                controller = !controller;
            pad_steering = myGamepad->axisX();
            pad_brake = myGamepad->brake();        // (0 - 1023): brake button
            pad_throttle = myGamepad->throttle();  // (0 - 1023): throttle (AKA gas) button
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
