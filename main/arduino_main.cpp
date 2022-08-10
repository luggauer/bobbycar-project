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
#include "inputreader.h"
#include "console_task.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include <PID_v1.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  //Hier wird das Display benannt (Adresse/Zeichen pro Zeile/Anzahl Zeilen). In
// unserem Fall „lcd“. Die Adresse des I²C Displays kann je nach Modul variieren.
SoftwareSerial HoverSerial_front(RX0, TX0);  // RX, TX
SoftwareSerial HoverSerial_rear(RX1, TX1);   // RX, TX
// BluetoothSerial ESP_BT; //Object for Bluetooth
// PID steering_calculator;
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...

float throttle_factor;

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


SerialFeedback NewFeedback_front;
SerialFeedback NewFeedback_rear;

SerialFeedback SerialFeedback_front;
SerialFeedback SerialFeedback_rear;

// Arduino setup function. Runs in CPU 1
void setup() {
    const int task_count = 4;
    printf("Hoverboard Serial v1.0");
    TaskHandle_t tasks[task_count] = {NULL, NULL, NULL, NULL};
    xTaskCreate(&init_gamepad, "init_gamepad", 2048 * 3, NULL, 5, &tasks[0]);
    xTaskCreate(&init_adc_task, "init_adc_task", 2048 * 2, NULL, 5, &tasks[1]);
    xTaskCreate(&init_sbus, "init_servo", 2048 * 2, NULL, 5, &tasks[2]);
    xTaskCreate(&usb_console_init, "usb_console_init", 2048 * 2, NULL, 5, &tasks[3]);
    for(int y = task_count; y > 0;){
        y = task_count;
        for(int x = 0; x < task_count; x++)
            if(eTaskGetState(tasks[x]))
                //printf("Hoverboard Serial v1.0");
                y--;
        vTaskDelay(10);
        printf("next %i\n",y);
    }
    // ESP_BT.begin("ESP32_BobbyCon"); //Name of your Bluetooth Signal
    // lcd.init(); //Im Setup wird der LCD gestartet
    // lcd.backlight(); //Hintergrundbeleuchtung einschalten (0 schaltet die Beleuchtung aus).
    xTaskCreate(&adc_task, "adc", 2048 * 2, NULL, 5, NULL);
    xTaskCreate(&tast_usb_console, "tast_usb_console", 2048 * 2, NULL, 5, NULL);
      SerialFeedback_front.speedL_meas = SerialFeedback_front.speedR_meas = SerialFeedback_rear.speedL_meas = SerialFeedback_rear.speedR_meas = 0;

    HoverSerial_front.begin(HOVER_SERIAL_BAUD);

    HoverSerial_rear.begin(HOVER_SERIAL_BAUD);
    pinMode(LED_BUILTIN, OUTPUT);
    // init_debug_screen();

    init_buffer();
    display_init();
    // Setup the Bluepad32 callbacks
    // addr:9C:AA:1B:E6:7D:13
}

inline float rad2deg(float rad){
    return rad * 45.0 / M_PI_4;
}

void display_state(int throttle, float steering, float steering_desired, int *torgue, int torgue_regulated, int speed, int input_src){
    char line_buffer[512];
    snprintf(line_buffer, 512, "T%i S%.1f SD%.1f", throttle,rad2deg(steering),rad2deg(steering_desired));
    lcd.setCursor(0, 0);              // Start at top-left corner
    lcd.printf(line_buffer);

    snprintf(line_buffer, 512, "%i%c%i %i%c%i", torgue[0], torgue_regulated<0 ? '+' : '-' , ABS(torgue_regulated), torgue[1], torgue_regulated>0 ? '+' : '-' , ABS(torgue_regulated));
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
byte incomingByte_front;
byte incomingBytePrev_front;
byte incomingByte_rear;
byte incomingBytePrev_rear;


bool Receive(SoftwareSerial* board, SerialFeedback* out, byte *incomingByte,byte *incomingBytePrev,SerialFeedback *NewFeedback) {
    uint16_t bufStartFrame;  // Buffer Start Frame
    // byte buffer[sizeof(SerialFeedback)];
    //  Check for new data availability in the Serial buffer
    bool data_complete = false;
    while(board->available()){
        *incomingByte = board->read();                                        // Read the incoming byte
        bufStartFrame = ((uint16_t)(*incomingByte) << 8) | *incomingBytePrev;  // Construct the start frame

            // Copy received data
        if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
            p = (byte*)NewFeedback;
            *p++ = *incomingBytePrev;
            *p++ = *incomingByte;
            idx = 2;
        } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
            *p++ = *incomingByte;
            idx++;
        }
        // Update previous states
        *incomingBytePrev = *incomingByte;
        // Check if we reached the end of the package
        if (idx == sizeof(SerialFeedback)) {
            uint16_t checksum;
            checksum =
                (uint16_t)(NewFeedback->start ^ NewFeedback->cmd1 ^ NewFeedback->cmd2 ^ NewFeedback->speedR_meas ^
                        NewFeedback->speedL_meas ^ NewFeedback->batVoltage ^ NewFeedback->boardTemp ^ NewFeedback->cmdLed);
            idx = 0;  // Reset the index_buff_vals (it prevents to enter in this if condition in the next cycle)

            // Check validity of the new data
            if (NewFeedback->start == START_FRAME && checksum == NewFeedback->checksum) {
                // Copy the new data
                memcpy(out, NewFeedback, sizeof(SerialFeedback));
                out->speedR_meas = -out->speedR_meas;
                // Print data to built-in Serial
                printf("1: %i 2: %i 3: %i 4: %i 5: %i 6: %i 7: %i\n",out->cmd1,out->cmd2,out->speedR_meas,out->speedL_meas,out->batVoltage,out->boardTemp,out->cmdLed);
                data_complete = true;
            } else {
                printf("Non-valid data skipped\n");
            }
        }
    }
    return data_complete;

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
    Serial.println(incomingByte, HEX);
#endif
}

bool controller = false;
int torgue[4];
int speed_per_wheel[4];
int speed = 0;
int send_cnt = 0;
// Arduino loop function. Runs in CPU 1
char sprint_buffer[256];
int16_t pad_steering;
uint16_t pad_throttle, pad_brake;
long last_time;
int last_throttle;
float last_steering;

bool isNear(float a,float b, float range){
    if(ABS(a-b)<range)
        return true;
    else
        return false;
}

void loop() {
    int voltage =0;
    int torgue_regulated=0;
    unsigned long timeNow = millis();
    //int throttle = throttle_calc(clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN),1)));
    //float steering = calc_steering_eagle(clean_adc_steering(value_buffer(analogRead(STEERING_PIN),0)));
    int throttle = get_throttle();
    float steering =  get_steering();
    if(!isNear(last_throttle,throttle,100) || !isNear(last_steering,steering,0.5)){
        printf("update time %li %li\n",last_time, timeNow);
        last_time = timeNow;
        last_throttle = throttle;
        last_steering = steering;
    }
    // Check for new received data
    if(Receive(&HoverSerial_front, &SerialFeedback_front, &incomingByte_front, &incomingBytePrev_front, &NewFeedback_front)
        || Receive(&HoverSerial_rear, &SerialFeedback_rear, &incomingByte_rear, &incomingBytePrev_rear, &NewFeedback_rear)){
        speed = 0;
      speed += speed_per_wheel[0] = SerialFeedback_front.speedL_meas;
      speed += speed_per_wheel[1] = SerialFeedback_front.speedR_meas;
      speed += speed_per_wheel[2] = SerialFeedback_rear.speedL_meas;
      speed += speed_per_wheel[3] = SerialFeedback_rear.speedR_meas;
      voltage = SerialFeedback_front.batVoltage + SerialFeedback_rear.batVoltage / 2;
      speed /= 4;
      last_time = timeNow;
    }  // Send commands
    //steering_calculator.Compute();
    torgue_regulated = throttle_factor * THROTTLE_MAX;
    if (iTimeSend > timeNow)
        return;
    iTimeSend = timeNow + TIME_SEND;
    if (!controller)
        calc_torque_per_wheel(throttle, steering,torgue_regulated, torgue);
    else
        calc_torque_per_wheel(pad_throttle - pad_brake, pad_steering * 2,torgue_regulated, torgue);
    Send(&HoverSerial_front, torgue[0], torgue[1]);
    Send(&HoverSerial_rear, torgue[2], torgue[3]);
    if (!((send_cnt++) % 7)) {
        if( last_time + 20000 < timeNow){
            display.clearDisplay();
            display.display();
        }else{
            sprintf(sprint_buffer, "Throttle: %i\nSteering: %f\n%i %c %i  \t  %i %c %i\n%i      \t      %i\n%i: S%i B%i T%i\nV:%i    SPEED: %i", throttle,
                    steering * 45.0 / M_PI_4, torgue[0], torgue_regulated<0 ? '+' : '-' , ABS(torgue_regulated), torgue[1], torgue_regulated>0 ? '+' : '-' , ABS(torgue_regulated), torgue[2], torgue[3], controller, pad_steering, pad_brake,
                    pad_throttle,voltage, speed);
            display.clearDisplay();
            draw_line(sprint_buffer, 0);
            display_state(throttle, steering, pad_steering, torgue, torgue_regulated, speed, controller);
        }
    }
    // Blink the LED
    digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
    vTaskDelay(10);
}
