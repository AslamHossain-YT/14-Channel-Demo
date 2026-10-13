#include "U8glib.h"
#include <EEPROM.h>
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);
#define right_motor_forward 10   //L298 -- IN1 || TB6612 -- AIN2
#define right_motor_backward 11  //L298 -- IN2 || TB6612 -- AIN1
#define left_motor_forward 3     //L298 -- IN3  || TB6612 --- BIN1
#define left_motor_backward 9    //L298 -- IN4  || TB6612 --- BIN2
#define right_motor_speed 6  //EnA (L298) //PWMA(TB6612)
#define left_motor_speed 5   //EnB (L298) //PWMB(TB6612)
#define BUTTON_UP_PIN A2      // pin for UP button
#define BUTTON_SELECT_PIN A0  // pin for SELECT button
#define BUTTON_DOWN_PIN A1    // pin for DOWN button
#define led 13
#define buzzer A3
#define S0 4
#define S1 7
#define S2 8
#define S3 12
#define SIG_PIN A7
//Sensor Variables
#define sensorNumber 14
uint16_t sensorADC[sensorNumber];
uint8_t s[sensorNumber];
uint16_t bitWeight[sensorNumber] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 };
const uint8_t WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140 };
uint8_t sumOnSensor;
uint16_t sensorWight;
uint16_t bitSensor;
uint16_t Max_ADC[sensorNumber];
uint16_t Min_ADC[sensorNumber];
uint16_t Reference_ADC[sensorNumber];
//PID Variables
float line_position;
unsigned long blackTimer = 0;
float error;
float center_position = 75;
float previous_error;
uint8_t base_speed;  //left/right scan speed
int16_t right_motor_correction;
int16_t left_motor_correction;
float kp;
float kd;
int16_t position = 0;
unsigned char rotate_ratio;
String lineColor="black";
//turns variable
uint8_t turnSpeed = 150;
unsigned char direction = 0;
uint16_t U_turn_time;  // Uturn adjust
//inverse parameter
bool inverseON = 0;  //0 = normal line, 1 = inverse line
//BT
int motorSpeed;
//OLED Related Parameters
const unsigned char upir_logo[] PROGMEM = {  // this is another way how to define images, using binary notation
  B00010101, B11010111,
  B00010101, B01000101,
  B00010101, B10010110,
  B00011001, B00010101
};
// 'scrollbar_background', 8x64px
const unsigned char bitmap_scrollbar_background[] PROGMEM = {
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00
};
// 'item_sel_outline', 128x21px
const unsigned char bitmap_item_sel_outline[] PROGMEM = {
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0,
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0
};
// ----------- end generated bitmaps from image2cpp -------------
const int NUM_ITEMS = 10;         // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20;  // maximum characters for the item name
char menu_items[NUM_ITEMS][MAX_ITEM_LENGTH] = {
  { "Start Follow" },
  { "Calibration" },
  { "Analog Value" },
  { "Speed" },
  { "KP" },
  { "KD" },
  {"U Turn"},
  {"Reset Setting"},
  {"Bluetooth"},
  {"Motor Config."},
};

// note - when changing the order of items above, make sure the other arrays referencing bitmaps
// also have the same order, for example array "bitmap_icons" for icons, and other arrays for screenshots and QR codes
bool button_up_clicked = 0;      // only perform action when button is clicked, and wait until another press
bool button_select_clicked = 0;  // same as above
bool button_down_clicked = 0;    // same as above
int8_t item_selected = 0;        // which item in the menu is selected
int8_t item_sel_previous;        // previous item - used in the menu screen to draw the item before the selected one
int8_t item_sel_next;            // next item - used in the menu screen to draw next item after the selected one
int8_t current_screen = 0;       // 0 = menu, 1 = screenshot, 2 = qr
//************************************SETUP***************************************
void setup() {
  Serial.begin(9600);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);      // up button
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);  // select button
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);    // down button
  pinMode(led, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  LoadCalibration();  //load calibration value from eeprom whenever the arduino is start or reset
  loadEEPROM();
}
//************************************LOOP***************************************
void loop() {
  while (1) {
    read_black_line();
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_profont12);
      u8g.drawStr(10, 15, "");  //Set Your Team Name
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 60);
      u8g.print(s[0]);
      u8g.setPrintPos(15, 60);
      u8g.print(s[1]);
      u8g.setPrintPos(25, 60);
      u8g.print(s[2]);
      u8g.setPrintPos(20, 40);
      u8g.print(s[3]);
      u8g.setPrintPos(30, 40);
      u8g.print(s[4]);
      u8g.setPrintPos(40, 40);
      u8g.print(s[5]);
      u8g.setPrintPos(50, 40);
      u8g.print(s[6]);
      u8g.setPrintPos(60, 40);
      u8g.print(s[7]);
      u8g.setPrintPos(70, 40);
      u8g.print(s[8]);
      u8g.setPrintPos(80, 40);
      u8g.print(s[9]);
      u8g.setPrintPos(90, 40);
      u8g.print(s[10]);
      u8g.setPrintPos(100, 60);
      u8g.print(s[11]);
      u8g.setPrintPos(110, 60);
      u8g.print(s[12]);
      u8g.setPrintPos(120, 60);
      u8g.print(s[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) menu();
  }
}
void BT_MODE() {
  motorSpeed=EEPROM.read(54);
  u8g.setFont(u8g_font_7x14B);
  u8g.firstPage();
  do {
    u8g.drawStr(30, 10, "Bluetooth");
    u8g.drawStr(35, 25, "Activated");
  } while (u8g.nextPage());
  delay(500);
  while (1) {
    if (digitalRead(BUTTON_DOWN_PIN) == LOW) {
      motor(0, 0);
      delay(200);
      return menu();
    }
    if (Serial.available()) {
      char command = Serial.read();
      // Convert lowercase to uppercase
      if (command >= 'a' && command <= 'z') {
        command = command - 32;
      }
      switch (command) {
        case 'F':
          digitalWrite(led, 1);
          motor(motorSpeed, motorSpeed);
          break;
        case 'B':
          digitalWrite(led, 1);
          motor(-motorSpeed, -motorSpeed);
          break;
        case 'L':
          digitalWrite(led, 1);
          motor(-motorSpeed, motorSpeed);
          break;
        case 'R':
          digitalWrite(led, 1);
          motor(motorSpeed, -motorSpeed);
          break;
        case 'S':
          digitalWrite(led, 0);
          motor(0, 0);
          break;
        case '0':
          motorSpeed = 80;
          EEPROM.write(54, motorSpeed);
          break;
        case '1':
          motorSpeed = 100;
          EEPROM.write(54, motorSpeed);
          break;
        case '2':
          motorSpeed = 120;
          EEPROM.write(54, motorSpeed);
          break;
        case '3':
          motorSpeed = 140;
          EEPROM.write(54, motorSpeed);
          break;
        case '4':
          motorSpeed = 160;
          EEPROM.write(54, motorSpeed);
          break;
        case '5':
          motorSpeed = 180;
          EEPROM.write(54, motorSpeed);
          break;
        case '6':
          motorSpeed = 200;
          EEPROM.write(54, motorSpeed);
          break;
        case '7':
          motorSpeed = 220;
          EEPROM.write(54, motorSpeed);
          break;
        case '8':
          motorSpeed = 240;
          EEPROM.write(54, motorSpeed);
          break;
        case '9':
          motorSpeed = 250;
          EEPROM.write(54, motorSpeed);
          break;
        case 'Q':
          motorSpeed = 255;
          EEPROM.write(54, motorSpeed);
          break;
      }
    }
  }
}
void menu() {
  while (1) {
    if (current_screen == 0) {
      if ((digitalRead(BUTTON_UP_PIN) == LOW) && (button_up_clicked == 0)) {  
        item_selected = item_selected - 1;button_up_clicked = 1;                                                
        if (item_selected < 0) {item_selected = NUM_ITEMS - 1;}}
        else if ((digitalRead(BUTTON_DOWN_PIN) == LOW) && (button_down_clicked == 0)) {  
        item_selected = item_selected + 1;                                 
        button_down_clicked = 1;if (item_selected >= NUM_ITEMS) {item_selected = 0;}
      }if ((digitalRead(BUTTON_UP_PIN) == HIGH) && (button_up_clicked == 1)) {button_up_clicked = 0;}
      if ((digitalRead(BUTTON_DOWN_PIN) == HIGH) && (button_down_clicked == 1)) {button_down_clicked = 0;}}
    if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) {button_select_clicked = 1;}
    if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) {button_select_clicked = 0;}
    item_sel_previous = item_selected - 1;
    if (item_sel_previous < 0) { item_sel_previous = NUM_ITEMS - 1; }
    item_sel_next = item_selected + 1;
    if (item_sel_next >= NUM_ITEMS) { item_sel_next = 0; }
    
    if (item_selected == 0 && button_select_clicked == 1) {current_screen = 0;linefollow();}
    if (item_selected == 1 && button_select_clicked == 1) {current_screen = 0;calibrateSensor();}
    if (item_selected == 2 && button_select_clicked == 1) {current_screen = 0;analog_value();}
    if (item_selected == 3 && button_select_clicked == 1) {current_screen = 0;setSpeed();}
    if (item_selected == 4 && button_select_clicked == 1) {current_screen = 0;setKP();}
    if (item_selected == 5 && button_select_clicked == 1) {current_screen = 0;setKD();}
    if (item_selected == 6 && button_select_clicked == 1) {current_screen = 0;set_U_turn();}
    if (item_selected == 7 && button_select_clicked == 1) {current_screen = 0;reset_Setting();}
    if (item_selected == 8 && button_select_clicked == 1) {current_screen = 0;BT_MODE();}
    if (item_selected == 9 && button_select_clicked == 1) {current_screen = 0;motor_test();}
    u8g.firstPage();
    do {if (current_screen == 0) {
        u8g.drawBitmapP(0, 22, 128 / 8, 21, bitmap_item_sel_outline);
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15, menu_items[item_sel_previous]);
        u8g.setFont(u8g_font_7x14B);
        u8g.drawStr(25, 15 + 20 + 2, menu_items[item_selected]);
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15 + 20 + 20 + 2 + 2, menu_items[item_sel_next]);
        u8g.drawBitmapP(128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);
        u8g.drawBox(125, 64 / NUM_ITEMS * item_selected, 3, 64 / NUM_ITEMS);}}
        while (u8g.nextPage());}}

void linefollow() {
  blackTimer = 0;
  u8g.setFont(u8g_font_7x14B);
  u8g.firstPage();
  do {
    u8g.drawStr(30, 35, "Line Follow");
    u8g.drawStr(40, 50, "Activated");
  } while (u8g.nextPage());
  delay(500);
  while (1) {
a:
    read_black_line();
    if (sumOnSensor > 0) {line_position = sensorWight / sumOnSensor;
      error = center_position - line_position;}
    if (s[0] == 0 && s[13] == 1) { direction = 1; }
    if (s[0] == 1 && s[13] == 0) { direction = 2; }
    if(s[0]||s[1]||s[2]){error=65;}
    if (sumOnSensor == 0) {
      uint32_t timer1 = millis();
      while (sumOnSensor == 0) {
        motor(turnSpeed, turnSpeed);read_black_line();
        if (millis() - timer1 > U_turn_time) {(direction == 1) ? motor(turnSpeed, -turnSpeed) : motor(-turnSpeed, turnSpeed);
          while (s[6] == 0 && s[7] == 0) { read_black_line(); }
          break;}}
    }else if (sumOnSensor == sensorNumber) {
      if (blackTimer == 0) { blackTimer = millis(); }
      if (millis() - blackTimer >= 25) {
        motor(0, 0);
        u8g.firstPage();
        do {u8g.drawStr(40, 30, "FINISH");
          u8g.drawStr(25, 50, "STOPPED");
        } while (u8g.nextPage());
        while (1) {if (digitalRead(BUTTON_DOWN_PIN) == LOW) {delay(200);return menu();}}}
    }else{blackTimer=0;}
    right_motor_correction = base_speed + ((error * kp) + ((error - previous_error) * kd));
    left_motor_correction = base_speed - ((error * kp) + ((error - previous_error) * kd));
    previous_error = error;
    motor(left_motor_correction, right_motor_correction);
    if (digitalRead(BUTTON_DOWN_PIN) == LOW) {motor(0, 0);delay(200);return menu();}}}

void calibrateSensor() {
  u8g.setFont(u8g_font_7x14B);  // Set the font
    u8g.firstPage();
    do {
      u8g.drawStr(10, 35, "Line Calibration");
      u8g.drawStr(30, 50, "Activated");
    } while (u8g.nextPage());
  // Initialize calibration limits
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;     // Set the maximum ADC value for each sensor to 0 (lowest possible)
    Min_ADC[i] = 1024;  // Set the minimum ADC value for each sensor to 1024 (highest possible, since ADC range is 0–1023)
  }
  // Define rotation directions for calibration sweeps
  int rotationDirections[][2] = {
    { -1, 1 },  // Rotate robot slightly to the left
    { 1, -1 },  // Rotate robot slightly to the right
    { 1, -1 },  // Repeat rotation to the right for better calibration coverage
    { -1, 1 }   // Rotate left again to cover the full sensor range
  };

  // Perform calibration sweeps (4 passes)
  for (int step = 0; step < 4; step++) {
    // Rotate the robot slowly in the defined direction for this step
    motor(base_speed * 0.5 * rotationDirections[step][0],
          base_speed * 0.5 * rotationDirections[step][1]);

    // Sweep through several readings while rotating
    for (int sweep = 0; sweep < 300; sweep++) {
      // Read each sensor one by one
      for (int i = 0; i < sensorNumber; i++) {
        selectChannel(i);                    // Select which sensor channel to read (if using multiplexer)
        sensorADC[i] = analogRead(SIG_PIN);  // Read the analog value from the sensor

        // Update the maximum and minimum ADC values recorded for this sensor
        Max_ADC[i] = max(Max_ADC[i], sensorADC[i]);
        Min_ADC[i] = min(Min_ADC[i], sensorADC[i]);
      }
    }
    if (step == 0 || step == 3) {
      motor(base_speed * 0.5, -(base_speed * 0.5));
      delay(30);
    } else {
      motor(-(base_speed * 0.5), base_speed * 0.5);
      delay(30);
    }
    motor(0, 0);  // Stop the motors after each rotation step
    delay(200);  // Small pause before changing rotation direction
  }

  // Store calibrated Reference, Max, and Min values in EEPROM
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.0;  // Calculate mid-point (reference) value

    EEPROM.write(i, Reference_ADC[i] / 4);           // Store reference value (divided by 4 because EEPROM stores 0–255)
    delay(10);                                       // Small delay for EEPROM write stability
    EEPROM.write(i + sensorNumber, Max_ADC[i] / 4);  // Store maximum calibrated value
    delay(10);
    EEPROM.write(i + (sensorNumber * 2), Min_ADC[i] / 4);  // Store minimum calibrated value
    delay(10);
  }
  show_calibration();
}

// Function to load calibration data from EEPROM
void LoadCalibration() {
  Serial.println("Reference Value for Digital Conversion: ");
  // Print header labels for serial monitor (sensor indices)
  Serial.print("S0: ");
  Serial.print("  S1: ");
  Serial.print("  S2: ");
  Serial.print("  S3: ");
  Serial.print("  S4: ");
  Serial.print("  S5: ");
  Serial.print("  S6: ");
  Serial.print("  S7: ");
  Serial.print("  S8: ");
  Serial.print("  S9: ");
  Serial.print("  S10: ");
  Serial.print(" S11: ");
  Serial.print(" S12: ");
  Serial.print(" S13: ");
  
  Serial.println();

  // Load the stored calibration data for each sensor
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 4;                 // Read and restore reference value
    Max_ADC[i] = EEPROM.read(i + sensorNumber) * 4;        // Read and restore maximum value
    Min_ADC[i] = EEPROM.read(i + (sensorNumber * 2)) * 4;  // Read and restore minimum value

    Serial.print(String(Reference_ADC[i]) + ",  ");  // Print loaded reference value
  }
  Serial.println();  // Move to new line after printing all sensor values
}


void show_calibration() {
  while (1) {
    // Set the font
    u8g.firstPage();
    do {
      //title print
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(15, 10, "LOW");
      u8g.drawStr(55, 10, "REF.");
      u8g.drawStr(90, 10, "HIGH");
      //value print
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(15, 30);
      u8g.print(Min_ADC[0]);
      u8g.setPrintPos(15, 45);
      u8g.print(Min_ADC[4]);
      u8g.setPrintPos(15, 60);
      u8g.print(Min_ADC[7]);
      //
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(55, 30);
      u8g.print(Reference_ADC[0]);
      u8g.setPrintPos(55, 45);
      u8g.print(Reference_ADC[3]);
      u8g.setPrintPos(55, 60);
      u8g.print(Reference_ADC[7]);
      //
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(95, 30);
      u8g.print(Max_ADC[0]);
      u8g.setPrintPos(95, 45);
      u8g.print(Max_ADC[7]);
      u8g.setPrintPos(95, 60);
      u8g.print(Max_ADC[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return menu();
  }
}

void loadEEPROM(){
  base_speed = EEPROM.read(50);
  kp = EEPROM.read(51);
  kd = EEPROM.read(52)*10;
  U_turn_time=EEPROM.read(53)*10;
  rotate_ratio=EEPROM.read(55);
  motorSpeed=EEPROM.read(54);
}
void reset_Setting(){
  u8g.setFont(u8g_font_profont12);  // Set the font
    u8g.firstPage();
    do {
      u8g.drawStr(5, 15, "Long Press UP Button");
      u8g.drawStr(5, 30, "to Reset");
      u8g.drawStr(5, 50, "Press DOWN Button");
      u8g.drawStr(5, 60, "to Cancel");
    } while (u8g.nextPage());

  while(1){
    if(button(BUTTON_UP_PIN)==2){
      EEPROM.write(50, 150);
      EEPROM.write(51, 5);
      EEPROM.write(52, 10);
      EEPROM.write(53, 14);
      EEPROM.write(55, 70); //Rotate Rate
      EEPROM.write(54, 127); //BT speed
      u8g.setFont(u8g_font_profont12);  // Set the font
    u8g.firstPage();
    do {
      u8g.drawStr(5, 15, "Reset Completed");
    } while (u8g.nextPage());
      delay(500);loadEEPROM();return menu();
      }
    if(button(BUTTON_DOWN_PIN)==1){delay(200);return menu();}
  }
}

uint8_t button(byte buttonNumber) {
  uint32_t delayCount = 0;
  if (!digitalRead(buttonNumber)) {    delay(50);
    while (!digitalRead(buttonNumber)) {
      delay(50);delayCount += 40;
      if (delayCount > 490) {digitalWrite(led, 1);digitalWrite(buzzer, 1);}}
    digitalWrite(led, 0);
    digitalWrite(buzzer, 0);
    if (delayCount > 490) {return 2;}else {return 1;}}return 0;}
void motor(int LPWM, int RPWM) {
  if (LPWM > 0) {digitalWrite(left_motor_forward, HIGH);digitalWrite(left_motor_backward, LOW);}
  else {digitalWrite(left_motor_forward, LOW);digitalWrite(left_motor_backward, HIGH);}
  if (RPWM > 0) {digitalWrite(right_motor_forward, HIGH);digitalWrite(right_motor_backward, LOW);}
  else {digitalWrite(right_motor_forward, LOW);digitalWrite(right_motor_backward, HIGH);}
  LPWM = constrain(LPWM, -255, 255);RPWM = constrain(RPWM, -255, 255);
  analogWrite(left_motor_speed, abs(LPWM));analogWrite(right_motor_speed, abs(RPWM));}

void motor_test() {
  while (1) {
    u8g.setFont(u8g_font_7x14B);  // Set the font
    u8g.firstPage();
    do {
      u8g.drawStr(5, 10, "Motor Testing...");
      u8g.drawStr(5, 25, "1. Both Forward");
      u8g.drawStr(5, 40, "2. LeftM Back");
    } while (u8g.nextPage());
    motor(150, 150);digitalWrite(led, HIGH);
    delay(2000);motor(0, 0);
    digitalWrite(led, LOW);delay(200);
    motor(-150, 0);digitalWrite(led, HIGH);
    delay(2000);motor(0, 0);
    return;}}

void brake(){motor(-255,-255);delay(10);motor(0,0);}
void read_black_line() {
  sumOnSensor = 0;sensorWight=0;bitSensor=0;
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    if (sensorADC[i] > Reference_ADC[i]) {
      if (inverseON == 0) {s[i] = 1;} else {s[i] = 0;}}
      else {if (inverseON == 0) {s[i] = 0;} else {s[i] = 1;}}
    sumOnSensor += s[i];
    sensorWight += s[i] * WeightValue[i];
    bitSensor += s[i] * bitWeight[(sensorNumber - 1) - i];}}

void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));digitalWrite(S3, bitRead(channel, 3));}

void analog_value() {
  while (1) {
    for (int channel = 0; channel < sensorNumber; channel++) {
      selectChannel(channel);
      sensorADC[channel] = analogRead(SIG_PIN);
    }
    u8g.setFont(u8g_font_profont12);  // Set the font
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 15);
      u8g.print(sensorADC[0]);
      u8g.setPrintPos(35, 15);
      u8g.print(sensorADC[1]);
      u8g.setPrintPos(65, 15);
      u8g.print(sensorADC[2]);
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(10, 30);
      u8g.print(sensorADC[3]);
      u8g.setPrintPos(40, 30);
      u8g.print(sensorADC[4]);
      u8g.setPrintPos(70, 30);
      u8g.print(sensorADC[5]);
      u8g.setPrintPos(100, 30);
      u8g.print(sensorADC[6]);
      u8g.setPrintPos(10, 45);
      u8g.print(sensorADC[7]);
      u8g.setPrintPos(40, 45);
      u8g.print(sensorADC[8]);
      u8g.setPrintPos(70, 45);
      u8g.print(sensorADC[9]);
      u8g.setPrintPos(100, 45);
      u8g.print(sensorADC[10]);
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(40, 60);
      u8g.print(sensorADC[11]);
      u8g.setPrintPos(70, 60);
      u8g.print(sensorADC[12]);
      u8g.setPrintPos(100, 60);
      u8g.print(sensorADC[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return;
  }
}
void digital_value() {
  while (1) {
    read_black_line();
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_profont12);
      u8g.drawStr(30, 15, "");  //Set Your Team Name
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 60);
      u8g.print(s[0]);
      u8g.setPrintPos(15, 60);
      u8g.print(s[1]);
      u8g.setPrintPos(25, 60);
      u8g.print(s[2]);
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(20, 40);
      u8g.print(s[3]);
      u8g.setPrintPos(30, 40);
      u8g.print(s[4]);
      u8g.setPrintPos(40, 40);
      u8g.print(s[5]);
      u8g.setPrintPos(50, 40);
      u8g.print(s[6]);
      u8g.setPrintPos(60, 40);
      u8g.print(s[7]);
      u8g.setPrintPos(70, 40);
      u8g.print(s[8]);
      u8g.setPrintPos(80, 40);
      u8g.print(s[9]);
      u8g.setPrintPos(90, 40);
      u8g.print(s[10]);
      u8g.setPrintPos(100, 60);
      u8g.print(s[11]);
      u8g.setPrintPos(110, 60);
      u8g.print(s[12]);
      u8g.setPrintPos(120, 60);
      u8g.print(s[13]);
    } while (u8g.nextPage());
    while (button(BUTTON_DOWN_PIN) == 1) return;}}
void setSpeed() {
  while (1) {
    if (button(BUTTON_UP_PIN) == 1) base_speed = base_speed + 5;
    if (button(BUTTON_DOWN_PIN) == 1) base_speed = base_speed - 5;
    if (base_speed > 250) base_speed = 250;
    if (base_speed < 5) base_speed = 5;
    // Set the font
    u8g.firstPage();
    do {
      //title print
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(20, 30, "Speed = ");
      //value print
      u8g.setPrintPos(80, 30);
      u8g.print(base_speed);
    } while (u8g.nextPage());
    while (button(BUTTON_SELECT_PIN) == 2) {
      EEPROM.write(50, base_speed);
      return menu();
    }
  }
}

void setKP() {
  while (1) {
    if (button(BUTTON_UP_PIN) == 1) kp = kp + 1;
    if (button(BUTTON_DOWN_PIN) == 1) kp = kp - 1;
    if (kp > 50) kp = 0;
    if (kp < 0) kp = 0;
    // Set the font
    u8g.firstPage();
    do {
      //title print
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(20, 30, "KP = ");
      //value print
      u8g.setPrintPos(80, 30);
      u8g.print(kp);
    } while (u8g.nextPage());
    while (button(BUTTON_SELECT_PIN) == 2) {
      EEPROM.write(51, kp);
      return menu();
    }
  }
}

void setKD() {
  while (1) {
    if (button(BUTTON_UP_PIN) == 1) kd = kd + 20;
    if (button(BUTTON_DOWN_PIN) == 1) kd = kd - 20;
    if (kd > 2500) kd = 0;
    if (kd < 0) kd = 0;
    // Set the font
    u8g.firstPage();
    do {
      //title print
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(20, 30, "KD = ");
      //value print
      u8g.setPrintPos(80, 30);
      u8g.print(kd);
    } while (u8g.nextPage());
    while (button(BUTTON_SELECT_PIN) == 2) {
      EEPROM.write(52, (kd / 10));
      return menu();
    }
  }
}

void set_U_turn() {
  while (1) {
    if (button(BUTTON_UP_PIN) == 1) U_turn_time = U_turn_time + 20;
    if (button(BUTTON_DOWN_PIN) == 1) U_turn_time = U_turn_time - 20;
    if (U_turn_time > 2500) U_turn_time = 0;
    if (U_turn_time < 0) U_turn_time = 0;
    // Set the font
    u8g.firstPage();
    do {
      //title print
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(20, 30, "U Turn = ");
      //value print
      u8g.setPrintPos(80, 30);
      u8g.print(U_turn_time);
    } while (u8g.nextPage());
    while (button(BUTTON_SELECT_PIN) == 2) {
      EEPROM.write(53, (U_turn_time / 10));
      return menu();
    }
  }
}