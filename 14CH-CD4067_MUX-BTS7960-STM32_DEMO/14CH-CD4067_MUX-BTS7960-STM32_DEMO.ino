#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//
#define button1 PB13
#define button2 PB12
#define button3 PB5
#define button4 PB3
//motorpins
#define SERIAL_PORT Serial1       // FTDI uses USART1
#define pwmL1 PB9
#define pwmL2 PB8
#define pwmR1 PA0
#define pwmR2 PA1
//
#define led PA4
#define buzzer PB5
#define S0 PA7
#define S1 PA6
#define S2 PA4
#define S3 PA3
#define SIG_PIN PA2
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
int16_t right_motor_correction;
int16_t left_motor_correction;
uint8_t base_speed=120; 
float kp=5; 
float kd=100;
int16_t position = 0;
//turns variable
uint8_t turnSpeed = 150;
unsigned char direction = 0;
uint16_t U_turn_time=120;  // Uturn adjust
//inverse parameter
bool inverseON = 0;  //0 = normal line, 1 = inverse line
//
const int menuCount = 4;
int selectedItem = 0;
enum ScreenState {
  MAIN_MENU,
  START_SCREEN,
  CALIBRATION_SCREEN,
  SENSOR_TEST_SCREEN,
  MOTOR_CONFIG_SCREEN
};
ScreenState currentScreen = MAIN_MENU;
unsigned long lastButtonTime = 0;
const unsigned long debounceDelay = 180;
const char* menuItems[] = {
  "Start",
  "Calibration",
  "Sensor Test",
  "Motor Configure"
};
//************************************SETUP***************************************
void setup() {
  SERIAL_PORT.begin(115200);
  delay(100);  // give FTDI time
  // Disable JTAG so PB12–PB15 work correctly
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  SERIAL_PORT.println("STM32 + FTDI MUX ADC READY");
  // STM32F103 hardware I2C
  // Buttons
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    while (1)
      ;
  }
  display.cp437(true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("DEMO CODE");
  //PWM Setup
  analogReadResolution(12);     // 0–4095
  analogWriteFrequency(10000);  // 10k Hz
  analogWriteResolution(8);     // 8-bit PWM (0–255)
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(pwmL1, OUTPUT);
  pinMode(pwmL2, OUTPUT);
  pinMode(pwmR1, OUTPUT);
  pinMode(pwmR2, OUTPUT);
  analogWrite(pwmL1, 0);
  analogWrite(pwmL2, 0);
  analogWrite(pwmR1, 0);
  analogWrite(pwmR2, 0);
  pinMode(led, OUTPUT);
  pinMode(buzzer, OUTPUT);
  motor(0, 0);
  LoadCalibration();
  drawMainMenu();
}
//************************************LOOP***************************************
void loop() {
  handleButtons();
}
void linefollow() {
  blackTimer = 0;
  delay(500);
  while (1) {
a:
    read_black_line();
    if (sumOnSensor > 0) {line_position = sensorWight / sumOnSensor;
      error = center_position - line_position;}
    if (s[0] == 0 && s[13] == 1) { direction = 1; }
    if (s[0] == 1 && s[13] == 0) { direction = 2; }
    if (sumOnSensor == 0) {
      uint32_t timer1 = millis();
      while (sumOnSensor == 0) {
        motor(base_speed, base_speed);read_black_line();
        if (millis() - timer1 > U_turn_time) {(direction == 1) ? motor(turnSpeed, -turnSpeed) : motor(-turnSpeed, turnSpeed);
          while (s[6] == 0 && s[7] == 0) { read_black_line(); }
          break;}}
    }else if (sumOnSensor == sensorNumber) {
      if (blackTimer == 0) { blackTimer = millis(); }
      if (millis() - blackTimer >= 25) {
        motor(0, 0);
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(50, 50);
        display.println("STOPPED");
        while (1) {if (digitalRead(button3) == LOW || digitalRead(button2) == LOW || digitalRead(button1) == LOW) {delay(200);}}}
    }else{blackTimer=0;}
    right_motor_correction = base_speed + ((error * kp) + ((error - previous_error) * kd));
    left_motor_correction = base_speed - ((error * kp) + ((error - previous_error) * kd));
    previous_error = error;
    motor(left_motor_correction, right_motor_correction);
    if (digitalRead(button3) == LOW || digitalRead(button2) == LOW || digitalRead(button1) == LOW) {motor(0, 0);delay(200);}}}


void calibrateSensor() {
  // Initialize calibration limits
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;     // Set the maximum ADC value for each sensor to 0 (lowest possible)
    Min_ADC[i] = 4096;  // Set the minimum ADC value for each sensor to 1024 (highest possible, since ADC range is 0–1023)
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
    for (int sweep = 0; sweep < 600; sweep++) {
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
      motor(80, -80);
      delay(20);
    } else {
      motor(-80, 80);
      delay(20);
    }
    motor(0, 0);  // Stop the motors after each rotation step
    delay(200);  // Small pause before changing rotation direction
  }

  // Store calibrated Reference, Max, and Min values in EEPROM
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.0;  // Calculate mid-point (reference) value

    EEPROM.write(i, Reference_ADC[i] / 20);           // Store reference value (divided by 4 because EEPROM stores 0–255)
    delay(10);                                       // Small delay for EEPROM write stability
    EEPROM.write(i + sensorNumber, Max_ADC[i] / 20);  // Store maximum calibrated value
    delay(10);
    EEPROM.write(i + (sensorNumber * 2), Min_ADC[i] / 20);  // Store minimum calibrated value
    delay(10);
  }
}

// Function to load calibration data from EEPROM
void LoadCalibration() {
  SERIAL_PORT.println("Reference Value for Digital Conversion: ");
  // Print header labels for SERIAL_PORT monitor (sensor indices)
  SERIAL_PORT.print("S0: ");
  SERIAL_PORT.print("  S1: ");
  SERIAL_PORT.print("  S2: ");
  SERIAL_PORT.print("  S3: ");
  SERIAL_PORT.print("  S4: ");
  SERIAL_PORT.print("  S5: ");
  SERIAL_PORT.print("  S6: ");
  SERIAL_PORT.print("  S7: ");
  SERIAL_PORT.print("  S8: ");
  SERIAL_PORT.print("  S9: ");
  SERIAL_PORT.print("  S10: ");
  SERIAL_PORT.print(" S11: ");
  SERIAL_PORT.print(" S12: ");
  SERIAL_PORT.print(" S13: ");
  
  SERIAL_PORT.println();

  // Load the stored calibration data for each sensor
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 20;                 // Read and restore reference value
    Max_ADC[i] = EEPROM.read(i + sensorNumber) * 20;        // Read and restore maximum value
    Min_ADC[i] = EEPROM.read(i + (sensorNumber * 2)) * 20;  // Read and restore minimum value

    SERIAL_PORT.print(String(Reference_ADC[i]) + ",  ");  // Print loaded reference value
  }
  SERIAL_PORT.println();  // Move to new line after printing all sensor values
}

void motor(int leftPWM, int rightPWM) {
  if (rightPWM>255){rightPWM=255;}else{if(rightPWM<-255){rightPWM=-255;}}
  if (leftPWM>255){leftPWM=255;}else{if(leftPWM<-255){leftPWM=-255;}}
  if (leftPWM>0){analogWrite(pwmL1,0);analogWrite(pwmL2,leftPWM);}
  else {analogWrite(pwmL1,(-1*leftPWM));analogWrite(pwmL2, 0);}
  if (rightPWM>0){analogWrite(pwmR1,0);analogWrite(pwmR2,rightPWM);}
  else {analogWrite(pwmR1,(-1*rightPWM));analogWrite(pwmR2,0);}}

void motor_test() {
  motor(100, 100);
  digitalWrite(led, HIGH);
  delay(2000);
  motor(0, 0);
  digitalWrite(led, LOW);
  delay(200);
  motor(-90, 0);
  digitalWrite(led, HIGH);
  delay(2000);
  motor(0, 0);
}

void brake() {
  motor(-255, -255);
  delay(10);
  motor(0, 0);
}
void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    if (sensorADC[i] > Reference_ADC[i]) {
      if (inverseON == 0) {
        s[i] = 1;
      } else {
        s[i] = 0;
      }
    } else {
      if (inverseON == 0) {
        s[i] = 0;
      } else {
        s[i] = 1;
      }
    }
    sumOnSensor += s[i];
    sensorWight += s[i] * WeightValue[i];
    bitSensor += s[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

void sensor_value_test() {
  while (1) {
    read_black_line();
    for (unsigned char i = 0; i < sensorNumber; i++) {
      SERIAL_PORT.print(String(sensorADC[i]) + "  ");
    }
    SERIAL_PORT.println();
    if (digitalRead(button1) == 0 || digitalRead(button2) == 0 || digitalRead(button4) == 0) {
      delay(500);
      break;
    }
    return handleButtons();
  }
}

void handleButtons() {
  // Simple debounce
  if (millis() - lastButtonTime < debounceDelay) {
    return;
  }
  if (digitalRead(button1) == LOW) {
    lastButtonTime = millis();
    if (currentScreen == MAIN_MENU) {
      selectedItem--;
      if (selectedItem < 0) {
        selectedItem = menuCount - 1;
      }
      drawMainMenu();
    }
  }
  if (digitalRead(button2) == LOW) {
    lastButtonTime = millis();
    if (currentScreen == MAIN_MENU) {
      selectedItem++;
      if (selectedItem >= menuCount) {
        selectedItem = 0;
      }
      drawMainMenu();
    }
  }
  if (digitalRead(button3) == LOW) {
    lastButtonTime = millis();
    if (currentScreen == MAIN_MENU) {
      enterSelectedMenu();
    }
  }
  if (digitalRead(button4) == LOW) {
    lastButtonTime = millis();
    if (currentScreen != MAIN_MENU) {
      currentScreen = MAIN_MENU;
      drawMainMenu();
    }
  }
}
void drawMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("DEMO CODE");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  for (int i = 0; i < menuCount; i++) {
    int y = 14 + (i * 12);
    if (i == selectedItem) {
      display.fillRect(0, y - 1, 128, 11, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(4, y);
    display.print(i + 1);
    display.print(". ");
    display.println(menuItems[i]);
    display.setTextColor(SSD1306_WHITE);
  }
  display.display();
}
void enterSelectedMenu() {
  switch (selectedItem) {
    case 0:
      currentScreen = START_SCREEN;
      drawStartScreen();
      break;
    case 1:
      currentScreen = CALIBRATION_SCREEN;
      drawCalibrationScreen();
      break;
    case 2:
      currentScreen = SENSOR_TEST_SCREEN;
      drawSensorTestScreen();
      break;
    case 3:
      currentScreen = MOTOR_CONFIG_SCREEN;
      drawMotorConfigScreen();
      break;
  }
}
void drawStartScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(25, 5);
  display.println("START");
  display.setTextSize(1);
  display.setCursor(10, 30);
  display.println("Reset to Stop");
  display.display();
  linefollow();
}
void drawCalibrationScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("CALIBRATION");
  display.display();
  calibrateSensor();
  LoadCalibration();
  return handleButtons();
}
void drawSensorTestScreen() {
  display.clearDisplay();
  display.display();
  while (1) {
    sensor_value_test();
    if (digitalRead(button4) == 0) {
      delay(200);
      return handleButtons();
    }
  }
}

void drawMotorConfigScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("MOTOR CONFIG");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(5, 20);
  display.println("1. Both Forward");
  display.setCursor(5, 40);
  display.println("2. Left Backward");
  display.display();
  motor(100, 100);
  delay(2000);
  motor(-100, 0);
  delay(2000);
  motor(0, 0);
  return handleButtons();
}
