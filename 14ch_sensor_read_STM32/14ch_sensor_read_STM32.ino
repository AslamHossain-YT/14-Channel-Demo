//MULTIPLEXER CONTROL PINS (STM32 BLUEPILL)
#define S0 PA7
#define S1 PA6
#define S2 PA5
#define S3 PA4
#define SIG_PIN PA3

#define sensorNumber 14
int sensor[sensorNumber];

void setup() {
  Serial.begin(9600);
  // Set control pins as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  // ADC input
  pinMode(SIG_PIN, INPUT_ANALOG);
  //set 12-bit ADC resolution
  analogReadResolution(12);  // Range: 0–4095
}

void loop() {
  for (int channel = 0; channel < sensorNumber; channel++) {
    selectChannel(channel);
    delayMicroseconds(5);  // MUX settling time
    sensor[channel] = analogRead(SIG_PIN);
    Serial.print(sensor[channel]);
    Serial.print("  ");
  }
  Serial.println();
}

//SELECT MUX CHANNEL
void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));  // LSB
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));  // MSB
}
