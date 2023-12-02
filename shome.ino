#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Servo.h>
#include <DHT.h>
#include <SoftwareSerial.h>

// ESP8266
SoftwareSerial Bluetooth(15, 14); // RX, TX

// current
const int ACS_PIN = A3;

// Time
bool ACSEnabled = true;                 // Flag to control motion sensor reading
unsigned long ACSPreviousMillis = 0;    // Previous time motion sensor was read
const unsigned long ACSInterval = 8000; // Interval between motion sensor readings in milliseconds

// Temperature
// DHT
#define DHT_PIN 22
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// fan
Servo fanServo;
const int FAN_PIN = 24;
const float thresholdTempC = 30.0; // Room Temperature

// Time
bool DHTEnabled = true;                 // Flag to control motion sensor reading
unsigned long DHTPreviousMillis = 0;    // Previous time motion sensor was read
const unsigned long DHTInterval = 9000; // Interval between motion sensor readings in milliseconds

// Lights
const int LDR = A2; // LDR motion sensor pin
const int LED_OUT = 12;

// Time
bool LDREnabled = true;                 // Flag to control motion sensor reading
unsigned long LDRPreviousMillis = 0;    // Previous time motion sensor was read
const unsigned long LDRInterval = 9500; // Interval between motion sensor readings in milliseconds

// Motion Sensor
#define redLed 28                 //   pin for red led
#define greenLed 30               // pin for green led
#define yellowLed 32              // pin for blue led
const int MOTION_SENSOR_PIN = 13; // Arduino pin connected to the OUTPUT pin of motion sensor
const int BUZZER_PIN = 11;        // Arduino pin connected to Buzzer's pin

bool motionEnabled = true;                  // Flag to control motion sensor reading
unsigned long motionPreviousMillis = 0;     // Previous time motion sensor was read
const unsigned long motionInterval = 10000; // Interval between motion sensor readings in milliseconds

// Security
// Keypad
const byte ROWS = 4; // four rows
const byte COLS = 3; // three columns
char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};
byte rowPins[ROWS] = {2, 3, 4, 5}; //
byte colPins[COLS] = {6, 7, 8};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// password
char correctPassword[] = "2020"; // Change this to your desired password
char enteredPIN[5];              // Array to store entered PIN
byte pinIndex = 0;               // Index for tracking PIN entry

// Servo
Servo doorServo;
const int servopin = 9;
const int ledPin = 10; // door

// TIME
bool KeyPadEnabled = true;              // Flag to control motion sensor reading
unsigned long KeyPadPreviousMillis = 0; // Previous time motion sensor was read
const unsigned long KeyPadInterval = 1; // Interval between motion sensor readings in milliseconds

// Display
// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

// Function prototypes
void processBluetoothCommand(char command);
void turnFanOn();
void turnFanOff();
void turnLightsOn();
void turnLightsOff();
void readDHT();
void readLDR();
void readMotionSensor();
void readKeypad();
bool checkPIN();
void openDoor();
void closeDoor();
void triggerBuzzer();
void EnergyReadings();

void setup()
{

  Bluetooth.begin(9600);
  Serial.begin(9600); // Start Serial Monitor to display current read value on Serial monitor

  // LCD
  lcd.init(); // initialize the lcd
  // Adjust contrast (try different values, typically between 0 and 255)
  lcd.setBacklight(25);
  lcd.command(0x21);       // Set LCD extended command mode
  lcd.command(0x2C | 200); // Set contrast(adjust as needed)
  lcd.command(0x20);       // Set LCD basic command mode
  lcd.setCursor(0, 0);

  // Servo
  doorServo.attach(servopin); // attaches the servo on pin 9 to the servo object
  closeDoor();

  // Buzzer
  // //Serial.begin(9600);                // initialize serial
  pinMode(MOTION_SENSOR_PIN, INPUT); // set arduino pin to input mode
  pinMode(BUZZER_PIN, OUTPUT);       // set arduino pin to output mode
  pinMode(redLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(LDR, INPUT);
  pinMode(LED_OUT, OUTPUT);
  // Serial.println("System startup"); // Used for troubleshooting

  // TEMPERATURE AND HUMIDITY
  pinMode(FAN_PIN, OUTPUT);
  pinMode(DHT_PIN, INPUT);
  dht.begin(); // initialize the sensor

  // CURRENT
  pinMode(ACS_PIN, INPUT);
}

void loop()
{

  // Bluetooth communication
  if (Serial.available() > 0)
  {
    char command = Serial.read();
    // Serial.println(command);
    processBluetoothCommand(command);
  }

  readKeypad();
  readLDR();
  readMotionSensor();
  EnergyReadings();
  readDHT();
}

void processBluetoothCommand(char command)
{
  switch (command)
  {
  case 'e':
    openDoor();
    break;
  case 'f':
    closeDoor();
    break;
  case 'c':
    turnFanOn();
    break;
  case 'd':
    turnFanOff();
    break;
  case 'g':
    turnLightsOn();
    break;
  case 'h':
    turnLightsOff();
    break;
    // Add more commands as needed
  }
}

void turnFanOn()
{
  // Serial.println("Turning fan on");
  digitalWrite(FAN_PIN, HIGH);
}

void turnFanOff()
{
  // Serial.println("Turning fan off");
  digitalWrite(FAN_PIN, LOW);
}

void turnLightsOn()
{
  // Serial.println("Turning lights on");
  digitalWrite(redLed, HIGH); //  turns ON the lights
  digitalWrite(greenLed, HIGH);
  digitalWrite(yellowLed, HIGH);
}

void turnLightsOff()
{
  // Serial.println("Turning lights off");
  digitalWrite(redLed, LOW); // turns OFF the lights
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
}

// Functions

// DHT22
void readDHT()
{
  unsigned long currentMillis = millis();
  if (currentMillis - DHTPreviousMillis >= DHTInterval)
  {
    DHTPreviousMillis = currentMillis;
    // read humidity
    float humi = dht.readHumidity();
    // read temperature as Celsius
    float tempC = dht.readTemperature();

    // Check for NaN in temperature readings
    if (!(tempC == tempC && humi == humi))
    {
      // Serial.println("Failed to read from DHT sensor!");
      delay(2000);
    }
    else
    {
      // Serial.print("Humidity: ");
      // Serial.print(humi);
      // Serial.println("%");

      // Serial.print("Temperature: ");
      // Serial.print(tempC);
      // Serial.println("°C");

      // Control the motor based on temperature
      if (tempC > thresholdTempC)
      {
        // It's hot, open the fan
        digitalWrite(FAN_PIN, HIGH);
        delay(5000);
      }
      else
      {
        // It's cold, close the FAN
        digitalWrite(FAN_PIN, LOW);
        delay(5000);
      }
    }
  }
}

// LDR
void readLDR()
{
  unsigned long currentMillis = millis();
  if (currentMillis - LDRPreviousMillis >= LDRInterval)
  {
    LDRPreviousMillis = currentMillis;
    int LDR_Status = analogRead(LDR);

    if (LDR_Status <= 150)
    {
      digitalWrite(LED_OUT, HIGH);

      // Serial.print(" LIGHT ON : ");
      Serial.println(LDR_Status);
    }
    else
    {
      digitalWrite(LED_OUT, LOW); // Turn LED off
      // Serial.println("LIGHT OFF: ");
      // Serial.println(LDR_Status); // Print LDR analog value on the serial port
    }
  }
}

// Motion Sensor
void readMotionSensor()
{
  unsigned long currentMillis = millis();

  if (currentMillis - motionPreviousMillis >= motionInterval)
  {
    motionPreviousMillis = currentMillis;

    // Motion Detection
    int Motion_Value = digitalRead(MOTION_SENSOR_PIN);

    if (Motion_Value == HIGH)
    {
      // Serial.println("Motion detected!");
      digitalWrite(redLed, HIGH); // Turn on the LED
      digitalWrite(yellowLed, HIGH);
      digitalWrite(greenLed, HIGH);
      delay(1000); // Adjust the delay as needed
    }
    else
    {
      // Serial.println("No motion detected");
      digitalWrite(redLed, LOW); // Turn on the LED
      digitalWrite(yellowLed, LOW);
      digitalWrite(greenLed, LOW); // Turn off the LED
    }
  }
}

// Keypad
void readKeypad()
{
  unsigned long currentMillis = millis();
  if (currentMillis - KeyPadPreviousMillis >= KeyPadInterval)
  {
    KeyPadPreviousMillis = currentMillis;
    char key = keypad.getKey();

    if (key != NO_KEY)
    {
      // Serial.print(key);

      // Check if special sequence "#01" is entered
      if (key == '#')
      {
        char nextKey = keypad.getKey();
        if (nextKey == '0')
        {
          char thirdKey = keypad.getKey();
          if (thirdKey == '1')
          {
            // Display energy readings without entering PIN
            EnergyReadings();
          }
        }
      }
      else
      {
        // Display entered character on LCD
        lcd.setCursor(pinIndex, 1);
        lcd.print('*');

        // Check if entered PIN after 4 characters
        enteredPIN[pinIndex++] = key;

        if (pinIndex == 4)
        {
          // Serial.println("\nChecking PIN...");

          // Check entered PIN
          if (checkPIN())
          {
            // Open the door only if the correct PIN is entered
            delay(1000);
            openDoor();
            lcd.clear();
            lcd.print("Door Opened");
            delay(5000); // Keep the door open for 5 seconds (adjust as needed)
            closeDoor();
            lcd.clear();
            lcd.print("Enter PIN:");
            // doorOpened = true; // Set the flag to indicate the door is open
          }
          else
          {
            // Serial.println("Incorrect PIN");
            lcd.clear();
            lcd.print("Incorrect PIN");
            triggerBuzzer(); // Trigger the buzzer for incorrect PIN
            delay(2000);     // Display "Incorrect PIN" for 2 seconds
            lcd.clear();
            lcd.print("Enter PIN:");
          }

          // Reset entered PIN for the next attempt
          pinIndex = 0;
          memset(enteredPIN, 0, sizeof(enteredPIN));
        }
      }
    }
  }
}

bool checkPIN()
{
  enteredPIN[pinIndex] = '\0'; // Null-terminate the entered PIN
  return (strcmp(enteredPIN, correctPassword) == 0);
}

void openDoor()
{
  doorServo.write(180);       // Assuming 90 degrees is the open position
  digitalWrite(ledPin, HIGH); // Turn on the LED to indicate door is open
  // Serial.println("Door Opened");
}

void closeDoor()
{
  doorServo.write(-180);     // Assuming 0 degrees is the closed position
  digitalWrite(ledPin, LOW); // Turn off the LED to indicate door is closed
  // Serial.println("Door Closed");
}

void triggerBuzzer()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(10000); // Buzz for 0.5 seconds (adjust as needed)
  digitalWrite(BUZZER_PIN, LOW);
}

void EnergyReadings()
{
  unsigned long currentMillis = millis();
  if (currentMillis - ACSPreviousMillis >= ACSInterval)
  {
    ACSPreviousMillis = currentMillis;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Energy Readings");
    // display of energy readings here
    unsigned int x = 0;
    int AcsValue = 0.0, Samples = 0.0, AvgAcs = 0.0, AcsValueF = 0.0;

    // Read current sensor values
    AcsValue = analogRead(ACS_PIN);
    // Serial.println("ACS value" + AcsValue);
    for (int x = 0; x < 15; x++)
    {                               // Get 15 samples
      Samples = Samples + AcsValue; // Add samples together
      delay(3);                     // let ADC settle before following sample 3ms
    }
    AvgAcs = Samples / 150.0; // Taking Average of Samples
    AcsValueF = (2.5 - (AvgAcs * (5.0 / 1024.0))) / 0.185;

    // Serial.println("Average Read" + AcsValueF); // Print the read current on Serial monitor
    lcd.clear();
    lcd.print("Current: " + String(AcsValue));
    delay(1000); // Display readings for 2 seconds
    lcd.clear();
    lcd.print("ENTER PIN: ");
  }
}
