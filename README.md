# Air-Quality-Monitoring-System-and-warnings-alert-using-ESP32-and-Blynk

/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "XXX"
#define BLYNK_TEMPLATE_NAME "XXX"
#define BLYNK_AUTH_TOKEN "XXX"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "XXX";
char pass[] = "XXX";

#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial mySerial(21, 22); // RX, TX
unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;
float co_ppm = 0; // Declare co_ppm variable globally

#define LED_PIN 27 // Define LED pin
#define BUZZER_PIN 26 // Define Buzzer pin

#define MQ7_ANALOG_PIN 34 // Analog input pin for the MQ-7 sensor
#define SAMPLING_TIME 200 // Time between measurements (in milliseconds)

float loadResistor = 10.0; // Load resistor value (in kilo-ohms)
float ro = 10.0; // Initial value for Ro (in kilo-ohms), which should be calibrated for your specific sensor

WidgetTerminal terminal(V4); // Add terminal widget instance

void setup() {
  Serial.begin(9600);
  while (!Serial);
  mySerial.begin(9600);
  pinMode(MQ7_ANALOG_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); // Initialize LED pin as output
  pinMode(BUZZER_PIN, OUTPUT); // Initialize Buzzer pin as output

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("MQ-7 CO Sensor and PM Sensor Example");

  // Buzzer test
  digitalWrite(BUZZER_PIN, HIGH);
  delay(5000);
  digitalWrite(BUZZER_PIN, LOW);
  
}

void loop() {
  Blynk.run();

  readPMSensor();
  readMQ7Sensor();
  checkAndSendWarning(); // Call the function to check and send warning messages

  delay(SAMPLING_TIME);
}

void readPMSensor() {
  int index = 0;
  char value;
  char previousValue;

  while (mySerial.available()) {
    value = mySerial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) {
      Serial.println("Cannot find the data header.");
      break;
    }

    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
      previousValue = value;
    }
    else if (index == 5) {
      pm1 = 256 * previousValue + value;
      Serial.print("{ ");
      Serial.print("\"pm1\": ");
      Serial.print(pm1);
      Serial.print(" ug/m3");
      Serial.print(", ");

      Blynk.virtualWrite(V2, pm1);
    }
    else if (index == 7) {
      pm2_5 = 256 * previousValue + value;
      Serial.print("\"pm2_5\": ");
      Serial.print(pm2_5);
      Serial.print(" ug/m3");
      Serial.print(", ");

      Blynk.virtualWrite(V3, pm2_5);
    }
    else if (index == 9) {
          pm10 = 256 * previousValue + value;
      Serial.print("\"pm10\": ");
      Serial.print(pm10);
      Serial.print(" ug/m3");

      Blynk.virtualWrite(V1, pm10);
    }
    else if (index > 15) {
      break;
    }
    index++;
  }
  while (mySerial.available()) mySerial.read();
  Serial.println(" }");
}

void readMQ7Sensor() {
  float rs = readSensorResistance(MQ7_ANALOG_PIN);
  co_ppm = calculateCOppm(rs, ro); // Update the global co_ppm variable

  Serial.print("CO Concentration: ");
  Serial.print(co_ppm);
  Serial.println(" ppm");

  Blynk.virtualWrite(V0, co_ppm);
}

float readSensorResistance(int pin) {
  int adc_value = analogRead(pin);
  float voltage = adc_value * (3.3 / 4095.0);
  float rs = ((3.3 - voltage) * loadResistor) / voltage;

  return rs;
}

float calculateCOppm(float rs, float ro) {
  float ratio = rs / ro;
  float co_ppm = 30.872 * pow(ratio, -1.431); // Values are based on the MQ-7 datasheet curve

  return co_ppm;
}

void checkAndSendWarning() {
  String warningMessage = "";
  bool isWarning = false; // Variable to store if a warning is triggered

  if (pm10 > 50) {
    warningMessage = "Warning! The PM10 in your area is " + String(pm10) + " ug/m3 and It's getting dangerous.";
    terminal.println(warningMessage);
    terminal.flush();
    isWarning = true;
  }

  if (pm2_5 > 35) {
    warningMessage = "Warning! The PM2.5 in your area is " + String(pm2_5) + " ug/m3 and It's getting dangerous.";
    terminal.println(warningMessage);
    terminal.flush();
    isWarning = true;
  }

  if (co_ppm > 9) {
    warningMessage = "Warning! The CO in your area is " + String(co_ppm) + " ppm and It's getting dangerous.";
    terminal.println(warningMessage);
    terminal.flush();
    isWarning = true;
  }

  // Turn on the LED and buzzer if there is a warning, otherwise turn them off
  if (isWarning) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
}
