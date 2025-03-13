
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// WiFi credentials
char ssid[] = "YourWiFiSSID";
char pass[] = "YourWiFiPassword";

// Blynk authentication token
char auth[] = "YourBlynkAuthToken";

// Pin definitions
#define MAIN_VOLTAGE_SENSOR 34    // Main AC voltage sensor
#define SOLAR_VOLTAGE_SENSOR 35   // Solar AC voltage sensor
#define WINDMILL_VOLTAGE_SENSOR 32 // Windmill AC voltage sensor
#define CURRENT_SENSOR 33         // AC current sensor
#define SOLAR_RELAY 25            // Relay for solar
#define WINDMILL_RELAY 26         // Relay for windmill

// Calibration constants
#define VOLTAGE_CALIBRATION 0.707  // RMS conversion factor
#define CURRENT_CALIBRATION 0.707  // RMS conversion factor
#define VOLTAGE_REFERENCE 3.3      // Reference voltage of ESP32 ADC
#define ADC_RESOLUTION 4095.0      // 12-bit ADC resolution
#define VOLTAGE_DIVIDER_RATIO 100  // Voltage divider ratio (adjust as per your hardware)
#define CURRENT_SENSITIVITY 0.066  // For ACS712 30A (66mV/A)
#define VOLTAGE_THRESHOLD 5.0      // Minimum voltage to consider a source active

// Variables
float mainVoltage = 0.0;
float solarVoltage = 0.0;
float windmillVoltage = 0.0;
float current = 0.0;
float power = 0.0;
float energy = 0.0;
bool solarActive = false;
bool windmillActive = false;
unsigned long lastMillis = 0;
unsigned long energyTimer = 0;
unsigned long switchingTimer = 0;
const unsigned long SWITCHING_DELAY = 10000; // 10 seconds delay between source switching

// Blynk virtual pins
#define V_MAIN_VOLTAGE V0
#define V_SOLAR_VOLTAGE V1
#define V_WINDMILL_VOLTAGE V2
#define V_CURRENT V3
#define V_POWER V4
#define V_ENERGY V5
#define V_ACTIVE_SOURCE V6
#define V_MANUAL_CONTROL V7

bool manualControl = false;


BLYNK_WRITE(V_MANUAL_CONTROL) {
  manualControl = param.asInt();
}


BLYNK_WRITE(V_ACTIVE_SOURCE) {
  if (manualControl) {
    int source = param.asInt();
    switch (source) {
      case 1: 
        activateSolar();
        break;
      case 2: 
        activateWindmill();
        break;
      case 0: 
      default:
        deactivateBoth();
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  

  pinMode(MAIN_VOLTAGE_SENSOR, INPUT);
  pinMode(SOLAR_VOLTAGE_SENSOR, INPUT);
  pinMode(WINDMILL_VOLTAGE_SENSOR, INPUT);
  pinMode(CURRENT_SENSOR, INPUT);
  pinMode(SOLAR_RELAY, OUTPUT);
  pinMode(WINDMILL_RELAY, OUTPUT);
  
  
  digitalWrite(SOLAR_RELAY, LOW);
  digitalWrite(WINDMILL_RELAY, LOW);
  

  Blynk.begin(auth, ssid, pass);
  
  energyTimer = millis();
  switchingTimer = millis();
}

void loop() {
  Blynk.run();
  
  
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
    
  
    mainVoltage = readVoltage(MAIN_VOLTAGE_SENSOR);
    solarVoltage = readVoltage(SOLAR_VOLTAGE_SENSOR);
    windmillVoltage = readVoltage(WINDMILL_VOLTAGE_SENSOR);
    current = readCurrent(CURRENT_SENSOR);
    
    
    power = mainVoltage * current;
    

    calculateEnergy();
    
 
    if (!manualControl) {
      autoSwitchEnergySource();
    }
    
 
    printValues();
    
  
    updateBlynk();
  }
}

void autoSwitchEnergySource() {
  
  if (millis() - switchingTimer >= SWITCHING_DELAY) {
   
    if (solarVoltage > windmillVoltage && solarVoltage > VOLTAGE_THRESHOLD) {
      
      if (!solarActive) {
        activateSolar();
        switchingTimer = millis();
      }
    } else if (windmillVoltage > solarVoltage && windmillVoltage > VOLTAGE_THRESHOLD) {
     
      if (!windmillActive) {
        activateWindmill();
        switchingTimer = millis();
      }
    } else if (solarVoltage < VOLTAGE_THRESHOLD && windmillVoltage < VOLTAGE_THRESHOLD) {
      
      deactivateBoth();
    }
  }
}

void activateSolar() {
  digitalWrite(WINDMILL_RELAY, LOW);
  digitalWrite(SOLAR_RELAY, HIGH);
  solarActive = true;
  windmillActive = false;
  Serial.println("Activated Solar Source");
  Blynk.virtualWrite(V_ACTIVE_SOURCE, 1);
}

void activateWindmill() {
  digitalWrite(SOLAR_RELAY, LOW);
  digitalWrite(WINDMILL_RELAY, HIGH);
  solarActive = false;
  windmillActive = true;
  Serial.println("Activated Windmill Source");
  Blynk.virtualWrite(V_ACTIVE_SOURCE, 2);
}

void deactivateBoth() {
  digitalWrite(SOLAR_RELAY, LOW);
  digitalWrite(WINDMILL_RELAY, LOW);
  solarActive = false;
  windmillActive = false;
  Serial.println("Deactivated Both Sources");
  Blynk.virtualWrite(V_ACTIVE_SOURCE, 0);
}

float readVoltage(int pin) {
 
  float sumSquares = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    int rawValue = analogRead(pin);
    float voltage = (rawValue / ADC_RESOLUTION) * VOLTAGE_REFERENCE;
   
    voltage -= (VOLTAGE_REFERENCE / 2);
    sumSquares += voltage * voltage;
    delayMicroseconds(200);  
  }
  
  // Calculate RMS
  float rms = sqrt(sumSquares / samples);
  
  
  return rms * VOLTAGE_DIVIDER_RATIO;
}

float readCurrent(int pin) {

  float sumSquares = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    int rawValue = analogRead(pin);
    float voltage = (rawValue / ADC_RESOLUTION) * VOLTAGE_REFERENCE;
  
    voltage -= 2.5;
   
    float instantCurrent = voltage / CURRENT_SENSITIVITY;
    sumSquares += instantCurrent * instantCurrent;
    delayMicroseconds(200); 
  }
  
 
  return sqrt(sumSquares / samples);
}

void calculateEnergy() {

  unsigned long currentMillis = millis();
  float hoursPassed = (currentMillis - energyTimer) / 3600000.0;  
  energy += power * hoursPassed / 1000.0; 
  energyTimer = currentMillis;
}

void printValues() {
  Serial.println("===== Smart Energy Meter Readings =====");
  Serial.print("Main Voltage: "); Serial.print(mainVoltage); Serial.println(" V");
  Serial.print("Solar Voltage: "); Serial.print(solarVoltage); Serial.println(" V");
  Serial.print("Windmill Voltage: "); Serial.print(windmillVoltage); Serial.println(" V");
  Serial.print("Current: "); Serial.print(current); Serial.println(" A");
  Serial.print("Power: "); Serial.print(power); Serial.println(" W");
  Serial.print("Energy: "); Serial.print(energy); Serial.println(" kWh");
  Serial.print("Active Source: ");
  if (solarActive) Serial.println("Solar");
  else if (windmillActive) Serial.println("Windmill");
  else Serial.println("None");
  Serial.print("Control Mode: "); Serial.println(manualControl ? "Manual" : "Automatic");
  Serial.println("======================================");
}

void updateBlynk() {
  Blynk.virtualWrite(V_MAIN_VOLTAGE, mainVoltage);
  Blynk.virtualWrite(V_SOLAR_VOLTAGE, solarVoltage);
  Blynk.virtualWrite(V_WINDMILL_VOLTAGE, windmillVoltage);
  Blynk.virtualWrite(V_CURRENT, current);
  Blynk.virtualWrite(V_POWER, power);
  Blynk.virtualWrite(V_ENERGY, energy);
  

  int activeSource = 0;
  if (solarActive) activeSource = 1;
  else if (windmillActive) activeSource = 2;
  Blynk.virtualWrite(V_ACTIVE_SOURCE, activeSource);
}

