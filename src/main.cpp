#include <Arduino.h>
/*
  Application:
  - Interface water flow sensor with ESP32 board.
  
  Board:
  - ESP32 Dev Module
    https://my.cytron.io/p-node32-lite-wifi-and-bluetooth-development-kit

  Sensor:
  - G 1/2 Water Flow Sensor
    https://my.cytron.io/p-g-1-2-water-flow-sensor
 */

// ======================
// ===== Define PIN =====
// ======================
#define LED_BUILTIN 2
#define FLOWSENSOR  27
#define PRESSURESENSOR 32
boolean ledState = LOW;
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;

// ======================
// === Flow Variables ===
// ======================
// F=11Q written on the flow sensor
float calibrationFactor = 11; //F
volatile byte pulseCount;
float flow = 0.0;

// ==========================
// === Pressure Variables ===
// ==========================
const float PressureMin = 1.4; // Pressure at wich the pump will start in bar
const float PressureMax = 3.0; // Minimum running pressure of the pump. The pump will not stop unless this pressure is achieved in bar
float PressureValue = 0.000;
// For a perfect 100psi sensor - A=25 ; B=-12.5
// La formule de la pression est 
// A * Voltage + B = Pressure
float PressureCalibrationA = 23.364;
float PressureCalibrationB = -5.1402;//-12.5


// ----- Flow Functions -----
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}
void SetupFlow()
{
  pinMode(FLOWSENSOR, INPUT_PULLUP);
  pulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(FLOWSENSOR), pulseCounter, FALLING);
}
void displayFlow(float FlowValue)//, unsigned long TotalValue)
{
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(FlowValue));  // Print the integer part of the variable
    Serial.println("L/min");
  
}
float GetFlow(long PreviousFlowMillis)
{
  float flowRate;
  byte pulse1Sec = 0;
  flowRate = 0.0;
  pulse1Sec = pulseCount;
  pulseCount = 0;

  // Because this loop may not complete in exactly 1 second intervals we calculate
  // the number of milliseconds that have passed since the last execution and use
  // that to scale the output. We also apply the calibrationFactor to scale the output
  // based on the number of pulses per second per units of measure (litres/minute in
  // this case) coming from the FLOWSENSOR.
  flowRate = ((1000.0 / (millis() - PreviousFlowMillis)) * pulse1Sec) / calibrationFactor;
  Serial.print("Pulse 1 s: ");
  Serial.print(pulse1Sec);
  Serial.print(" ; ");
  return flowRate;
}
// --------------------------

// ----- Pressure Functions -----
void SetupPressure()
{
  analogSetPinAttenuation(PRESSURESENSOR, ADC_0db);
  pinMode(PRESSURESENSOR,INPUT_PULLUP);
}
void displayPressure(float PressureValue)//, unsigned long TotalValue)
{
    // Print the flow rate for this second in litres / minute
    Serial.print("Pressure: ");
    Serial.print(float(PressureValue));  // Print the integer part of the variable
    Serial.println(" Bar");
  
}
float GetPressure(const char PinSensor)
{  
  float pressure = 0.00;
  float pressure1 = analogRead(PinSensor);//read the pressure
  delay(10);
  float pressure2 = analogRead(PinSensor);
  delay(10);
  float pressure3 = analogRead(PinSensor);
  pressure = (pressure1+pressure2+pressure3)/3; //average the readings
  pressure = pressure*4.8/4095; //voltage
  Serial.print("Voltage =");Serial.print(pressure1);Serial.print(" - ");Serial.print(pressure2);Serial.print(" - ");Serial.print(pressure3);Serial.print(" -  ");Serial.print(pressure);Serial.println(" V ");
  pressure = PressureCalibrationA*pressure+PressureCalibrationB;// Pressure in PSI
  Serial.print("Pressure: ");Serial.print(pressure);Serial.println(" PSI");
  pressure = pressure/14.503773800722; // Convert PSI to BAR

  return pressure;
}
// ------------------------------

// ----- Pump Function -----
void StartPump()
{
  
}

//==========================================
//================ Setup ===================
//==========================================
void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  
  previousMillis = 0;
  SetupFlow();
  SetupPressure();
  
}


//=========================================
//================ Loop ===================
//=========================================
void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    flow = GetFlow(previousMillis);
    displayFlow(flow);

    PressureValue = GetPressure(PRESSURESENSOR);
    displayPressure(PressureValue);
    previousMillis = millis();
    Serial.println("---------------------------------------------------------");
  }

}
