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

#define LED_BUILTIN 2
#define SENSOR  27

boolean ledState = LOW;
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;

// ======================
// === Flow Variables ===
// ======================
float calibrationFactor = 11; //F
volatile byte pulseCount;
float flow = 0.0;

// unsigned int flowMilliLitres;
// unsigned long totalMilliLitres;

// --- Flow Functions ---
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}
void SetupFlow()
{
  pulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
}
void displayFlow(float FlowValue)//, unsigned long TotalValue)
{
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(FlowValue));  // Print the integer part of the variable
    Serial.println("L/min");

    // Print the cumulative total of litres flowed since starting
    // Serial.print("Output Liquid Quantity: ");
    // Serial.print(TotalValue);
    // Serial.print("mL / ");
    // Serial.print(TotalValue / 1000);
    // Serial.println("L");
  
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
  // this case) coming from the sensor.
  flowRate = ((1000.0 / (millis() - PreviousFlowMillis)) * pulse1Sec) / calibrationFactor;

  // Divide the flow rate in litres/minute by 60 to determine how many litres have
  // passed through the sensor in this 1 second interval, then multiply by 1000 to
  // convert to millilitres.
  // flowMilliLitres = (flowRate / 60) * 1000;

  // Add the millilitres passed in this second to the cumulative total
  //totalMilliLitres += flowMilliLitres;
  // displayFlow(flowRate,totalMilliLitres);
  
  return flowRate;
}

//==========================================
//================ Setup ===================
//==========================================
void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR, INPUT_PULLUP);
  previousMillis = 0;
  SetupFlow();
  
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
    previousMillis = millis();
  }

}
