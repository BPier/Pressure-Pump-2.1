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
// ===== Blynk Parameters =====
#define BLYNK_TEMPLATE_ID           "TMPLyOjK1eom"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "oKLzWGjqnvWtjT-iXjAlUw-vuiKLCmrk"
// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
char auth[] = BLYNK_AUTH_TOKEN;
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "TownHouse";
char pass[] = "Itsraining";
BlynkTimer timer;

bool YNBlynkSetup = false;
int BlynkPumpSwitch = 0;
// ======================
// ===== Define PIN =====
// ======================
#define LED_BUILTIN 2
#define FLOWSENSOR  27
#define PRESSURESENSOR 35
#define PumpPin 18
boolean ledState = LOW;
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
// === PumpVariable ===
int PumpStatus = 0;
int PumpRunningTime = 0;

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
float PressureCalibrationA = 25;
float PressureCalibrationB = -12.5;//-12.5


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


// ----- Blynk Functions -----
BLYNK_WRITE(V0)
{
  // This function is called every time the Virtual Pin 0 state changes
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
}
BLYNK_CONNECTED()
{
  // This function is called every time the device is connected to the Blynk.Cloud
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}
void myTimerEvent()
{
// This function sends Arduino's uptime every second to Virtual Pin 2.
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}
void BlynkSetup()
{
  // Blynk
  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
  YNBlynkSetup=true;
}
void SendBlynkValue()
{
  Blynk.virtualWrite(V5, float(PressureValue));
  Blynk.virtualWrite(V6, flow);
  Blynk.virtualWrite(V7,int(PumpStatus));
}

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
  int PressureReading[10];
  //Get a set of values
  for (byte i = 0; i<10; i = i+1 )
  {
    PressureReading[i] = analogRead(PinSensor);
    delay(10);
    pressure = pressure+PressureReading[i];
  }

  pressure = pressure/10; //average the readings
  pressure = pressure*4.8/4095; //voltage
  Serial.print("Voltage =");Serial.print(PressureReading[1]);Serial.print(" - ");Serial.print(PressureReading[2]);Serial.print(" - ");Serial.print(PressureReading[3]);Serial.print(" -  ");Serial.print(pressure);Serial.println(" V ");
  pressure = PressureCalibrationA*pressure+PressureCalibrationB;// Pressure in PSI
  Serial.print("Pressure: ");Serial.print(pressure);Serial.println(" PSI");
  pressure = pressure/14.503773800722; // Convert PSI to BAR

  return pressure;
}
// ------------------------------

// ----- Pump Function -----
void SetupPump()
{
  pinMode(PumpPin,OUTPUT);
}
void StartPump()
{
  digitalWrite(PumpPin,HIGH);
  PumpStatus=1;
  //Blynk.logEvent("pump_start", String("The Pump has started"));
  
}
void StopPump()
{
  digitalWrite(PumpPin,LOW);
  PumpStatus=0;
  PumpRunningTime=0;
  Blynk.virtualWrite(V10,0);
}
void PumpStartCycle()
{
  StartPump();
  delay(1000);
  StopPump();
}
void PumpManagement(){
  if (BlynkPumpSwitch == 1){
    StartPump();
  }
  Blynk.virtualWrite(V7,PumpStatus);
  if (PumpStatus==1)
  {
    PumpRunningTime=PumpRunningTime+interval/1000;
  }
  if (PumpRunningTime>3 && flow<1)
  {
    StopPump();
  }
  if (flow>1){
    StartPump();
  }
}

BLYNK_WRITE(V8){
  PressureCalibrationA = param.asFloat();
}
BLYNK_WRITE(V9){
  PressureCalibrationB = param.asFloat();
}
BLYNK_WRITE(V10){
  BlynkPumpSwitch = param.asInt();
}
// ====== Loops MultiCore ======
void loop1(void *pvParameters){
  while (1) {
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      flow = GetFlow(previousMillis);
      displayFlow(flow);
      PressureValue = GetPressure(PRESSURESENSOR);
      displayPressure(PressureValue);
      previousMillis = millis();
      Serial.print("V10: ");Serial.print(BlynkPumpSwitch);Serial.print(" - Wifi Connexion: ");Serial.println(YNBlynkSetup);
      Serial.println("---------------------------------------------------------");
      PumpManagement();
    }
  }
}
void loop2(void *pvParameters){
  
  while (1) {
    currentMillis = millis();
    if (currentMillis - previousMillis > 100) {
      SendBlynkValue();
    }
    Blynk.run();
    timer.run();
  }
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
  SetupPump();
  //PumpStartCycle();
  xTaskCreatePinnedToCore(loop1, "loop1", 16000, NULL, 1, NULL, 0);
  BlynkSetup();
  xTaskCreatePinnedToCore(loop2, "loop2", 16000, NULL, 10, NULL, 1);
}


//=========================================
//================ Loop ===================
//=========================================
void loop()
{
  
}
