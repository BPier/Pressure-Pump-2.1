#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
// ------------------------------------------------------------------------------------------------
// ===== Blynk Parameters =====
#define BLYNK_TEMPLATE_ID           "TMPLyOjK1eom"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "oKLzWGjqnvWtjT-iXjAlUw-vuiKLCmrk"
//#define BLYNK_PRINT Serial // Comment this out to disable prints and save space
#define WIFI_TIMEOUT_MS 20000 // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt
char auth[] = BLYNK_AUTH_TOKEN;
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "TownHouse";
char pass[] = "Itsraining";
BlynkTimer timer;
bool BlynkConnectionResult = false;
int BlynkPumpSwitch = 0;

// ===== Define PIN =====
#define LED_BUILTIN 2
#define FLOWSENSOR  27
#define PRESSURESENSOR 35
#define PumpPin 18
#define WifiPin 16
boolean ledState = LOW;
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;

// === LED Variables ===
int LEDWifiValue = 0;
int LEDPumpValue = 0;
int LEDWifiState=0;
int LEDPumpState=0;
// === PumpVariable ===
int PumpStatus = 0;
long PumpRunningTime = 0;
long PumpStartTime = 0;

// === Flow Variables ===
float calibrationFactor = 11; // F=11Q written on the flow sensor
volatile byte pulseCount;
float flow = 0.0;

// === Pressure Variables ===
const float PressureMin = 1.4; // Pressure at wich the pump will start in bar
const float PressureMax = 3.0; // Minimum running pressure of the pump. The pump will not stop unless this pressure is achieved in bar
float PressureValue = 0.000;
// For a perfect 100psi sensor - A=25 ; B=-12.5
// La formule de la pression est 
// A * Voltage + B = Pressure
float PressureCalibrationA = 25.1;
float PressureCalibrationB = -12.5;//-12.5

// ------------------------------------------------------------------------------------------------
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
  // Serial.print("Pulse 1 s: ");
  // Serial.print(pulse1Sec);
  // Serial.print(" ; ");
  return flowRate;
}
// --------------------------


// ----- Blynk Functions -----
void WifiConnect(){
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi ..");
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
    Serial.print('.');
    delay(1000);
  }
}
BLYNK_WRITE(V0)
{
  // This function is called every time the Virtual Pin 0 state changes
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
}
BLYNK_WRITE(V8){
  PressureCalibrationA = param.asFloat();
}
BLYNK_WRITE(V9){
  PressureCalibrationB = param.asFloat();
}
BLYNK_CONNECTED()
{
  // This function is called every time the device is connected to the Blynk.Cloud
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
  Serial.println("[Blynk] Connected To Blynk");
}
void myTimerEvent()
{
// This function sends Arduino's uptime every second to Virtual Pin 2.
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
  Blynk.virtualWrite(V5, float(PressureValue));
  Blynk.virtualWrite(V6, flow);
  Blynk.virtualWrite(V7,int(PumpStatus));
}
void BlynkSetup()
{
  // Blynk
  for(;;){
    //Wait for wifi to be connected before setting up the connexion to Blynk
    if(WiFi.status() == WL_CONNECTED){
      Blynk.config(auth,"blynk.cloud", 80);
      // Setup a function to be called every second
      timer.setInterval(1000L, myTimerEvent);
      break;
    }

  }

}

// ----- Pressure Functions -----
void bubbleSort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        for(int o=0; o<(size-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    int t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}
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
  int NumberofReadings = 10;
  int PressureReading[NumberofReadings];
  //Get a set of values
  for (byte i = 0; i<NumberofReadings; i++ )
  {
    PressureReading[i] = analogRead(PinSensor);
    delay(100/NumberofReadings);
  }
  int PressureSorted[NumberofReadings];
  bubbleSort(PressureReading,NumberofReadings);
  for (byte i=3; i<NumberofReadings-3;i++){
    pressure=pressure+PressureReading[i];
    // Serial.print(i);Serial.print(" - ");Serial.print(PressureReading[i]);Serial.print(" ; ");
  }
  pressure = pressure/(NumberofReadings-6); //average the readings
  // Serial.println();Serial.print("Average Reading: ");Serial.println(pressure);
  pressure = pressure*4.8/4095; //voltage
  // Serial.print("Voltage =");Serial.print(PressureReading[1]);Serial.print(" - ");Serial.print(PressureReading[2]);Serial.print(" - ");Serial.print(PressureReading[3]);Serial.print(" -  ");Serial.print(pressure);Serial.println(" V ");
  pressure = PressureCalibrationA*pressure+PressureCalibrationB;// Pressure in PSI
  // Serial.print("Pressure: ");Serial.print(pressure);Serial.println(" PSI");
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
  PumpStartTime = millis();
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
  if(PumpStatus == 0) {// Pump is Stopped
    if (PressureValue<1.5){
      StartPump();
    } 
  } else if (PumpStatus==1){// Pump is running
    PumpRunningTime=millis()-PumpStartTime;
    Serial.print("[Pump] Running time (s): ");Serial.println(PumpRunningTime);
    if (PumpRunningTime>5000){
      if(flow<1){
        StopPump();
      }
    }
  }
}


BLYNK_WRITE(V10){
  BlynkPumpSwitch = param.asInt();
}
// ====== Loops MultiCore ======
// ------ Loop Pump ------------
void loop1(void *pvParameters){
  while (1) {
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      flow = GetFlow(previousMillis);
      displayFlow(flow);
      PressureValue = GetPressure(PRESSURESENSOR);
      displayPressure(PressureValue);
      previousMillis = millis();
      // Serial.print("V10: ");Serial.print(BlynkPumpSwitch);Serial.print(" - Wifi Status: ");Serial.println(Blynk.connected());
      Serial.println("---------------------------------------------------------");
      PumpManagement();
    }
  }
}
// ----- Loop Blynk -------------
void loop2(void *pvParameters){
  while (1) {
    currentMillis = millis();
    if(WiFi.status() == WL_CONNECTED){
      Blynk.run();
      timer.run();
    }
  }
}
// ----- Loop to keep the wifi connection -----
void keepWiFiAlive(void * parameter){
    for(;;){
        if(WiFi.status() == WL_CONNECTED){
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            Serial.println("[Wifi] Connected");           
            continue;
        }
        digitalWrite(WifiPin,LOW);
        WiFi.begin(ssid, pass);
        Serial.print("[Wifi] Connecting ..");
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
          Serial.print('.');
          delay(1000);
        }
        // When we couldn't make a WiFi connection (or the timeout expired)
		  // sleep for a while and then retry.
        if(WiFi.status() != WL_CONNECTED){
            Serial.println("[WIFI] Connection FAILED");
            vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
			  continue;
        }

        Serial.println("[WIFI] Connected: " + WiFi.localIP());
    }
}

//==========================================
//================ Setup ===================
//==========================================
void setup()
{
  Serial.begin(9600);
  previousMillis = 0;
  SetupFlow();
  SetupPressure();
  SetupPump();
  //PumpStartCycle();
  xTaskCreatePinnedToCore(loop1, "loop1", 16000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(keepWiFiAlive, "keepWiFiAlive", 5000, NULL, 9, NULL, 1);
  BlynkSetup();
  xTaskCreatePinnedToCore(loop2, "loop2", 16000, NULL, 10, NULL, 1);
  
}


//=========================================
//================ Loop ===================
//=========================================
void loop()
{
  
}
