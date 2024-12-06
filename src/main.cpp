#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "SPIFFS.h"
#include <ESPmDNS.h>

// https://sandboxelectronics.com/?p=165
/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (36)     //define which analog input channel you are going to use
#define         RL_VALUE                     (1)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


// WiFi credentials
const char *mdns = "esp32gas";
const char *ssid = "####";
const char *password = "$$$$$";

// Create an instance of the server
AsyncWebServer server(80);

unsigned long lastUpdateTime = 0;
int currentAdcValue = 0;
double dataPointsLPG[60];
double dataPointsCO[60];
double dataPointsSmoke[60];

float MQCalibration(int mq_pin);
float MQResistanceCalculation(int raw_adc);
float MQRead(int mq_pin);
int MQGetGasPercentage(float rs_ro_ratio, int gas_id);
int  MQGetPercentage(float rs_ro_ratio, float *pcurve);

void setup()
{
  Serial.begin(115200);                               //UART setup, baudrate = 9600bps

  analogReadResolution(10);// 1023 values

  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.println();



  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi. IP:");
  Serial.println(WiFi.localIP().toString());

  // Initialize mDNS
  if (!MDNS.begin(mdns)) {   // Set the hostname to "esp32.local"
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  Serial.print("mDNS name = ");
  Serial.println(mdns);


  // Initialize data storage with random values (simulating sensor data)
  for (int i = 0; i < 60; i++) {
    dataPointsLPG[i] = 0;
    dataPointsCO[i] = 0;
    dataPointsSmoke[i] = 0;
  }

  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");

  // Set up time (for 1-hour data visualization)
  configTime(0, 0, "pool.ntp.org");  // Use NTP for time synchronization


  // Serve the index.html file
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    // Open the HTML file from LittleFS and send it as response
    if (SPIFFS.exists("/index.html")) {
      request->send(SPIFFS, "/index.html", "text/html");
    } else {
      request->send(404, "text/plain", "File not found");
    }
  });

  // Serve JSON for current value and the last hour of data
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<1024> doc;
    doc["currentAdcValue"] = currentAdcValue;
    doc["Ro"] = Ro;

    JsonArray dataLPGArray = doc.createNestedArray("dataLPG");
    JsonArray dataCOArray = doc.createNestedArray("dataCO");
    JsonArray dataSmokeArray = doc.createNestedArray("dataSmoke");
    for (int i = 0; i < 60; i++) {
      dataLPGArray.add(dataPointsLPG[i]);
      dataCOArray.add(dataPointsCO[i]);
      dataSmokeArray.add(dataPointsSmoke[i]);
    }

    JsonArray labelsArray = doc.createNestedArray("labels");
    for (int i = 0; i < 60; i++) {
      labelsArray.add(String(i));
    }

    String jsonResponse;
    serializeJson(doc, jsonResponse);
    request->send(200, "application/json", jsonResponse);
  });

  // Start the server
  server.begin();
}

void loop()
{

unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime > 1000) {
    lastUpdateTime = currentTime;

    // Shift the data points (remove the oldest and add the new one)
    for (int i = 0; i < 59; i++) {
      dataPointsLPG[i] = dataPointsLPG[i + 1];
      dataPointsCO[i] = dataPointsCO[i + 1];
      dataPointsSmoke[i] = dataPointsSmoke[i + 1];
    }
    
    // Add new data point (simulate a new sensor reading)
    currentAdcValue = analogRead(MQ_PIN);
    
    dataPointsLPG[59] = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
    dataPointsCO[59] = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
    dataPointsSmoke[59] = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);

    Serial.print("ADC:"); 
    Serial.print(analogRead(MQ_PIN));
    Serial.print("    ");   
    Serial.print("Ro:"); 
    Serial.print(Ro);
    Serial.print("    ");   
    Serial.print("LPG:"); 
    Serial.print( dataPointsLPG[59] );
    Serial.print( "ppm" );
    Serial.print("    ");   
    Serial.print("CO:"); 
    Serial.print( dataPointsCO[59] );
    Serial.print( "ppm" );
    Serial.print("    ");   
    Serial.print("SMOKE:"); 
    Serial.print( dataPointsSmoke[59] );
    Serial.print( "ppm" );
    Serial.print("\n");
  }
}

/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 

  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}