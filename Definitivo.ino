#include <MAX3010x.h>
#include "filters.h"
#include <Wire.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

SCD4x mySensor(SCD4x_SENSOR_SCD41);

MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;
const float kEdgeThreshold = -2000.0;
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

const char* ssid     = "MIEL";
const char* password = "Androide2000";
const char* serverName = "https://192.168.0.107/php_data.php";
String apiKeyValue = "781A";
String F_Cardiaca = "150";
String F_Respiratoria = "100";
String P_Arterial = "50";
String O_Sangre = "100";
String EKG="150";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  randomSeed(9);
  pinMode(16,INPUT);
  pinMode(17,INPUT);
  pinMode(5,OUTPUT);
  
  Wire.begin();
   if (mySensor.begin(false, true, false) == false){
    Serial.println(F("Sensor not detected. Please check wiring. Freezing..."));
    while (1);
  }
  bool success = mySensor.measureSingleShot();
  if (success == false){
    Serial.println(F("measureSingleShot failed. Are you sure you have a SCD41 connected? Freezing..."));
    while (1);    
  }
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    digitalWrite(5,LOW);
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }

}

LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

long last_heartbeat = 0;

long finger_timestamp = 0;
bool finger_detected = false;

float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void loop() {
  P_Arterial=String(random(60,90));
  CO2();//F_Resp
  Corazon();//EKG
  SpO2_Heart();//F_Card,O_Sang
  Conexion();
  delay(5000);
}

void Conexion(){
  if(WiFi.status()== WL_CONNECTED){
    WiFiClientSecure *client = new WiFiClientSecure;
    client->setInsecure();
    HTTPClient https;
    https.begin(*client, serverName);
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String httpRequestData = "api_key=" + apiKeyValue + "&P_Arterial=" + P_Arterial
                          + "&F_Cardiaca=" + F_Cardiaca + "&F_Respiratoria=" + F_Respiratoria
                          + "&O_Sangre=" + O_Sangre + "&EKG=" + EKG + "";
    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);
   
    int httpResponseCode = https.POST(httpRequestData);
    
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    https.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
}

void CO2(){
  while (mySensor.readMeasurement() == false){
    delay(500);
  }
  F_Cardiaca=String(mySensor.getCO2());
  mySensor.measureSingleShotRHTOnly();
  while (mySensor.readMeasurement() == false) // readMeasurement will return true when fresh data is available
  {
    delay(5);
  }  
  mySensor.measureSingleShot();
}

void Corazon(){
  if((digitalRead(16) == 1)||(digitalRead(15) == 1)){
    Serial.println('!');
  }
  else{
    EKG=String(analogRead(A0));
  }
}
void SpO2_Heart(){
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);
    if(!isnan(current_diff) && !isnan(last_diff)) {      
      // Detect Heartbeat - Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }      
      if(current_diff > 0) {
        crossed = false;
      }
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          int bpm = 60000/(crossed_time - last_heartbeat);
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          float r = rred/rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          
          if(bpm > 50 && bpm < 250) {
            if(kEnableAveraging) {
              int average_bpm = averager_bpm.process(bpm);
              int average_r = averager_r.process(r);
              int average_spo2 = averager_spo2.process(spo2);
              if(averager_bpm.count() >= kSampleThreshold) {
                //Serial.print("Heart Rate (avg, bpm): ");
                //Serial.println(average_bpm);
                //Serial.print("SpO2 (avg, %): ");
                //Serial.println(average_spo2);  
              }
            }
            else {
              //Serial.print("Heart Rate (current, bpm): ");
              F_Cardiaca=String(bpm);  
              //Serial.print("SpO2 (current, %): ");
              O_Sangre=String(spo2);   
            }
          }
          stat_red.reset();
          stat_ir.reset();
        }
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    last_diff = current_diff;
  }
}
