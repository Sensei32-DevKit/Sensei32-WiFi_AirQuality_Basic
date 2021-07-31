/* 
  SENSEI32 - WiFi Air Quality Station with basic integrated sensor read
  
  Obtains data from the embedded sensors, manage deep-sleep and pubblish hourly to ThingSpeak

  ####################################################################################################################################
  This software, the ideas and concepts is Copyright (c) Davide Raggini & Davide Samori 2021. All rights to this software are reserved.

  Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author Davide Raggini & Davide Samori as the source of the material.
  3. You may not, except with our express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

  The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
  software use is visible to an end-user.

  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
  OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  Contact us:   davide.raggini@gmail.com
                davide.samori@gmail.com

  ####################################################################################################################################
  This software uses ideas and concepts which are Copyright (c) David Bird 2018. All rights to this software are reserved.

  Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
  3. You may not, except with my express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

  The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
  software use is visible to an end-user.

  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
  OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  See more at http://www.dsbird.org.uk
*/

/*******************************************************************************************
 *
 * Dependencies
 *
 *******************************************************************************************/

#include <EEPROM.h> 
#include <WiFi.h>
#include <time.h>

#include <Wire.h>
#include <ClosedCube_HDC1080.h>
#include "Adafruit_BME680.h"

// Wifi Network and Thingspeak access credentials
#include "network_credentials.h"
#include "thingspeak_push.h"

/*******************************************************************************************
 *
 * Private Definitions
 *
 *******************************************************************************************/

// Define to avoid the usage of the Board LED - POWER SAVING
#define USE_BOARD_LED

// Define to ignore the battery voltage - DEBUG ONLY
//#define FAKE_BATTERY 

// Define to avoid pushing data to ThingSpeak - DEBUG ONLY
#define PushToThingSpeak

/*******************************************************************************************
 *
 * Global Variables
 *
 *******************************************************************************************/

/*******************************************************************************************
 *
 * Private Variables
 *
 *******************************************************************************************/

// Sensei32 Board Led
static const uint8_t BOARD_LED        = 2; 

// Sensei32 Analog Input for Li-Ion Battery Voltage
static const uint8_t ANALOG_VBAT      = 35;

// Sensei32 Analog Input for VBUS and InCharge Status Read
static const uint8_t ANALOG_VBUS      = 36;
static const uint8_t ANALOG_INCHARGE  = 39;

// Sensei32 I2C Sensors Interface
static const uint8_t I2C_SDA          = 21; // Default I2C SDA line
static const uint8_t I2C_SCL          = 22; // Default I2C SCL line

// Humdity and Temperature Sensor - HDC1080
ClosedCube_HDC1080 hdc1080;

// Humdity, Temperature, Pressure and Air Quality Sensor - BME688
// BME688 add an AI core, not yet supported in Arduino, over the BME680 functions:
// Existing libraries for BME680 are however compatible with BME688  
Adafruit_BME680 bme;
bool  BME688_InitDone = false;

// Sensei32 embedded sensors: Last Read Values 
static float env_temperature           = 0;
static float env_humidity              = 0;
static float env_pressure              = 0;
static float env_gasresistance         = 0;

// Sensei32 embedded sensors: Mean Values (hourly computed) 
static float env_temperature_mean      = 0;
static float env_humidity_mean         = 0;
static float env_pressure_mean         = 0;
static float env_gasresistance_mean    = 0;

// Sensei32 battery voltage and battery safety margin
float BATT_voltage;
float BATT_MinimumVoltage               = 2.90;

// Miscellanea and strings/variable to present time and date
bool    TimeStatus = false, Connected = true, BattLow = false, VBUS_Status = false, InCharge_Status = false;
String  time_str, date_str;
int     wifi_signal, CurrentHour = 0, CurrentMin = 0, CurrentSec = 0;
long    StartTime = 0;
const char* weekday_D[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };  //Day of the week
const char* month_M[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" }; //Month

/*******************************************************************************************
 *
 * RTC Stored Variables (retained in deep-sleep)
 *
 *******************************************************************************************/

// Miscellanea for deep-sleep managment
RTC_DATA_ATTR int bootCount                 = 0;
RTC_DATA_ATTR long OnTime_s                 = 0;
RTC_DATA_ATTR int offlineCycles             = 0;
RTC_DATA_ATTR long offlineTimeOffset_ms     = 0;

// Sensei32 embedded sensors: Number of accumulated values for mean computation
RTC_DATA_ATTR int sensorReads               = 0; 
// Sensei32 embedded sensors: Accumulators for mean computation
RTC_DATA_ATTR float env_temperature_acc     = 0;
RTC_DATA_ATTR float env_humidity_acc        = 0;
RTC_DATA_ATTR float env_pressure_acc        = 0;
RTC_DATA_ATTR float env_gasresistance_acc   = 0;
// Sensei32 embedded sensors: MAX and MIN Values (hourly computed) 
RTC_DATA_ATTR float env_temperature_max     = 0;
RTC_DATA_ATTR float env_humidity_max        = 0;
RTC_DATA_ATTR float env_pressure_max        = 0;
RTC_DATA_ATTR float env_gasresistance_max   = 0;
RTC_DATA_ATTR float env_temperature_min     = 0;
RTC_DATA_ATTR float env_humidity_min        = 0;
RTC_DATA_ATTR float env_pressure_min        = 0;
RTC_DATA_ATTR float env_gasresistance_min   = 0;

/*******************************************************************************************
 *
 * Arduino Setup
 *
 ******************************************************************************************/

/**
  * @brief	Arduino Environment Setup Callback
  *
  * @param  None
  * @retval None
  */
void setup() {
 
  // Slowdown the cpu
  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz to low the power consumption
  getCpuFrequencyMhz();   //Get CPU clock
  
  // setup stuff
  Serial.begin(115200);
  while(!Serial);
  
}

/*******************************************************************************************
 *
 * Arduino Loop
 *
 ******************************************************************************************/

/**
  * @brief	Arduino Environment Loop Callback
  *
  * @param  None
  * @retval None
  */
void loop() {
  
  // Set the start time
  StartTime = millis();

  // LED ON - Signal activity
  #ifdef USE_BOARD_LED
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, HIGH);
    Serial.println("[" + String(millis()) + "] LED ON");
  #endif

  // Read battery voltage
  ReadBatteryOCV();
  // Read Sensei32 status
  ReadVBUS();
  ReadInCharge();

  // Reset connected status
  Connected = false;

  // Reset network update flag
  bool NetworkUpdate = false;
  
  // 1) Execute only if there is enaugh voltage (this should preserve battery if not used for long time)
  //    Exception, if USB vbus is detected, execute even if the battery is under UVLO.
  if( (BATT_voltage > BATT_MinimumVoltage) | (VBUS_Status == true) ){

    // Signal to the next steps that operations can happen
    BattLow = false;

    // Sensor read happens in backgroun each 5 minutes (300s) as requested by BME688 library
    // When offlineCycles reach zero, an hour is elapsed and connected functions should be triggered
    if ( offlineCycles <= 0 ){

      // Request a NetworkUpdate
      NetworkUpdate = true;
      
      // Connect to the network to retrieve Time and Weather. Display and Push Sensor Data 
      if ( (StartWiFi() == WL_CONNECTED) && (SetupTime() == true) ) {
  
          // Signal to the next steps that internet connection is available
          Connected = true;
                
      } else {
  
          // >> Reduces power consumption as every other action do not requires the network
          StopWiFi();
          
      }
      
    }
    
    // Manage internet connection not available
    if( Connected == false ){

      // REMARK: SENSEI32 StandAlone doesn't need any function here...
      
    }

    // Manually update sensor OnTime
    // BME688 needs to be called each 300s in ULP mode, OnTime_s variable
    // is internelly used to decide if update data or not...
    OnTime_s = OnTime_s + 300;

  } else {
    
    // Signal to the next steps that operation must be suspended
    BattLow = true;
     
  }

  // 2) Manage operation modes
  if( BattLow == true ){

    // REMARK: No operation done here, BeginSleep will manage this situation...
    
  } else {

    // 3) Enaugh energy to update of the sensor measurments (mean values and so on...)
    // The value pushed to Thingspeak are the "mean" one, valid after calling ComputeMeanSensors()
    InitialiseSensors();

    // When offlineCycles reach zero, an hour is elapsed and connected functions should be triggered
    if( NetworkUpdate == true ){

      // 4) Manage Connected Status
      if( Connected == true ){
        
        // b) Full update every 60 minutes with sensor data push to cloud (TS)
        
        // wifi client object
        WiFiClient client;
        
        // Compute mean sensor values
        ComputeMeanSensors();
        #ifdef PushToThingSpeak
          // Update Readings to TS
          ThingSpeakSend(client, env_temperature_mean, env_humidity_mean, env_pressure_mean, env_gasresistance_mean, BATT_voltage);
        #endif
        
      } else {

        // REMARK: No operation done here, BeginSleep will manage this situation...
        
      }
    }
  }
  
  // Manage Deep-Sleep
  BeginSleep( TimeStatus, Connected, BattLow );
  
}

/*******************************************************************************************
 *
 * Private Functions
 *
 *******************************************************************************************/

/**
  * @brief	Function that manage Deep-Sleep
  *
  * @param  [IN] bool isTimeGood
  *         [IN] bool isConnected
  *         [IN] bool isBattLow
  * @retval None
  */
void BeginSleep( bool isTimeGood, bool isConnected, bool isBattLow ) {

  #ifdef USE_BOARD_LED
    pinMode(BOARD_LED, INPUT); // If it's On, turn it off and some boards use GPIO-5 for SPI-SS, which remains low after screen use
    digitalWrite(BOARD_LED, LOW);
    Serial.println("[" + String(millis()) + "] LED OFF");
  #endif

  long sleepTime;
  float awakeTime_s;

  // Compute awake time and increment boot counter
  awakeTime_s = (millis() - StartTime) / 1000.0;
  bootCount = bootCount + 1;

  // Switch operating modes    
  if( isBattLow == true ){

    // Must go to deepsleep without a timeout to preserve battery
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_36,1); //1 = High, Hardware pull-down on VBUS
    esp_deep_sleep_start();
    
  } else {

    Serial.println("[Sensei32 - Core] Awake for: " + String(awakeTime_s, 3) + "s");

    if( offlineCycles <= 0 ){

      // We have plenty of battery to keep the standard loop
      // > Different sleep times based on the fact that the
      //   network link is UP or DOWN
      if( isConnected == true ){

        if( isTimeGood == true ){

          long offlineTimeOffset_ms_tmp = 0;
                    
          // Compute nuber of offline cycles (5minutes each) until next hour
          offlineCycles = ((60 - CurrentMin) / 5) % 12; 
          if( offlineCycles <= 0 ){
            // We missed the hour mark by less than 5 minutes
            // This can be recovered by the offlineTimeOffset_ms
            offlineCycles = 12 - 1; // - 1, Counting towards zero
          } 

          // Avoid divide by zero when computing offlineTimeOffset_ms
          if( offlineCycles > 0 ){
            // Compute number the cycle by cycle time offset to recover the time drift
            // We want to sync the wakeup times with the NTP time (hourly)
            offlineTimeOffset_ms_tmp = ( (CurrentMin % 5) * 60 * 1000 ) + ( CurrentSec * 1000 );
            
            if( (offlineTimeOffset_ms_tmp >= 150000) & (offlineTimeOffset_ms_tmp < 300000) ){
              // Exception: If we go faster, we need to skip one offlineCycle with compensation
              offlineCycles = offlineCycles - 1;
              // The value is more than 2.5min (150000ms). This means that we our timer is
              // counting slower than NTP, so we need to add the offset... 
              if( offlineCycles <= 0 ){
                // All the compensation will go in one cycle
                offlineTimeOffset_ms = (300000 - offlineTimeOffset_ms_tmp);
              } else {
                // Standard split
                offlineTimeOffset_ms = (300000 - offlineTimeOffset_ms_tmp) / (offlineCycles + 1);
              }
              
            } else if( offlineTimeOffset_ms_tmp <= 150000 ){
              // The value is less than 2.5min (150000ms). This means that we our timer is
              // counting faster than NTP, so we need to subtract the offset... 
              offlineTimeOffset_ms = (0 - offlineTimeOffset_ms_tmp) / (offlineCycles + 1);
              
            } else {
  
              // An error has occurred
              offlineTimeOffset_ms = 0;
                        
            }
            
          } else {
            
            // An error has occurred
            offlineTimeOffset_ms = offlineTimeOffset_ms_tmp;
            
          }

          Serial.println("[Sensei32 - Core] Computed offlineCycles #: " + String(offlineCycles) + ", Total drift from NTP: " + String(offlineTimeOffset_ms_tmp) + "ms, offline time offset per cycle:" +  String(offlineTimeOffset_ms) + "ms" );
           
        } else {

          // There is internet connection but NTP is not available, there is no mean to compute the time drift "offlineTimeOffset_ms"
          // So it will be assumed equal to zero. Also, there is no mean to know how many offline cycles are needed to
          // sync the wakeup times with the NTP time (hourly), so "offlineCycles" is simply set to 12 == 1 hour in 5 minutes steps
          offlineCycles = 12 - 1; // - 1, Counting towards zero
          offlineTimeOffset_ms = 0;

          Serial.println("[Sensei32 - Core] NTP server not found: offlineCycles #: " + String(offlineCycles) + ", offline time offset per cycle:" +  String(offlineTimeOffset_ms) + "ms" );
        }
        
      } else {
        
        // If no internet connection is available there is no mean to compute the time drift "offlineTimeOffset_ms"
        // So it will be assumed equal to zero. Also, there is no mean to know how many offline cycles are needed to
        // sync the wakeup times with the NTP time (hourly), so "offlineCycles" is simply set to 12 == 1 hour in 5 minutes steps
        offlineCycles = 12 - 1; // - 1, Counting towards zero
        offlineTimeOffset_ms = 0;

        Serial.println("[Sensei32 - Core] Network not found: offlineCycles #: " + String(offlineCycles) + ", offline time offset per cycle:" +  String(offlineTimeOffset_ms) + "ms" );
      }
    
    } else {

      // Decrement the number of offline cycles
      offlineCycles = offlineCycles - 1;
      
    }

    // Compute the sleep timeout to 5 minute
    // BME688 in ULP mode need a run every 300s to keep accurate readings
    sleepTime = 302 - (int)awakeTime_s; // 300 + 2 second sleep for roundings - awakeTime_s
    Serial.println("[Sensei32 - Core] Starting " + String(sleepTime) + "s + (" + String(offlineTimeOffset_ms) + ")ms of deep-sleep, bootCycle #:" + String(bootCount) + ", offlineCycle to next Uplink #:" + String(offlineCycles) +"...");
    // Start the light-sleep cycle
    esp_sleep_enable_timer_wakeup( (sleepTime  * 1000000LL) + (offlineTimeOffset_ms * 1000LL) ); 
    esp_deep_sleep_start();  // Sleep for e.g. 30 minutes
    
  }
  
}

/**
  * @brief	Function that starts WiFi subsystem
  *
  * @param  None
  * @retval None
  */
uint8_t StartWiFi() {
  Serial.println("[" + String(millis()) + "] Connecting to: " + String(ssid));
  IPAddress dns(8, 8, 8, 8); // Google DNS
  WiFi.disconnect();
  WiFi.mode(WIFI_STA); // switch off AP
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  uint8_t connectionStatus;
  bool AttemptConnection = true;
  while (AttemptConnection) {
    connectionStatus = WiFi.status();
    if (millis() > start + 15000) { // Wait 15-secs maximum
      AttemptConnection = false;
    }
    if (connectionStatus == WL_CONNECTED || connectionStatus == WL_CONNECT_FAILED) {
      AttemptConnection = false;
    }
    delay(50);
  }
  if (connectionStatus == WL_CONNECTED) {
    wifi_signal = WiFi.RSSI(); // Get Wifi Signal strength now, because the WiFi will be turned off to save power!
    Serial.println("[" + String(millis()) + "] WiFi connected at: " + WiFi.localIP().toString());
  }
  else Serial.println("[" + String(millis()) + "] WiFi connection *** FAILED ***");
  return connectionStatus;
}

/**
  * @brief	Function that stops WiFi subsystem
  *
  * @param  None
  * @retval None
  */
void StopWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

/**
  * @brief	Function that setup local time from NTP server
  *
  * @param  None
  * @retval bool TimeStatus
  */
boolean SetupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov"); //(gmtOffset_sec, daylightOffset_sec, ntpServer)
  setenv("TZ", Timezone, 1);  //setenv()adds the "TZ" variable to the environment with a value TimeZone, only used if set to 1, 0 means no change
  tzset(); // Set the TZ environment variable
  delay(100);
  TimeStatus = UpdateLocalTime();
  return TimeStatus;
}

/**
  * @brief	Worker Function that get and decode the Local Time
  *
  * @param  None
  * @retval bool TimeStatus
  */
boolean UpdateLocalTime() {
  struct tm timeinfo;
  char   time_output[30], day_output[30], update_time[30];
  while (!getLocalTime(&timeinfo, 5000)) { // Wait for 5-sec for time to synchronise
    Serial.println("[" + String(millis()) + "] TimeServer: Failed to obtain time");
    return false;
  }
  CurrentHour = timeinfo.tm_hour;
  CurrentMin  = timeinfo.tm_min;
  CurrentSec  = timeinfo.tm_sec;

  //See http://www.cplusplus.com/reference/ctime/strftime/
  //Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S");      // Displays: Saturday, June 24 2017 14:05:49
  sprintf(day_output, "%s  %02u-%s-%04u", weekday_D[timeinfo.tm_wday], timeinfo.tm_mday, month_M[timeinfo.tm_mon], (timeinfo.tm_year) + 1900);
  strftime(update_time, sizeof(update_time), "%H:%M:%S", &timeinfo);  // Creates: '@ 14:05:49'   and change from 30 to 8 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  sprintf(time_output, "%s", update_time);

  date_str = day_output;
  time_str = time_output;
  Serial.println("[" + String(millis()) + "] TimeServer: Human Time: " + date_str + ", " + time_str);
  time_t now_time;
  time(&now_time);
  
  return true;
}

/**
  * @brief  Function to get the Li-Ion battery voltage
  *
  * @param  None
  * @retval None
  */
void ReadBatteryOCV() {
  #ifdef FAKE_BATTERY
    BATT_voltage = 3.6f;
  #else
    // Battery should be read as first thing (so it will be similar to OCV value...)
    BATT_voltage = analogRead(ANALOG_VBAT) / 4096.0 * 6.77395927613 * 0.8843421307;  //(1/0.48715970461) * 3.3 - R1= 470K, R2=(680K//1.3M_leak)
                                                                                     //fine_tuning offset was added: 0.8843421307
  #endif
  Serial.println("[Sensei32 - Core] BATT Voltage = " + String(BATT_voltage));
}

/**
  * @brief  Function to get USB BUS Voltage
  *
  * @param  None
  * @retval None
  */
void ReadVBUS() {
  float VBUS_voltage = analogRead(ANALOG_VBUS) / 4096.0 * 3.3 * 2.076923077;  // R1 = 100K, R2 = (100K//1.3M_leak)
  Serial.println("[Sensei32 - Core] VBUS Voltage = " + String(VBUS_voltage));
  // On Sensei32 VBUS is read with a 1/2 divider from USB VBUS
  // Digital status is computed by evaluating the pin voltage
  if( VBUS_voltage >= 3.3 ){
    VBUS_Status = true;
  } else {
    VBUS_Status = false;
  }
  Serial.println("[Sensei32 - Core] VBUS State Digital State = " + String(VBUS_Status) );
}

/**
  * @brief  Function to get the charge status of the Li-Ion Battery
  *
  * @param  None
  * @retval None
  */
void ReadInCharge() {
  float InCharge_voltage = analogRead(ANALOG_INCHARGE) / 4096.0 * 3.3 * 2.098461538;  // R1 = 102K, R2 = (100K//1.3M_leak)
  Serial.println("[Sensei32 - Core] InCharge Voltage = " + String(InCharge_voltage));
  // On Sensei32 InCharge_Status is read with a 1/2 divider from LiIon Charge Pin
  // Digital status is computed by evaluating the pin voltage, with inverted logic
  if( InCharge_voltage >= 3.3 ){
    InCharge_Status = false;
  } else {
    InCharge_Status = true;
  }
  Serial.println("[Sensei32 - Core] InCharge Digital State = " + String(InCharge_Status) );
}

/**
  * @brief	Function to initialise and read Sensei32 embedded sensors
  *
  * @param  None
  * @retval None
  */
void InitialiseSensors() {
    
    // Default settings: 
    //  - Heater off
    //  - 14 bit Temperature and Humidity Measurement Resolutions
    hdc1080.begin(0x40);
    Serial.println("[" + String(millis()) + "] HDC1080: Init: Done");

    // Read the Temperature and humidity sensor
    env_temperature = hdc1080.readTemperature();
    env_humidity = hdc1080.readHumidity();
    Serial.println("[" + String(millis()) + "] HDC1080: Temperature: " + String(env_temperature) + "*C");
    Serial.println("[" + String(millis()) + "] HDC1080: Humidity:    " + String(env_humidity) + "%");
    
    // Accumulate last value to later compute mean value
    env_temperature_acc = env_temperature_acc + env_temperature;
    env_humidity_acc    = env_humidity_acc + env_humidity;

    // Adafruit simple BME688 sensor read
    if (!bme.begin(0x76)) {
      Serial.println("[" + String(millis()) + "] BME688: Error: Cannot perform read!");
    } else {
      Serial.println("[" + String(millis()) + "] BME688: Init: Done");
      // Set up oversampling and filter initialization
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(320, 150); // 320*C for 150 ms
      // Read sample
      if (!bme.performReading()) {
        Serial.println("Failed to perform reading :(");
      } else {
        Serial.println("[" + String(millis()) + "] BME688: Temperature = " + String(bme.temperature) + " *C");
        Serial.println("[" + String(millis()) + "] BME688: Humidity    = " + String(bme.humidity) + " %");
        Serial.println("[" + String(millis()) + "] BME688: Pressure    = " + String(bme.pressure / 100.0) + " hPa");
        Serial.println("[" + String(millis()) + "] BME688: Gas         = " + String(bme.gas_resistance) + " Ohms");
      }
      // Temperature and Humidity are retrieved by HDC1080 which is more accurate
      env_pressure = bme.pressure / 100.0;
      env_gasresistance = bme.gas_resistance;
      // Accumulate last value to later compute mean value
      env_pressure_acc = env_pressure_acc + env_pressure;
      env_gasresistance_acc = env_gasresistance_acc + env_gasresistance;
    }

    // Compute min-max values
    if( sensorReads <= 0 ){
      env_temperature_max = env_temperature;
      env_temperature_min = env_temperature;
      env_humidity_max = env_humidity;
      env_humidity_min = env_humidity;
      env_pressure_max = env_pressure;
      env_pressure_min = env_pressure;
      env_gasresistance_max = env_gasresistance;
      env_gasresistance_min = env_gasresistance;
    } else {
      if( env_temperature > env_temperature_max ){ env_temperature_max = env_temperature; } else if( env_temperature < env_temperature_min ){ env_temperature_min = env_temperature; }
      if( env_humidity > env_humidity_max ){ env_humidity_max = env_humidity; } else if( env_humidity < env_humidity_min ){ env_humidity_min = env_humidity; }
      if( env_pressure > env_pressure_max ){ env_pressure_max = env_pressure; } else if( env_pressure_min < env_pressure_min ){ env_pressure_min = env_pressure; }
      if( env_gasresistance > env_gasresistance_max ){ env_gasresistance_max = env_gasresistance; } else if( env_gasresistance < env_gasresistance_min ){ env_gasresistance_min = env_gasresistance; }
    }
    
    // Add a reading point
    sensorReads = sensorReads + 1;
    
}

/**
  * @brief	Function to compute Sensei32 embedded sensors mean values
  *
  * @param  None
  * @retval None
  */
void ComputeMeanSensors() {

  // Compute mean values
  env_temperature_mean = env_temperature_acc / sensorReads;
  env_humidity_mean = env_humidity_acc / sensorReads;
  env_pressure_mean = env_pressure_acc / sensorReads;
  env_gasresistance_mean = env_gasresistance_acc / sensorReads;

  // Reset used variables
  sensorReads             = 0;
  env_temperature_acc     = 0;
  env_humidity_acc        = 0;
  env_pressure_acc        = 0;
  env_gasresistance_acc   = 0;

  // Output mean value
  Serial.println("[" + String(millis()) + "] mean Temperature       = " + String(env_temperature_mean) + " *C");
  Serial.println("[" + String(millis()) + "] mean Relative Humidity = " + String(env_humidity_mean) + " %");
  Serial.println("[" + String(millis()) + "] mean Pressure          = " + String(env_pressure_mean) + " hPa");
  Serial.println("[" + String(millis()) + "] mean Gas Resistance    = " + String(env_gasresistance_mean) + " Ohms");
  
}
