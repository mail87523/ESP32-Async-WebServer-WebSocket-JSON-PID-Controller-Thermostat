
/***********************************************************************************************/
/*** ESP32 ASYNC WEBSOCKET PID THERMOSTAT ******************************************************/
/***********************************************************************************************/
#include <Arduino.h>

#include <EEPROM.h>
#include <SPIFFS.h>

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP280.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include <RBDdimmer.h>
#include <PID.h>

/***********************************************************************************************/
/*** LED GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN ********************************************/
/***********************************************************************************************/
#define TEMP_LED 23
#define LED_PIN 2
#define LED_ON 0x01
#define LED_OFF 0x00
/***********************************************************************************************/
/*** LED GLOBAL DEFINE VARIABLE AND FUNCTIONS END **********************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** IO GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN *********************************************/
/***********************************************************************************************/
bool I_01 = false;
bool I_02 = false;
bool I_03 = false;
bool I_04 = false;
bool I_05 = false;
bool I_06 = false;
bool I_07 = false;
bool I_08 = false;

#define I_01_PIN 18
#define I_02_PIN 19
#define I_03_PIN 23
#define I_04_PIN 27
#define I_05_PIN 32
#define I_06_PIN 33
#define I_07_PIN 34
#define I_08_PIN 35

bool Q_01 = false;
bool Q_02 = false;
bool Q_03 = false;
bool Q_04 = false;
bool Q_05 = false;
bool Q_06 = false;
bool Q_07 = false;
bool Q_08 = false;

#define Q_01_PIN 2
#define Q_02_PIN 4
#define Q_03_PIN 5
#define Q_04_PIN 12
#define Q_05_PIN 13
#define Q_06_PIN 14
#define Q_07_PIN 15
#define Q_08_PIN 16

uint32_t I_Status = 0;
uint32_t Q_Status = 0;
/***********************************************************************************************/
/*** IO GLOBAL DEFINE VARIABLE AND FUNCTIONS END ***********************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** DIMMER GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN *****************************************/
/***********************************************************************************************/
#define DIMMER_OUT_PIN 25
#define DIMMER_ZC_PIN 26 

dimmerLamp Dimmer(DIMMER_OUT_PIN, DIMMER_ZC_PIN);
int DimmerOut = 0;
/***********************************************************************************************/
/*** DIMMER GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN END *************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** BMP280 GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN *****************************************/
/***********************************************************************************************/
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
Adafruit_BMP280 BMP280;

float Offset_T = 0.0f;
float Offset_P = 0.0f;
/***********************************************************************************************/
/*** BMP280 GLOBAL DEFINE VARIABLE AND FUNCTIONS END *******************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** RTOS GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN *******************************************/
/***********************************************************************************************/
#define MAIN_TASK_SIGNAL (1 << 0)
#define PID_TASK_SIGNAL (1 << 1)
#define PRINT_TASK_SIGNAL (1 << 2)
#define I_TASK_SIGNAL (1 << 3)
#define Q_TASK_SIGNAL (1 << 4)

EventGroupHandle_t TaskSignal;
SemaphoreHandle_t PID_1_Mutex;

TaskHandle_t MAIN_Task_Handle;
TaskHandle_t PID_Task_Handle;
TaskHandle_t PRINT_Task_Handle;
TaskHandle_t I_Task_Handle;
TaskHandle_t Q_Task_Handle;

static void MAIN_Task(void *parameter);
static void PID_Task(void *parameter);
static void PRINT_Task(void *parameter);
static void I_Task(void *parameter);
static void Q_Task(void *parameter);

volatile uint32_t PID_1_TaskTime;
volatile uint32_t MQTT_TaskTime;
volatile uint32_t PRINT_TaskTime;

/***********************************************************************************************/
/*** RTOS GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN END ***************************************/
/***********************************************************************************************/
/***********************************************************************************************/
/*** EEPROM GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN *****************************************/
/***********************************************************************************************/
typedef struct EepromPidParam_t
{
  bool initialized;
  float_t Setpoint;
  float_t Kp;
  float_t Ki;
  float_t Kd;
  float_t Offset;
} EepromPidParam_t;

EepromPidParam_t EepromPidParam;

#define EEPROM_SIZE 21
#define EEPROM_ADDR_OFFSET_INIT_FLAG  0
#define EEPROM_ADDR_OFFSET_SP  1
#define EEPROM_ADDR_OFFSET_KP  5
#define EEPROM_ADDR_OFFSET_KI  9
#define EEPROM_ADDR_OFFSET_KD  13
#define EEPROM_ADDR_OFFSET_OFFSET  17

/***********************************************************************************************/
/*** EEPROM GLOBAL DEFINE VARIABLE AND FUNCTIONS END *******************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** PID GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN ********************************************/
/***********************************************************************************************/
#define DEF_SP 23.0f    //Default PID Setpoint
#define DEF_KP 100.0f   //Default PID Kp gain
#define DEF_KI 35.0f    //Default PID Ki gain
#define DEF_KD 2.0f     //Default PID Kd gain
#define DEF_OFFSET 0.0f //Default Temperature Offset for BMP280

#define PID_TAU 12.5f //0.2f

#define PID_LIM_MIN 0.0f   /*Limit min for Output of PID Calculation */
#define PID_LIM_MAX 100.0f /*Limit max for Output of PID Calculation */

#define PID_LIM_MIN_INT -50.0f /*Limit min for Integral part of PID */
#define PID_LIM_MAX_INT 50.0f  /*Limit max for Integral part of PID */

#define SAMPLE_TIME_S 1.0f /*PID Loop Time in seconds */

PIDController Pid_1 = {HEATING_AUTO_STD, /*Mode of PID Controller */
                       DEF_SP,
                       DEF_KP,
                       DEF_KI,
                       DEF_KD,
                       PID_TAU,
                       PID_LIM_MIN,
                       PID_LIM_MAX,
                       PID_LIM_MIN_INT,
                       PID_LIM_MAX_INT,
                       SAMPLE_TIME_S};
/***********************************************************************************************/
/*** PID GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN END ****************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** WIFI GLOBAL DEFINE VARIABLE AND FUNCTIONS BEGIN *******************************************/
/***********************************************************************************************/
const char *WIFI_SSID = "Your Network";
const char *WIFI_PASSWORD = "Password";
const char *AP_SSID = "ESP32_THERMOSTAT_AP1";
const char *AP_PASSWORD = "123456789";

#define WIFI_MODE_STATION
#define HTTP_PORT 80

AsyncWebServer server(HTTP_PORT); // -> Web server
AsyncWebSocket ws("/ws");

/***********************************************************************************************/
/*** WIFI GLOBAL DEFINE VARIABLE AND FUNCTIONS END *********************************************/
/***********************************************************************************************/


/***********************************************************************************************/
/*** GPIO FUNCTIONS BEGIN **********************************************************************/
/***********************************************************************************************/

void initGPIO()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIMMER_OUT_PIN, OUTPUT);

  pinMode(Q_01_PIN, OUTPUT);
  pinMode(Q_02_PIN, OUTPUT);
  pinMode(Q_03_PIN, OUTPUT);
  pinMode(Q_04_PIN, OUTPUT);
  pinMode(Q_05_PIN, OUTPUT);
  pinMode(Q_06_PIN, OUTPUT);
  pinMode(Q_06_PIN, OUTPUT);
  pinMode(Q_07_PIN, OUTPUT);

  digitalWrite(LED_PIN, LED_OFF);
  digitalWrite(DIMMER_OUT_PIN, LOW);

  digitalWrite(Q_01_PIN, LOW);
  digitalWrite(Q_02_PIN, LOW);
  digitalWrite(Q_03_PIN, LOW);
  digitalWrite(Q_04_PIN, LOW);
  digitalWrite(Q_05_PIN, LOW);
  digitalWrite(Q_06_PIN, LOW);
  digitalWrite(Q_07_PIN, LOW);
  digitalWrite(Q_08_PIN, LOW);
  
  pinMode(DIMMER_ZC_PIN, INPUT);

  pinMode(I_01_PIN, INPUT);
  pinMode(I_02_PIN, INPUT);
  pinMode(I_03_PIN, INPUT);
  pinMode(I_04_PIN, INPUT);
  pinMode(I_05_PIN, INPUT);
  pinMode(I_06_PIN, INPUT);
  pinMode(I_07_PIN, INPUT);
  pinMode(I_08_PIN, INPUT);
}

/***********************************************************************************************/
/*** GPIO FUNCTIONS END ************************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** SERIAL FUNCTIONS BEGIN ********************************************************************/
/***********************************************************************************************/
void initSerial()
{
  Serial.begin(921600);
}
/***********************************************************************************************/
/*** SERIAL FUNCTIONS END **********************************************************************/
/***********************************************************************************************/
/***********************************************************************************************/
/*** EEPROM FUNCTIONS BEGIN ********************************************************************/
/***********************************************************************************************/
void Init_EEPROM()
{
  Serial.print(F("2. Initializing EEPROM ... \n"));
  if (EEPROM.begin(EEPROM_SIZE))
  {
    if (EEPROM.readByte(EEPROM_ADDR_OFFSET_INIT_FLAG))
    {
      EepromPidParam.initialized = 1;
      EepromPidParam.Setpoint = EEPROM.readFloat(EEPROM_ADDR_OFFSET_SP);
      EepromPidParam.Kp = EEPROM.readFloat(EEPROM_ADDR_OFFSET_KP);
      EepromPidParam.Ki = EEPROM.readFloat(EEPROM_ADDR_OFFSET_KI);
      EepromPidParam.Kd = EEPROM.readFloat(EEPROM_ADDR_OFFSET_KD);
      EepromPidParam.Offset = EEPROM.readFloat(EEPROM_ADDR_OFFSET_OFFSET);

      Serial.printf("   EEPROM DATA ARE INITIALAZED !!!\n");
      Serial.printf("   IF [0x%02x] = %u\n", EEPROM_ADDR_OFFSET_INIT_FLAG, EepromPidParam.initialized);
      Serial.printf("   SP [0x%02x] = %.2f\n", EEPROM_ADDR_OFFSET_SP, EepromPidParam.Setpoint);
      Serial.printf("   KP [0x%02x] = %.2f\n", EEPROM_ADDR_OFFSET_KP, EepromPidParam.Kp);
      Serial.printf("   KI [0x%02x] = %.2f\n", EEPROM_ADDR_OFFSET_KI, EepromPidParam.Ki);
      Serial.printf("   KD [0x%02x] = %.2f\n", EEPROM_ADDR_OFFSET_KD, EepromPidParam.Kd);
      Serial.printf("   OFFSET [0x%02x] = %.2f\n", EEPROM_ADDR_OFFSET_OFFSET, EepromPidParam.Offset);
    }
    else
    {
      EepromPidParam.initialized = 0;
      EepromPidParam.Setpoint = DEF_SP;
      EepromPidParam.Kp = DEF_KP;
      EepromPidParam.Ki = DEF_KI;
      EepromPidParam.Kd = DEF_KD;
      EepromPidParam.Offset = DEF_OFFSET;

      Serial.printf("   NO EEPROM DATA DEFAULT DATA ARE INITIALAZED !!!\n");
    }
  }
  else
  {
    Serial.println("error!");
  }
}

void SaveToEeprom(float_t Setpoint, float_t Kp, float_t Ki, float_t Kd, float_t Offset)
{
  bool hasToBeSaved = false;

  // Remember that `EepromPidParam` contains the values that were
  // read from the EEPROM during initialization.

  if (Setpoint != EepromPidParam.Setpoint)
  {
    EepromPidParam.Setpoint = Setpoint;
    EEPROM.writeFloat(EEPROM_ADDR_OFFSET_SP, Setpoint);
    hasToBeSaved = true;
  }

  if (Kp != EepromPidParam.Kp)
  {
    EepromPidParam.Kp = Kp;
    EEPROM.writeFloat(EEPROM_ADDR_OFFSET_KP, Kp);
    hasToBeSaved = true;
  }
  if (Ki != EepromPidParam.Ki)
  {
    EepromPidParam.Ki = Ki;
    EEPROM.writeFloat(EEPROM_ADDR_OFFSET_KI, Ki);
    hasToBeSaved = true;
  }
  if (Kd != EepromPidParam.Kd)
  {
    EepromPidParam.Kd = Kd;
    EEPROM.writeFloat(EEPROM_ADDR_OFFSET_KD, Kd);
    hasToBeSaved = true;
  }
  if (Offset != EepromPidParam.Offset)
  {
    EepromPidParam.Offset = Offset;
    EEPROM.writeFloat(EEPROM_ADDR_OFFSET_OFFSET, Offset);
    hasToBeSaved = true;
  }

  if (hasToBeSaved)
  {
    // If no storage has ever taken place in the EEPROM,
    // a control value is also stored so that it can be remembered
    // that it has actually taken place at least once.
    if (!EepromPidParam.initialized)
    {
      EEPROM.writeByte(EEPROM_ADDR_OFFSET_INIT_FLAG, 1);
      EepromPidParam.initialized = true;
    }
    // we end by actually writing in the EEPROM:
    EEPROM.commit();
    Serial.println(F("-> Has been stored in EEPROM\n"));
  }
  else
  {
    Serial.println(F("Already stored in EEPROM (no change)\n"));
  }
}

/***********************************************************************************************/
/*** EEPROM FUNCTIONS END **********************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** TEMPERATURE SENSOR BMP280 FUNCTIONS BEGIN *************************************************/
/***********************************************************************************************/
void initTempSensor()
{
  if (!BMP280.begin(0x76))
  {
    Serial.println("ERROR BMP280 sensor");
    while (1)
    {
      ;
    }
  }
  BMP280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X16,    /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X4,       /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println(F("4. BMP280 temperature sensor activated"));
}
/***********************************************************************************************/
/*** TEMPERATURE SENSOR BMP280 FUNCTIONSS END **************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** DIMMER FUNCTIONS BEGIN ********************************************************************/
/***********************************************************************************************/
void initDimmer()
{
  Dimmer.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)
  Dimmer.setPower(0);            // setPower(0-100%);
}
/***********************************************************************************************/
/*** DIMMER FUNCTIONS END **********************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** SPIFS FUNCTIONS BEGIN *********************************************************************/
/***********************************************************************************************/
/**
 * The web user interface will be stored on the ESP32 Flash memory file system
 * as 5 separate files :
 * - index.html  (the interface structure)
 * - index.css   (the graphical layout of the interface)
 * - index.js    (the dynamic interface management program)
 * - favicon.ico (the tiny icon for the browser)
 */

void initSPIFFS()
{
  if (!SPIFFS.begin())
  {
    Serial.println(F("ERROR - Cannot mount SPIFFS volume..."));
    while (1)
      digitalWrite(LED_PIN, millis() % 200 < 20 ? HIGH : LOW);
  }
  Serial.println(F("5. SPIFFS volume is mounted"));
}

/***********************************************************************************************/
/*** SIFS FUNCTIONS END ************************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** WIFI FUNCTIONS BEGIN **********************************************************************/
/***********************************************************************************************/
void initWiFi()
{
  #ifdef WIFI_MODE_STATION
    uint8_t TryCnt=0;
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("6. Trying to connect [%s] ", WiFi.macAddress().c_str());
    while (WiFi.status() != WL_CONNECTED)
    {
      TryCnt++;
      Serial.print(".");
      if (TryCnt>30) //Try to Connect 30 time (sek) if not then exit loop without connetion to router
      {
         break;
         Serial.printf("\n   ERROR - Not Connected to Wifi Network after %d time !\n", TryCnt);
      }  
      delay(1000);
    }
    Serial.printf("\n   Successfully Connected to Wifi Network after %d time with IP address:", TryCnt);
    Serial.printf(" %s\n", WiFi.localIP().toString().c_str());
  #endif

  WiFi.softAP(AP_SSID, AP_PASSWORD,1,0,4);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("   AP Successfully Established with IP address: ");
  Serial.println(IP);
}

/***********************************************************************************************/
/*** WIFI FUNCTIONS END ************************************************************************/
/***********************************************************************************************/


/***********************************************************************************************/
/*** ASYNC WEBSERVER FUNCTIONS BEGIN ***********************************************************/
/***********************************************************************************************/

// ----------------------------------------------------------------------------
// HTTP route definition & request processing
// ----------------------------------------------------------------------------

// Processing of the `index.html` template
// ---------------------------------------

/**
 * The HTML page (index.html) that is stored in SPIFFS has generic markers
 * of the form `%TAG%`. This routine is responsible for substituting these
 * markers with the actual values that correspond to them and must be included
 * in the page that is sent to the browser.
 * 
 * There are 4 of these markers:
 * - %TEMP%       (the current temperature read by the sensor)
 * - %DEF_SP%   (factory setting of the minimum temperature)
 * - %DEF_KP%   (Factory setting of the maximum temperature)
 * - %SP% (the Setpoint limit of the temperature range set by the operator)
 * - %KP% (the Kp limit of the temperature range set by the operator)
 */

String processor(const String &var)
{
  
  if (var == "TEMP")
  {
    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    float Measurement = Pid_1.Measurement;
    xSemaphoreGive(PID_1_Mutex);
    return String(Measurement, 1);
  }
  else if (var == "DEF_SP")
  {
    return String(DEF_SP, 1);
  }
  else if (var == "DEF_KP")
  {
    return String(DEF_KP, 1);
  }
  else if (var == "DEF_KI")
  {
    return String(DEF_KI, 1);
  }
  else if (var == "DEF_KD")
  {
    return String(DEF_KD, 1);
  }
  else if (var == "DEF_OFFSET")
  {
    return String(DEF_OFFSET, 1);
  }
  else if (var == "SP")
  {
    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    float Setpoint = Pid_1.Setpoint;
    xSemaphoreGive(PID_1_Mutex);
    return String(Setpoint, 1);
  }
  else if (var == "KP")
  {
    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    float Kp = Pid_1.Kp;
    xSemaphoreGive(PID_1_Mutex);
    return String(Kp, 1);
  }
  else if (var == "KI")
  {
    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    float Ki = Pid_1.Ki;
    xSemaphoreGive(PID_1_Mutex);
    return String(Ki, 1);
  }
  else if (var == "KD")
  {
    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    float Kd = Pid_1.Kd;
    xSemaphoreGive(PID_1_Mutex);
    return String(Kd, 1);
  }
  else if (var == "OFFSET")
  {
    return String(Offset_T, 1);
  }
  else if (var == "STATE_01")
  {
    return String(var == "STATE_01" && Q_01 ? "on" : "off");
  }
  else if (var == "STATE_02")
  {
    return String(var == "STATE_02" && Q_02 ? "on" : "off");
  }
  else if (var == "STATE_03")
  {
    return String(var == "STATE_03" && Q_03 ? "on" : "off");
  }
  else if (var == "STATE_04")
  {
    return String(var == "STATE_04" && Q_04 ? "on" : "off");
  }
  else if (var == "STATE_05")
  {
    return String(var == "STATE_05" && Q_05 ? "on" : "off");
  }
  else if (var == "STATE_06")
  {
    return String(var == "STATE_06" && Q_06 ? "on" : "off");
  }
  else if (var == "STATE_07")
  {
    return String(var == "STATE_07" && Q_07 ? "on" : "off");
  }
  else if (var == "STATE_08")
  {
    return String(var == "STATE_08" && Q_08 ? "on" : "off");
  }
  return String();
  
}


/**
 * When the browser requests access to the main page `index.html`,
 * the server must first replace the generic markers declared above
 * with their respective values.
 */

void onRootRequest(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/index.html", "text/html", false, processor);
}


void onNotFound(AsyncWebServerRequest *request)
{
  request->send(404);
}

void onTemp(AsyncWebServerRequest *request)
{
  xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
  float Measurement = Pid_1.Measurement;
  xSemaphoreGive(PID_1_Mutex);

  if (isnan(Measurement))
  {
    request->send(200, "text/plain", String("Error"));
  }
  else
  {
    request->send(200, "text/plain", String(Measurement));
  }
  
}

void onDefault(AsyncWebServerRequest *request)
{
  xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
  Pid_1.Setpoint = DEF_SP;
  Pid_1.Kp = DEF_KP;
  Pid_1.Ki = DEF_KI;
  Pid_1.Kd = DEF_KD;
  Offset_T = DEF_OFFSET;
  Pid_1.Integrator = 0.0f;
  
  Serial.printf("...Factory reset...\n");
  Serial.printf("...PID Parameters is set to:\n");
  Serial.printf("...Sp = %.2f\n", Pid_1.Setpoint);
  Serial.printf("...Kp = %.2f\n", Pid_1.Kp);
  Serial.printf("...Ki = %.2f\n", Pid_1.Ki);
  Serial.printf("...Kd = %.2f\n", Pid_1.Kd);
  Serial.printf("...Offset = %.2f\n", Offset_T);
  
  xSemaphoreGive(PID_1_Mutex);
  
  request->send(200);
  
}

void onReboot(AsyncWebServerRequest *request)
{
  request->send(200);
  Serial.println(F("...Rebooting...\n"));
  Serial.flush();
  ESP.restart();
}

void onUpload(AsyncWebServerRequest *request)
{
  if (request->hasParam("Setpoint") && request->hasParam("Kp") && request->hasParam("Ki") && request->hasParam("Kd") && request->hasParam("Offset"))
  {
    float Setpoint = request->getParam("Setpoint")->value().toFloat();
    float Kp = request->getParam("Kp")->value().toFloat();
    float Ki = request->getParam("Ki")->value().toFloat();
    float Kd = request->getParam("Kd")->value().toFloat();
    float Offset = request->getParam("Offset")->value().toFloat();

    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    Pid_1.Setpoint = Setpoint;
    Pid_1.Kp = Kp;
    Pid_1.Ki = Ki;
    Pid_1.Kd = Kd;
    Offset_T = Offset;
    Pid_1.Integrator = 0.0f;
    xSemaphoreGive(PID_1_Mutex);

    Serial.printf("...PID Parameters is set:\n");
    Serial.printf("...Sp = %.2f\n", Setpoint);
    Serial.printf("...Kp = %.2f\n", Kp);
    Serial.printf("...Ki = %.2f\n", Ki);
    Serial.printf("...Kd = %.2f\n", Kd);
    Serial.printf("...Offset = %.2f\n", Offset);
  }
  request->send(200);
}

void onSave(AsyncWebServerRequest *request)
{
  
  if (request->hasParam("Setpoint") && request->hasParam("Kp") && request->hasParam("Ki") && request->hasParam("Kd") && request->hasParam("Offset"))
  {

    float Setpoint = request->getParam("Setpoint")->value().toFloat();
    float Kp = request->getParam("Kp")->value().toFloat();
    float Ki = request->getParam("Ki")->value().toFloat();
    float Kd = request->getParam("Kd")->value().toFloat();
    float Offset = request->getParam("Offset")->value().toFloat();

    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    Pid_1.Setpoint = Setpoint;
    Pid_1.Kp = Kp;
    Pid_1.Ki = Ki;
    Pid_1.Kd = Kd;
    Offset_T = Offset;
    Pid_1.Integrator = 0.0f;
    xSemaphoreGive(PID_1_Mutex);

    SaveToEeprom(Setpoint, Kp, Ki, Kd, Offset);

    Serial.printf("...PID Parameters is set:\n");
    Serial.printf("...Sp = %.2f\n", Setpoint);
    Serial.printf("...Kp = %.2f\n", Kp);
    Serial.printf("...Ki = %.2f\n", Ki);
    Serial.printf("...Kd = %.2f\n", Kd);
    Serial.printf("...Offset = %.2f\n", Offset);   
  }
    request->send(200);
    
}

void initWebServer()
{
  server.on("/", onRootRequest);

  server.serveStatic("/script.js", SPIFFS, "/script.js");
  server.serveStatic("/style.css", SPIFFS, "/style.css");
  server.serveStatic("/favicon.ico", SPIFFS, "/favicon.ico");

  server.onNotFound(onNotFound);

  server.on("/temp", onTemp);
  server.on("/default", onDefault);
  server.on("/reboot", onReboot);
  server.on("/upload", onUpload);
  server.on("/save", onSave);

  server.begin();
  Serial.println(F("8. Web server started"));
}

/***********************************************************************************************/
/*** ASYNC WEBSERVER FUNCTIONS END *************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** WEBSOCKET FUNCTIONS BEGIN *****************************************************************/
/***********************************************************************************************/

void notifyClients(uint8_t temp)
{
  const uint8_t size = JSON_OBJECT_SIZE(1);
  StaticJsonDocument<size> json;
  Q_Status = 0;
  Q_Status = Q_Status | (Q_01 << 0);
  Q_Status = Q_Status | (Q_02 << 1);
  Q_Status = Q_Status | (Q_03 << 2);
  Q_Status = Q_Status | (Q_04 << 3);
  Q_Status = Q_Status | (Q_05 << 4);
  Q_Status = Q_Status | (Q_06 << 5);
  Q_Status = Q_Status | (Q_07 << 6);
  Q_Status = Q_Status | (Q_08 << 7);

  json["status"] = Q_Status;

  char buffer[32];
  size_t len = serializeJson(json, buffer);
  ws.textAll(buffer, len);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {

    const uint8_t size = JSON_OBJECT_SIZE(1);
    StaticJsonDocument<size> json;
    DeserializationError err = deserializeJson(json, data);
    if (err)
    {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(err.c_str());
      return;
    }

    const char *action = json["action"];

    if (strcmp(action, "q01") == 0)
    {
      Q_01 = !Q_01;
      notifyClients(1);
    }
    else if (strcmp(action, "q02") == 0)
    {
      Q_02 = !Q_02;
      notifyClients(2);
    }
    else if (strcmp(action, "q03") == 0)
    {
      Q_03 = !Q_03;
      notifyClients(3);
    }
    else if (strcmp(action, "q04") == 0)
    {
      Q_04 = !Q_04;
      notifyClients(4);
    }
    else if (strcmp(action, "q05") == 0)
    {
      Q_05 = !Q_05;
      notifyClients(5);
    }
    else if (strcmp(action, "q06") == 0)
    {
      Q_06 = !Q_06;
      notifyClients(6);
    }
    else if (strcmp(action, "q07") == 0)
    {
      Q_07 = !Q_07;
      notifyClients(7);
    }
    else if (strcmp(action, "q08") == 0)
    {
      Q_08 = !Q_08;
      notifyClients(8);
    }
  }
}

void onEvent(AsyncWebSocket *server,
             AsyncWebSocketClient *client,
             AwsEventType type,
             void *arg,
             uint8_t *data,
             size_t len)
{

  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}


/***********************************************************************************************/
/*** WEBSOCKET FUNCTIONS END *******************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** SETUP RTOS BEGIN **************************************************************************/
/***********************************************************************************************/
void initRTOS()
{
TaskSignal = xEventGroupCreate();
PID_1_Mutex = xSemaphoreCreateMutex(); // MUTually EXclusive
xTaskCreatePinnedToCore(MAIN_Task, "MAIN_Task", 2000, NULL, 2, &MAIN_Task_Handle, 1);
xTaskCreatePinnedToCore(PID_Task, "PID_Task", 2000, NULL, 2, &PID_Task_Handle, 1);
xTaskCreatePinnedToCore(PRINT_Task, "PRINT_Task", 2000, NULL, 2, &PRINT_Task_Handle, 1);
xTaskCreatePinnedToCore(I_Task, "I_Task", 2000, NULL, 2, &I_Task_Handle, 1);
xTaskCreatePinnedToCore(Q_Task, "Q_Task", 2000, NULL, 2, &Q_Task_Handle, 1);
}
/***********************************************************************************************/
/*** SETUP RTOS END ****************************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** PID FUNCTIONS BEGIN ***********************************************************************/
/***********************************************************************************************/
void Init_PID_Parameters()
{
  if (EepromPidParam.initialized == 1)
  {
    Pid_1.Setpoint = EepromPidParam.Setpoint;
    Pid_1.Kp = EepromPidParam.Kp;
    Pid_1.Ki = EepromPidParam.Ki;
    Pid_1.Kd = EepromPidParam.Kd;
    Offset_T = EepromPidParam.Offset;
    Pid_1.Integrator = 0.0f;
  }
  else
  {
    Pid_1.Setpoint = DEF_SP;
    Pid_1.Kp = DEF_KP;
    Pid_1.Ki = DEF_KI;
    Pid_1.Kd = DEF_KD;
    Pid_1.Integrator = 0.0f;
    Offset_T = DEF_OFFSET;
  }

  Serial.printf("3. PID Parameters Initialazed \n");
}

void Init_PID()
{
  PIDController_Init(&Pid_1);
  Init_PID_Parameters();
}
/***********************************************************************************************/
/*** PID FUNCTIONS END *************************************************************************/
/***********************************************************************************************/


/***********************************************************************************************/
/*** SETUP BEGIN *******************************************************************************/
/***********************************************************************************************/
void setup()
{
  initGPIO();
  initSerial();
  Init_EEPROM();
  initTempSensor();
  initDimmer();

  initSPIFFS();
  initWiFi();
  initWebServer();
  initWebSocket();

  initRTOS();
  Init_PID();
}
/*************************************************************************************************/
/*** SETUP END ***********************************************************************************/
/*************************************************************************************************/

/*************************************************************************************************/
/*** LOOP BEGIN **********************************************************************************/
/*************************************************************************************************/
void loop()
{
  delay(1000);
}
/*************************************************************************************************/
/*** LOOP END ************************************************************************************/
/*************************************************************************************************/


/***********************************************************************************************/
/*** MAIN TASK BEGIN ****************************************************************************/
/***********************************************************************************************/
static void MAIN_Task(void *parameter)
{
  uint32_t MainTaskTime = millis();
  uint32_t MainTaskDelayTime = 0;
  uint32_t EventGroupReturn = 0;
  for (;;)
  {
    
    MainTaskTime = millis();
    EventGroupReturn = xEventGroupSetBits(TaskSignal, PID_TASK_SIGNAL);
    EventGroupReturn = xEventGroupWaitBits(TaskSignal, MAIN_TASK_SIGNAL, pdTRUE, pdTRUE, 10);

    EventGroupReturn = xEventGroupSetBits(TaskSignal, PRINT_TASK_SIGNAL);
    EventGroupReturn = xEventGroupWaitBits(TaskSignal, MAIN_TASK_SIGNAL, pdTRUE, pdTRUE, 10);    

    ws.cleanupClients();
    MainTaskDelayTime = ((uint32_t)Pid_1.LoopTime * 1000) - (millis() - MainTaskTime);
    delay(MainTaskDelayTime);
  }
}
/***********************************************************************************************/
/*** MAIN TASK END ******************************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** PID TASK BEGIN ****************************************************************************/
/***********************************************************************************************/
static void PID_Task(void *parameter)
{
  uint32_t TimeNow = millis();

  Pid_1.Measurement = BMP280.readTemperature();

  if (isnan(Pid_1.Measurement))
  {
    Dimmer.setPower(0); // setPower(0-100%);
    Pid_1.Measurement =  Pid_1.PrevMeasurement;
    Serial.println(F("ERROR - Failed to read from BMP280 sensor!\n"));
    Serial.println(F("Rebooting...\n"));
    Serial.flush();
    ESP.restart();
  }
  else
  {
    Pid_1.Measurement = Pid_1.PrevMeasurement+Offset_T;
    Pid_1.PrevMeasurement = Pid_1.Measurement;
  }
  

  if (Pid_1.Mode == HEATING_AUTO_STD || Pid_1.Mode == HEATING_AUTO_PAR || Pid_1.Mode == HEATING_MANUAL)
  {
    Pid_1.Error = Pid_1.Setpoint - Pid_1.Measurement;
  }
  else if (Pid_1.Mode == COOLING_AUTO_STD || Pid_1.Mode == COOLING_AUTO_PAR || Pid_1.Mode == COOLING_MANUAL)
  {
    Pid_1.Error = Pid_1.Measurement - Pid_1.Setpoint;
  }

  Pid_1.PrevError1 = 0;
  Pid_1.PrevError2 = 0;

  for (;;)
  {
    xEventGroupWaitBits(TaskSignal, PID_TASK_SIGNAL, pdTRUE, pdTRUE, portMAX_DELAY);

    TimeNow = millis();

    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    
    Pid_1.Measurement = BMP280.readTemperature();

    if (isnan(Pid_1.Measurement))
    {
      Dimmer.setPower(0); // setPower(0-100%);
      Pid_1.Measurement = Pid_1.PrevMeasurement;
      Serial.println(F("ERROR - Failed to read from BMP280 sensor!\n"));
      Serial.println(F("Rebooting...\n"));
      Serial.flush();
      ESP.restart();
    }
    else
    {
      Pid_1.Measurement = Pid_1.Measurement + Offset_T;
      Pid_1.PrevMeasurement = Pid_1.Measurement;
      Dimmer.setPower(PIDController_Update(&Pid_1)); // setPower(0-100%);
    }

    xSemaphoreGive(PID_1_Mutex);

    PID_1_TaskTime = millis() - TimeNow;

    xEventGroupSetBits(TaskSignal, MAIN_TASK_SIGNAL);
  }
}
/***********************************************************************************************/
/*** pid TASK END ******************************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** I TASK BEGIN ******************************************************************************/
/***********************************************************************************************/
static void I_Task(void *parameter)
{

  for (;;)
  {
   // xEventGroupWaitBits(TaskSignal, I_TASK_SIGNAL, pdTRUE, pdTRUE, portMAX_DELAY);

    I_01 = digitalRead(I_01_PIN);
    I_02 = digitalRead(I_02_PIN);
    I_03 = digitalRead(I_03_PIN);
    I_04 = digitalRead(I_04_PIN);
    I_05 = digitalRead(I_05_PIN);
    I_06 = digitalRead(I_06_PIN);
    I_07 = digitalRead(I_07_PIN);
    I_08 = digitalRead(I_08_PIN);

    if (I_01 | I_02 | I_03 | I_04 | I_05 | I_06 | I_07 | I_08)
    {
      if (I_01){Q_01 = !Q_01;}
      if (I_02){Q_02 = !Q_02;}
      if (I_03){Q_03 = !Q_03;}
      if (I_04){Q_04 = !Q_04;}
      if (I_05){Q_05 = !Q_05;}
      if (I_06){Q_06 = !Q_06;}
      if (I_07){Q_07 = !Q_07;}
      if (I_08){Q_08 = !Q_08;}

      notifyClients(1);
      delay(500);
    }
   // xEventGroupSetBits(TaskSignal, MAIN_TASK_SIGNAL);
    delay(100);
  }
}
/***********************************************************************************************/
/*** I TASK END ********************************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** Q TASK BEGIN ******************************************************************************/
/***********************************************************************************************/
static void Q_Task(void *parameter)
{

  for (;;)
  {
   //xEventGroupWaitBits(TaskSignal, Q_TASK_SIGNAL, pdTRUE, pdTRUE, portMAX_DELAY);
    digitalWrite(Q_01_PIN, Q_01 ? HIGH : LOW);
    digitalWrite(Q_02_PIN, Q_02 ? HIGH : LOW);
    digitalWrite(Q_03_PIN, Q_03 ? HIGH : LOW);
    digitalWrite(Q_04_PIN, Q_04 ? HIGH : LOW);
    digitalWrite(Q_05_PIN, Q_05 ? HIGH : LOW);
    digitalWrite(Q_06_PIN, Q_06 ? HIGH : LOW);
    digitalWrite(Q_07_PIN, Q_07 ? HIGH : LOW);
    digitalWrite(Q_08_PIN, Q_08 ? HIGH : LOW);
    //xEventGroupSetBits(TaskSignal, MAIN_TASK_SIGNAL);
    delay(100);

  }
}
/***********************************************************************************************/
/*** Q TASK END ********************************************************************************/
/***********************************************************************************************/

/***********************************************************************************************/
/*** PRINT TASK BEGIN **************************************************************************/
/***********************************************************************************************/
static void PRINT_Task(void *parameter)
{
  uint32_t TimeNow = millis();

  for (;;)
  {

    xEventGroupWaitBits(TaskSignal, PRINT_TASK_SIGNAL, pdTRUE, pdTRUE, portMAX_DELAY);
    TimeNow = millis();
    xSemaphoreTake(PID_1_Mutex, portMAX_DELAY);
    Serial.print("PID_1");
    Serial.print("  M: ");
    Serial.print(Pid_1.Mode);
    Serial.print("  SP: ");
    Serial.print(Pid_1.Setpoint);
    Serial.print("  MV: ");
    Serial.print(Pid_1.Measurement);
    Serial.print("  OUT: ");
    Serial.print(Pid_1.Out);

    if ((Pid_1.Mode == HEATING_AUTO_PAR) || (Pid_1.Mode == COOLING_AUTO_PAR))
    {
      Serial.print("  P: ");
      Serial.print(Pid_1.Proportional);
      Serial.print("  I: ");
      Serial.print(Pid_1.Integrator);
      Serial.print("  D: ");
      Serial.print(Pid_1.Differentiator);
    }

    Serial.print("  Kp: ");
    Serial.print(Pid_1.Kp);
    Serial.print("  Ki: ");
    Serial.print(Pid_1.Ki);
    Serial.print("  Kd: ");
    Serial.print(Pid_1.Kd);
    Serial.print("  Err: ");
    Serial.print(Pid_1.Error);
    Serial.print("  LT: ");
    Serial.print((int)(Pid_1.LoopTime * 1000));
    Serial.print("  TT1: ");
    Serial.print(PID_1_TaskTime);
    Serial.print("  TT2: ");
    Serial.print(PRINT_TaskTime);
    // Serial.print("  TT3: ");
    // Serial.print(MQTT_TaskTime);

    Serial.println();
    Serial.println();
    xSemaphoreGive(PID_1_Mutex);

    PRINT_TaskTime = millis() - TimeNow;

    xEventGroupSetBits(TaskSignal, MAIN_TASK_SIGNAL);
  }
}
/***********************************************************************************************/
/*** PRINT TASK END ****************************************************************************/
/***********************************************************************************************/



