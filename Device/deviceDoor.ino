#include "Arduino.h"
#include "AZ3166WiFi.h"
#include "AzureIotHub.h"
#include "DevKitMQTTClient.h"
#include "LIS2MDLSensor.h"
#include "OledDisplay.h"
#include "Sensor.h"
#include "ShakeUI.h"
#include "SystemTickCounter.h"
#include "SystemVersion.h"
#include "parson.h"
#include "Telemetry.h"
#include "SystemTime.h"

DevI2C *ext_i2c;
HTS221Sensor *ht_sensor;
LPS22HBSensor *lp_sensor;

#define APP_VERSION "ver=1.0"
#define LOOP_DELAY 1000
#define EXPECTED_COUNT 5
#define INTERVAL 10000

// The magnetometer sensor
static DevI2C *i2c;
static LIS2MDLSensor *lis2mdl;

// The Temp-sensors
const char *roomSchema = "tuer-sensor1;v1";
float temperature = 50;
char  temperatureUnit = 'C';
float humidity = 50;
char  humidityUnit = '%';
float pressure = 55;
const char *pressureUnit = "psig";

// The RGB_LED
static int userLEDState = 0;
static int rgbLEDState = 0;
static int rgbLEDR = 0;
static int rgbLEDG = 0;
static int rgbLEDB = 0;

RGB_LED rgbLed;
uint8_t colors[][3] = {
        {255, 0, 0},                // red
        {0, 255, 0},                // green
        {0, 0, 255},                // blue
        {0, 0, 0},
        {255, 255, 0},
        {0, 255, 255},
        {255, 0, 255},
        {255, 255, 255}
    };
uint8_t colorindex;
bool doReset=false;

// Data from magnetometer sensor
static int axes[3];
static int base_x;
static int base_y;
static int base_z;
static double base_mittelwert;

// Indicate whether the magnetometer sensor has been initialized
static bool initialized = false;

// Indicate whether the temperatur sensor has been initialized
static bool sensorinitialized = false;

// The open / close status of the door
static bool preOpened = false;
static bool curOpened = true;

// Indicate whether DoorMonitorSucceed event has been logged
static bool telemetrySent = false;

// Indicate whether WiFi is ready
static bool hasWifi = false;

// Indicate whether IoT Hub is ready
static bool hasIoTHub = false;

// Monitor 
static uint64_t send_interval_ms;
static uint64_t reset_interval_ms;



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities
static void InitWiFi()
{
  Screen.print(2, "Verbindung...");

  if (WiFi.begin() == WL_CONNECTED)
  {
    IPAddress ip = WiFi.localIP();
    Screen.print(1, ip.get_address());
    hasWifi = true;
    Screen.print(2, "   Verbindung erfolgreich... \r\n");
    DrawCheckBox(2, 0, 1);
  }
  else
  {
    hasWifi = false;
    Screen.print(1, "   Kein Wifi\r\n ");
    DrawCheckBox(1, 0, 0);
  }
}

void sendData(const char *data, const char *schema){
  time_t t = time(NULL);
  char buf[sizeof "2011-10-08T07:07:09Z"];
  strftime(buf, sizeof buf, "%FT%TZ", gmtime(&t));

  EVENT_INSTANCE* message = DevKitMQTTClient_Event_Generate(data, MESSAGE);

  DevKitMQTTClient_Event_AddProp(message, "$$CreationTimeUtc", buf);
  DevKitMQTTClient_Event_AddProp(message, "$$MessageSchema", schema);
  DevKitMQTTClient_Event_AddProp(message, "$$ContentType", "JSON");
  
  DevKitMQTTClient_SendEventInstance(message);
}

static void InitMagnetometer()
{
  Screen.print(2, "Initialisierung...");
  i2c = new DevI2C(D14, D15);
  lis2mdl = new LIS2MDLSensor(*i2c);
  lis2mdl->init(NULL);

  lis2mdl->getMAxes(axes);
  base_x = axes[0];
  base_y = axes[1];
  base_z = axes[2];

  int count = 0;
  int delta = 10;
  char buffer[20];
  while (true)
  {
    delay(LOOP_DELAY);
    lis2mdl->getMAxes(axes);
    // --Filter--
    // Waiting for the data from sensor to become stable
    if (abs(base_x - axes[0]) < delta && abs(base_y - axes[1]) < delta && abs(base_z - axes[2]) < delta)
    {
      count++;
      if (count >= EXPECTED_COUNT)
      {
        // Done
        base_mittelwert = sqrt((base_x) * (base_x) + (base_y) * (base_y) + (base_z) * (base_z));
        Screen.print(0, "   Ueberwachung <0>.0>");
        DrawCheckBox(0, 0, 1);
        break;
      }
    }
    else
    {
      count = 0;
      base_x = axes[0];
      base_y = axes[1];
      base_z = axes[2];
    }
    sprintf(buffer, "      %d", EXPECTED_COUNT - count);
    Screen.print(1, buffer);
  }
}

void CheckMagnetometerStatus()
{
  char *message;
  int delta = 250;
  double magnetmittelwert = sqrt((axes[0]) * (axes[0]) + (axes[1]) * (axes[1]) + (axes[2]) * (axes[2]));
  // //--------------------------------DEBUG-start
  //    char buffer[50];

  //     sprintf(buffer, "base:  %f", base_mittelwert);
  //     Screen.print(1, buffer);

  //     sprintf(buffer, "magnet:  %f", magnetmittelwert);
  //     Screen.print(2, buffer);
  //     int i=0;
  //     if (curOpened==true){
  //       i=1;
  //     }
  //     else{
  //       i =0;
  //     }
  //     sprintf(buffer, "curOpened, %d", i);
  //     Screen.print(3, buffer);
  //     // -----------------------------DEBUG-end
  if ((curOpened == true) && (magnetmittelwert >= (base_mittelwert - delta)))
  {
    Screen.print(0, "   Tuere zu");
    DrawCheckBox(0, 0, 1);
    message = "{\"DoorStatus\":\"Closed\"}";
    curOpened = false;
  }
  else if ((curOpened == false) && (magnetmittelwert <= (base_mittelwert - delta)))
  {
    Screen.print(0, "   Tuere offen");
    DrawCheckBox(0, 0, 0);
    message = "{\"DoorStatus\":\"Opened\"}";
    curOpened = true;
  }

  // send message when status change
  if (curOpened != preOpened)
  {
    if (DevKitMQTTClient_SendEvent(message))
    {
      if (!telemetrySent)
      {
        telemetrySent = true;
        LogTrace("DoorMonitorSucceed", APP_VERSION);
      }
    }
    preOpened = curOpened;
  }
}

//init Temperature-Sensor
static void InitTempSensor()
{

  ext_i2c = new DevI2C(D14, D15);
  
  ht_sensor = new HTS221Sensor(*ext_i2c);
  ht_sensor->init(NULL);

  lp_sensor= new LPS22HBSensor(*ext_i2c);
  lp_sensor->init(NULL);
}

//check Temperature-Sensor
void CheckTemperatureStatus()
{
  try
  {
    ht_sensor->reset();
    ht_sensor->getTemperature(&temperature);
    //convert from C to F
    //temperature = temperature*1.8 + 32;

    ht_sensor->getHumidity(&humidity);

    lp_sensor->getPressure(&pressure);
    
    char buff[128];
    sprintf(buff, "Environment \r\n Temp:%s%c    \r\n Humidity:%s%c  \r\n Atm: %s%s",f2s(temperature, 1),temperatureUnit, f2s(humidity, 1), humidityUnit, f2s(pressure,1), pressureUnit);
    Screen.print(1, buff);

    char sensorData[200];
    snprintf(sensorData, sizeof(sensorData), "{\"temperature\":%s,\"temperature_unit\":\"%c\",\"humidity\":%s,\"humidity_unit\":\"%c\",\"pressure\":%s,\"pressure_unit\":\"%s\"}", f2s(temperature, 1), temperatureUnit,f2s(humidity, 1), humidityUnit,f2s(pressure, 1), pressureUnit);
    sendData(sensorData,roomSchema);
  }
  catch(int error)
  {
    LogError("*** Read sensor failed: %d",error);
  }

  
  // send message when temperature is higher than 45C
  // if (curOpened != preOpened)
  // {
  //   if (DevKitMQTTClient_SendEvent(message))
  //   {
  //     if (!telemetrySent)
  //     {
  //       telemetrySent = true;
  //       LogTrace("DoorMonitorSucceed", APP_VERSION);
  //     }
  //   }
  //   preOpened = curOpened;
  // }
}



int device_method_callback(const char *methodName, const unsigned char *payload, int length, unsigned char **response, int *responseLength){

  LogInfo("*** Remote method: %s",methodName);  

  if(strcmp(methodName,"LedColor")==0){
    int n=sizeof(colors)/sizeof(colors[0]);
    uint8_t *color=colors[(colorindex++)%n];
    rgbLed.setColor(color[0],color[1],color[2]);

    const char *ok="{\"result\":\"OK\"}";
    *responseLength=strlen(ok);
    *response = (unsigned char*)malloc(*responseLength);
    strncpy((char *)(*response), ok, *responseLength);
    return 200;
  }

  if(strcmp(methodName,"Reboot")==0){
    doReset=true;
    
    const char *reset="{\"result\":\"RESET\"}";    
    *responseLength=strlen(reset);
    *response = (unsigned char*)malloc(*responseLength);
    strncpy((char *)(*response), reset, *responseLength);
    return 201;
  }

  LogError("*** Remote method: %s not found",methodName);
  return 500;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino sketch
void setup()
{
  Screen.init();
  Screen.print(0, "Tuer Sicherheit");

  Screen.print(2, "Startet...");
  Screen.print(3, " > Serial");
  Serial.begin(115200);

  // Initialize the WiFi module
  Screen.print(3, " > WiFi");
  hasWifi = false;
  InitWiFi();
  if (!hasWifi)
  {
    return;
  }
  LogTrace("DoorMonitor", APP_VERSION);

  // IoT hub
  Screen.print(3, " > IoT Hub");
  DevKitMQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "DoorMonitor");
  if (!DevKitMQTTClient_Init())
  {
    Screen.clean();
    Screen.print(0, "DoorMonitor");
    Screen.print(2, "No IoT Hub");
    hasIoTHub = false;
    return;
  }
  hasIoTHub = true;

  Screen.print(3, " > Magnetometer");
  InitMagnetometer();
  Screen.print(3, " > TempSensor");
  InitTempSensor();
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (hasIoTHub && hasWifi)
  {
    char buff[128];

    // replace the following line with your data sent to Azure IoTHub
    if (hasWifi && hasIoTHub)
    {
      // Get data from magnetometer sensor
      lis2mdl->getMAxes(axes);
      Serial.printf("Axes: x - %d, y - %d, z - %d\r\n", axes[0], axes[1], axes[2]);
      
      //Screen.print(1, " ");
      //Screen.print(2, " ");
      //Screen.print(3, " ");
      /*
    
    char buffer[50];
    
    sprintf(buffer, "x:  %d", axes[0]);
    Screen.print(1, buffer);

    sprintf(buffer, "y:  %d", axes[1]);
    Screen.print(2, buffer);

    sprintf(buffer, "z:  %d", axes[2]);
    Screen.print(3, buffer);
    */
    
    if(hasWifi)
  {
    if((int)(SystemTickCounterRead() - send_interval_ms)>INTERVAL)
    {
      CheckTemperatureStatus();
      send_interval_ms = SystemTickCounterRead();
    }

    if((int)(SystemTickCounterRead() - reset_interval_ms)>INTERVAL){
      if(doReset){
         NVIC_SystemReset();
      }
      reset_interval_ms = SystemTickCounterRead();
    }
  }
    CheckMagnetometerStatus();
    }
    delay(LOOP_DELAY);
  }
}