#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h> //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGPS;

//Wifi credentials
const char* ssid     = "UniFi";
const char* password = "Logitech";


IPAddress subnet(255,255,252,0); 
IPAddress gateway(192, 168, 0, 1); 
IPAddress ip(192, 168, 0, 200); // IP address at Hannebjerg

WiFiServer server(200);
WiFiClient client;

unsigned long int delayTime = 0;

const bool serialDebug = false;
char t;

long lastSentRTCM_ms = 0; //Time of last data pushed to socket
uint32_t serverBytesSent = 0; //Just a running total
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
long lastReport_ms = 0; //Time of last report of bytes sent

void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)
{
 if (client.connected() == true)
  {
    client.write(incoming); //Send this byte to socket
    if (serialDebug)
  Serial.print(incoming);
    
    serverBytesSent++;
    lastSentRTCM_ms = millis();
  }
}



void setup()
{

 // if (serialDebug)
  Serial.begin(115200);
  
  //while (!Serial); //WAIT FOR SERIAL LOGON ---------------------DELETE-----------------------------

  pinMode(BUILTIN_LED, OUTPUT);      // set the LED pin mode

  delay(10);

digitalWrite(BUILTIN_LED,true);

  // Start GPS connection
  Wire.begin();
  //myGPS.enableDebugging(); // Uncomment this line to enable debug messages
  myGPS.disableDebugging();


  while(myGPS.begin(Wire,0x42) == false) //Connect to the u-blox module using Wire port
  {
     
    if (serialDebug)
  Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    delay(100);
    //while (1);
  }

digitalWrite(BUILTIN_LED, false);

  // Start by connecting to a WiFi network
  if (serialDebug)
  Serial.println();
  if (serialDebug)
  Serial.print("Connecting to ");
  if (serialDebug)
  Serial.println(ssid);

  if (!WiFi.config(ip, gateway, subnet)) 
  {
    if (serialDebug)
  Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
    
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(10);
    if (serialDebug)
  Serial.print(".");
  }



  // Print network status:
  if (serialDebug)
  Serial.println("WiFi connected!");
  if (serialDebug)
  Serial.println("");
  if (serialDebug)
  Serial.print("IP address:      ");
  if (serialDebug)
  Serial.println(WiFi.localIP());
  if (serialDebug)
  Serial.print("MAC address:     ");
  if (serialDebug)
  Serial.println(WiFi.macAddress());
  if (serialDebug)
  Serial.print("Gateway address: ");
  if (serialDebug)
  Serial.println(WiFi.gatewayIP());
  if (serialDebug)
  Serial.print("Subnet address:  ");
  if (serialDebug)
  Serial.println(WiFi.subnetMask());
  if (serialDebug)
  Serial.print("SSID Power:      ");
  if (serialDebug)
  Serial.println(WiFi.getTxPower());
  if (serialDebug)
  Serial.println("");


  //Setup GPS
  myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //UBX+RTCM3 is not a valid option so we enable all three.

  myGPS.setNavigationFrequency(1); //Set output in Hz. RTCM rarely benefits from >1Hz.

  //Disable all NMEA sentences
  bool response = true;
  response &= myGPS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C);
  response &= myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
  response &= myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
  response &= myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
  response &= myGPS.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_I2C);
  response &= myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C);
  response &= myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);

  if (response == false)
  {
    if (serialDebug)
  Serial.println(F("Failed to disable NMEA. Freezing..."));
    while (1);
  }
  else
    if (serialDebug)
  Serial.println(F("NMEA disabled"));

  //Enable necessary RTCM sentences
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); //Enable message 1005 to output through UART2, message every second
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); //Enable message every 10 seconds

  if (response == false)
  {
    if (serialDebug)
  Serial.println(F("Failed to enable RTCM. Freezing..."));
    while (1);
  }
  else
    if (serialDebug)
  Serial.println(F("RTCM sentences enabled"));

  //-1280208.308,-4716803.847,4086665.811 is SparkFun HQ so...
  //Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
  //For more infomation see Example12_setStaticPosition
  //Note: If you leave these coordinates in place and setup your antenna *not* at SparkFun, your receiver
  //will be very confused and fail to generate correction data because, well, you aren't at SparkFun...
  //See this tutorial on getting PPP coordinates: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
  //response &= myGPS.setStaticPosition(347803029, 00, 62746441, 00, 529252570, 00); //With high precision 0.1mm parts ********************** STUE TEST **************
  response &= myGPS.setStaticPosition(347806165, 23, 61588601, 51, 529303506, 16); //With high precision 0.1mm parts ************************* Garage pos *************
  if (response == false)
  {
    if (serialDebug)
  Serial.println(F("Failed to enter static position. Freezing..."));
    while (1);
  }
  else
    if (serialDebug)
  Serial.println(F("Static position set"));

  //You could instead do a survey-in but it takes much longer to start generating RTCM data. See Example4_BaseWithLCD
  //myGPS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m

  if (myGPS.saveConfiguration() == false) //Save the current settings to flash and BBR
  { 
    if (serialDebug)
  Serial.println(F("Module failed to save."));
}
 
  if (serialDebug)
  Serial.println(F("Module configuration complete"));


  // Start server
  server.begin();

}


void loop()
{
  
  //digitalWrite(BUILTIN_LED, false);

  client = server.available();   // listen for incoming clients
  
  if (client) 
  {                             // if you get a client,
    if (serialDebug)
  Serial.print("New Client: ");           // print a message out the serial port
    if (serialDebug)
  Serial.println(client.remoteIP());
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) 
    {            // loop while the client's connected
      if (client.available()) 
      {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (serialDebug)
  Serial.write(c);                    // print it out the serial monitor
        if(c == 'X')
        {
          client.stop();
          digitalWrite(LED_BUILTIN, HIGH);
          if (serialDebug)
  Serial.println("");
          if (serialDebug)
  Serial.println("Remote restart with X command!");
          if (serialDebug)
  Serial.println("");




          delay(1000);
          digitalWrite(LED_BUILTIN, LOW);
          ESP.restart();
        }
      }
      
      /*
      if(millis()>delayTime)
      {
        client.println(millis());
        delayTime = millis() + 1000;
      }
      */

      if (millis() - lastSentRTCM_ms > 20)
      {
        digitalWrite(LED_BUILTIN, LOW); 
      }

      myGPS.checkUblox();
 
      //Close socket if we don't have new data for 10s
      //RTK2Go will ban your IP address if you abuse it. See http://www.rtk2go.com/how-to-get-your-ip-banned/
      //So let's not leave the socket open/hanging without data
      if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms)
      {
        if (serialDebug)
  Serial.println("RTCM timeout. Disconnecting...");
        client.stop();
       /*
        if (serialDebug)
  Serial.println("");
        if (serialDebug)
  Serial.println("Restart with timeout!");
        if (serialDebug)
  Serial.println("");
        delay(1000);
        ESP.restart();
        */
        return;
      }

      /* 
      if (millis() - lastReport_ms > 1000)
      {
        lastReport_ms += 1000;
        if (serialDebug)
  Serial.printf("Total sent: %d\n", serverBytesSent);
      }
      */

      digitalWrite(LED_BUILTIN, HIGH); //Show that client is connected.
    }
    digitalWrite(LED_BUILTIN, LOW); 
    // close the connection:
    client.stop();
    if (serialDebug)
  Serial.println("Client Disconnected.");
  }

    
}

