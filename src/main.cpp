#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h> //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;

//Wifi credentials
const char* ssid     = "UniFi";
const char* password = "Logitech";

//IPAddress ip(192, 168, 0, 200); 
IPAddress subnet(255,255,255,0); 
//IPAddress gateway(192, 168, 0, 1); 
IPAddress gateway(192, 168, 1, 1); 
IPAddress ip(192, 168, 1, 240);

WiFiServer server(200);
WiFiClient client;

unsigned long int delayTime = 0;


long lastSentRTCM_ms = 0; //Time of last data pushed to socket
uint32_t serverBytesSent = 0; //Just a running total
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
long lastReport_ms = 0; //Time of last report of bytes sent

void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
 if (client.connected() == true)
  {
    client.write(incoming); //Send this byte to socket
    //Serial.print(incoming);
    
    serverBytesSent++;
    lastSentRTCM_ms = millis();
  }
}



void setup()
{
  Serial.begin(115200);
  //while (!Serial); //WAIT FOR SERIAL LOGON ---------------------DELETE-----------------------------

  pinMode(BUILTIN_LED, OUTPUT);      // set the LED pin mode

  delay(10);

  // Start GPS connection
  Wire.begin();
  //myGPS.enableDebugging(); // Uncomment this line to enable debug messages
  if (myGPS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }



  // Start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  if (!WiFi.config(ip, gateway, subnet)) 
  {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
    
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(10);
    Serial.print(".");
  }



  // Print network status:
  Serial.println("WiFi connected!");
  Serial.println("");
  Serial.print("IP address:      ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address:     ");
  Serial.println(WiFi.macAddress());
  Serial.print("Gateway address: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Subnet address:  ");
  Serial.println(WiFi.subnetMask());
  Serial.print("SSID Power:      ");
  Serial.println(WiFi.getTxPower());
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
    Serial.println(F("Failed to disable NMEA. Freezing..."));
    while (1);
  }
  else
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
    Serial.println(F("Failed to enable RTCM. Freezing..."));
    while (1);
  }
  else
    Serial.println(F("RTCM sentences enabled"));

  //-1280208.308,-4716803.847,4086665.811 is SparkFun HQ so...
  //Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
  //For more infomation see Example12_setStaticPosition
  //Note: If you leave these coordinates in place and setup your antenna *not* at SparkFun, your receiver
  //will be very confused and fail to generate correction data because, well, you aren't at SparkFun...
  //See this tutorial on getting PPP coordinates: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
  response &= myGPS.setStaticPosition(347806165, 23, 61588601, 51, 529303506, 16); //With high precision 0.1mm parts
  if (response == false)
  {
    Serial.println(F("Failed to enter static position. Freezing..."));
    while (1);
  }
  else
    Serial.println(F("Static position set"));

  //You could instead do a survey-in but it takes much longer to start generating RTCM data. See Example4_BaseWithLCD
  //myGPS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m

  if (myGPS.saveConfiguration() == false) //Save the current settings to flash and BBR
    Serial.println(F("Module failed to save."));

  Serial.println(F("Module configuration complete"));


  // Start server
  server.begin();

}


void loop()
{
  client = server.available();   // listen for incoming clients
  
  if (client) 
  {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) 
    {            // loop while the client's connected
      if (client.available()) 
      {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if(c == 'X')
        {
          client.stop();
          Serial.println("");
          Serial.println("Remote restart with X command!");
          Serial.println("");
          delay(1000);
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

      myGPS.checkUblox();
      
      //Close socket if we don't have new data for 10s
      //RTK2Go will ban your IP address if you abuse it. See http://www.rtk2go.com/how-to-get-your-ip-banned/
      //So let's not leave the socket open/hanging without data
      if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms)
      {
        Serial.println("RTCM timeout. Disconnecting...");
        client.stop();
       /*
        Serial.println("");
        Serial.println("Restart with timeout!");
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
        Serial.printf("Total sent: %d\n", serverBytesSent);
      }
      */

      digitalWrite(LED_BUILTIN, HIGH); //Show that client is connected.
    }
    digitalWrite(LED_BUILTIN, LOW); 
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }

    
}

