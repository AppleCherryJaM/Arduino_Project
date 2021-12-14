#include <WiFiNINA.h>
#include <RTCZero.h>
#include <Wire.h>
#include <WDTZero.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "LedControl.h"

// WIFI Settings
char ssid[] = "XXX";       // your WiFi network name
char pass[] = "YYY";       // your WiFi password

// Timezone Settings
int GMT = 1;
int DST = 0;

const String Version = "1.7";

// some instances
RTCZero rtc;
WiFiServer server(80);
WDTZero MyWatchDoggy;

// Define all displays :
Adafruit_7segment Time7SegDis = Adafruit_7segment();      // 7 seg display for Time Display
Adafruit_7segment Date7SegDis = Adafruit_7segment();      // 7 seg display for Date Display
Adafruit_7segment TempL7SegDis = Adafruit_7segment();     // 7 seg display for Temp Display
Adafruit_7segment TempR7SegDis = Adafruit_7segment();     // 7 seg display for Temp Display
Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix(); // 8x8 dot matrix display


int status = WL_IDLE_STATUS;
unsigned long lastepochrec=0;
unsigned long lastNTPupdate=0;
unsigned long tempupdaterec=0;
unsigned long starttime;

// display controls
int brightness=14;
int dim_brightness=4;
int dimstart=21;
int dimend=6;
boolean drawDots = false;
boolean drawCol = false;
int ledpin=6;
bool led1=false;
int showdotmatrix=-1;

// date and time
int myhours, mins, secs;
int myday, mymonth, myyear;
int myClock = 24;
int dateOrder = 0;
bool IsPM = false;
String resetTime;
String currenttime,currentdate;

// temperatures
float temp_in=-99.0;
float temp_out=-99.0;

// log buffer
const unsigned int logsize=50;
String loglines[logsize];
unsigned int logpointer=0;
signed int logstart=-logsize;

// battery and ext voltage monitoring
int sensorValue;
float voltage;
float batlevel;
float batbars;
float power5;
boolean onbattery=false;

#include "symbols.h"

void setup() 
{

  starttime=millis();
  addlogentry("Start after reset t="+String(millis()-starttime, DEC));
 
  // init IOs
  analogReadResolution(12);  // set ADC resolution (4096)
  pinMode(ledpin, OUTPUT);   // for built-in LED
  
  // init UART0 for serial monitor
  Serial.begin(9600); //Initialize serial port

   // Init Time&Date 7seg displays (on I2C bus)
   Time7SegDis.begin(0x70);    // 3 Address Select pins: 0x70 thru 0x77
   Time7SegDis.setBrightness(brightness);  // 0..15
   Time7SegDis.blinkRate(3);  //  0 is no blinking. 1, 2 or 3 is for display blinking
   Time7SegDis.print(88.88);  // show "88:88" initially
   Time7SegDis.writeDisplay();
   Date7SegDis.begin(0x71);    // 3 Address Select pins: 0x70 thru 0x77
   Date7SegDis.setBrightness(brightness);  // 0..15
   Date7SegDis.blinkRate(3);  //  0 is no blinking. 1, 2 or 3 is for display blinking
   Date7SegDis.print(88.88);  // show "88:88" initially
   Date7SegDis.writeDisplay();

   // Init Temperature 7 seg display setup (on I2C bus)
   TempL7SegDis.begin(0x74);       // 3 Address Select pins: 0x70 thru 0x77 
   TempL7SegDis.setBrightness(brightness); // 0..15
   TempL7SegDis.blinkRate(0);      //  0 is no blinking. 1, 2 or 3 is for display blinking
   TempL7SegDis.drawColon(false);  // no ":" will be shown
   TempL7SegDis.writeDigitRaw(0, 0b01000000);  // display "---C" initially
   TempL7SegDis.writeDigitRaw(1, 0b01000000);
   TempL7SegDis.writeDigitRaw(3, 0b01000000);
   TempL7SegDis.writeDigitNum(4, 0xC, false); 
   TempL7SegDis.writeDisplay();    // update display
   TempR7SegDis.begin(0x73);       // 3 Address Select pins: 0x70 thru 0x77 
   TempR7SegDis.setBrightness(brightness); // 0..15
   TempR7SegDis.blinkRate(0);      //  0 is no blinking. 1, 2 or 3 is for display blinking
   TempR7SegDis.drawColon(false);  // no ":" will be shown
   TempR7SegDis.writeDigitRaw(0, 0b01000000);  // display "---C" initially
   TempR7SegDis.writeDigitRaw(1, 0b01000000);
   TempR7SegDis.writeDigitRaw(3, 0b01000000);
   TempR7SegDis.writeDigitNum(4, 0xC, false); 
   TempR7SegDis.writeDisplay();    // update display

  
    // Init 8x8 Dot Matrix Display
    matrix.begin(0x72);  // I2C address of display 
    matrix.setBrightness(15);
    matrix.blinkRate(0);
    matrix.setRotation(3);  // set rotation orientation of display - here : pin connectors are top 
    matrix.clear();
    matrix.drawPixel(0, 0, LED_RED);   // show one pixel
    matrix.writeDisplay();  // write the changes we just made to the display
     
    // first UART message
     int t = 10; //wait for serial port to open, max 5 seconds
     while (!Serial) {
        delay(500);
        if ( (t--) == 0 ) break;
     }
    Serial.print("\nHomematic Display Clock\n"); 
    Serial.println("======================="); 
    Serial.print("Version =");
    Serial.println(Version);
    
    // check if there is the WiFi module accessable
    if (WiFi.status() == WL_NO_MODULE) 
    {
     Serial.println("Fatal Error : Communication with WiFi module failed! - Abort...!!");
     addlogentry("Fatal Error : Communication with WiFi module failed!");
     matrix.print("E");
     matrix.writeDisplay();
     matrix.blinkRate(1);
     // don't continue
     while (true);
     }
    Serial.println("Found WiFi module.");

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Warning : Please upgrade the WiFi module firmware !");
    addlogentry("Warning : Please upgrade the WiFi module firmware !");
  }
  else
  {
    Serial.println("WiFi module Firmware OK.");
    addlogentry("WiFi module Firmware OK");
  }

   // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  printMacAddress(mac);

   // show a blinking red WiFi Symbol 
  matrix.clear();
  matrix.drawBitmap(0, 0, WiFi_bmp, 8, 8, LED_RED);
  matrix.blinkRate(1);
  matrix.writeDisplay();

  // Begin WiFi
  Serial.println("Scanning available networks...");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) // Fatal Error: Couldn't get a WiFi connection
  {
    Serial.println("WiFi ERROR...");
    matrix.blinkRate(3);  // fast blinking WiFi symbol 
    matrix.writeDisplay();    
    while (true); // application will hang...
  }

  // print the list of networks found:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  if (numSsid == 0) // no WiFi networks found
  {
    Serial.println("!! No WiFi Networks found ... change position of device for better reception !!");
   // show a fast blinking red WiFi Symbol 
   matrix.clear();
   matrix.drawBitmap(0, 0, WiFi_bmp, 8, 8, LED_RED);
   matrix.blinkRate(3);
   matrix.writeDisplay();
   delay(1000);
  }
  else // WiFi networks found
  {
   matrix.blinkRate(0);
   matrix.writeDisplay();
  }

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.println(" dBm");
  }
  
  // connect to WiFi network
  int attempts=0;
  Serial.println("Try to connect to WiFi...");

  while (status != WL_CONNECTED) // as we need WiFi, keep trying until WiFi is connected...
  {
    if (status != WL_CONNECTED)
    {
      delay(250);
      matrix.drawPixel(7, 7, 0);  // indicate connection attempts by flashing pixel
      matrix.writeDisplay();   
    }
    Serial.print(attempts);
    Serial.print(" attempts to connect to SSID: ");
    Serial.print(ssid);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print(" - signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
    
    attempts++;

    delay(250);
    matrix.drawPixel(7, 7, LED_RED);
    matrix.writeDisplay();  

  }
 
  // WiFi connected, show a green WiFi Symbol
  matrix.clear();
  matrix.drawBitmap(0, 0, WiFi_bmp, 8, 8, LED_GREEN);
  matrix.writeDisplay();
  Serial.println("Connected to WiFi.");
  addlogentry("Connected to WiFi t="+String(millis()-starttime, DEC));
  delay(250);

  // Try to connect to NTP and get time intially
  Serial.println("trying to get NTP time...");
  addlogentry("trying to get NTP time...");
  WiFi.setDNS(IPAddress(8, 8, 8, 8));  
  rtc.begin();
  
  // show a Red NTP Symbol
  matrix.clear();
  matrix.drawBitmap(0, 0, NTP_bmp, 8, 8, LED_RED);
  matrix.writeDisplay();

  // get Epoch time from Internet Time Service and set RTC
  setRTC();  
  fixTimeZone();

  // NTP received, show a green NTP Symbol
  matrix.clear();
  matrix.blinkRate(0);
  matrix.drawBitmap(0, 0, NTP_bmp, 8, 8, LED_GREEN);
  matrix.writeDisplay();
  showdotmatrix=8;

  // output initial time
  Serial.println("\nInitial Time after Reset : ");
  print2digitsSer(myhours);
  Serial.print(":");
  print2digitsSer(mins);
  Serial.print(":");
  print2digitsSer(secs);
  Serial.print(" ");
  Serial.print(myday);
  Serial.print(".");
  Serial.print(mymonth);
  Serial.print(" ");  

  resetTime = String(myday)+"."+String(mymonth)+" "+String(myhours)+":"+String(mins); 
  addlogentry("Reset Time= "+resetTime);
  
  // start to act as Internet Server to enable the HTML menu
  server.begin();  

  // end of setup routine
  Time7SegDis.blinkRate(0);  // stop blinking
  Date7SegDis.blinkRate(0);
  Serial.print("Setup completed t="+String(millis()-starttime, DEC));
  addlogentry("Setup completed t="+String(millis()-starttime, DEC));
  addlogentry("Start normal operation...");
  
}

void loop() 
{
  secs = rtc.getSeconds();      // get current time from real time clock
  if (secs == 0) fixTimeZone(); // each minute at 0s, correct time zone

  showTime();                   // Display time and date

  showTemp();                   // Display temperatures

  show_battery_status();        // Detect Supply voltage and get battery level

  showMatrixDisplay();          // Display Symbols on 8x8 dot matrix display (if required)

  adjustDimming();              // adjust 7seg LED brightness depending on time
  
  handleWifiClient();           // handle WiFi clients (browser for HTML Menu etc)

  // wait for next second ...
  while (secs == rtc.getSeconds())delay(10); // wait until seconds change
  
  if (mins==59 && secs ==0)   setRTC();      // get NTP time every hour at minute 59 and update RTC
  
}

void showTime()
{
  if (secs==0)    // print full time&date, temp and and battery stats each minute at 0 secs 
  {
    Serial.println();
    print2digitsSer(myhours);
    Serial.print(":");
    print2digitsSer(mins);
    Serial.print(":");
    print2digitsSer(secs);
    if (myClock==12) 
    {
      if(IsPM) Serial.print("  PM");
      else Serial.print("  AM");
    }
    Serial.print(" ");
    Serial.print(myday);
    Serial.print(".");
    Serial.print(mymonth);
    Serial.print(" TempI=");
    Serial.print(temp_in);    
    Serial.print(" TempO=");
    Serial.print(temp_out);    
    if (onbattery)
    {
      Serial.print(" on battery (Bat=");  
      Serial.print(batlevel);
      Serial.print("% =");
      Serial.print(voltage);
      Serial.print("V) |");
    }
    else
      Serial.print(" USB powered |");  
    
  }
  else
  {
    if ((secs==10) || (secs==20) || (secs==40) || (secs==50)) Serial.print("*");
    else if ((secs==30) || (secs==59)) Serial.print("|");
    else Serial.print(".");      
    currenttime=String(myhours)+":"+String(mins)+":"+String(secs);  
    currentdate=String(myday)+"."+String(mymonth);   
  }
  
 //7-seg display Time output
  Time7SegDis.writeDigitNum(0, (myhours / 10) % 10, drawDots);
  Time7SegDis.writeDigitNum(1, myhours % 10, drawDots); 
  Time7SegDis.drawColon(drawCol); 
  Time7SegDis.writeDigitNum(3, (mins / 10) % 10, drawDots);
  Time7SegDis.writeDigitNum(4, (mins % 10), drawDots);
  Time7SegDis.writeDisplay();
  drawCol=!drawCol;
   
  //7-seg display Date output
  Date7SegDis.writeDigitNum(0, (myday / 10) % 10, false);
  Date7SegDis.writeDigitNum(1, myday % 10, true); 
  Date7SegDis.drawColon(false); 
  Date7SegDis.writeDigitNum(3, (mymonth / 10) % 10, false);
  Date7SegDis.writeDigitNum(4, (mymonth % 10), false);
  Date7SegDis.writeDisplay();

  
}

void showTemp()
{
  if (temp_in > -99.0)
  {
      displayTempL(temp_in);  // display on left display
  }
  
  
  if (temp_out > -99.0)
  {
      displayTempR(temp_out);  // display on right display
  }
  
}

void setRTC() 
{ 
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 15;

  // try to access NTP service until successful
  do
  {
      do 
      {
        numberOfTries++;
        epoch = WiFi.getTime(); // Try to get NTP time 
        if (epoch == 0)
          delay(500);           // failed, wait until next try
        else
        {
          lastNTPupdate=epoch; // success, remember last update time
          lastepochrec = millis()-starttime;
        }
      }
      while ((epoch == 0) && (numberOfTries < maxTries));  // repeat a number of times
    
      if (numberOfTries == maxTries) // not successful
      {
        Serial.print(currenttime+" "+currentdate+" could not reach NTP");
        addlogentry(currenttime+" "+currentdate+" could not reach NTP");        
        // show a Red blining NTP Symbol
        matrix.clear();
        matrix.drawBitmap(0, 0, NTP_bmp, 8, 8, LED_RED);
        matrix.blinkRate(2);
        matrix.writeDisplay();

      }
      else  // success
      {
        rtc.setEpoch(epoch);
        Serial.print(currenttime+" "+currentdate+" NTP update successful. Epoch= ");
        Serial.println(epoch);
        addlogentry(currenttime+" "+currentdate+" NTP update successful. Epoch="+String(epoch));        
         // show a green NTP Symbol for a few seconds
         matrix.clear();
         matrix.blinkRate(0);
         matrix.drawBitmap(0, 0, NTP_bmp, 8, 8, LED_GREEN);
         matrix.writeDisplay();
         showdotmatrix=8;  

         if (onbattery) addlogentry("Currently running on battery. Remaining="+String(batlevel)+"%");        // add a note here if on bat
                         
      }
  }
  while (lastNTPupdate==0);   // repeat again and again if no NTP access was achieved yet (endless loop only can happen at start-up)
}

void adjustDimming()
{
  if (mins==0 && secs==0 && myhours==dimstart) // if start-time of dimming has been reached
  {
    Time7SegDis.setBrightness(dim_brightness);  // set to reduce brightness
    Date7SegDis.setBrightness(dim_brightness); 
    TempL7SegDis.setBrightness(dim_brightness); 
    TempR7SegDis.setBrightness(dim_brightness);
    Serial.print("brightness level dimmed");
    addlogentry("brightness level dimmed");        
 
  }

  if (mins==0 && secs==0 && myhours==dimend) // if end-time of dimming has been reached
  {
    Time7SegDis.setBrightness(brightness);  // set brightness back to normal level
    Date7SegDis.setBrightness(brightness); 
    TempL7SegDis.setBrightness(brightness); 
    TempR7SegDis.setBrightness(brightness); 
    Serial.print("brightness level set to normal");
    addlogentry("brightness level set to normal");        
  }
}

void showMatrixDisplay()
{
  if (showdotmatrix > 0)  
    {
      matrix.setBrightness(showdotmatrix); // adjust brightness of showing symbol on dotmatrix  
      showdotmatrix--;                     // decrease seconds to show
      if (showdotmatrix==0)                // if zero, clear matrix display again
      {
          matrix.clear();
          matrix.writeDisplay();
          matrix.setBrightness(15);
          showdotmatrix=-1;
      }
   }
   else if (showdotmatrix==-1 && onbattery) show_batbars(batbars);  // else show battery bar display if on battery 
  
}

void handleWifiClient()
{
  String currentLine = "";                // a String to hold incoming data from the client
  String commandLine = "";                // holds additional commands (after "?")
  String cmdpar = "";
  int clientcmd;                          // position of "?" 
  int len;
  boolean cmdproc = false;               // indicates if a command was executed with this current client alreday
  String lastepochrec2, tempupdaterec2;

  // first check if still connected with WiFi network
  if (WiFi.status() != WL_CONNECTED)  // if no longer connected...
  {
     // ...show a red WiFi Symbol ...
     matrix.clear();
     matrix.drawBitmap(0, 0, WiFi_bmp, 8, 8, LED_RED);
     matrix.writeDisplay();
     showdotmatrix=5;        

    // ... and (re-)connect to network.
    WiFi.disconnect();
    status = WiFi.begin(ssid, pass);
    Serial.println(currenttime+" re-connecting to WiFi...");
    addlogentry(currenttime+" re-connecting to WiFi ");   
     
  }
  else // if still connected to WiFi
  {
    WiFiClient client = server.available();
    cmdproc=false;

    if (client) // if an outside client request was received (from a browser etc)
    {
     boolean currentLineIsBlank = true;
     currentLine = "";
     while (client.connected())  // as long as client is connected 
     {  
      if (client.available())   // and available
      {
        // prepare variables to show on HTML page
        IPAddress ip = WiFi.localIP();

        unsigned int ler_min = ((millis() - lastepochrec)/60000);
        unsigned int ler_sec = ((millis() - lastepochrec)/1000) - (ler_min*60);
        lastepochrec2 = String(ler_min)+":"+String(ler_sec);  // contains how long ago the last NTP update was

        if (tempupdaterec!=0)
        {
          unsigned int ltr_min = ((millis() - tempupdaterec)/60000);
          unsigned int ltr_sec = ((millis() - tempupdaterec)/1000) - (ltr_min*60);
          tempupdaterec2 = String(ltr_min)+":"+String(ltr_sec); // contains how long ago the last temperture update was
        }
        else tempupdaterec2 = "never";

        int ll=logstart;  // for the log buffer dump  
        if (ll<0) ll=0; 
        int li=0;

        // parse client message
        char c = client.read();
        if (c == '\n' && currentLineIsBlank) // repond to client after CR
        {
          // first send a standard HTTP response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
//          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();

#include "menu_inline_css7.h"
          
          break;
        }
        if (c == '\n') 
        {
          currentLineIsBlank = true;
          currentLine = "";
        } 
        else if (c != '\r') 
        {

          currentLineIsBlank = false;
          currentLine += c;      // add it to the end of the currentLine
        }


       // process received WiFi commands : 
       
       if ((c == '\r') && (cmdproc==false))  // if CR, look for additional commands received, if no already done (otherwise just a request to display the menu)
        {
            clientcmd = currentLine.indexOf('?');  // position of "?" in the request line ("?" indicates a command)
            if (clientcmd>0)    // if a command have been detected
            {
              // extract command out of received line
              len=currentLine.length();  
              commandLine = currentLine.substring(clientcmd+1,len);            // extract string from "?"..
              commandLine = commandLine.substring(0,commandLine.indexOf(' ')); // ..until first SPACE

              // output command on UART
              Serial.print("\nWiFi Command ");
              Serial.print(commandLine);

              // command handling :
              
              if (commandLine.endsWith("ibr"))   // adjust 7seg LED display brightness
              {
                Serial.println(" --> Inc LED backlight");
                if (brightness<14) brightness++;
                Time7SegDis.setBrightness(brightness);  
                Date7SegDis.setBrightness(brightness); 
                TempL7SegDis.setBrightness(brightness); 
                TempR7SegDis.setBrightness(brightness); 
                cmdproc=true; 
              }              
              if (commandLine.endsWith("dbr"))
              {
                Serial.println(" --> Dec LED backlight");
                if (brightness>0) brightness--;
                Time7SegDis.setBrightness(brightness);  
                Date7SegDis.setBrightness(brightness);  
                TempL7SegDis.setBrightness(brightness); 
                TempR7SegDis.setBrightness(brightness); 
                cmdproc=true;
              }
              
              if (commandLine.endsWith("idims"))   // adjust auto reduced brightness dimming time start and end times
              {
                Serial.println(" --> Inc dim time start");
                dimstart++;
                cmdproc=true;
              }
              if (commandLine.endsWith("ddims"))
              {
                Serial.println(" --> Dec dim time start");
                dimstart--;
                cmdproc=true;
              }
              if (commandLine.endsWith("idime"))
              {
                Serial.println(" --> Inc dim time end");
                dimend++;
                cmdproc=true;
              }
              if (commandLine.endsWith("ddime"))
              {
                Serial.println(" --> Dec dim time end");
                dimend--;
                cmdproc=true;
              }
                
              if (commandLine.startsWith("tempL="))  // set value on left temp display
              {
                cmdpar = commandLine.substring(6, commandLine.length());
                Serial.print(" --> set L temperature to ");
                Serial.println(cmdpar);
                temp_in=cmdpar.toFloat();
                // show a green Temp Symbol for a few seconds
                matrix.clear();
                matrix.blinkRate(0);
                matrix.drawBitmap(0, 0, temp_bmp, 8, 8, LED_GREEN);
                matrix.writeDisplay();
                showdotmatrix=5;  
                tempupdaterec = millis()-starttime;
                cmdproc=true;
              }              
              if (commandLine.startsWith("tempR=")) // set value on right temp display
              {
                cmdpar = commandLine.substring(6, commandLine.length());
                Serial.print(" --> set R temperature to ");
                Serial.println(cmdpar);
                temp_out=cmdpar.toFloat();
                // show a green Temp Symbol for a few seconds
                matrix.clear();
                matrix.blinkRate(0);
                matrix.drawBitmap(0, 0, temp_bmp, 8, 8, LED_GREEN);
                matrix.writeDisplay();
                showdotmatrix=5;        
                tempupdaterec = millis()-starttime;
                cmdproc=true;                
              }              

              if (commandLine.startsWith("mxteston")) // display a bell picture on matrix display for some seconds
              {
                // show a Symbol for a few seconds
                matrix.clear();
                matrix.blinkRate(0);
                matrix.drawBitmap(0, 0, bell_bmp, 8, 8, LED_RED);
                matrix.writeDisplay();
                showdotmatrix=10;        
                cmdproc=true;                
              }     
              if (commandLine.endsWith("ledon")) // mostly for testing only : toggle built-in LED
              {
                Serial.println(" --> Built-in LED ON");
                led1=true;
                digitalWrite(ledpin,HIGH);
                cmdproc=true;
              }              
              if (commandLine.endsWith("ledoff"))
              {
                Serial.println(" --> Built-in LED OFF");
                led1=false;
                digitalWrite(ledpin,LOW);
                cmdproc=true;
              }
            }
         }
      }
    }
    delay(1);         // give the web browser time to receive the data
    client.stop();   // close the connection
   }
  } 
}

void fixTimeZone() 
{
  int daysMon[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (myyear % 4 == 0) daysMon[2] = 29; // fix for leap year
  
  // get current time from real time clock module
  myhours = rtc.getHours();
  mins = rtc.getMinutes();
  myday = rtc.getDay();
  mymonth = rtc.getMonth();
  myyear = rtc.getYear();

  myhours +=  GMT; // adjust time zone 

  myhours += DST;  // and correct DST  

  // do corrections in case necessary :
  
  if (myhours < 0)  // if hours rolls negative
  {  
    myhours += 24;   // keep in range of 0-23
    myday--;        // fix the day
    if (myday < 1)  // fix the month if necessary
    {  
      mymonth--;
      if (mymonth == 0) mymonth = 12;
      myday = daysMon[mymonth];
      if (mymonth == 12) myyear--; // fix the year if necessary
    }
  }
  if (myhours > 23)   // if hours rolls over 23
  {  
    myhours -= 24; // keep in range of 0-23
    myday++; // fix the day
    if (myday > daysMon[mymonth]) // fix the month if necessary
    {  
      mymonth++;
      if (mymonth > 12) mymonth = 1;
      myday = 1;
      if (mymonth == 1)myyear++; // fix the year if necessary
    }
  }

  if (myClock == 12) // this is for 12 hour clock
  {  
    IsPM = false;
    if (myhours > 11)IsPM = true;
    myhours = myhours % 12; // convert to 12 hour clock
    if (myhours == 0) myhours = 12;  // show noon or midnight as 12
  }
}

void show_battery_status()
{
  // Get Battery Level (available on Mkr1010 board)
  sensorValue = analogRead(ADC_BATTERY);
  
  // Convert the analog reading (which goes from 0 - 4095) to a real voltage (0 - 4.208V):
  voltage = sensorValue * (4.208 / 4095.000); // (0-4.208V)
  batlevel = 100*(voltage-3.3)/0.8; // calc bat level in % 

  if (batlevel < 0) batlevel=0;     // if no battery connected
  if (batlevel > 100) batlevel=100; // if charging

  // determine "bars" for battery gauge
  if (voltage < 3.4) 
    batbars =0;
  else if (voltage>=3.4 && voltage<3.46)
    batbars = 1;
  else if (voltage>=3.46 && voltage<3.62)
    batbars = 2;
  else if (voltage>=3.62 && voltage<3.78)
    batbars = 3;
  else if (voltage>=3.78 && voltage<3.94)
    batbars = 4;
  else if (voltage>=3.94)
    batbars = 5;

 // Detect USB-Power (Charging) or Battery Mode by measuring the 5V output pin of the Mkr1010
 // use 2:1 voltage divider (2xR) on 5V pin to A1 and GND to monitor voltage level
  power5 = analogRead(A1) * (3.3 / 4095.000);   // USB(5V)>=2.5V ; Bat(3.3V)<=1.6V

  // detect USB Power (Charge) or Battery Mode
  if (power5 > 2.2) // ext.power,charging
  { 
    if (onbattery)
    {
     MyWatchDoggy.setup(WDT_HARDCYCLE4S);  // initialize hardware watchdog after 4S if not cleared before

      //  show a blinking reset Symbol 
      matrix.clear();
      matrix.drawBitmap(0, 0, reset_bmp, 8, 8, LED_RED);
      matrix.blinkRate(2);
      matrix.writeDisplay();
      delay(1000);

      Serial.println(currenttime+" Watchdog reset in 4 sec - wait for reset \n");
      addlogentry(currenttime+" Switch from bat to USB power -> will RESET in 4 sec... ");        
      
      for (int t = 1; t > 0; ++t) {
        Serial.print(t);Serial.print(".");
        delay(950);
        }      
    }
     
  }
  else     // on backup battery
  {
      if (onbattery==false)  // if just switched from USB power to battery, output message
      {
        Serial.println(currenttime+" Swichted to Battery Supply! Remaining="+String(batlevel)+"%");
        addlogentry(currenttime+" Swichted to Battery Supply! Remaining="+String(batlevel)+"%");       
      }
      onbattery=true; // remember status "on battery" (also for next time it changes to USB power ...needs a reset then)
  }

}

void show_batbars(int bar)
{
      // draw the battery status using 5 bars (green and red)
      for (int i=1; i<=bar; i++)
      {
        matrix.drawLine(2,7-i, 5,7-i, LED_GREEN);
        matrix.writeDisplay();
      }
      for (int i=bar+1; i<=5; i++)
      {
        matrix.drawLine(2,7-i, 5,7-i, LED_RED);
        matrix.writeDisplay();
      }

     // show battery bitmap in 3 colors around
     if (bar>2)
      {
      matrix.drawBitmap(0, 0, battery_bmp, 8, 8, LED_GREEN);
      matrix.writeDisplay();
      }
      else if (bar==2)
      {
      matrix.drawBitmap(0, 0, battery_bmp, 8, 8, LED_YELLOW);
      matrix.writeDisplay();
      }
      else 
      {
      matrix.drawBitmap(0, 0, battery_bmp, 8, 8, LED_RED);
      matrix.writeDisplay();
      }            
  
}

void displayTempR(float temp0)    
{
  temp0=truncf(temp0 * 10.0) / 10.0;            // cut off value after 1 decimal place
  if (temp0!=0.0f) TempR7SegDis.printFloat(temp0); // display float variable in range -99C...99.9C
  else   TempR7SegDis.printFloat(0.01f,2);      // workaround to show "0.0" correctly
  TempR7SegDis.writeDigitNum(4, 0xC, false);  // show "C" on last digit 
  TempR7SegDis.writeDisplay();                // update display  
}

void displayTempL(float temp0)    
{
  temp0=truncf(temp0 * 10.0) / 10.0;            // cut off value after 1 decimal place
  if (temp0!=0.0f) TempL7SegDis.printFloat(temp0); // display float variable in range -99C...99.9C
  else   TempL7SegDis.printFloat(0.01f,2);      // workaround to show "0.0" correctly
  TempL7SegDis.writeDigitNum(4, 0xC, false);  // show "C" on last digit 
  TempL7SegDis.writeDisplay();                // update display  
}

void addlogentry(String newlogentry)  
{
  loglines[logpointer]=newlogentry; // write String to next line
  logpointer++;                     // increment and adjust write pointer
  if (logpointer==logsize) logpointer=0;
  logstart++;                       // increment and adjust read pointer
  if (logstart==logsize) logstart=0;
}

void print2digitsSer(int number) 
{
  if (number < 10) Serial.print("0");
  Serial.print(number);
}

void printMacAddress(byte mac[]) 
{
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
