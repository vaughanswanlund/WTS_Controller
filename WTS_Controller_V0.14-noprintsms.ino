/**
 *  WTS Controller
 *  
 *  v0.1  - Parse Cleral RS232 and send to Serial Monitor
 *  v0.2  - Add GSM modem to send data via an SMS
 *  v0.3  - Add a bluetooth printer and send raw data to printer
 *  v0.4  - Add a GPS functionality and replace V1.1 GSM functionality using DFRobot lib
 *  v0.5  - Store received data in data struct for easy retrieval. Only transmit meaningful information
 *  v0.6  - migrate rs232 and bluetooth from software serial to hardware serial.
 *          Turn sim808 module on/off using transistor on pin 5
 *  v0.7  - Replace DFRobot_SIM808 code for GPRS, GPS and HTTP requests as it does not work well.
 *        - Repeat connection attempts if there are failures. Applies to GPRS connections, SMS sending, HTTP requests
 *  v0.8  - Add feature to send multiple SMSs
 *  v0.9  - Check state of HC-05 and restart it if not connected.
 *  v0.10 - Add SD card module and load configuration information.
 *  v0.11 - Save incoming messages to log file.
 *          Save each incoming message to a separate file in the TX folder on the SD card for processing.
 *          Rename recvWithStartEndMarkers to processIncomingMessage.
 *          Modify processIncomingMessage to discard malformed messages and log
 *  v0.11-noprint - removed print function for live test.
 *                  added leds to indicate system status.
 *  v0.12-noprint - Check failed tramission queue and resend
 *                  Read API key from config file
 *                  Read the Administrators mobile number in from config file.
 *                  Reduce memory usage by using F() for Serial.print
 *  v0.13
 *  - noprintsms    urlencoded the GET URL to allow date, time, truckid , latitude and longitude to be send to Thingspeak
 *                  Changing format of data being sent to Thingspeak 
 *                    From: field1=<Gross S> field2=<Gross A> field3=<Gross B> field4=<Gross C> field5=<Net S> field6=<Net A> field7=<Net B> field8=<Net C>
 *                    To:   field1=<Gross S> field2=<Gross A> field3=<Gross B> field4=<Gross C> field5=<Gross Total> field6=<Date/Time> field7=<UID>/<TruckID> field8=<latitude/longitude>
 *                  Added method to check status of SIM808 and restart if need be.
 *                  Reduced failed attempt counter thresholds to speed system up.
 *                  Fix minor bugs
 *  v0.14
 *  - noprintsms    replace RS232 communications with new functionality that communicates with the WTS_Cleral_Comms_Handler module.         
 *          
 */

#define LOGGING
// Select your modem:
#define TINY_GSM_MODEM_SIM808
// Increase RX buffer to capture the entire response.Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response else data will be lost (and the http library will fail).
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

// set GSM PIN, if any
#define GSM_PIN ""                  // The pin used to unlock the SIM card. Leave as empty string if no pin is required. @TODO pull this in from config file

//#include <SoftwareSerial.h>
#include "Adafruit_Thermal.h"
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

#define cleralMessageStartMarker 2  // Cleral uses 0x02 to denote the start of a message
#define cleralMessageEndMarker 3    // Cleral uses 0x03 to denote the start of a message
#define cleralDelimiter ";"         // Cleral cleralDelimiter for each field 
#define MESSAGE_LENGTH 160          // the SMS message length.
#define NUM_SENSORS 9               // the maximum number of sensors data that could be received.
#define wtsRS232Serial Serial1      // Serial1 for comms with WTS_Cleral_Comms_Handler. Arduino TX1 pin to RX on . RX1 to RX on RS232 module. (RX RS232 <-> TX Cleral)
#define sim808Serial Serial2        // Serial2 for GPS/GSM/GPRS comms.   Arduino RX2 connected to the TX SIM808 module, TX2 connected to the RX of SIM808
#define printerSerial Serial3       // Serial3 for bluetooth comms. Arduino TX3 to RX on HC-05 bluetooth module. RX3 to TX on HC-05 bluetooth module
#define SIM808_POWER_PIN 5          // pin used to tun SIM808 ON/OFF. 
#define MAX_MOBILE_NUMBERS 5        // SMS will be sent to a maximum of this number of mobile numbers
#define MOBILE_NUM_LEN 13           // Mobile number length in ZA is +27711277683 + \0 = 13
#define TRUCK_ID_LEN 21             // Truck identifier length. 20 characters plus 1 for '\0'
#define MAX_SP_LEN 30               // length of the Service Provider GPRS credential string
#define MAX_API_LEN 50              // length of the API Key credential string
#define RECEIVED_NUM_CHARS 310      // The max number of chars that can be recevied from Cleral
#define SystemReadyStatusPin 36     // pin used to indicate if contoller is ready to receive data. 
#define SystemBusyStatusPin 37      // pin used to indicate if contoller busy initialising or processing message. 
#define MODEM_BAUD 9600             // The SIM808 modem baud rate
#define MODEM_RESTART_THRESHOLD 2   // The number of failed attempts to modem.restart(); before restarting SIM808
#define NETWORK_CONNECT_THRESHOLD 2   // The number of failed attempts to connect to the GSM network before giving up
#define SD_CLOUD_TX_DIR "tx/cloud"  // directory for storing the Cloud TX queue
#define SD_SMS_TX_DIR "tx/sms"      // directory for storing the SMS TX queue
#define GPS_RETRY_LIMIT 10          // The number of times to retry obtaining the GPS location
#define GPS_RESTART_THRESHOLD 4     // the number of times to retry gps location before restarting the GPS module.

// Bluetooth Variables
#define BT_CONNECTED 1
#define BT_DISCONNECTED 0
#define BT_POWER_PIN 4
#define BT_STATE_PIN 3

// EEPROM - used to keep track of filename increments
#define CLOUD_FILE_EEPROM_ADDRESS 43
#define SMS_FILE_EEPROM_ADDRESS 45
#define MAX_FILE_NUM 9999

// SD Card & Configuration
#define SD_CS_PIN 48                            // this is the Chip Select pin of the SD Card Moudule
#define CLOUD_TX_FILE_LENGTH 18                 // Length of cloud tx filename which includes directory,filename and terminating \0
#define SMS_TX_FILE_LENGTH 16                   // Length of sms tx filename which includes directory filename and terminating \0
#define SD_CHECK_INTERVAL 60000                 // interval between SD card checks
bool isSDCardValid = false;
long lastSDCheckTime;

File configFile;                                // placeholder that is used to access the configuration files.
int  rxFileNumber = 0;                          // index for naming files that must be transmitted.
int  prevBTState = 1;                           // The previous bluetooth connection state. 1 is connected and 0 is dis-connected.
long connectTime = millis();                    // Last time the bluetooth module was connected to the printer.
bool configMobileLoaded = false;                // Indicates if the mobile numbers have been loaded successfully
bool configTruckLoaded = false;                 // Indicates if the truck ID has been loaded successfully
bool configServiceProviderLoaded = false;       // Indicates if the Service Provider details have been loaded successfully
bool configAPILoaded = false;                   // Indicates if the thingspeak API has been loaded successfully
bool configAdminMobileLoaded = false;           // Indicates if the Administrators mobile number had been loaded successfully
char truckID[TRUCK_ID_LEN] = {"TRUCK_DEFAULT"}; // The truck ID. This is read from config file.
int  receivedMobileLength = 0;                  // number of chars received.
int  receivedSPLength = 0;                      // number of chars received.
int  receivedAPILength = 0;                     // number of API chars received.
int  receivedAdminMobileLength = 0;             // number of chars received.
int  cloudFileCount = 0;                        // Cloud Transmission file name sequence counter
int  smsFileCount = 0;                          // SMS Transmission file name sequence counter
char fileNum[5];                                // char representation of file sequence
bool configComplete = true;                     // indicates if crucial config has been completed. IF NOT CONTACT ADMINISTRATOR

// Message Sending variables
long lastTransmitCloudQueueCheckTime;             // last time the Cloud queue was processed
long lastTransmitSMSQueueCheckTime;               // last time the SMS queue was processed
char currentCloudTXFile [CLOUD_TX_FILE_LENGTH];  // The name of the Cloud file currently being processed. 
char currentSMSTXFile [SMS_TX_FILE_LENGTH];     // The name of the SMS currently being processed.

char currentCloudFileName [CLOUD_TX_FILE_LENGTH];  // The name of the Cloud file currently being proccessed.   ¬remove 
char currentSMSFileName [SMS_TX_FILE_LENGTH];     // The name of the SMS file currently being proccessed.     ¬remove 
bool receivedMessageSuccessfully = false;           // flag showing if message was received and stored successfully

// GPRS/GSM Modem
long lastModemCheckTime;                          // last time the failed queue was processed
boolean isModemWorking = false;                   // global flag to show status of modem

// Core Processor Timings
long MODEM_CHECK_INTERVAL = 600000;               // The time the system waits before trying to resend failed messages. 600000ms = 10min
long CLOUD_RESEND_INTERVAL = 120000; //¬420000;              // The time the system waits before trying to send message to the Cloud. 420000ms = 7 min
long SMS_RESEND_INTERVAL = 200000;                // The time the system waits before trying to send an SMS. 300000ms = 5 min
long CLERAL_MSG_CHECK_INTERVAL = 60000;              // The time interval between each request to the WTS_Cleral_Comms_handler for a new message

long lastMsgCheckTime = 0;                        // keeps track of the last time the WTS_Cleral_Comms_Handler was polled

boolean newData = false;                           // Flag to indicate that a new message has arrived.
Adafruit_Thermal printer(&printerSerial);          // Pass Serial3 pointer addr to printer constructor
                                                                                                                       
// RS232 message settings
const byte numChars = 50;
const String sensorOrder = "MABCDEF";   //Sensor order used to retrieve Sensor structures from array.  @TODO this shoud be read from configuration file.
char receivedChars[RECEIVED_NUM_CHARS];
char tempChars[RECEIVED_NUM_CHARS];           // temporary array for use when parsing   
static boolean recvInProgress = false;  // flag indicating if new message is being processed. 
                                                                                                                        
// GPS variables
//char message[MESSAGE_LENGTH];
int messageIndex = 0;
//char MESSAGE[300];
char lat[12];
char lon[12];
char wspeed[12];
char phone[16];
char datetime[20];
bool gpsReady = false;
bool gotGpsData = false;
int failedInitCounter = 0;

//GSM Variables
char gprsBuffer[64];
char *s = NULL;

// SMS & GPRS Variables
char apn[10];
char gprsUser[10];
char gprsPass[10];

char smsRecipients[MAX_MOBILE_NUMBERS][MOBILE_NUM_LEN];     //MAKE SURE MOBILE NUMBERS ARE CORRECT CAUSE SERVICE PROVIDE DOES NOT CHECK
char readMobileChars[MAX_MOBILE_NUMBERS*MOBILE_NUM_LEN];
char readServiceProviderChars[MAX_SP_LEN];
char readAPIKeyChars[MAX_API_LEN];
char readTruckIDChars[TRUCK_ID_LEN];
char readAdminMobileChars[MOBILE_NUM_LEN];    //MAKE SURE MOBILE NUMBERS ARE CORRECT CAUSE SERVICE PROVIDE DOES NOT CHECK

char adminMobile[MOBILE_NUM_LEN] = "+27711277683";      //setup with default mobile in case config doesn't load

// Cloud Server details
const char server[]   = "api.thingspeak.com";
const int  port       = 80;
char apiKey[18];  
char thingspeakChannel[8];

TinyGsm       modem(sim808Serial);
TinyGsmClient client(modem);
HttpClient    http(client, server, port);


struct Sensor{
  char id[2] = {'Z','\0'};     // 'Z' indicates that no data has been added to the sensor yet
  char netWeight[7];
  char grossWeight[7];
  char sensorId[5];
};

struct cleralMessage{
  char uid[4];
  char truckId[TRUCK_ID_LEN];
  char dateTime[20];
  char unit[3];
  char checksum[4];
  int numSensors = 0;
  char netWeightTotal[10];
  char grossWeightTotal[10];
  char latitude[12] = "N/A";
  char longitude[12] = "N/A";
  Sensor sensors[NUM_SENSORS]={};
};

struct cleralMessage cMessage;
struct Sensor sensorArray[8];
char positions[9] = {'S','A','B','C','D','E','F','G','N'};


void setup() {
  Serial.begin(115200);               // Serial monitor
  delay(400);
  pinMode(SIM808_POWER_PIN,OUTPUT);   // difine the SIM808 power pin for restarting module
  pinMode(SystemReadyStatusPin,OUTPUT);   
  pinMode(SystemBusyStatusPin,OUTPUT);   
  setSystemStatusLED(false);          // Change Status LED to RED to indicate that system is not ready to receive.
  digitalWrite(SIM808_POWER_PIN,LOW); 
  wtsRS232Serial.begin(9600);         // WTS serial to receive Cleral RS232 data
  printerSerial.begin(38400);         // start using BAUD 38400 which is what the BLE module is configured to use.
  printer.begin();                    // Init printer
  delay(1000);

  // Obtain the latest filenumber sequences
  cloudFileCount = readIntFromEEPROM(CLOUD_FILE_EEPROM_ADDRESS);
  smsFileCount = readIntFromEEPROM(SMS_FILE_EEPROM_ADDRESS);
  
  sim808Serial.begin(9600);           // GPS module is set to 9600
  Serial.println(__FILE__);           // Print the file location

// Bluetooth Module init
  pinMode(BT_STATE_PIN,INPUT);                  
  pinMode(BT_POWER_PIN,OUTPUT);
  digitalWrite(BT_POWER_PIN,LOW);
  Serial.println(F("Turning HC-05 ON"));
  delay(1000);
  digitalWrite(BT_POWER_PIN,HIGH);
  Serial.print(F("HC-05 State: "));
  Serial.println(digitalRead(BT_STATE_PIN));


  // SD Module Setup
  pinMode(SD_CS_PIN,OUTPUT);            // Chip Select Pin for SD Module. SPI pins MISO, MOSI and SCK are on pins 50, 51 and 52 respectively
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("initialization failed. Things to check:"));
    Serial.println(F("1. is a card inserted?"));
    Serial.println(F("2. is your wiring correct?"));
    Serial.println(F("3. did you change the chipSelect pin to match your shield or module?"));
    Serial.println(F("Note: press reset or reopen this serial monitor after fixing your issue!"));
    isSDCardValid=false;
  }else{
    isSDCardValid=true;
  }

  // Load configuration from SD card.
  if(!loadConfiguration()){     //¬ @TODO add a check for isSDCardValid and handle accordingly
    Serial.println(F("loadConfiguration failed"));
    configComplete = false;
  }

  //If SIM808 is not responding turn in ON
  if(modem.testAT()==0){
    startSIM808();
    if(modem.testAT()==0){
      Serial.println(F("SIM808 did not start"));
      isModemWorking = false;
    }else{
      Serial.println(F("SIM808 powered up"));
    }
  }

  // Unlock the SIM card if required.
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }       

  //******** Initialize SIM808 module *************
  while(!modem.restart()){ //init()){
    failedInitCounter++;
    if(modem.testAT() != 0 && failedInitCounter > MODEM_RESTART_THRESHOLD){
      //restartSIM808();
      isModemWorking = false;
      failedInitCounter=0;
      break;
    }else if( modem.testAT() == 0){
      startSIM808();
    }
    Serial.println(F("Failed to init modem...retrying"));
    gpsReady = false;
    delay(1000);
  }

  if(modem.testAT() !=0){isModemWorking = true;}
 
  Serial.println(F("Waiting for network..."));
  int networkCounter =0;
  while(!modem.waitForNetwork(20000L)) {
    Serial.println(F("Still waiting...wait 5 sec..."));
    delay(5000);
    if(networkCounter >= NETWORK_CONNECT_THRESHOLD){
      Serial.println(F("Failed to connect"));
      break;    
    }
    networkCounter++;
  }

  if (modem.isNetworkConnected()) {
    Serial.println(F("Network connected")); 
  }

  failedInitCounter=0;
  gpsReady = modem.enableGPS();
 
  if(configComplete){
    setSystemStatusLED(true);
  }else{
    setSystemStatusLED(false);
    bool adminRes = modem.sendSMS(adminMobile, "Loading configuration from SD card failed");
    Serial.print(F("Admin Notified of config failure: "));
    Serial.println(adminRes);
  }
  lastTransmitCloudQueueCheckTime = millis();
  lastTransmitSMSQueueCheckTime = millis();
  lastModemCheckTime = millis();
  lastSDCheckTime = millis();
  Serial.println(F("System Initialisation Complete"));
}

void loop() {
    processIncomingMessage();
    if (newData == true) {
      Serial.println();
      // Change the status LEDs to show BUSY
      setSystemStatusLED(false);
      strcpy(tempChars, receivedChars);   // temporary copy is necessary to protect the original data because strtok() used in parseData() replaces the commas with \0
      
      //@TODO If SIM808 is not responding turn in ON 

      if(!logMessage("Message Received",receivedChars)){
        Serial.println(F("Saving Received Message FAILED"));
      }

      if(!parseData()){      // populate cMessage with received data
        Serial.println(F("parseData() failed"));                        
        delay(200);
      }
      if(gpsReady){// Obtain GPS information and time/date 
        if(!getLocation()){
          Serial.println(F("getLocation() failed"));                        
          delay(200);
        }
      }
      
      if(!queueCloudMessage(receivedChars,cMessage.latitude,cMessage.longitude)){
        Serial.println(F("Adding to Cloud TX queues FAILED"));
        receivedMessageSuccessfully=false;
      }else{
        receivedMessageSuccessfully = true;
      }

      if(!queueSMSMessage(receivedChars,cMessage.latitude,cMessage.longitude)){
        Serial.println(F("Adding to SMS TX queues FAILED"));
        receivedMessageSuccessfully = false;
      }else{
        receivedMessageSuccessfully = true;
      }
      sendResponse(receivedMessageSuccessfully);
             
      newData = false;                                                                            // set flag to indicate system is ready for the next message
      // Clear receivedChars
      clearCharArray(receivedChars);
      
      Serial.println(F("Ready..."));

      /*
      // Clear the current tx file name
      for(int currTXChar = 0; currTXChar < sizeof(currentCloudFileName); currTXChar++){
        currentCloudFileName[currTXChar] = '\0';
      }
      */
      
      // Change the status LEDs to show READY
      setSystemStatusLED(true);
    }

    // Check the SD card
    if(!isSDCardValid && (millis() - lastSDCheckTime > SD_CHECK_INTERVAL)){
      if (!SD.begin(SD_CS_PIN)) {
        Serial.println(F("SD card initialization failed. Things to check:"));
        Serial.println(F("1. is a card inserted?"));
        Serial.println(F("2. is your wiring correct?"));
        Serial.println(F("3. did you change the chipSelect pin to match your shield or module?"));
        Serial.println(F("Note: press reset or reopen this serial monitor after fixing your issue!"));
        isSDCardValid=false;
      }else{
        isSDCardValid=true;
      }
      lastSDCheckTime=millis();
    }

    // Send a request to the WTS_Cleral_Comms_Handler for a new message.
    if(isSDCardValid && !newData && !recvInProgress && (millis() - lastMsgCheckTime > CLERAL_MSG_CHECK_INTERVAL)){
      Serial.println("requesting message");
      wtsRS232Serial.write(cleralMessageStartMarker);
      wtsRS232Serial.write('A');
      wtsRS232Serial.write(cleralMessageEndMarker);
      lastMsgCheckTime = millis();
    }


    if(!recvInProgress && !newData && (millis() - lastTransmitCloudQueueCheckTime > CLOUD_RESEND_INTERVAL)){
      setSystemStatusLED(false);
      Serial.println(F("******Process Cloud Queue*****"));    
      if(isModemRunning()){
        processCloudQueue();
        lastTransmitCloudQueueCheckTime = millis();
        setSystemStatusLED(true);
      }else{
        Serial.println(F("Warning: modem not running, skip sending"));    
      }
    }

    if(!recvInProgress && !newData && (millis() - lastTransmitSMSQueueCheckTime > SMS_RESEND_INTERVAL)){
      setSystemStatusLED(false);
      Serial.println(F("******Process SMS Queue*****"));    
      if(isModemRunning()){
        //Serial.println("sms temporarily removed - add back later");
        processSMSQueue();
        lastTransmitSMSQueueCheckTime = millis();
        setSystemStatusLED(true);
      }else{
        Serial.println(F("Warning: modem not running, skip sending"));    
      }
    }

    if(!recvInProgress && !newData && (millis() - lastModemCheckTime > MODEM_CHECK_INTERVAL)){
      setSystemStatusLED(false);
      Serial.println(F("***Check Modem Status***"));    
      failedInitCounter=0;
      if(modem.testAT() == 0){
        Serial.println(F("Modem Not Responding"));
        while(!modem.restart()){
          failedInitCounter++;
          if(modem.testAT() != 0 && failedInitCounter >= MODEM_RESTART_THRESHOLD){ 
            //Restart SIM808
            Serial.println(F("Restarting modem"));
            restartSIM808();
            delay(1500);
            modem.setBaud(MODEM_BAUD);
            failedInitCounter=0;
          }else if( modem.testAT() == 0 && failedInitCounter >= MODEM_RESTART_THRESHOLD){
            // modem has failed to respond.
            break;
          }else if(modem.testAT() == 0){
            Serial.println(F("Toggling modem power"));
            startSIM808();
            modem.setBaud(MODEM_BAUD);
          }
          Serial.println(F("Failed to init modem, delaying 2s and retrying"));
          gpsReady = false;
          delay(2000);
        } 

        isModemWorking = modem.testAT();
        if(isModemWorking){
          if (modem.isNetworkConnected()) {
            Serial.println(F("Network connected")); 
          }else{
            Serial.print(F("Connecting.....")); 
            int cnt = 0;
            while(!modem.waitForNetwork(600000L, true)) {
              Serial.println(F("Still waiting...wait 5 sec..."));
              delay(5000);
              
              if(cnt >= 3){
                break;
              }
              cnt++;
            }
            Serial.println(F("Connected")); 
            
          }
          gpsReady = modem.enableGPS();
          setSystemStatusLED(true);
        }else{
          Serial.println("MODEM NOT RESPONDING - waiting until next modem check");
        }
   
      }else{
        Serial.println(F("Modem OK"));
        isModemWorking = true;
        setSystemStatusLED(true);
      }
      lastModemCheckTime = millis();
      failedInitCounter=0;
    }
}

/**
 *  Read incoming data on Serial1 (RS232) store in receivedChars array
 *  
 *  Ascii Message Format
 *  0x02;                                         Start marker
 *  UID;1;TIME;2022-03-23 21:44:52;UNIT;kg;       Header information
 *  GWM;5960;GWA;11980;GWB;6960;                  Gross weights
 *  NTM;3960;NTA;7980;NTB;3960;                   Net Weights
 *  GWT;24900;NTT;15900;                          Gross and Net totals
 *  IDM;0000;IDA;B226;IDB;ACFD;                   Sensor IDs
 *  CKS;92                                        check sum
 *  0x03                                          End marker
 */
void processIncomingMessage() {
    static int index = 0;
    char rc;
    int num;
    while (wtsRS232Serial.available() > 0 && newData == false) {
        rc = wtsRS232Serial.read();             // read the first byte
        Serial.print(rc);                       // print char to serial monitor for debug purposes
        //num = (int)rc;
        //Serial.print(" ");
        //Serial.println(num);
        
        if (recvInProgress == true) {
          
          //Check for partial message in serial buffer. Log and Discard.
          if (rc == cleralMessageStartMarker) {
            logMessage("Partial Message Received",receivedChars);
            for(int recChar = 0; recChar < sizeof(receivedChars); recChar++){     
              receivedChars[recChar] = '\0';
            }
            index = 0;                                  // Reset receiving index
            continue;                                   // Go read next byte from buffer
          }
          
          if (rc != cleralMessageEndMarker) {
              receivedChars[index] = rc;
              index++;
              if (index >= RECEIVED_NUM_CHARS) {
                  index = RECEIVED_NUM_CHARS - 1;
              }
          }else {
              receivedChars[index] = '\0'; // terminate the string
              recvInProgress = false;
              index = 0;
              newData = true;
          }
        }
        else if (rc == cleralMessageStartMarker) {        // message start marker received
            recvInProgress = true;
        }
    }
}

// ¬ must add code to check modem here!!
bool isModemRunning(){
  return true;
}


// clears a char array by setting each character in a string to \0
void clearCharArray(char charArray[]){
  for(int charCounter = 0; charCounter < sizeof(charArray); charCounter++){
    charArray[charCounter] = '\0';
  }
}

/** 
 * Load system configuration from config files on SD card
 *    Mobile numbers
 *    Truck ID
 *    GPRS APN setting
 *    Thingspeak API Key
 */
bool loadConfiguration(){
  Serial.println(F("Loading configuration data")); 
  int filePos = 0;
  char cf;

  //***Load the Admin's mobile number
  configFile = SD.open("config/admin.txt");
  if (configFile) {
    while (configFile.available()) {
      cf = configFile.read();
      // # indicates a comment in the first line of the file. 
      if(cf == '#'){
        //skip to end of the line.
        while(cf != '\n'){
          cf=configFile.read();  
        }
        // Reading position is now at the last character of the line. 
        cf = configFile.read(); // Read one more character to get to start of the next line
      }
      readAdminMobileChars[filePos] = cf;
      filePos++;
      if(filePos > (MOBILE_NUM_LEN)){
        Serial.println(F("WARNING: admin.txt contains more numbers than is allowed."));
        break;
      }
    }
    receivedAdminMobileLength = filePos-1;
    filePos=0;
    if(parseAdminMobileData()){
      Serial.println(F("Admin Mobile Number Loaded"));
      configAdminMobileLoaded = true;
    }else{
      configAdminMobileLoaded = false;
    }
    configFile.close();
  }else{                                                  //Reading from file failed
    Serial.println(F("'config/admin.txt' missing or corrupt"));
    Serial.println(F("Loading admin mobile numbers Failed!"));
    configAdminMobileLoaded = false;
  }

  //***Load mobile numbers
  configFile = SD.open("config/mobiles.txt");
  if (configFile) {
    while (configFile.available()) {
      cf = configFile.read();
      // # indicates a comment in the first line of the file. 
      if(cf == '#'){
        //skip to end of the line.
        while(cf != '\n'){
          cf=configFile.read();  
        }
        // Reading position is now at the last character of the line. 
        cf = configFile.read(); // Read one more character to get to start of the next line
      }
      
      readMobileChars[filePos] = cf;
      filePos++;
      if(filePos >= (MAX_MOBILE_NUMBERS*MOBILE_NUM_LEN)){
        Serial.println(F("WARNING: mobiles.txt contains more numbers than is allowed."));
        break;
      }
    }
    receivedMobileLength = filePos-1;
    filePos=0;
    if(parseMobileData()){
      Serial.println(F("Mobile Numbers Loaded"));
      configMobileLoaded = true;
    }else{
      configMobileLoaded = false;
    }
    configFile.close();
  }else{                                                  //Reading from file failed
    Serial.println(F("'config/mobiles.txt' missing or corrupt"));
    Serial.println(F("Loading Mobile Numbers Failed!"));
    configMobileLoaded = false;
  }
   
  //***Load Truck ID
  filePos = 0;
  configFile = SD.open("config/truck.txt");
  if (configFile) {
    // clear the value of truckID
    clearCharArray(truckID);
    //¬for (int u = 0; u < sizeof(truckID); u++) {
    //  truckID[u] = '\0';
    //}
    while (configFile.available()) {
      cf = configFile.read();
      // # indicates a comment in the first line of the file. 
      if(cf == '#'){
        //skip to end of the line.
        while(cf != '\n'){
          cf=configFile.read();  
        }
        // Reading position is now at the last character of the line. 
        cf = configFile.read(); // Read one more character to get to start of the next line
      }

      readTruckIDChars[filePos] = cf;
      filePos++; 
      if(filePos >= TRUCK_ID_LEN-1){
        Serial.println(F("WARNING: truck.txt contains more characters/numbers than is allowed."));
        break;
      }
    }

    if(parseTruckIDData()){
      Serial.println(F("Truck ID Loaded"));
      configTruckLoaded = true;  
    }else{
      Serial.println(F("Loading Truck ID Failed!"));
      configTruckLoaded = false;  
    }
    configFile.close();
  }else{                                                  //Reading from file failed
    Serial.println(F("'config/truck.txt' missing or corrupt"));
    Serial.println(F("Loading Truck ID Failed!"));
    strcpy(truckID,"N/A");
    configTruckLoaded = false;
  }
  
  //***Load Service Provider Details
  filePos = 0;
  configFile = SD.open("config/spconfig.txt");
  if (configFile) {
    while (configFile.available()){
      cf = configFile.read();
      
      // # indicates a comment in the first line of the file. 
      if(cf == '#'){
        //skip to end of the line.
        while(cf != '\n'){
          cf=configFile.read();  
        }
        // Reading position is now at the last character of the line. 
        cf = configFile.read(); // Read one more character to get to start of the next line
      }
      readServiceProviderChars[filePos] = cf;
      filePos++;
      if(filePos >= MAX_SP_LEN){
        Serial.println(F("WARNING: spconfig.txt contains more text than is allowed."));
        break;
      }
    }
    receivedSPLength = filePos-1;
    if(parseServiceProviderData()){
      configServiceProviderLoaded = true;
    }else{
      configServiceProviderLoaded = false;
    }
    configFile.close();
  }else{                                                  
    Serial.println(F("'config/spconfig.txt' missing or corrupt"));
    Serial.println(F("Loading Service Provider GPRS details FAILED!"));
    configServiceProviderLoaded = false;
  }

  // Load API Details
  filePos = 0;
  configFile = SD.open("config/api.txt");
  if (configFile) {
    while (configFile.available()){
      cf = configFile.read();
      
      // # indicates a comment in the first line of the file. 
      if(cf == '#'){
        //skip to end of the line.
        while(cf != '\n'){
          cf=configFile.read();  
        }
        // Reading position is now at the last character of the line. 
        cf = configFile.read(); // Read one more character to get to start of the next line
      }
      readAPIKeyChars[filePos] = cf;
      filePos++;
      if(filePos >= MAX_API_LEN){
        Serial.println(F("WARNING: api.txt contains more text than is allowed."));
        break;
      }
    }
    receivedAPILength = filePos-1;
    if(parseAPIData()){
      configAPILoaded = true;
    }else{
      configAPILoaded = false;
    }
    configFile.close();
  }else{                                                  
    Serial.println(F("'config/api.txt' missing or corrupt"));
    Serial.println(F("Loading API Key FAILED!"));
    configAPILoaded = false;
  }
    
  if(configMobileLoaded && configTruckLoaded && configServiceProviderLoaded && configAPILoaded && configAdminMobileLoaded){
    Serial.println(F("Loading Configuration Finished"));
    return true;
  }else{
    return false;
  }
}

// Check that a character is on of: alphanumeric, hyphen or Underscore 
bool checkIfValidChar(char checkChar){
  if(isDigit(checkChar) || isAlpha(checkChar) || checkChar == '-' ||  checkChar == '_'){
    return true;
  }else{
    // Not an allowed char
    return false;
  }
}


// read the GPRS credentials from the configuration file 
bool parseServiceProviderData(){
  char * idx;        // this is used by strtok() as an index
  
  // get the APN from the string
  idx = strtok(readServiceProviderChars,",");      
  strcpy(apn,idx);

  idx = strtok(NULL,",");
  if(idx != NULL){
    strcpy(gprsUser,idx);
  }else{
    strcpy(gprsUser,"");
  }

  idx = strtok(NULL,",");
  if(idx != NULL){
    strcpy(gprsPass,idx);
  }else{
    strcpy(gprsPass,"");
  }

  Serial.println(F("GPRS Credentials Loaded"));
 
  return true;
} 

// send response message over wtsRS232Serial to confirm status of message processing. 'S' success and 'F' failure
void sendResponse(bool processingStatus){
  Serial.println("send response message to WTS_Cleral_Comms_Handler");
  wtsRS232Serial.write(cleralMessageStartMarker);
  if(processingStatus){
    wtsRS232Serial.write('S');
  }else{
    wtsRS232Serial.write('F');
  }
  wtsRS232Serial.write(cleralMessageEndMarker);
  lastMsgCheckTime = millis();
}

/** 
 *  Read the API key from the configuration file 
 */
bool parseAPIData(){
  char * idex;        // this is used by strtok() as an index
  
  // get the APN from the string
  idex = strtok(readAPIKeyChars,",");      
  strcpy(apiKey,idex);
  
  idex = strtok(NULL,",");
  if(idex != NULL){
    strcpy(thingspeakChannel,idex);
  }else{
    strcpy(thingspeakChannel,"");
  }
  Serial.println(F("API KEY Credentials Loaded"));
  return true;
}

/** 
 *  Read the Trcuk ID from the configuration file 
 */
bool parseTruckIDData(){
  char * tridx;        // this is used by strtok() as an index
  
  // get the truck ID from the string
  tridx = strtok(readTruckIDChars , ",");      
  if(tridx != NULL){
    strcpy(truckID,tridx);
    for(int cnt = 0 ; cnt < strlen(truckID); cnt++){
      if(!checkIfValidChar(truckID[cnt])){  
          Serial.print(F("WARNING: truck.txt contains invalid character: "));
          Serial.println(truckID[cnt]);
          truckID[cnt] = '_';
      }
    }
  }else{
    return false;
  }
  return true;
}

/** 
 *  Read the Administrator's mobile number from the configuration file 
 */
bool parseAdminMobileData(){
  char * rdx;        // this is used by strtok() as an index
  
  // get the number from the string
  rdx = strtok(readAdminMobileChars,",");      
  if(rdx != NULL)strcpy(adminMobile,rdx);
  
  Serial.println(F("Admin Credentials Loaded"));
  return true;
}

// store an int value in the EEPROM 
void writeIntIntoEEPROM(int address, int number){
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

// retrieve an int value from the EEPROM 
int readIntFromEEPROM(int address){
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}


// read the mobile numbers from the configuration file 
bool parseMobileData(){
  int arrPos = 0;
  char * indx;        // this is used by strtok() as an index

  indx = strtok(readMobileChars,",");      // get the first number from the string
  for(arrPos = 0; arrPos < MAX_MOBILE_NUMBERS; arrPos++){
    if(indx != NULL){
      strcpy(smsRecipients[arrPos],indx);
    }else{
      strcpy(smsRecipients[arrPos],"Z");
    }
    indx = strtok(NULL,",");      // get the first part - the string
  }
  if(smsRecipients[0] == NULL){
    Serial.println(F("NO MOBILE NUMBERS LOADED`"));
    return false;  
  }
  return true;
}

// Read one message from Cloud TX Queue
bool processCloudQueue() {
  setSystemStatusLED(false);
  File dir = SD.open(SD_CLOUD_TX_DIR);   
  File entry =  dir.openNextFile();
  // @TODO check if the file is corrup
  char et;
  byte filePosition = 0;
  if (! entry) {
    // no more files
    Serial.println(F("processCloudQueue: No files to process"));
    return true;
  }
  Serial.print(F("dir.name(): "));  Serial.println(dir.name());  Serial.print(F("entry.name(): "));  Serial.println(entry.name());
  String fn = SD_CLOUD_TX_DIR; 
  fn.concat("/");
  fn.concat(entry.name());
  fn.toCharArray(currentCloudTXFile,CLOUD_TX_FILE_LENGTH);
  
  Serial.print(F(" currentCloudTXFile being processed: "));  Serial.println(currentCloudTXFile);
  Serial.print("currentCloudTXFile size:");  Serial.println(entry.size());

  // Delete the file if it is empty.
  if(entry.size() == 0){
    entry.close();
    Serial.print(F("ERROR - '"));
    Serial.print(currentCloudTXFile);
    Serial.print(F("' Is Empty. Deleting empty file"));
    Serial.println(removeFileFromSDCard(currentCloudTXFile) ? "...Success" : "...Failed");
    char adminMessage[80] = "Empty TX file encountered. Truck: ";
    strcat(adminMessage,truckID);
    strcat(adminMessage, " , File: ");
    strcat(adminMessage,currentCloudTXFile);
    modem.sendSMS(adminMobile, adminMessage);//"Empty TX file encountered. Encountered an empty Loading configuration from SD card failed");
    return false;
  }

  //Serial.print(F("sizeof(tempChars): "));  Serial.println(sizeof(tempChars));  Serial.print(F("strlen(tempChars): "));  Serial.println(strlen(tempChars));
  // Clear tempChars
  clearCharArray(tempChars);
  //¬for(int rChar = 0; rChar < sizeof(tempChars); rChar++){
  //  tempChars[rChar] = '\0';
  //}
  while (entry.available()) {
    et = entry.read();
    Serial.print(et);
    tempChars[filePosition] = et;
    filePosition++;
    if(filePosition >= (RECEIVED_NUM_CHARS)){
      Serial.print(F("WARNING: "));Serial.print(currentCloudTXFile);Serial.println(F(" contains more chars than is allowed."));
      break;
    }
  }
  entry.close();

Serial.println();   // for debug purposes  ¬remove

  parseData();
  
  if(!printToSerial()){
     Serial.println(F("printToSerial() failed"));
     delay(200);
  }

  Serial.println(F("about to print to cloud"));
  
  if(printToCloud2()){
    // transmit was successful so delete file from the TX queue
    Serial.println(F("MSG transmitted to cloud."));
    Serial.println(removeFileFromSDCard(currentCloudTXFile) ? "...Success" : "...Failed");
    Serial.println(F("HTTP Request Success"));
  }else {
    Serial.println(F("HTTP Request Failed"));
  }
    
  delay(1000);
  //newData = false;
  Serial.println(F("Ready..."));

  // Clear the current tx file name
  clearCharArray(currentCloudTXFile);
  //¬for(int currTX = 0; currTX < sizeof(currentCloudTXFile); currTX++){
  //  currentCloudTXFile[currTX] = '\0';
  //}
  
  // Change the status LEDs to show READY
  setSystemStatusLED(true);
 
  return true;
}

// Read one message from SMS TX Queue
bool processSMSQueue() {
  setSystemStatusLED(false);
  File dir = SD.open(SD_SMS_TX_DIR);
  File entry =  dir.openNextFile();
  char et;
  byte filePosition = 0;
  if (! entry) {
    // no more files
    Serial.println(F("processSMSQueue: No files to process"));
    return true;
  }
  //Serial.print(F("dir.name(): "));  Serial.println(dir.name());  Serial.print(F("entry.name(): "));  Serial.println(entry.name());
  String fn = SD_SMS_TX_DIR;  
  fn.concat("/");
  fn.concat(entry.name());
  fn.toCharArray(currentSMSTXFile,SMS_TX_FILE_LENGTH);
  Serial.print(F("currentSMSTXFile being processed: "));  Serial.println(currentSMSTXFile);
  Serial.print("currentSMSTXFile size:");  Serial.println(entry.size());

  // Delete the file if it is empty.
  if(entry.size() == 0){
    entry.close();
    Serial.print(F("ERROR - '"));
    Serial.print(currentSMSTXFile);
    Serial.print(F("' Is Empty. Deleting empty file"));
    Serial.println(removeFileFromSDCard(currentSMSTXFile) ? "...Success" : "...Failed");
    char adminMessage[80] = "Empty TX file encountered. Truck: ";
    strcat(adminMessage,truckID);
    strcat(adminMessage, " , File: ");
    strcat(adminMessage,currentSMSTXFile);
    modem.sendSMS(adminMobile, adminMessage);//"Empty TX file encountered. Encountered an empty Loading configuration from SD card failed");
    return false;
  }
  
  //Serial.print(F("sizeof(tempChars): "));  Serial.println(sizeof(tempChars));  Serial.print(F("strlen(tempChars): "));  Serial.println(strlen(tempChars));
  // Clear tempChars
  clearCharArray(tempChars);
  //¬for(int rChar = 0; rChar < sizeof(tempChars); rChar++){
  //  tempChars[rChar] = '\0';
  //}
  while (entry.available()) {
    et = entry.read();
    Serial.print(et);
    tempChars[filePosition] = et;
    filePosition++;
    if(filePosition >= (RECEIVED_NUM_CHARS)){
      Serial.print(F("WARNING: "));Serial.print(currentSMSTXFile);Serial.println(F(" contains more chars than is allowed."));
      break;
    }
  }
  entry.close();
  
Serial.println();   // for debug purposes  ¬remove

  parseData();
  
  if(!printToSerial()){
     Serial.println(F("printToSerial() failed"));
     delay(200);
  }

  Serial.println(F("about to send SMS"));

  int statusOfSMS = transmitSMS();
  if(statusOfSMS == 0){
    // transmit was successful so delete file from the TX queue
    Serial.println(F("MSG transmitted via SMS."));
    Serial.println(removeFileFromSDCard(currentSMSTXFile) ? "...Success" : "...Failed");
    Serial.println(F("SMS Success"));
  }else{ 
    Serial.println(F("transmitSMS() failed to send  "));                        
    Serial.print(statusOfSMS);
    Serial.println(F(" SMS"));                        
    delay(200);
  }
      
  delay(1000);
  //newData = false;
  Serial.println(F("Ready..."));

  // Clear the current tx file name
  clearCharArray(currentSMSTXFile);
  //¬for(int currTX = 0; currTX < sizeof(currentSMSTXFile); currTX++){
  //  currentSMSTXFile[currTX] = '\0';
  //}
  
  // Change the status LEDs to show READY
  setSystemStatusLED(true);
 
  return true;
}

/*
 *  Add a received message to the Cloud Queue
 */
bool queueCloudMessage(char receivedChars[],char latitude[], char longitude[]){
  // Save message for transmitting
  bool fileNameUnique = false;
  char fileName[CLOUD_TX_FILE_LENGTH];
    
  while(!fileNameUnique){
    cloudFileCount++;
    if(cloudFileCount > MAX_FILE_NUM)cloudFileCount = 1;  //Reset file number sequence
    // reset the fileName
    for (int v = 0; v <= sizeof(fileName); v++) {                           // Add directory tx/ to filename
      if(v==0){
        fileName[v]='t';
      }else if(v==1){
        fileName[v]='x';
      }else if(v==2){
        fileName[v]='/';
      }else if(v==3){
        fileName[v]='c';
      }else if(v==4){
        fileName[v]='l';
      }else if(v==5){
        fileName[v]='o';
      }else if(v==6){
        fileName[v]='u';
      }else if(v==7){
        fileName[v]='d';
      }else if(v==8){
        fileName[v]='/';
      }else{
        fileName[v] = '\0';
      }
    }
    sprintf(fileNum,"%04d",cloudFileCount);
    strcat(fileName,fileNum);
    strcat(fileName,".txt");
    if(!SD.exists(fileName)){
      fileNameUnique = true;   
    }
  }
  
  configFile = SD.open(fileName,FILE_WRITE);
  if (configFile) {
    strcpy(currentCloudFileName, fileName);                      //Store global filename for error handling
    configFile.print(receivedChars);
    configFile.print(";LAT;");
    configFile.print(latitude);
    configFile.print(";LON;");
    configFile.print(longitude);
///////DEBUG ¬remove
    Serial.print(receivedChars);
    Serial.print(";LAT;");
    Serial.print(latitude);
    Serial.print(";LON;");
    Serial.println(longitude);
///////DEBUG
    
    Serial.print(F("Added '"));
    Serial.print(fileName);
    Serial.println(F("' Queue"));
    configFile.close();
    writeIntIntoEEPROM(CLOUD_FILE_EEPROM_ADDRESS, cloudFileCount);
    return true;
  }else{
    Serial.print(F("Writing file '"));
    Serial.print(fileName);
    Serial.print(F("' to "));
    Serial.print(SD_CLOUD_TX_DIR);
    Serial.println(F(" directory FAILED"));
    configFile.close();
    isSDCardValid=false;
    return false;
  }
  return true;
}


/*
 *  Add a received message to the SMS Queue
 */
bool queueSMSMessage(char receivedChars[],char latitude[], char longitude[]){
  // Save message for transmitting
  bool fileNameUnique = false;
  char fileName[SMS_TX_FILE_LENGTH];  
  while(!fileNameUnique){
    smsFileCount++;
    if(smsFileCount > MAX_FILE_NUM)smsFileCount = 1;  //Reset file number sequence
    // reset the fileName
    for (int v = 0; v <= sizeof(fileName); v++) {                           // Add directory tx/ to filename
      if(v==0){
        fileName[v]='t';
      }else if(v==1){
        fileName[v]='x';
      }else if(v==2){
        fileName[v]='/';
      }else if(v==3){
        fileName[v]='s';
      }else if(v==4){
        fileName[v]='m';
      }else if(v==5){
        fileName[v]='s';
      }else if(v==6){
        fileName[v]='/';
      }else{
        fileName[v] = '\0';
      }
    }
    sprintf(fileNum,"%04d",smsFileCount);
    strcat(fileName,fileNum);
    strcat(fileName,".txt");
    if(!SD.exists(fileName)){
      fileNameUnique = true;   
    }
  }
  
  configFile = SD.open(fileName,FILE_WRITE);
  if (configFile) {
    strcpy(currentSMSFileName, fileName);                      //Store global filename for error handling
    configFile.print(receivedChars);
    configFile.print(";LAT;");
    configFile.print(latitude);
    configFile.print(";LON;");
    configFile.print(longitude);
///////DEBUG ¬remove
    Serial.print(receivedChars);
    Serial.print(";LAT;");
    Serial.print(latitude);
    Serial.print(";LON;");
    Serial.println(longitude);
///////DEBUG
    Serial.print(F("Added '"));
    Serial.print(fileName);
    Serial.println(F("' Queue"));
    configFile.close();
    writeIntIntoEEPROM(SMS_FILE_EEPROM_ADDRESS, smsFileCount);
    return true;
  }else{
    Serial.print(F("Writing file '"));
    Serial.print(fileName);
    Serial.print(F("' to "));
    Serial.print(SD_SMS_TX_DIR);
    Serial.println(F(" directory FAILED"));
    configFile.close();
    isSDCardValid=false;
    return false;
  }
  return true;
}


// FileNameToRemove must include the full path to the file that must be deleted.
bool removeFileFromSDCard(char fileNameToRemove[]){
  Serial.print("Removing ");
  Serial.println(fileNameToRemove);
  if(SD.exists(fileNameToRemove)){
    ///////DEBUG ¬remove
    File tmp = SD.open(fileNameToRemove);
    while (tmp.available()) {
      Serial.print((char)tmp.read());
    }
    tmp.close();
    Serial.println();
    ///////DEBUG
    
    return SD.remove(fileNameToRemove);
  }
  else{ 
    Serial.print(F("removeFileFromSDCard - could not find file: "));
    Serial.println(fileNameToRemove);
    isSDCardValid=false;
    return false;
  }
}

bool logMessage(char prefix[], char receivedChars[]){
  // Save message in log
  configFile = SD.open("rx/log.txt",FILE_WRITE);
  if (configFile) {
    configFile.print(millis());
    configFile.print("  ");
    configFile.print(prefix);
    configFile.print(": ");
    configFile.println(receivedChars);
  }else{
    configFile.close();
    return false;
  }
  configFile.close();

  return true;
}

// sends data to cloud. gross S,A,B,C net S,A,B,C
bool printToCloud(){
  int gprsCounter = 0;
  int getCounter = 0;
  int responseCounter = 0;
  int waitForNetworkCounter = 0;
  bool GETSuccess = false;

//If SIM808 is not responding turn in ON
  if(modem.testAT()==0){
    Serial.println(F("Starting SIM808"));
    startSIM808();
    Serial.println(F("SIM808 started"));
  }
  delay(500);
  modem.setBaud(MODEM_BAUD);                // set the baud rate to 9600
  delay(1500);
  
  Serial.print(F("Waiting for network..."));
  delay(200);
  while(!modem.waitForNetwork()) {
    Serial.println(F("in printTtoCloud 3"));
    delay(300);
    waitForNetworkCounter++;
    delay(10000);
    Serial.print(F("10s.."));
    if(waitForNetworkCounter>3){
      Serial.println(F("...failed"));      
      return false;
    }
  }
  Serial.println(F("....success"));

  if (modem.isNetworkConnected()) { Serial.println(F("Network connected")); }
  
  while(!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    gprsCounter++;
    Serial.println(F("GPRS connection fail...attempt again"));
    delay(10000);
    if(gprsCounter>3){
      Serial.println(F("GPRS connection failed"));  
      return false;
    }
  }
  Serial.println(F("GPRS success"));

  String lnk="/update?api_key=";
  lnk.concat("D2VRQOI0NNQZPXGG");  //apiKey); //add API key 
  // Add gross weights
  for(int z = 0 ; z < NUM_SENSORS ; z++){
    if(cMessage.sensors[z].id[0] == 'Z'){
      continue;   //Sensor not in use so skip to next one
    }
    lnk.concat("&field");
    lnk.concat(z+1);
    lnk.concat("=");
    lnk.concat(String(cMessage.sensors[z].grossWeight));
  }

  // Add net weights
  for(int y = 0 ; y < NUM_SENSORS ; y++){
    if(cMessage.sensors[y].id[0] == 'Z'){
      continue;   //Sensor not in use so skip to next one
    }
    lnk.concat("&field");
    lnk.concat(y+1+cMessage.numSensors);
    lnk.concat("=");
    lnk.concat(String(cMessage.sensors[y].netWeight));
  }

  char resource[(lnk.length())+1];
  lnk.toCharArray(resource,(lnk.length()+1));
  Serial.println(resource); // if u want to see the GET request uncomment this line
  
  Serial.println(F("Performing HTTP GET request... "));
  int err = http.get(resource);
  Serial.print(F("http.get() response: "));  Serial.println(err);
  while(err != 0) {    // Failed to complete GET, retry!
    getCounter++; 
    Serial.print(F("failed to connect...attempt "));
    Serial.println(getCounter);
    err = http.get(resource);
    delay(2000);
    if(getCounter>4){
      Serial.println(F("GET request failed 4 times. returning false"));
      http.stop();
      modem.gprsDisconnect();
      return false;
    }
  }
  if(err==0)GETSuccess =true;
/*
The following codes could be received from our library
static const int HTTP_SUCCESS = 0; 
static const int HTTP_ERROR_CONNECTION_FAILED = -1;  // The end of the headers has been reached.  This consumes the '\n'. Could not connect to the server
static const int HTTP_ERROR_API = -2;          // This call was made when the HttpClient class wasn't expecting it to be called.  Usually indicates your code is using the class incorrectly
static const int HTTP_ERROR_TIMED_OUT = -3;    // Spent too long waiting for a reply
static const int HTTP_ERROR_INVALID_RESPONSE = -4;   // The response from the server is invalid, is it definitely an HTTP server?

Also can receive standard HTTP responses e.g 200
*/
  int status = http.responseStatusCode();
  Serial.print(F("Response status code: "));
  Serial.println(status);
  if(GETSuccess && status != 200){
    while(status != 200){
      responseCounter++;
      http.stop();
      err = http.get(resource);
      Serial.print(F("re-attempt http.get() response: "));
      Serial.println(err);
      status = http.responseStatusCode();
      if(responseCounter >= 3){
        Serial.println(F("HTTP GET FAILED"));
          http.stop();
          modem.gprsDisconnect();
        return false;
      }
    }
  }

  
  if (!status) {
    //delay(10000);
    http.stop();
    modem.gprsDisconnect();
    return false;
  }

  http.stop();
  Serial.println(F("Server disconnected"));

  modem.gprsDisconnect();
  Serial.println(F("GPRS disconnected"));

  return true;
  
}

// sends data to cloud. gross S,A,B,C, grosstotal ,datetime, uid/truckid, lat/long
bool printToCloud2(){
  int gprsCounter2 = 0;
  int getCounter2 = 0;
  int responseCounter2 = 0;
  int waitForNetworkCounter2 = 0;
  bool GETSuccess2 = false;

  //If SIM808 is not responding turn in ON ¬TODO make this a function that can be called anywhere
  if(modem.testAT()==0){
    Serial.println(F("Starting SIM808"));
    startSIM808();
    Serial.println(F("SIM808 started"));
  }
  delay(500);
  modem.setBaud(MODEM_BAUD);                // set the baud rate to 9600
  delay(1500);
  
  Serial.print(F("Waiting for network..."));
  delay(200);
  while(!modem.waitForNetwork()) {
    delay(300);
    waitForNetworkCounter2++;
    delay(10000);
    Serial.print(F("10s.."));
    if(waitForNetworkCounter2>3){
      Serial.println(F("...failed"));      
      return false;
    }
  }
  Serial.println(F("....success"));

  if (modem.isNetworkConnected()) { Serial.println(F("Network connected")); }
  
  while(!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    gprsCounter2++;
    Serial.println(F("GPRS connection fail...attempt again"));
    delay(10000);
    if(gprsCounter2>3){
      Serial.println(F("GPRS connection failed"));  
      return false;
    }
  }
  Serial.println(F("GPRS success"));

  String lnk2="/update?api_key=";    
  if(apiKey[2]== '\0'){
    lnk2.concat("AKXL977L1CFIKPMI");  
  }else{
    lnk2.concat(apiKey); //add API key 
  }

  // Add gross weights
  for(int z = 0 ; z < NUM_SENSORS ; z++){
    if(cMessage.sensors[z].id[0] == 'Z'){
      continue;   //Sensor not in use so skip to next one
    }
    lnk2.concat("&field");
    lnk2.concat(z+1);
    lnk2.concat("=");
    lnk2.concat(String(cMessage.sensors[z].grossWeight));
  }

  //Gross Total
  lnk2.concat("&field5=");
  lnk2.concat(String(cMessage.grossWeightTotal));

  //DateTime
  lnk2.concat("&field6=");
  String datTim = urlEncodeDateTime();
  lnk2.concat(datTim);

  // Message ID and TruckID
  String uidTruck = "";
  uidTruck.concat(String(cMessage.uid));
  uidTruck.concat("%2F");   // Add '/' separator between uid and truckId
  uidTruck.concat(String(cMessage.truckId));
  lnk2.concat("&field7=");
  lnk2.concat(urlEncodeUIDTruckID(uidTruck)); 
  
  
  //Latitude/Longitude
  String ll = urlEncodeLatitudeLongitude();
  lnk2.concat("&field8=");
  lnk2.concat(ll);

  char resource2[(lnk2.length())+1];

  lnk2.toCharArray(resource2,(lnk2.length()+1));
  Serial.println(resource2);
  int err2 = http.get(resource2);
  Serial.print(F("http.get() connected: ")); Serial.print((err2 == 0) ? "Success (" : "Failed ("); Serial.print(err2); Serial.println(")");

  //////debug
  int statusDebug = http.responseStatusCode();
  Serial.print(F("Debug:Response status code: "));
  Serial.println(statusDebug);
  //////debug
  
  while(err2 != 0) {    // Failed to complete GET, retry!
    getCounter2++; 
    Serial.print(F("failed to connect...attempt "));
    Serial.println(getCounter2);
    err2 = http.get(resource2);
    delay(2000);
    if(getCounter2>4){
      Serial.println(F("GET request failed 4 times. returning false"));
      http.stop();
      modem.gprsDisconnect();
      return false;
    }
  }
  if(err2==0)GETSuccess2 =true;
/*
The following codes could be received from our library
static const int HTTP_SUCCESS =0; 
static const int HTTP_ERROR_CONNECTION_FAILED =-1;  // The end of the headers has been reached.  This consumes the '\n'. Could not connect to the server
static const int HTTP_ERROR_API =-2;          // This call was made when the HttpClient class wasn't expecting it to be called.  Usually indicates your code is using the class incorrectly
static const int HTTP_ERROR_TIMED_OUT =-3;    // Spent too long waiting for a reply
static const int HTTP_ERROR_INVALID_RESPONSE =-4;   // The response from the server is invalid, is it definitely an HTTP server?

Also can receive standard HTTP responses e.g 200
*/
  int status2 = http.responseStatusCode();
  Serial.print(F("Response status code: "));
  Serial.println(status2);
  if(GETSuccess2 && status2 != 200){
    while(status2 != 200){
      responseCounter2++;
      http.stop();
      err2 = http.get(resource2);
      Serial.print(F("re-attempt http.get() response: "));
      Serial.println(err2);
      status2 = http.responseStatusCode();
      if(responseCounter2 >= 3){
        Serial.println(F("HTTP GET FAILED"));
        http.stop();
        modem.gprsDisconnect();
        return false;
      }
    }
  }

  
  if (!status2) {
    //delay(10000);
    http.stop();
    modem.gprsDisconnect();
    return false;
  }

  http.stop();
  Serial.println(F("Server disconnected"));

  modem.gprsDisconnect();
  Serial.println(F("GPRS disconnected"));

  return true;
  
}

// Return cMessage.dateTime as a String in format dd%2Dmm%2Dyyyy%20HH%3AMM%3ASS
String urlEncodeDateTime(){
  String retDateTime;
  if(cMessage.dateTime[0] != '\0'){
    retDateTime="";
    //06-06-2022 13:23:23
    char dti;
    
    for(int dTime = 0; dTime < strlen(cMessage.dateTime); dTime++){
      dti = cMessage.dateTime[dTime];
      if(dti == '-'){
        retDateTime.concat("%2D");
      }else if(dti == ' '){
        retDateTime.concat("%20");
      }else if(dti == ':'){
        retDateTime.concat("%3A");
      }else{
        retDateTime.concat(dti);
      }
    }
  }else{
    retDateTime = "N%2FA";
  }
  return retDateTime;
}


// return uid and truckid string in format -25.123456/28.123456
String urlEncodeLatitudeLongitude(){
  String strLatLong = "N%2FA%2FN%2FA";
  if(cMessage.latitude[0] != '\0' && cMessage.longitude[0] != '\0'){
    if(cMessage.latitude[0] == 'N')return strLatLong;
    strLatLong = "";
    char latloni;
    for(int lati = 0; lati < strlen(cMessage.latitude); lati++){
        latloni = cMessage.latitude[lati];
        if(latloni == '-'){
          strLatLong.concat("%2D");
        }else if(latloni == '.'){
          strLatLong.concat("%2E");
        }else{
          strLatLong.concat(latloni);
        }
    }
    strLatLong.concat("%2F");   // Add '/' separator between latitude and longitude
    
    for(int longi = 0; longi < strlen(cMessage.longitude); longi++){
        latloni = cMessage.longitude[longi];
        if(latloni == '-'){
          strLatLong.concat("%2D");
        }else if(latloni == '.'){
          strLatLong.concat("%2E");
        }else{
          strLatLong.concat(latloni);
        }
    }
  }
  return strLatLong;
}

// encode hyphens, slash, spaces and underscores
String urlEncodeUIDTruckID(String uidTruck){
  String strUidTruckID = "N%2FA%2FN%2FA";
  if(cMessage.uid[0] != '\0' && cMessage.truckId[0] != '\0'){
    strUidTruckID = "";
    char tempChar;
    for(unsigned int cnt = 0; cnt < uidTruck.length(); cnt++){
      tempChar = uidTruck.charAt(cnt);
      if(tempChar == '-'){
        strUidTruckID.concat("%2D");
      }else if(tempChar == '_'){
        strUidTruckID.concat("%5F");
      }else if(tempChar == ' '){
        strUidTruckID.concat("%20");
      }else{
        strUidTruckID.concat(tempChar);
      }
    }
  }    
  return strUidTruckID;
}

// Turn the SIM808 module ON
bool startSIM808(){
  Serial.println(F("Toggling power to modem"));
  digitalWrite(SIM808_POWER_PIN,HIGH);
  delay(1300);
  digitalWrite(SIM808_POWER_PIN,LOW);
  delay(500);
  modem.setBaud(MODEM_BAUD);
  delay(500);
}

// Restart the SIM808 module
bool restartSIM808(){
  Serial.println(F("Restarting modem"));
  //Turn OFF
  digitalWrite(SIM808_POWER_PIN,HIGH);
  delay(1300);
  digitalWrite(SIM808_POWER_PIN,LOW);
  Serial.println(F("SIM808 is OFF"));        
   //Turn ON
  delay(500);
  digitalWrite(SIM808_POWER_PIN,HIGH);
  delay(1300);
  digitalWrite(SIM808_POWER_PIN,LOW);
  Serial.println(F("SIM808 is ON"));
  delay(500);
  modem.setBaud(MODEM_BAUD);
  delay(500);
}

/* 
 *  Parse cleral data and assign to global structs.
 *  
 *  Ascii Message Format
 *  UID;1;TIME;2022-03-23 21:44:52;UNIT;kg;       Header information
 *  GWM;5960;GWA;11980;GWB;6960;                  Gross weights
 *  NTM;3960;NTA;7980;NTB;3960;                   Net Weights
 *  GWT;24900;NTT;15900;                          Gross and Net totals
 *  IDM;0000;IDA;B226;IDB;ACFD;                   Sensor IDs
 *  CKS;92                                        check sum
 *  LAT;-25.814693;LON;28.154558                  ERROR HANDLING ONLY. The latitude and longitude will already be in the message.
*/
bool parseData() {
    char * strtokIndx;        // this pointer is used by strtok() as an index
    int dataCount = 1;        // used to keep track of data pos. e.g key or value
    int arrayPos = 0;         // used to insert to correct position in array
    char *cleralData = NULL;
    cleralData = strtok(tempChars,cleralDelimiter); 
    char key[5] = {0};
    char val[23] = {0};
    char check[30]={0};
    char tc[2] = {0};
    clearcMessageStruct();
    strcpy(cMessage.truckId,truckID);  
    while(cleralData != NULL){

      if(dataCount %2 ==0 ){
        strcpy(val,cleralData);

        // Read Total Gross Weight
        if(( key[0] == 'G' ) && ( key[1] == 'W' )&& ( key[2] == 'T' )){
          strcpy(cMessage.grossWeightTotal, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }
        // Read Total Net Weight
        if(( key[0] == 'N' ) && ( key[1] == 'T' )&& ( key[2] == 'T' )){
          strcpy(cMessage.netWeightTotal, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }
        // Read Checksum
        if(( key[0] == 'C' ) && ( key[1] == 'K' )&& ( key[2] == 'S' )){
          strcpy(cMessage.checksum, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }
        // Read Message ID
        if(( key[0] == 'U' ) && ( key[1] == 'I' )&& ( key[2] == 'D' )){
          strcpy(cMessage.uid, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }
        if (( key[0] == 'T' ) && ( key[1] == 'I' )&& ( key[2] == 'M' )){
          strcpy(cMessage.dateTime, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }
        if (( key[0] == 'U' ) && ( key[1] == 'N' )&& ( key[2] == 'I' )){
          strcpy(cMessage.unit, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }

        // ERROR Handling. read latitude
        if (( key[0] == 'L' ) && ( key[1] == 'A' )&& ( key[2] == 'T' )){
          strcpy(cMessage.latitude, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }

        // ERROR Handling. read latitude
        if (( key[0] == 'L' ) && ( key[1] == 'O' )&& ( key[2] == 'N' )){
          strcpy(cMessage.longitude, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }

        // Sensor Data
        // Read a Sensor's Gross Weight
        if (( key[0] == 'G' ) && ( key[1] == 'W' )){
          int p = getSensorPos(key[2]);
          char sc = key[2];
          if(sc == 'M')sc = 'S';
          if(cMessage.sensors[p].id[0] == 'Z'){
            memset(tc, '\0', sizeof(tc));
            tc[0] = sc;
            tc[1] = '\0';
            strcpy(cMessage.sensors[p].id, tc);
          }
          strcpy(cMessage.sensors[p].grossWeight, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }
        // Read a Sensor's Net Weight
        if (( key[0] == 'N' ) && ( key[1] == 'T' )){
          int p = getSensorPos(key[2]);
          strcpy(cMessage.sensors[p].netWeight, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }
        // Read a Sensor' ID
        if (( key[0] == 'I' ) && ( key[1] == 'D' )){
          int p = getSensorPos(key[2]);
          strcpy(cMessage.sensors[p].sensorId, val);
          cleralData = strtok(NULL,cleralDelimiter);
          dataCount++;
          continue;
        }

         arrayPos++;
      }else{
         strcpy(key, cleralData); 
      }
      
      cleralData = strtok(NULL,cleralDelimiter);
      dataCount++;
    }
    cMessage.numSensors = getNumberOfSensors();
    return true;
}

int getNumberOfSensors(){
   int numSensors = 0; 
   for(int c = 0 ; c < NUM_SENSORS ; c++){
      if(cMessage.sensors[c].id[0] == 'Z'){
        continue;   //Sensor not in use so skip to next one
      }
      numSensors++;
   }
   return numSensors;
}

// Clear the contents of the global message (cMessage)
bool clearcMessageStruct() {
  memset(cMessage.truckId, '\0', sizeof(cMessage.truckId));
  memset(cMessage.grossWeightTotal, '\0', sizeof(cMessage.grossWeightTotal));
  memset(cMessage.netWeightTotal, '\0', sizeof(cMessage.netWeightTotal));
  memset(cMessage.checksum, '\0', sizeof(cMessage.checksum));
  memset(cMessage.uid, '\0', sizeof(cMessage.uid));
  memset(cMessage.dateTime, '\0', sizeof(cMessage.dateTime));
  memset(cMessage.unit, '\0', sizeof(cMessage.unit));
  memset(cMessage.latitude, '\0', sizeof(cMessage.latitude));
  memset(cMessage.longitude, '\0', sizeof(cMessage.longitude));
  cMessage.numSensors = 0;
  for(int m = 0 ; m < NUM_SENSORS; m++){
    cMessage.sensors[m].id[0] = 'Z';                                                         // 'Z' indicates that no data has been added to the sensor yet
    cMessage.sensors[m].id[1] = '\0';                                                         // '0' indicates the end of the string
    memset(cMessage.sensors[m].grossWeight, '\0', sizeof(cMessage.sensors[m].grossWeight));
    memset(cMessage.sensors[m].netWeight, '\0', sizeof(cMessage.sensors[m].netWeight));
    memset(cMessage.sensors[m].sensorId, '\0', sizeof(cMessage.sensors[m].sensorId));
  }
  return true;
}




int getSensorPos(char sensorChar){
  if(sensorChar == 'M')sensorChar = 'S';
  for( int c = 0 ; c < NUM_SENSORS ; c++ ){
    if(positions[c] == sensorChar) return c;
  }
  return -1;
}


/*
 *  send data via SMS to all listed recipients 
 *  return int  -  0 = All Successful, 1 = one failure, 2 = all failed
 * 
 */
int transmitSMS(){
  int smsSendStatus = 0;
  char msg[500] = "";
  strcat(msg,"WTS Weight Info");
  strcat(msg,"\n");
  
  strcat(msg,"Truck:    ");
  strcat(msg,cMessage.truckId);
  strcat(msg,"\n");

  strcat(msg,"Date:     ");
  strcat(msg,cMessage.dateTime);
  strcat(msg,"\n");
  strcat(msg,"number of channels:     ");
  char buf[2];
  strcat(msg,itoa(cMessage.numSensors,buf,10));
  strcat(msg,"\n");
  
  strcat(msg,"Unit:     ");
  strcat(msg,cMessage.unit);
  strcat(msg,"\n");
  strcat(msg,"\n");
  
  strcat(msg,"Channel         GWT");
  strcat(msg,"\n");
  
  for(int c = 0 ; c < NUM_SENSORS ; c++){
    if(cMessage.sensors[c].id[0] == 'Z'){
      continue;   //Sensor not in use so skip to next one
    }
    strcat(msg,"     ");
    strcat(msg,cMessage.sensors[c].id);
    strcat(msg,"               ");
    strcat(msg,cMessage.sensors[c].grossWeight);
    strcat(msg,"\n");
  }
  strcat(msg,"\n");
  strcat(msg,"Total         ");
  strcat(msg,cMessage.grossWeightTotal);
  strcat(msg,"\n");
  strcat(msg,"\n");
  strcat(msg,"Truck Position");
  strcat(msg,"\n");
 // sprintf(tempMsg, "http://maps.google.com/maps?q=%s,%s\n", cMessage.latitude, cMessage.longitude);
  delay(300);
  strcat(msg,"Lat: ");
  strcat(msg,cMessage.latitude);

  strcat(msg,"\n");
  strcat(msg,"Long: ");
  strcat(msg,cMessage.longitude);

  strcat(msg,"\n");
  if(gotGpsData){
    strcat(msg,"http://maps.google.com/maps?q=");
    strcat(msg,cMessage.latitude);
    strcat(msg,",");
    
    strcat(msg,cMessage.longitude);
    strcat(msg,"\n");
  }

  for(int sm = 0; sm < (sizeof(smsRecipients) / sizeof(smsRecipients[0])) ; sm++){
    if(smsRecipients[sm][0] == 'Z'){
      continue;
    }
    if(!sendSMS(msg,smsRecipients[sm])){
      Serial.print(F("transSMS() to "));
      Serial.print(smsRecipients[sm]);
      Serial.println(F(" failed"));
      smsSendStatus++;
    }
  }
  return smsSendStatus;
}

bool sendSMS(char msg[],char number[]){
  int smsResendCounter = 0;
  Serial.print(F("Sending SMS..."));
  Serial.println(number); 
  Serial.print(F("Signal: "));
  Serial.println(modem.getSignalQuality());
  bool res = modem.sendSMS(String(number), String(msg));
  while(!res){
    Serial.println(F("SMS Failed....Resending"));
    smsResendCounter++;
    res = modem.sendSMS(String(number), String(msg));
    if(smsResendCounter > 2) break;
    delay(500);
  }
  Serial.print(F("SMS: "));
  if(res){
    Serial.println(F("OK"));
    return true;
  }else{
    Serial.println(F("Fail"));
    saveSMS(msg);
    return false;
  }
}

//@TODO save failed SMSes to SD card for later attempt.
bool saveSMS(char msg[]){
  return true;
}

/*
bool sendSMS(char msg[]){
  Serial.print(F("Sending SMS..."));               
  Serial.println(smsNumber);               

  
  if( sim808.sendSMS(smsNumber,msg) ){
    Serial.println();
    Serial.println(F("SMS transmission SUCCESS"));
  }else{
    Serial.println();
    Serial.println(F("SMS transmission FAILED"));
     saveSMSMessageForRetry(msg);   // store the sms for resending later.
  }
  
  delay(500);
  Serial.println(F("end sendSMS"));
  return true;
}
*/

// Store an SMS message for resending at a later stage
bool saveSMSMessageForRetry(char msg[]){
//  Serial.println(F("save sms  "));
//  Serial.println(sizeof(msg));
//  Serial.println(msg);
  
  return true;
}


bool printData(){
  int btFailCount = 0;
  Serial.println(F("PrintData()"));
  printer.wake();       // MUST wake() before printing again, even if reset
  delay(400);
  
  //////////////NEW CODE//////////////////
  int btState = digitalRead(BT_STATE_PIN);
  Serial.print(F("btState: "));
  Serial.println(btState);
  if(prevBTState != btState){
   // Serial.println(F("Bluetooth Connection State Change"));
    if(btState == BT_CONNECTED){
      Serial.print(btState);
      Serial.println(F(" : Bluetooth Connected"));  
      prevBTState = BT_CONNECTED;
    }else{  
      Serial.print(btState);
      Serial.println(F(" : Bluetooth Disconnected"));
      prevBTState = BT_DISCONNECTED;
    }
  }
  while(btState == BT_DISCONNECTED){
    if((millis() - connectTime > 10000)){
      btFailCount++;
      digitalWrite(BT_POWER_PIN,LOW);
      Serial.println(F("Restarting bluetooth...."));
      delay(2000);
      digitalWrite(BT_POWER_PIN,HIGH);
      Serial.println(F("Bluetooth Restarted"));
      digitalWrite(BT_STATE_PIN,LOW);
      connectTime = millis();
      prevBTState = BT_DISCONNECTED;
      delay(200);
    }
    btState = digitalRead(BT_STATE_PIN);
    if(btFailCount > 4){
      Serial.println(F("Bluetooth can't connect to printer."));
      return false;
    }
    btState = digitalRead(BT_STATE_PIN);
  }
  //////////////END NEW CODE//////////////////
  delay(100);
  printerSerial.println("      -------------------");
  printerSerial.println("        WTS Weight Info");
  printerSerial.println("      -------------------");
  printerSerial.println();
  printerSerial.print("Vehicle:  ");
  printerSerial.println(cMessage.truckId);
  printerSerial.print("Date:   ");
  printerSerial.println(cMessage.dateTime);
  printerSerial.print("number of channels:   ");
  printerSerial.println(cMessage.numSensors);
  printerSerial.print("Unit:   ");
  printerSerial.println(cMessage.unit);
  printerSerial.println();
  printerSerial.println("Channel   Gross Weight");

  for(int c = 0 ; c < NUM_SENSORS ; c++){
    if(cMessage.sensors[c].id[0] == 'Z'){
      continue;   //Sensor not in use so skip to next one
    }
    printerSerial.print("   ");
    printerSerial.print(cMessage.sensors[c].id);
    printerSerial.print("         ");
    printerSerial.println(cMessage.sensors[c].grossWeight);
  }
  printerSerial.println();
  printerSerial.print("Total         ");
  printerSerial.println(cMessage.grossWeightTotal);
  printerSerial.println();
  printerSerial.println("Truck Position");
  printerSerial.print("Latitude:   ");
  printerSerial.println(cMessage.latitude);
  printerSerial.print("Longitude:   ");
  printerSerial.print(cMessage.longitude);
  printerSerial.println(" ");
  printerSerial.println(" ");
  printerSerial.println(" ");
  
  //printer.sleep();      // Tell printer to sleep
  //delay(3000L);         // Sleep for 3 seconds
  //printer.wake();       // MUST wake() before printing again, even if reset
  //printer.feed(2);
  Serial.println(F("PrintData() end"));
  return true;
  
}

bool printDataORIG(){
  Serial.println(F("Printing via BLE Printer"));      
  printer.wake();       // MUST wake() before printing again, even if reset
  delay(400);
  int btFailCount = 0;
  long printDelay = 100L;
  
  
  //////////////NEW CODE//////////////////
  int btState = digitalRead(BT_STATE_PIN);
  Serial.print(F("btState: "));
  Serial.println(btState);
  if(prevBTState != btState){
   // Serial.println(F("Bluetooth Connection State Change"));
    if(btState == BT_CONNECTED){
      Serial.print(btState);
      Serial.println(F(" : Bluetooth Connected"));  
      prevBTState = BT_CONNECTED;
    }else{  
      Serial.print(btState);
      Serial.println(F(" : Bluetooth Disconnected"));
      prevBTState = BT_DISCONNECTED;
    }
  }
  while(btState == BT_DISCONNECTED){
    if((millis() - connectTime > 10000)){
      btFailCount++;
      digitalWrite(BT_POWER_PIN,LOW);
      Serial.println(F("Restarting bluetooth...."));
      delay(2000);
      digitalWrite(BT_POWER_PIN,HIGH);
      Serial.println(F("Bluetooth Restarted"));
      //digitalWrite(BT_STATE_PIN,LOW);
      connectTime = millis();
      prevBTState = BT_DISCONNECTED;
      delay(200);
    }
    if(btFailCount > 4){
      Serial.println(F("Bluetooth can't connect to printer."));
      return false;
    }
    btState = digitalRead(BT_STATE_PIN);
    Serial.print(F("Bluetooth state: "));  
    Serial.println(btState);
    delay(1000);
    btState = digitalRead(BT_STATE_PIN);
    Serial.print(F("Bluetooth state: "));
    Serial.println(btState);
delay(1000);
        btState = digitalRead(BT_STATE_PIN);
    Serial.print(F("Bluetooth state: "));
    Serial.println(btState);
delay(1000);
        btState = digitalRead(BT_STATE_PIN);
    Serial.print(F("Bluetooth state: "));
    Serial.println(btState);
    delay(1000);
    
  }
  //////////////END NEW CODE//////////////////

  
  printerSerial.println("      -------------------");
  delay(printDelay);
  printerSerial.println("        WTS Weight Info");
  delay(printDelay);
  printerSerial.println("      -------------------");
  delay(printDelay);
  printerSerial.println();
  delay(printDelay);
  printerSerial.print("Vehicle:  ");
  delay(printDelay);
  printerSerial.println(cMessage.truckId);
  delay(printDelay);
  printerSerial.print("Date:   ");
  delay(printDelay);
  printerSerial.println(cMessage.dateTime);
  delay(printDelay);
  printerSerial.print("number of channels:   ");
  delay(printDelay);
  printerSerial.println(cMessage.numSensors);
  delay(printDelay);    
  printerSerial.print("Unit:   ");
  delay(printDelay);
  printerSerial.println(cMessage.unit);
  delay(printDelay);
  printerSerial.println();
  delay(printDelay);
  printerSerial.println("Channel   Gross Weight");
  delay(printDelay);

  for(int c = 0 ; c < NUM_SENSORS ; c++){
    if(cMessage.sensors[c].id[0] == 'Z'){
      continue;   //Sensor not in use so skip to next one
    }
    printerSerial.print("   ");
    delay(printDelay);
    printerSerial.print(cMessage.sensors[c].id);
    delay(printDelay);
    printerSerial.print("         ");
    delay(printDelay);
    printerSerial.println(cMessage.sensors[c].grossWeight);
    delay(printDelay);
  }
  printerSerial.println();
  printerSerial.print("Total         ");
  delay(printDelay);
  printerSerial.println(cMessage.grossWeightTotal);
  delay(printDelay);
  
  printerSerial.println();
  delay(printDelay);
  printerSerial.println("Truck Position");
  delay(printDelay);
  printerSerial.print("Latitude:   ");
  delay(printDelay);
  printerSerial.println(cMessage.latitude);
  delay(printDelay);
  printerSerial.print("Longitude:   ");
  delay(printDelay);
  printerSerial.print(cMessage.longitude);
  delay(printDelay);
  printerSerial.println(" ");
  delay(printDelay);
  printerSerial.println(" ");
  delay(printDelay);
  printerSerial.println(" ");
  delay(printDelay);
  
  //printer.sleep();      // Tell printer to sleep
  //delay(3000L);         // Sleep for 3 seconds
  //printer.wake();       // MUST wake() before printing again, even if reset
  //printer.feed(2);

  Serial.println(F("end printData"));
  return true;
}

// get the GPS location and update cMessage.
bool getLocation(){
  bool gotGPS = false;
  int counter = 0;

  //modem.enableGPS();
  //delay(15000L);
  float lat2      = 0;
  float lon2      = 0;
  float speed2    = 0;
  float alt2      = 0;
  int   vsat2     = 0;
  int   usat2     = 0;
  float accuracy2 = 0;
  int   year2     = 0;
  int   month2    = 0;
  int   day2      = 0;
  int   hour2     = 0;
  int   min2      = 0;
  int   sec2      = 0;
  for (int8_t i = GPS_RETRY_LIMIT; i; i--) {
    Serial.println(F("Requesting current GPS location"));
    if (modem.getGPS(&lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,&year2, &month2, &day2, &hour2, &min2, &sec2)) {
      float la = lat2;
      float lo = lon2;
      float ws = speed2;
    
      dtostrf(la, 6, 6, lat); //put float value of la into char array of lat. 6 = number of digits before decimal sign. 2 = number of digits after the decimal sign.
      dtostrf(lo, 6, 6, lon); //put float value of lo into char array of lon
      dtostrf(ws, 6, 2, wspeed);  //put float value of ws into char array of wspeed

      strcpy(cMessage.latitude,lat);
      strcpy(cMessage.longitude,lon);
      
      gotGPS=true;   
      gotGpsData=true;    

      char dat[20];
      sprintf(dat, "%02d-%02d-%02d %02d:%02d:%02d", day2, month2, year2, (hour2 + 2), min2, sec2);
      counter=0;
      break;
    } else {
      Serial.println(F("Couldn't get GPS location, retrying in 5s."));
      delay(5000L);
      counter++;
      if(counter == GPS_RESTART_THRESHOLD){ 			//¬ use define value
        //Check if modem is running
        if(modem.testAT()==0){
          restartSIM808();
          if(modem.testAT()==0){
            Serial.println(F("SIM808 did not start"));
            isModemWorking = false;
          }else{
            Serial.println(F("SIM808 powered up"));
          }
        }else{
          Serial.print(F("Restart GPS: "));
          modem.disableGPS();
          delay(1000);
          gpsReady = modem.enableGPS();  
          Serial.println(gpsReady);      
        }
      }
      
    }
  }
  if(counter == 0){
    Serial.println(F("getLocation Successful"));
    return true;
  }else{
    Serial.println(F("getLocation Failed"));
    strcpy(cMessage.latitude,"N/A");
    strcpy(cMessage.longitude,"N/A" );
    return false;
  }
  //Serial.println(F("Disabling GPS"));
  //modem.disableGPS();
}

bool printToSerial(){
  Serial.println(F("*********PRINTING DATA**********"));
  Serial.print(F("Message ID: "));  Serial.println(cMessage.uid);
  Serial.print(F("Truck ID: "));    Serial.println(cMessage.truckId);
  Serial.print(F("date time: "));   Serial.println(cMessage.dateTime);
  Serial.print(F("Unit: "));        Serial.println(cMessage.unit);
  Serial.print(F("checksum: "));    Serial.println(cMessage.checksum);
  Serial.print(F("number of sensors: "));  Serial.println(cMessage.numSensors);
  Serial.print(F("Total NET: "));   Serial.println(cMessage.netWeightTotal);
  Serial.print(F("Total Gross: ")); Serial.println(cMessage.grossWeightTotal);
  for(int c = 0 ; c < NUM_SENSORS ; c++){
    if(cMessage.sensors[c].id[0] == 'Z'){
      continue;   //Sensor not in use so skip to next one
    }
    Serial.print(F(" Sensor:"));           Serial.println(cMessage.sensors[c].id);
    Serial.print(F("    Net Weight: "));   Serial.println(cMessage.sensors[c].netWeight);
    Serial.print(F("    Gross Weight: ")); Serial.println(cMessage.sensors[c].grossWeight);
    Serial.print(F("    Sensor ID: "));    Serial.println(cMessage.sensors[c].sensorId);
  }
  Serial.print(F("Latitude: "));   Serial.println(cMessage.latitude);
  Serial.print(F("Longitude: ")); Serial.println(cMessage.longitude);
  return true;
}

// sets the status LEDs. status=true will set green HIGH.
void setSystemStatusLED(bool status){
  if(status){
    digitalWrite(SystemReadyStatusPin,HIGH);
    digitalWrite(SystemBusyStatusPin,LOW);
  }else{
    digitalWrite(SystemReadyStatusPin,LOW);
    digitalWrite(SystemBusyStatusPin,HIGH);
  }
}
