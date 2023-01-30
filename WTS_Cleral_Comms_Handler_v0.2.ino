/* 
 *  Handle the incoming RS232 data from the Cleral system as well as the sending of messages to WTS_Controller.
 *  
 *  Note: Use a Nano Every board for this.
 *        SPI=> D11 (COPI), D12 (CIPO), D13 (SCK). Use any GPIO for Chip Select (CS).  
 *  
 *  v0.1 - Receive Cleral serial data and store in an array.
 *         Receive a WTS ready signal then send one message from the array via serial.
 *  v0.2 - Add an SD card module and use it to store the incoming messages.
 *         Retrieve messages from SD when sending to WTS_Controller
 *    
 */
#include <SoftwareSerial.h>                                   // used for communicating with WTS_Controller
#include <SPI.h>                                              // needed for SD card
#include <SD.h>                                               // used for file management on SD card
#include <EEPROM.h>                                           // used for storing filenames 

// Pin Definitions
#define OUT_SERIAL_TX_PIN 10                                  // pin of Arduino that connects to RX pin of the WTS_Controller
#define OUT_SERIAL_RX_PIN 9                                   // pin of Arduino that connects to TX pin of the WTS_Controller
#define RESET_PIN 3                                           // pin used to reset WTS_Controller
#define SystemReadyStatusPin 4                                // pin used to indicate if contoller is ready to receive data. 
#define SystemBusyStatusPin 5                                 // pin used to indicate if contoller busy initialising or processing message. 
#define SD_CS_PIN 8                                           // The Chip Select pin connected to CS on the SD Card Moudule


// Cleral Message Definitions
#define cleralMessageStartMarker 2                            // Cleral uses 0x02 to denote the start of a message
#define cleralMessageEndMarker 3                              // Cleral uses 0x03 to denote the start of a message
#define cleralDelimiter ";"                                   // Cleral cleralDelimiter for each field 
#define CLERAL_RECEIVED_NUM_CHARS 210                         // The max number of chars that can be recevied from Cleral

// WTS_Controller
#define WTS_RECEIVED_NUM_CHARS 3                              // The max number of chars that can be recevied from WTS Controller
#define RECEIVED_MSG_QUEUE_LEN 10                             // the number of messages to hold for later transmission

// Message Queue
#define SD_TX_DIR "tx"                                        // directory for storing the Cloud TX queue

// EEPROM - used to keep track of filename increments
#define FILE_EEPROM_ADDRESS 43
#define MAX_FILE_NUM 9999

// SD Card & Configuration
#define TX_FILE_LENGTH 12                                                 // Length of cloud tx filename which includes directory,filename and terminating \0
#define cleralRS232Serial Serial1                                         // Serial1 for RS232 comms.     Arduino TX1 pin to TX on RS232 module. RX1 to RX on RS232 module. 


SoftwareSerial outSerial(OUT_SERIAL_RX_PIN, OUT_SERIAL_TX_PIN);           // RX, TX. for Serial communications with the WTS_Controller 
File txFile;                                                              // the file used when storing a new message on the SD card
boolean newData = false;                                                  // Flag to indicate that a new message has arrived from Cleral.
                                                                                                                       
// RS232 message settings
char receivedChars[CLERAL_RECEIVED_NUM_CHARS];                            // Array for storing the characters received from Cleral. This represents one message.
char tempChars[CLERAL_RECEIVED_NUM_CHARS];                                // temporary array for use when sending   
static boolean recvInProgress = false;                                    // flag indicating if new message is being processed. 

// WTS controller message
boolean outNewData = false;                                               // Flag to indicate that a new message has arrived from WTS_Controller.
char outReceivedChars[4];                                                 // format 2<character>3.  'A' = ready to receive a new message. 'S' - successfully received last message, 'F' failed...resend
static boolean outRecvInProgress = false;                                 // flag indicating if new message request has been received. 

int  txFileCount = 0;                                                     // WTS Transmission file name sequence counter
char fileNum[5];                                                          // char representation of file sequence number
char currentTXFileName [TX_FILE_LENGTH];                                  // The name of the tx file currently being processed. 
char lastSentTXFileName [TX_FILE_LENGTH];                                 // The name of the last tranmitted tx file. 

long resetTime;  //¬ delete
bool debugResetFunction = false;   // USE TO TURN ON/OFF the reset loop  ¬delete

void setup() {
  digitalWrite(RESET_PIN,HIGH);                                           // set high immediatly prevent restart
  pinMode(SystemReadyStatusPin,OUTPUT);                                   // status LED to indicate system is ready to receive print
  pinMode(SystemBusyStatusPin,OUTPUT);                                    // status LED that indicates system is ready to receive print
  setSystemStatusLED(false);
  pinMode(OUT_SERIAL_TX_PIN,OUTPUT);
  pinMode(OUT_SERIAL_RX_PIN,INPUT);
  outSerial.begin(9600);                                                  // start software serial for communication with WTS_Controller
  Serial.begin(115200);                                                   // Serial monitor
  cleralRS232Serial.begin(9600);                                          // WTS hardware serial to receive Cleral RS232 data

  pinMode(RESET_PIN,OUTPUT);                                              // pin used for resetting the WTS_Controller


  
  // Obtain the last filenumber sequences
  txFileCount = readIntFromEEPROM(FILE_EEPROM_ADDRESS);
  Serial.println(__FILE__);                                               // Print the file location

  // SD Module Setup
  pinMode(SD_CS_PIN,OUTPUT);                                              // Chip Select Pin for SD Module. SPI pins MISO, MOSI and SCK are on pins 12, 11 and 13 respectively
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("initialization failed. Things to check:"));
    Serial.println(F("1. is a card inserted?"));
    Serial.println(F("2. is your wiring correct?"));
    Serial.println(F("3. did you change the chipSelect pin to match your shield or module?"));
    Serial.println(F("Note: press reset or reopen this serial monitor after fixing your issue!"));
    //¬ would be good to try restart the SD if possible
  }
  setSystemStatusLED(true);
  resetTime = millis();
  Serial.println(F("System Initialisation Complete"));
  outSerial.println(0xFF);
}

void loop() {
    processIncomingCleralMessage();
    if (newData == true) {
      Serial.println();
      setSystemStatusLED(false);                                            // Change the status LEDs to show BUSY
      if(!queueTXMessage(receivedChars)){
        Serial.println(F("Adding file to TX message queues FAILED"));
      }
      newData = false;
      // Clear receivedChars
      clearCharArray(receivedChars);
      //¬  for(int recChar = 0; recChar < sizeof(receivedChars); recChar++){
      //  receivedChars[recChar] = '\0';
      //}
      setSystemStatusLED(true);
      Serial.println(F("Ready..."));
    }

    processIncomingWTSMessage();
    if (outNewData == true) {
      Serial.println();
      setSystemStatusLED(false);
      bool readyToSend = false;
      byte msgNum = 0;

      //Check if msg = 'A' which indicates WTS_Controller is ready for new message.
      if(outReceivedChars[0] == 'A'){
        processTXQueue();
      }else if(outReceivedChars[0] == 'S'){                 //Successfully received
        // set a flag to indicate the status of last sent message.          
        Serial.println("LAST SENT MESSAGE SUCCESSFULLY RECEIVED");
        Serial.print(F("Removing file:'")); Serial.print(lastSentTXFileName);   Serial.print(F("' "));
        Serial.println(removeFileFromSDCard(lastSentTXFileName) ? "...Success" : "...Failed");
        for(int cChar = 0; cChar < sizeof(lastSentTXFileName); cChar++){
          lastSentTXFileName[cChar] = '\0';
        }
      }else if(outReceivedChars[0] == 'F'){                 // message not received properly. must resend
        // set a flag to indicate the status of last sent message.        
        Serial.println("LAST SENT MESSAGE FAILED - DON'T DELETE FILE");
      }else{
        // msg received is invalid
      }
                  
      outNewData = false;
      // Clear outReceivedChars
      clearCharArray(outReceivedChars);
      //for(int outChar = 0; outChar < sizeof(outReceivedChars); outChar++){
      //  outReceivedChars[outChar] = '\0';
     //¬ }
      setSystemStatusLED(true);
      Serial.println(F("Ready..."));
    }

    // Watchdog functionality for restarting the WTS_Controller Mega board. Only runs if you set debugResetFunction to true ¬must still handle watchdog features.
    if(debugResetFunction && !outNewData && !newData && (millis() - resetTime > 30000)){
      setSystemStatusLED(false);
      Serial.println("WTS_Controller Timeout....resetting WTS_Controller");
      resetWTSController();
      resetTime = millis();
      setSystemStatusLED(true);
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
void processIncomingCleralMessage() {
    static int index = 0;
    char rc;
    int num;
    while (cleralRS232Serial.available() > 0 && newData == false) {
        rc = cleralRS232Serial.read();             // read the first byte
        Serial.print(rc);                       // print char to serial monitor for debug purposes
        //num = (int)rc;
        //Serial.print(" ");
        //Serial.println(num);
        if (recvInProgress == true) {
          //Check for partial message in serial buffer. Log and Discard.
          if (rc == cleralMessageStartMarker) {
            Serial.print(F("Partial Message Received"));
            Serial.println(receivedChars);
            for(int recChar = 0; recChar < sizeof(receivedChars); recChar++){     
              receivedChars[recChar] = '\0';
            }
            index = 0;                                  // Reset receiving index
            continue;                                   // Go read next byte from buffer
          }
          
          if (rc != cleralMessageEndMarker) {
              receivedChars[index] = rc;
              index++;
              if (index >= CLERAL_RECEIVED_NUM_CHARS) {
                  index = CLERAL_RECEIVED_NUM_CHARS - 1;
              }
          }else {
              receivedChars[index] = rc;
              index++;
              receivedChars[index] = '\0'; // terminate the string
              recvInProgress = false;
              index = 0;
              newData = true;
          }
        }
        else if (rc == cleralMessageStartMarker) {        // message start marker received
            receivedChars[index] = rc;
            index++;
            recvInProgress = true;
        }
    }
}


void processIncomingWTSMessage() {
  static int index = 0;
  char rc;
  //outSerial.listen();
  while (outSerial.available() > 0 && outNewData == false) {
    rc = outSerial.read();             // read the first byte
    Serial.print(rc);
    
    if (outRecvInProgress == true) {
      
      //Check for partial message in serial buffer. Log and Discard.
      if (rc == cleralMessageStartMarker) {
        Serial.print(F("Partial Message Received"));
        Serial.println(outReceivedChars);
        for(int recChar = 0; recChar < sizeof(outReceivedChars); recChar++){     
          outReceivedChars[recChar] = '\0';
        }
        index = 0;                                  // Reset receiving index
        continue;                                   // Go read next byte from buffer
      }
      
      if (rc != cleralMessageEndMarker) {
          outReceivedChars[index] = rc;
          index++;
          if (index >= WTS_RECEIVED_NUM_CHARS) {
              index = WTS_RECEIVED_NUM_CHARS - 1;
          }
      }else {
         // Serial.println("END marker");
          outReceivedChars[index] = '\0'; // terminate the string
          outRecvInProgress = false;
          index = 0;
          outNewData = true;
      }
    }
    else if (rc == cleralMessageStartMarker) {        // message start marker received
       // Serial.println("START marker");
        outRecvInProgress = true;
    }
  }
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


void setSystemStatusLED(bool status){
  if(status){
    digitalWrite(SystemReadyStatusPin,HIGH);
    digitalWrite(SystemBusyStatusPin,LOW);
  }else{
    digitalWrite(SystemReadyStatusPin,LOW);
    digitalWrite(SystemBusyStatusPin,HIGH);
  }
}

// FileNameToRemove must include the full path to the file that must be deleted.
bool removeFileFromSDCard(char fileNameToRemove[]){
  if(SD.exists(fileNameToRemove)){
    return SD.remove(fileNameToRemove);
  }
  else{ 
    Serial.print(F("removeFileFroSDCard - could not find file: "));
    Serial.println(fileNameToRemove);
    return false;
  }
}

// Read one message from Cloud TX Queue
bool processTXQueue() {
  setSystemStatusLED(false);
  File dir = SD.open(SD_TX_DIR);   
  File entry =  dir.openNextFile();
  // @TODO check if the file is corrupt
  char et;
  byte filePosition = 0;
  if (! entry) {
    // no more files
    Serial.println(F("processTXQueue: No files to process"));
    dir.close();
    entry.close();
    return true;
  }
  
  String fn = SD_TX_DIR; 
  fn.concat("/");
  fn.concat(entry.name());
  fn.toCharArray(currentTXFileName,TX_FILE_LENGTH);
  
  Serial.print(F("currentTXFileName being processed: "));  Serial.println(currentTXFileName);
  Serial.print("currentTXFileName size:");  Serial.println(entry.size());

  // Delete the file if it is empty.
  if(entry.size() == 0){
    entry.close();
    Serial.print(F("ERROR - '"));
    Serial.print(currentTXFileName);
    Serial.print(F("' Is Empty. Deleting file"));
    Serial.println(removeFileFromSDCard(currentTXFileName) ? "...Success" : "...Failed");
    return false;
  }

  // Clear tempChars
  clearCharArray(tempChars);
  while (entry.available()) {
    et = entry.read();
    tempChars[filePosition] = et;
    filePosition++;
    if(filePosition >= (CLERAL_RECEIVED_NUM_CHARS)){
      Serial.print(F("WARNING: "));Serial.print(currentTXFileName);Serial.println(F(" contains more chars than is allowed."));
      break;
    }
  }
  entry.close();

  for(int cnt = 0; cnt < strlen(tempChars); cnt++){
    outSerial.write(tempChars[cnt]);
    Serial.write(tempChars[cnt]);
  }
  Serial.println();

  //Degug removing temporarily ¬
  strcpy(lastSentTXFileName,currentTXFileName); 
   
  // Clear the current tx file name
  clearCharArray(currentTXFileName);
  
  // Change the status LEDs to show READY
  setSystemStatusLED(true);
 
  return true;
}


/*
 *  Add a received message to the Cloud Queue
 */
bool queueTXMessage(char receivedCleralChars[]){
  // Save message for transmitting
  bool fileNameUnique = false;
  char fileName[TX_FILE_LENGTH];
    
  while(!fileNameUnique){
    txFileCount++;
    if(txFileCount > MAX_FILE_NUM)txFileCount = 1;  //Reset file number sequence
    // reset the fileName
    for (int v = 0; v <= sizeof(fileName); v++) {                           // Add directory tx/ to filename
      if(v==0){
        fileName[v]='t';
      }else if(v==1){
        fileName[v]='x';
      }else if(v==2){
        fileName[v]='/';
      }else{
        fileName[v] = '\0';
      }
    }
    sprintf(fileNum,"%04d",txFileCount);
    strcat(fileName,fileNum);
    strcat(fileName,".txt");
    if(!SD.exists(fileName)){
      fileNameUnique = true;   
    }
  }
  
  txFile = SD.open(fileName,FILE_WRITE);
  if (txFile) {
    strcpy(currentTXFileName, fileName);                      //Store global filename for error handling
    txFile.print(receivedCleralChars);
    Serial.print(F("Added '"));
    Serial.print(fileName);
    Serial.println(F("' to TX queue"));
    txFile.close();
    writeIntIntoEEPROM(FILE_EEPROM_ADDRESS, txFileCount);
    return true;
  }else{
    Serial.print(F("Writing file '"));
    Serial.print(fileName);
    Serial.print(F("' to "));
    Serial.print(SD_TX_DIR);
    Serial.println(F(" directory FAILED"));
    txFile.close();
    return false;
  }
  return true;
}

// Reset the WTS_Controller. set mega reset pin to low to restart the board.
void resetWTSController(){
  digitalWrite(RESET_PIN,LOW);
  delay(1);
  digitalWrite(RESET_PIN,HIGH);
}

// clears a char array by setting each character in a string to \0
void clearCharArray(char charArray[]){
  for(int charCounter = 0; charCounter < sizeof(charArray); charCounter++){
    charArray[charCounter] = '\0';
  }
}
