
// Include the SX1272 
#include "SX1276.h"
#include <SoftwareSerial.h>


#ifdef ARDUINO
// IMPORTANT when using an Arduino. For a Raspberry-based gateway the distribution uses a radio.makefile file
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
//
// it seems that both HopeRF and Modtronix board use the PA_BOOST pin and not the RFO. Therefore, for these
// boards we set the initial power to 'x' and not 'M'. This is the purpose of the define statement 
//
// uncomment if your radio is an HopeRF RFM92W or RFM95W
//#define RADIO_RFM92_95
// uncomment if your radio is a Modtronix inAir9B (the one with +20dBm features), if inAir9, leave commented
//#define RADIO_INAIR9B
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
#endif

// IMPORTANT
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// please uncomment only 1 choice
#define BAND868
//#define BAND900
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef RASPBERRY
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h> 
#include  <signal.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#endif

#ifdef ARDUINO
// and SPI library on Arduino platforms
#include <SPI.h>
  
  #define PRINTLN                   Serial.println("")              
  #define PRINT_CSTSTR(fmt,param)   Serial.print(F(param))
  #define PRINT_STR(fmt,param)      Serial.print(param)
  #define PRINT_VALUE(fmt,param)    Serial.print(param)
  #define FLUSHOUTPUT               Serial.flush();
#else

  #define PRINTLN                   printf("\n")
  #define PRINT_CSTSTR(fmt,param)   printf(fmt,param)
  #define PRINT_STR(fmt,param)      PRINT_CSTSTR(fmt,param)
  #define PRINT_VALUE(fmt,param)    PRINT_CSTSTR(fmt,param)
  #define FLUSHOUTPUT               fflush(stdout);
#endif

#ifdef DEBUG
  #define DEBUGLN                 PRINTLN
  #define DEBUG_CSTSTR(fmt,param) PRINT_CSTSTR(fmt,param)  
  #define DEBUG_STR(fmt,param)    PRINT_CSTSTR(fmt,param)  
  #define DEBUG_VALUE(fmt,param)  PRINT_VALUE(fmt,param)  
#else
  #define DEBUGLN
  #define DEBUG_CSTSTR(fmt,param)
  #define DEBUG_STR(fmt,param)    
  #define DEBUG_VALUE(fmt,param)  
#endif
 
 
 // la partie  pour les configuration====================
 //====================================s 
//#define RECEIVE_ALL 
#define IS_RCV_GATEWAY
#define IS_SEND_GATEWAY
//#define CAD_TEST
//#define LORA_LAS
//#define WINPUT
//#define WITH_SEND_LED

// the special mode to test BW=125MHz, CR=4/5, SF=12
// on the 868.1MHz channel
//#define LORAMODE 11

#define LORAMODE 4

#ifdef BAND868
#define MAX_NB_CHANNEL 9
#define STARTING_CHANNEL 10
#define ENDING_CHANNEL 18
uint8_t loraChannelIndex=0;
uint32_t loraChannelArray[MAX_NB_CHANNEL]={CH_10_868,CH_11_868,CH_12_868,CH_13_868,CH_14_868,CH_15_868,CH_16_868,CH_17_868,CH_18_868};
#else // assuming #defined BAND900
#define MAX_NB_CHANNEL 13
#define STARTING_CHANNEL 0
#define ENDING_CHANNEL 12
uint8_t loraChannelIndex=5;
uint32_t loraChannelArray[MAX_NB_CHANNEL]={CH_00_900,CH_01_900,CH_02_900,CH_03_900,CH_04_900,CH_05_900,CH_06_900,CH_07_900,CH_08_900,
                                            CH_09_900,CH_10_900,CH_11_900,CH_12_900};
#endif

// use the dynamic ACK feature of our modified SX1272 lib
#define GW_AUTO_ACK 

#define DEFAULT_DEST_ADDR 6

#ifdef IS_SEND_GATEWAY
#define LORA_ADDR 1
// packet size for periodic sending
uint8_t MSS=40;
#else
#define LORA_ADDR 1
// to unlock remote configuration feature
#define UNLOCK_PIN 1234
// will use 0xFF0xFE to prefix data received from LoRa, so that post-processing stage can differenciate
// data received from radio
#define WITH_DATA_PREFIX

#ifdef WITH_DATA_PREFIX
#define DATA_PREFIX_0 0xFF
#define DATA_PREFIX_1 0xFE
#endif
#endif

int dest_addr=DEFAULT_DEST_ADDR;

char cmd[260]="****************";
char sprintf_buf[100];
int msg_sn=0;

// number of retries to unlock remote configuration feature
uint8_t unlocked_try=3;
boolean unlocked=false;
boolean receivedFromSerial=false;
boolean receivedFromLoRa=false;
boolean withAck=false;

#ifndef ARDUINO
char keyPressBuff[30];
uint8_t keyIndex=0;
int ch;
#endif

// configuration variables
//////////////////////////
bool radioON=false;
bool RSSIonSend=true;
uint8_t loraMode=LORAMODE;
uint32_t loraChannel=loraChannelArray[loraChannelIndex];
#if defined RADIO_RFM92_95 || defined RADIO_INAIR9B || defined RADIO_20DBM
// HopeRF 92W/95W and inAir9B need the PA_BOOST
// so 'x' set the PA_BOOST but then limit the power to +14dBm 
char loraPower='x';
#else
// other radio board such as Libelium LoRa or inAir9 do not need the PA_BOOST
// so 'M' set the output power to 15 to get +14dBm
char loraPower='M';
#endif
uint8_t loraAddr=LORA_ADDR;

unsigned int inter_pkt_time=0;
unsigned int random_inter_pkt_time=0;
long last_periodic_sendtime=0;

unsigned long startDoCad, endDoCad;
bool extendedIFS=true;
uint8_t SIFS_cad_number;
// set to 0 to disable carrier sense based on CAD
uint8_t send_cad_number=3;
uint8_t SIFS_value[11]={0, 183, 94, 44, 47, 23, 24, 12, 12, 7, 4};
uint8_t CAD_value[11]={0, 62, 31, 16, 16, 8, 9, 5, 3, 1, 1};

bool optAESgw=false;
uint16_t optBW=0; 
uint8_t optSF=0;
uint8_t optCR=0;
uint8_t optCH=0;
bool  optRAW=false;
double optFQ=-1.0;
uint8_t optSW=0x12;
  
//////////////////////////

#if defined ARDUINO && not defined _VARIANT_ARDUINO_DUE_X_ && not defined __MK20DX256__
int freeMemory () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif

long getCmdValue(int &i, char* strBuff=NULL) {
        
        char seqStr[7]="******";

        int j=0;
        // character '#' will indicate end of cmd value
        while ((char)cmd[i]!='#' && (i < strlen(cmd)) && j<strlen(seqStr)) {
                seqStr[j]=(char)cmd[i];
                i++;
                j++;
        }
        
        // put the null character at the end
        seqStr[j]='\0';
        
        if (strBuff) {
                strcpy(strBuff, seqStr);        
        }
        else
                return (atol(seqStr));
}   

void startConfig() {

  int e;

  // has customized LoRa settings    
  if (optBW!=0 || optCR!=0 || optSF!=0) {

    e = sx1276.setCR(optCR-4);
    PRINT_CSTSTR("%s","^$LoRa CR ");
    PRINT_VALUE("%d", optCR);    
    PRINT_CSTSTR("%s",": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

    e = sx1276.setSF(optSF);
    PRINT_CSTSTR("%s","^$LoRa SF ");
    PRINT_VALUE("%d", optSF);    
    PRINT_CSTSTR("%s",": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;
    
    e = sx1276.setBW( (optBW==125)?BW_125:((optBW==250)?BW_250:BW_500) );
    PRINT_CSTSTR("%s","^$LoRa BW ");
    PRINT_VALUE("%d", optBW);    
    PRINT_CSTSTR("%s",": state ");
    PRINT_VALUE("%d", e);
    PRINTLN;

    // indicate that we have a custom setting
    loraMode=0;
  
    if (optSF<10)
      SIFS_cad_number=6;
    else 
      SIFS_cad_number=3;
      
  }
  else {
    
    // Set transmission mode and print the result
    PRINT_CSTSTR("%s","^$LoRa mode ");
    PRINT_VALUE("%d", loraMode);
    PRINTLN;
        
    e = sx1276.setMode(loraMode);
    PRINT_CSTSTR("%s","^$Setting mode: state ");
    PRINT_VALUE("%d", e);
    PRINTLN;
  
  #ifdef LORA_LAS
    loraLAS.setSIFS(loraMode);
  #endif
  
    if (loraMode>7)
      SIFS_cad_number=6;
    else 
      SIFS_cad_number=3;

  }
  
  // Select frequency channel
  if (loraMode==11) {
    // if we start with mode 11, then switch to 868.1MHz for LoRaWAN test
    // Note: if you change to mode 11 later using command /@M11# for instance, you have to use /@C18# to change to the correct channel
    e = sx1276.setChannel(CH_18_868);
    PRINT_CSTSTR("%s","^$Channel CH_18_868: state ");    
  }
  else {
    // work also for loraMode 0
    e = sx1276.setChannel(loraChannel);

    if (optFQ>0.0) {
      PRINT_CSTSTR("%s","^$Frequency ");
      PRINT_VALUE("%f", optFQ);
      PRINT_CSTSTR("%s",": state ");      
    }
    else {
#ifdef BAND868      
      PRINT_CSTSTR("%s","^$Channel CH_1");
      PRINT_VALUE("%d", loraChannelIndex);
      PRINT_CSTSTR("%s","_868: state ");
#else
      PRINT_CSTSTR("%s","^$Channel CH_");
      PRINT_VALUE("%d", loraChannelIndex);
      PRINT_CSTSTR("%s","_900: state ");
#endif
    }
  }  
  PRINT_VALUE("%d", e);
  PRINTLN; 
  
  // Select output power (Max, High or Low)
  e = sx1276.setPower(loraPower);

  PRINT_CSTSTR("%s","^$Set LoRa Power to ");
  PRINT_VALUE("%c",loraPower);  
  PRINTLN;
                
  PRINT_CSTSTR("%s","^$Power: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;
 
  // get preamble length
  e = sx1276.getPreambleLength();
  PRINT_CSTSTR("%s","^$Get Preamble Length: state ");
  PRINT_VALUE("%d", e);
  PRINTLN; 
  PRINT_CSTSTR("%s","^$Preamble Length: ");
  PRINT_VALUE("%d", sx1276._preamblelength);
  PRINTLN;
  
  // Set the node address and print the result
  //e = sx1272.setNodeAddress(loraAddr);
  sx1276._nodeAddress=loraAddr;
  e=0;
  PRINT_CSTSTR("%s","^$LoRa addr ");
  PRINT_VALUE("%d", loraAddr);
  PRINT_CSTSTR("%s",": state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  if (optAESgw)
      PRINT_CSTSTR("%s","^$Handle AES encrypted data\n");

  if (optRAW) {
      PRINT_CSTSTR("%s","^$Raw format, not assuming any header in reception\n");  
      // when operating n raw format, the SX1272 library do not decode the packet header but will pass all the payload to stdout
      // note that in this case, the gateway may process packet that are not addressed explicitly to it as the dst field is not checked at all
      // this would be similar to a promiscuous sniffer, but most of real LoRa gateway works this way 
      sx1276._rawFormat=true;
  }
  
  // Print a success message
  PRINT_CSTSTR("%s","^$SX1272/76 configured ");
#ifdef IS_SEND_GATEWAY
  PRINT_CSTSTR("%s","as device. Waiting serial input for serial-RF bridge\n");
#else
  PRINT_CSTSTR("%s","as LR-BS. Waiting RF input for transparent RF-serial bridge\n");
#endif  
}
SoftwareSerial bluetoothSerial(6, 5); // (RX, TX) (pin Rx BT, pin Tx BT)
String donnees;
void setup()
{
  donnees="";
  int e;
#ifdef ARDUINO
  delay(3000);
  randomSeed(analogRead(14));
#else
  srand (time(NULL));
#endif

#ifdef _VARIANT_ARDUINO_DUE_X_
  Serial.begin(115200);  
#else  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  bluetoothSerial.write("AT+NAMEnode1");
#if defined ARDUINO && not defined __MK20DX256__
    // Print a start message
  Serial.print(freeMemory());
  Serial.println(F(" bytes of free memory.")); 
#endif  
#endif 

  // Power ON the module
  sx1276.ON();
  
  PRINT_CSTSTR("%s","^$**********Power ON: state ");
  PRINT_VALUE("%d", e);
  PRINTLN;

  e = sx1276.getSyncWord();

  if (!e) {
    PRINT_CSTSTR("%s","^$Default sync word: 0x");
#ifdef ARDUINO
    Serial.print(sx1276._syncWord, HEX);
#else  
    PRINT_VALUE("%X", sx1276._syncWord);
#endif  
    PRINTLN;
  }    

  if (optSW!=0x12) {
    e = sx1276.setSyncWord(optSW);

    PRINT_CSTSTR("%s","^$Set sync word to 0x");
#ifdef ARDUINO
    Serial.print(optSW, HEX);
#else  
    PRINT_VALUE("%X", optSW);
#endif
    
    PRINTLN;
    PRINT_CSTSTR("%s","^$LoRa sync word: state ");
    PRINT_VALUE("%d",e);  
    PRINTLN;
  }
  
  if (!e) {
    radioON=true;
    startConfig();
  }
  
  FLUSHOUTPUT;
  delay(1000);

#ifdef LORA_LAS
  loraLAS.ON(LAS_ON_WRESET);
  
#ifdef IS_SEND_GATEWAY  
  //delay(random(LAS_REG_RAND_MIN,LAS_REG_RAND_MAX));
  //loraLAS.sendReg();
#endif  
#endif

#ifdef CAD_TEST
  PRINT_CSTSTR("%s","Do CAD test\n");
#endif 
}

// we could use the CarrierSense function added in the SX1272 library, but it is more convenient to duplicate it here
// so that we could easily modify it for testing
void CarrierSense() {
  int e;
  bool carrierSenseRetry=false;
  
  if (send_cad_number) {
    do { 
      do {
        
        // check for free channel (SIFS/DIFS)        
        startDoCad=millis();
        e = sx1276.doCAD(send_cad_number);
        endDoCad=millis();
        
        PRINT_CSTSTR("%s","--> CAD duration ");
        PRINT_VALUE("%ld",endDoCad-startDoCad);
        PRINTLN;
        
        if (!e) {
          PRINT_CSTSTR("%s","OK1\n");
          
          if (extendedIFS)  {          
            // wait for random number of CAD
#ifdef ARDUINO                
            uint8_t w = random(1,8);
#else
            uint8_t w = rand() % 8 + 1;
#endif
  
            PRINT_CSTSTR("%s","--> waiting for ");
            PRINT_VALUE("%d",w);
            PRINT_CSTSTR("%s"," CAD = ");
            PRINT_VALUE("%d",CAD_value[loraMode]*w);
            PRINTLN;
            
            delay(CAD_value[loraMode]*w);
            
            // check for free channel (SIFS/DIFS) once again
            startDoCad=millis();
            e = sx1276.doCAD(send_cad_number);
            endDoCad=millis();
 
            PRINT_CSTSTR("%s","--> CAD duration ");
            PRINT_VALUE("%ld",endDoCad-startDoCad);
            PRINTLN;
        
            if (!e)
              PRINT_CSTSTR("%s","OK2");            
            else
              PRINT_CSTSTR("%s","###2");
            
            PRINTLN;
          }              
        }
        else {
          PRINT_CSTSTR("%s","###1\n");  

          // wait for random number of DIFS
#ifdef ARDUINO                
          uint8_t w = random(1,8);
#else
          uint8_t w = rand() % 8 + 1;
#endif
          
          PRINT_CSTSTR("%s","--> waiting for ");
          PRINT_VALUE("%d",w);
          PRINT_CSTSTR("%s"," DIFS (DIFS=3SIFS) = ");
          PRINT_VALUE("%d",SIFS_value[loraMode]*3*w);
          PRINTLN;
          
          delay(SIFS_value[loraMode]*3*w);
          
          PRINT_CSTSTR("%s","--> retry\n");
        }

      } while (e);
    
      // CAD is OK, but need to check RSSI
      if (RSSIonSend) {
    
          e=sx1276.getRSSI();
          
          uint8_t rssi_retry_count=10;
          
          if (!e) {
          
            PRINT_CSTSTR("%s","--> RSSI ");
            PRINT_VALUE("%d", sx1276._RSSI);
            PRINTLN;
            
            while (sx1276._RSSI > -90 && rssi_retry_count) {
              
              delay(1);
              sx1276.getRSSI();
              PRINT_CSTSTR("%s","--> RSSI ");
              PRINT_VALUE("%d",  sx1276._RSSI);       
              PRINTLN; 
              rssi_retry_count--;
            }
          }
          else
            PRINT_CSTSTR("%s","--> RSSI error\n"); 
        
          if (!rssi_retry_count)
            carrierSenseRetry=true;  
          else
      carrierSenseRetry=false;            
      }
      
    } while (carrierSenseRetry);  
  }
}

void loop(void)
{ 
  bluetoothSerial.flush();
  int i=0, e;
  int cmdValue;
///////////////////////  
// ONLY FOR TESTING CAD

// ONLY FOR TESTING CAD
///END/////////////////

//////////////////////////  
// START OF PERIODIC TASKS

  receivedFromSerial=false;
  receivedFromLoRa=false;
  // check if we received data from the input serial port
  //bluetoothSerial.flush();
  if (bluetoothSerial.available()){
    
    i=0;
    while (bluetoothSerial.available()) {
         //Serial.write(bluetoothSerial.read());
          cmd[i]=(char)bluetoothSerial.read();
          i++;  
      }
      cmd[i]='\0';

    PRINT_CSTSTR("%s","Rcv bluetooth: ");
    PRINT_STR("%s",cmd);
    PRINTLN;
    
    receivedFromSerial=true; 
  }
      
  /*if (Serial.available()) {

    i=0;  

    while (Serial.available() && i<80) {
      cmd[i]=Serial.read();
      i++;
      delay(50);
    }
    
    cmd[i]='\0';

    PRINT_CSTSTR("%s","Rcv serial: ");
    PRINT_STR("%s",cmd);
    PRINTLN;
    
    receivedFromSerial=true; 
  }*/
  
// handle keyboard input from a UNIX terminal  


  if (radioON && !receivedFromSerial) {

///////////////////////////////////////////////////////
// ONLY FOR END-DEVICE SENDING MESSAGES TO BASE STATION
#ifdef IS_SEND_GATEWAY  
      // periodic message sending? (mainly for tests)
      if (inter_pkt_time)
      
        if (millis()-last_periodic_sendtime > (random_inter_pkt_time?random_inter_pkt_time:inter_pkt_time)) {
          
          PRINT_CSTSTR("%s","inter_pkt ");
          PRINT_VALUE("%ld",millis()-last_periodic_sendtime);  
          PRINTLN;
          
          sprintf(cmd, "msg %3.d***", msg_sn++);
          for (i=strlen(cmd); i<MSS; i++)
            cmd[i]='*';
          
          cmd[i]='\0';
          
          PRINT_CSTSTR("%s","Sending : ");
          PRINT_STR("%s",cmd);  
          PRINTLN;
          
          CarrierSense();
          
          PRINT_CSTSTR("%s","Packet number ");
          PRINT_VALUE("%d",sx1276._packetNumber);
          PRINTLN;
          
          long startSend=millis();
          
#ifdef WITH_SEND_LED
          digitalWrite(SEND_LED, HIGH);
#endif       
          e = sx1276.sendPacketTimeout(dest_addr, (uint8_t*)cmd, strlen(cmd), 10000);
          
#ifdef WITH_SEND_LED
          digitalWrite(SEND_LED, LOW);
#endif           
          PRINT_CSTSTR("%s","LoRa Sent in ");
          PRINT_VALUE("%ld",millis()-startSend);  
          PRINTLN;
          PRINT_CSTSTR("%s","Packet sent, state ");
          PRINT_VALUE("%d",e);
          PRINTLN;
          //Serial.flush();
          
          if (random_inter_pkt_time) {
#ifdef ARDUINO                
            random_inter_pkt_time=random(2000,inter_pkt_time);
#else
            random_inter_pkt_time = rand() % inter_pkt_time + 2000;
#endif            
            PRINT_CSTSTR("%s","next in ");
            PRINT_VALUE("%ld",random_inter_pkt_time);
            PRINTLN;
          }
            
          last_periodic_sendtime=millis();       
        }  
        
      // TODO
      // the end-device should also open a receiving window to receive 
      // INIT & UPDT messages
      e=1;
#ifndef CAD_TEST        
      // open a receive window
      uint16_t w_timer=10000;
      
      if (loraMode==1)
        w_timer=2500;
        
      e = sx1276.receivePacketTimeout(w_timer);
      
#endif 

// ONLY FOR END-DEVICE SENDING MESSAGES TO BASE STATION
///END/////////////////////////////////////////////////
#else 
///////////////////////////////////////////////////////
// ONLY FOR BASE STATION RECEIVING MESSAGES FROM DEVICE
      uint16_t w_timer=10000;
      
      if (loraMode==1)
        w_timer=2500;
        
      e=1;
#ifndef CAD_TEST

      // check if we received data from the receiving LoRa module
#ifdef RECEIVE_ALL
      e = sx1276.receiveAll(w_timer);
#else
#ifdef GW_AUTO_ACK  

      e = sx1276.receivePacketTimeout(w_timer);
      
      if (!e && sx1276._requestACK_indicator) {
         sprintf(sprintf_buf,"^$ACK requested by %d\n", sx1276.packet_received.src);                   
         PRINT_STR("%s",sprintf_buf);        
      }
#else
      // Receive message
      if (withAck)
        e = sx1276.receivePacketTimeoutACK(w_timer);
      else      
        e = sx1276.receivePacketTimeout(w_timer);
#endif          
#endif
#endif
#endif 
// ONLY FOR BASE STATION RECEIVING MESSAGES FROM DEVICE
///END/////////////////////////////////////////////////

      if (!e) {
         int a=0, b=0;
         uint8_t tmp_length;

         receivedFromLoRa=true;
         sx1276.getSNR();
         sx1276.getRSSIpacket();

         tmp_length=sx1276._payloadlength;
         
         sprintf(sprintf_buf,"--- rxlora. dst=%d type=0x%.2X src=%d seq=%d len=%d SNR=%d RSSIpkt=%d BW=%d CR=4/%d SF=%d\n", 
                   sx1276.packet_received.dst,
                   sx1276.packet_received.type, 
                   sx1276.packet_received.src,
                   sx1276.packet_received.packnum,
                   tmp_length, 
                   sx1276._SNR,
                   sx1276._RSSIpacket,
                   (sx1276._bandwidth==BW_125)?125:((sx1276._bandwidth==BW_250)?250:500),
                   sx1276._codingRate+4,
                   sx1276._spreadingFactor);
                   
         //PRINT_STR("%s",sprintf_buf);

         // provide a short output for external program to have information about the received packet
         // ^psrc_id,seq,len,SNR,RSSI
         sprintf(sprintf_buf,"^p%d,%d,%d,%d,%d,%d,%d\n",
                   sx1276.packet_received.dst,
                   sx1276.packet_received.type,                   
                   sx1276.packet_received.src,
                   sx1276.packet_received.packnum, 
                   tmp_length,
                   sx1276._SNR,
                   sx1276._RSSIpacket);
                   
         //PRINT_STR("%s",sprintf_buf);          

         // ^rbw,cr,sf
         sprintf(sprintf_buf,"^r%d,%d,%d\n", 
                   (sx1276._bandwidth==BW_125)?125:((sx1276._bandwidth==BW_250)?250:500),
                   sx1276._codingRate+4,
                   sx1276._spreadingFactor);
                   
         //PRINT_STR("%s",sprintf_buf);  

// for Linux-based gateway only
///////////////////////////////
#ifdef LORA_LAS        
         if (loraLAS.isLASMsg(sx1276.packet_received.data)) {
           
           //tmp_length=sx1272.packet_received.length-OFFSET_PAYLOADLENGTH;
           tmp_length=sx1276._payloadlength;
           
           int v=loraLAS.handleLASMsg(sx1276.packet_received.src,
                                      sx1276.packet_received.data,
                                      tmp_length);
           
           if (v==DSP_DATA) {
             a=LAS_DSP+DATA_HEADER_LEN+1;
#ifdef WITH_DATA_PREFIX
             PRINT_STR("%c",(char)DATA_PREFIX_0);      
             PRINT_STR("%c",(char)DATA_PREFIX_1);
#endif             
           }
           else
             // don't print anything
             a=tmp_length; 
         }
         else
           PRINT_CSTSTR("%s","No LAS header. Write raw data\n");
#else
#ifdef WITH_DATA_PREFIX
         PRINT_STR("%c",(char)DATA_PREFIX_0);        
         PRINT_STR("%c",(char)DATA_PREFIX_1);
#endif
#endif
         for ( ; a<tmp_length; a++,b++) {
           PRINT_STR("%c",(char)sx1276.packet_received.data[a]);
           
           cmd[b]=(char)sx1276.packet_received.data[a];
           bluetoothSerial.write((char)sx1276.packet_received.data[a]);
           
         }
         
         // strlen(cmd) will be correct as only the payload is copied
         cmd[b]='\0';    
         PRINTLN;
         FLUSHOUTPUT;
         
#if not defined ARDUINO && defined WINPUT
        // if we received something, display again the current input 
        // that has still not be terminated
        if (keyIndex) {
              PRINT_CSTSTR("%s","keyboard input : ");
              PRINT_STR("%s",keyPressBuff);
              PRINTLN;
        }

#endif           
      }  
  }  
  
  if (receivedFromSerial || receivedFromLoRa) {
    
    boolean sendCmd=false;
    boolean withTmpAck=false;
    int forTmpDestAddr=-1;
    
    
    i=0;
    
    if (cmd[i]=='/' && cmd[i+1]=='@') {

      PRINT_CSTSTR("%s","^$Parsing command\n");      
      i=2;
      
      PRINT_CSTSTR("%s","^$");
      PRINT_STR("%s",cmd);
      PRINTLN;
      
      if ( (receivedFromLoRa && cmd[i]!='U' && !unlocked) || !unlocked_try) {
        PRINT_CSTSTR("%s","^$Remote config locked\n");
        // just assign an unknown command
        cmd[i]='*';  
      }
      

      FLUSHOUTPUT;
    }
    else
     sendCmd=true; 
    
#ifdef IS_SEND_GATEWAY 
///////////////////////////////////////////////////////
// ONLY FOR END-DEVICE SENDING MESSAGES TO BASE STATION
    if (sendCmd && receivedFromSerial) {
      
      uint8_t pl=strlen((char*)(&cmd[i]));
      PRINT_CSTSTR("%s","Sending. Length is ");
      PRINT_VALUE("%d",pl);
      PRINTLN;
      PRINT_STR("%s",(char*)(&cmd[i]));
      PRINTLN;
      
#ifdef LORA_LAS
      if (forTmpDestAddr>=0)
        e = loraLAS.sendData(forTmpDestAddr, (uint8_t*)(&cmd[i]), pl, 0, 
              LAS_FIRST_DATAPKT+LAS_LAST_DATAPKT, withAck | withTmpAck);
      else
        e = loraLAS.sendData(dest_addr, (uint8_t*)(&cmd[i]), pl, 0,
              LAS_FIRST_DATAPKT+LAS_LAST_DATAPKT, withAck | withTmpAck);
      
      if (e==TOA_OVERUSE) {
          PRINT_CSTSTR("%s","^$Not sent, TOA_OVERUSE\n");  
      }
      
      if (e==LAS_LBT_ERROR) {
          PRINT_CSTSTR("%s","^$LBT error\n");  
      }      
      
      if (e==LAS_SEND_ERROR || e==LAS_ERROR) {
          PRINT_CSTSTR("%s","Send error\n");  
      }      
#else  
      // only the DIFS/SIFS mechanism
      // we chose to have a complete control code insytead of using the implementation of the LAS class
      // for better debugging and tests features if needed.    
      PRINT_CSTSTR("%s","Payload size is ");  
      PRINT_VALUE("%d",pl);
      PRINTLN;
      
      uint32_t toa = sx1276.getToA(pl+5);      
      PRINT_CSTSTR("%s","ToA is w/5B Libelium header ");
      PRINT_VALUE("%d",toa);
      PRINTLN;
      
      long startSend, endSend;
      long startSendCad;
      
      startSendCad=millis();

      CarrierSense();

      startSend=millis();

#ifdef WITH_SEND_LED
      digitalWrite(SEND_LED, HIGH);
#endif

      PRINT_CSTSTR("%s","Packet number ");
      PRINT_VALUE("%d",sx1276._packetNumber);
      PRINTLN;

      // to test with appkey + encrypted
      //sx1272.setPacketType(PKT_TYPE_DATA | PKT_FLAG_DATA_WAPPKEY | PKT_FLAG_DATA_ENCRYPTED);
      
      sx1276.setPacketType(PKT_TYPE_DATA); 
        
      if (forTmpDestAddr>=0) {
        if (withAck)
          e = sx1276.sendPacketTimeoutACK(forTmpDestAddr, (uint8_t*)(&cmd[i]), pl, 10000);  
        else    
          e = sx1276.sendPacketTimeout(forTmpDestAddr, (uint8_t*)(&cmd[i]), pl, 10000);
      }
      else {
        if (withAck || withTmpAck)   
          e = sx1276.sendPacketTimeoutACK(dest_addr, (uint8_t*)(&cmd[i]), pl, 10000);    
         else 
          e = sx1276.sendPacketTimeout(dest_addr, (uint8_t*)(&cmd[i]), pl, 10000);
      }

#ifdef WITH_SEND_LED
      digitalWrite(SEND_LED, LOW);
#endif

      endSend=millis();  
      
      if ((withAck || withTmpAck) && !e) {
        sx1276.getSNR();
        sx1276.getRSSIpacket();
         
        sprintf(sprintf_buf,"--- rxlora ACK. SNR=%d RSSIpkt=%d\n", 
                   sx1276._SNR,
                   sx1276._RSSIpacket);
                   
        PRINT_STR("%s",sprintf_buf);
        
        PRINT_CSTSTR("%s","LoRa (ACK) Sent in ");  
      }
      else      
        PRINT_CSTSTR("%s","LoRa Sent in ");
      
      PRINT_VALUE("%ld",endSend-startSend);
      PRINTLN;
      
      PRINT_CSTSTR("%s","LoRa Sent w/CAD in ");
      PRINT_VALUE("%ld",endSend-startSendCad);
      PRINTLN;      
#endif    
      PRINT_CSTSTR("%s","Packet sent, state ");
      PRINT_VALUE("%d",e);
      PRINTLN;
      //Serial.flush();
    }
#endif 
// ONLY FOR END-DEVICE SENDING MESSAGES TO BASE STATION
///END/////////////////////////////////////////////////

  } // end of "if (receivedFromSerial || receivedFromLoRa)"
} 

// for Linux-based gateway only
///////////////////////////////

