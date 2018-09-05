/*  
 *  LoRa 868 / 915MHz SX1272 Module
 *  
 *  Copyright (C) Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  This program is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version. 
 *  
 *  This program is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License 
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *  
 *  Version:           1.0
 *  Design:            David Gascón 
 *  Implementation:    Victor Boria & Luis Miguel Marti
 */
 
// Include the SX1272 and SPI library: 
#include "SX1276.h"
#include <SPI.h>
#include <OneWire.h>
#define PRINTLN                   Serial.println("")              
  #define PRINT_CSTSTR(fmt,param)   Serial.print(F(param))
  #define PRINT_STR(fmt,param)      Serial.print(param)
  #define PRINT_VALUE(fmt,param)    Serial.print(param)
  #define FLUSHOUTPUT               Serial.flush();

int e;
char message1 [60];
int ntc;
int i=0;
boolean receivedFromSerial=false;
uint16_t w_timer=10000;
long startSend,endSend,debut,fin,finT;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  
  // Power ON the module
  sx1276.ON();
  
  // Set transmission mode and print the result
  e = sx1276.setMode(1);
  Serial.println(F("Setting Mode: state "));
  Serial.println(e, DEC);
/*
  e = sx1276.getMode();
  Serial.println(F("coding rate:  "));
  Serial.println(sx1276._codingRate);

  
  e = sx1276.getBW();
  Serial.println(F("BW:  "));
  Serial.println(sx1276._bandwidth);

  
  e = sx1276.getSF();
  Serial.println(F("SF:  "));
  Serial.println(sx1276._spreadingFactor);*/
  
  // Select frequency channel
  e = sx1276.setChannel(CH_10_868);
  Serial.println(F("Setting Channel: state "));
  Serial.println(e, DEC);
  
  // Select output power (Max, High or Low)
  e = sx1276.setPower('M');
  Serial.println(F("Setting Power: state "));
  Serial.println(e, DEC);
  
  // Set the node address and print the result
  e = sx1276.setNodeAddress(6);
  Serial.println(F("Setting node address: state "));
  Serial.println(e, DEC);
  
  // Print a success message
  Serial.println(F("SX1272 successfully configured"));
}

void loop(void)
{
  debut=millis();
  //Serial.println("on y est");
  if (Serial.available()) {

    i=0;  

    while (Serial.available() && i<80) {
      message1[i]=Serial.read();
      i++;
      delay(50);
    }
    
    receivedFromSerial=true; 
  }
  fin=millis();
  //PRINT_CSTSTR("%s","Temps verification serial ");
  //PRINT_VALUE("%ld",fin-debut);
  PRINTLN;
  ntc = 123;
  /*message1[0]="d";
  message1[1]="e";
  message1[2]="b";
  message1[3]="u";
  message1[4]="t";
  message1[48]="f";
  message1[49]="i";
  message1[50]="n";
  for (ntc=5; ntc<48; ntc++)
     message1[ntc]='#';
     message1[ntc]='\0';*/   
  while(ntc<1000){
  sprintf(message1,"%d",ntc);
 // if (receivedFromSerial){
    uint8_t pl=strlen((char*)(&message1[i]));
    startSend=millis();
    e = sx1276.sendPacketTimeout(1, (uint8_t*)(&message1[i]), pl, 10000); 
    endSend=millis();
    Serial.println(message1);
    //PRINT_CSTSTR("%s","LoRa Sent in ");
    //PRINT_VALUE("%ld",endSend-startSend);
    PRINTLN;
    //Serial.print(F("Packet sent, state "));
    //Serial.println(e, DEC);   
    receivedFromSerial=false;
    ntc++; 
    delay(5000);
    }
  //}
  //else
  //{
   /* e = sx1272.receivePacketTimeout(w_timer);
    //Serial.println(e);
    if(e==0){
    int a;
    char sprintf_buf[100];
    uint8_t tmp_length;
    tmp_length=sx1272._payloadlength;
    sprintf(sprintf_buf,"--- rxlora. dst=%d type=0x%.2X src=%d seq=%d len=%d SNR=%d RSSIpkt=%d BW=%d CR=4/%d SF=%d\n", 
                   sx1272.packet_received.dst,
                   sx1272.packet_received.type, 
                   sx1272.packet_received.src,
                   sx1272.packet_received.packnum,
                   tmp_length, 
                   sx1272._SNR,
                   sx1272._RSSIpacket,
                   (sx1272._bandwidth==BW_125)?125:((sx1272._bandwidth==BW_250)?250:500),
                   sx1272._codingRate+4,
                   sx1272._spreadingFactor);
                   
         PRINT_STR("%s",sprintf_buf);
     for (a=0 ; a<tmp_length; a++) {
           //Serial.print("%c",(char)sx1272.packet_received.data[a]);
           
           sprintf_buf[a]=(char)sx1272.packet_received.data[a];
         }
         PRINT_STR("%s",sprintf_buf);
         PRINTLN;
    //}
  }*/
  //delay(2000);
  finT=millis();
  //PRINT_CSTSTR("%s","Duree programme ");
  //PRINT_VALUE("%ld",finT-debut);
  //PRINTLN;
}

