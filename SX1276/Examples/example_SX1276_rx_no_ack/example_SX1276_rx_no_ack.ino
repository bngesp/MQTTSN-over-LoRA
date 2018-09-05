/*
 *  Semtech SX1272 module managing with Arduino
 *
 *  Copyright (C) 2014 Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.

 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version:		1.0
 *  Design:		David Gascón
 *  Implementation:	Covadonga Albiñana
 */
 
// Include the SX1272 and SPI library: 
#include "SX1276.h"
#include <SPI.h>

int e;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  
  // Print a start message
  Serial.println("SX1276 module and Arduino: receive packets without ACK");
  
  // Power ON the module
  sx1276.ON();
  
  // Set transmission mode and print the result
  e = sx1276.setMode(1);
  Serial.println(e, DEC);
  
  // Select frequency channel
  e = sx1276.setChannel(CH_16_868);
  Serial.println("Setting Channel: state ");
  Serial.println(e, DEC);
  
  // Select output power (Max, High or Low)
  e = sx1276.setPower('M');
  Serial.println("Setting Power: state ");
  Serial.println(e);
  
  // Set the node address and print the result
  e = sx1276.setNodeAddress(8);
  Serial.println(e, DEC);
  
  // Print a success message
  Serial.print("SX1276 successfully configured ");
}

void loop(void)
{
  // Receive message
  
  e = sx1276.receivePacketTimeout(10000);
  e = sx1276.getRSSIpacket();
  
  Serial.print(("Receive packet timeout, state "));
  Serial.println(e, DEC);
}

