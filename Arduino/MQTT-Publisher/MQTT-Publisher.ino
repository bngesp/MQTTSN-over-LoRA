/*
	(c) Diego Fernández <bigomby@gmail.com>
	Originally published on https://github.com/ESIBot/MQTT-SN-Arduino
*/
#include <Arduino.h>
#include <mqttsn-messages.h>

#define TOPIC "test"

#define LORA_GATEWAY 8

#define LORA_MODE 4
#define LORA_CHANNEL CH_10_868
#define LORA_POWER 'H'
#define LORA_ADDR 3

MQTTSN mqttsn;

uint16_t u16TopicID;

void initLoRa(){
  
  sx1276.ON();
  Serial.println("LoRa On");
  sx1276.setMode(LORA_MODE);
  Serial.print("loRa setMode : ");
  Serial.println(LORA_MODE);
  sx1276.setChannel(CH_10_868);
  Serial.print("loRa setChannel : ");
  Serial.println(CH_10_868);
  sx1276.setPower(LORA_POWER);
  Serial.print("loRa setPower : ");
  Serial.println(LORA_POWER);
  sx1276.setNodeAddress(LORA_ADDR);
  Serial.print("loRa set node address : ");
  Serial.println(LORA_ADDR);
  Serial.println("*********************************\n");
  Serial.println("SX1276 successfully configured s\n\n");
}




void setup() {
	Serial.begin(115200);
  
  // Print a start message
  Serial.println("SX1276 module loRa Configuration\n\n");
  Serial.println("*********************************\n");
  initLoRa();
  mqttsn.setLoraConnection(sx1276);
	//Serial1.begin(9600);
}

void loop() {
	uint8_t index;


	if (!mqttsn.connected()) {
    Serial.println("connexion");
		mqttsn.connect(0, 10, "arduino");
		return;
	}

	u16TopicID = mqttsn.find_topic_id(TOPIC, &index);
	if (u16TopicID == 0xffff) {
		mqttsn.register_topic(TOPIC);
   Serial.println("register");
		return;
	}

	char str[50] = "Hello World!";
	mqttsn.publish(0, u16TopicID, str, strlen(str));
}

void MQTTSN_serial_send(uint8_t *message_buffer, int length) {
	
}

void MQTTSN_publish_handler(const msg_publish *msg) {

}

void MQTTSN_gwinfo_handler(const msg_gwinfo *msg) {
}

void CheckSerial() {

}
