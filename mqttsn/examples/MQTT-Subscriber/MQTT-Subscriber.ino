/*
	(c) Diego Fernández <bigomby@gmail.com>
	Originally published on https://github.com/ESIBot/MQTT-SN-Arduino
*/

#include <Arduino.h>
#include <mqttsn-messages.h>

#define TOPIC "test"

MQTTSN mqttsn;

uint16_t u16TopicID;

void setup() {
	Serial.begin(115200);
	Serial1.begin(9600);
}

void loop() {
	uint8_t index;

	CheckSerial();
	delay(2000);

	if (mqttsn.wait_for_response()) {
		return;
	}

	if (!mqttsn.connected()) {
		mqttsn.connect(0, 10, "arduino");
		return;
	}

	u16TopicID = mqttsn.find_topic_id(TOPIC, &index);
	if (u16TopicID == 0xffff) {
		mqttsn.register_topic(TOPIC);
		mqttsn.subscribe_by_name(0, TOPIC);
		return;
	}
}

void MQTTSN_serial_send(uint8_t *message_buffer, int length) {
	Serial1.write(message_buffer, length);
	Serial1.flush();
}

void MQTTSN_publish_handler(const msg_publish *msg) {

}

void MQTTSN_gwinfo_handler(const msg_gwinfo *msg) {
}

void CheckSerial() {
	uint16_t cnt = 0;
	uint8_t buffer[512];
	uint8_t *buf = &buffer[0];

	while (Serial1.available()) {
		buffer[cnt++] = Serial1.read();
	}

	if (cnt > 0) {
		for (int i = 0; i < cnt ; i++) {
			Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(" ");
		}
		Serial.println("");
		mqttsn.parse_stream(buf, cnt);
	}
}
