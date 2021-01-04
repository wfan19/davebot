#include <Arduino.h>
#include "QuadEncoder.h"

#include <ros.h>
#include <std_msgs/Float64.h>

int rw_A_pin = 0;
int rw_B_pin = 1;

int lw_A_pin = 2;
int lw_B_pin = 3;

ros::NodeHandle nh;

std_msgs::Float64 lw_pos;
std_msgs::Float64 rw_pos;

ros::Publisher rw_pub("davebot_rw", &rw_pos);
ros::Publisher lw_pub("davebot_lw", &lw_pos);

QuadEncoder rw_encoder(1, rw_A_pin, rw_B_pin);
QuadEncoder lw_encoder(2, lw_A_pin, lw_B_pin);

int lastMessageTime = 0;

void publishMessage() {
	rw_pub.publish(&rw_pos);
	lw_pub.publish(&lw_pos);

	nh.spinOnce();
	lastMessageTime = millis();
	digitalWrite(13, HIGH);
}

void setup() {
	rw_encoder.setInitConfig();
	rw_encoder.init();

	lw_encoder.setInitConfig();
	lw_encoder.init();

	nh.initNode();
	nh.advertise(lw_pub);
	nh.advertise(rw_pub);

	rw_pos.data = 0;
	lw_pos.data = 0;

	publishMessage();
}

void loop() {
	int rw_currentPos = rw_encoder.read();
	int lw_currentPos = lw_encoder.read();

	if ((rw_currentPos != rw_pos.data) || (lw_currentPos != lw_pos.data)) {
		rw_pos.data = rw_currentPos;
		lw_pos.data = lw_currentPos;

		publishMessage();	
	} else {
		if (millis() - lastMessageTime > 1000) {
			publishMessage();
		} else {
			digitalWrite(13, LOW);
		}
	}
	delay(10);
}
