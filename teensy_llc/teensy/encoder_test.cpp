#include <Arduino.h>
#include "QuadEncoder.h"

int rw_A_pin = 0;
int rw_B_pin = 1;
QuadEncoder rwEncoder(1, rw_A_pin, rw_B_pin);

int rw_lastPos = 0;

void setup(){
	rwEncoder.setInitConfig();
	rwEncoder.init();
}

void loop(){
	int rw_currentPos = rwEncoder.read();
	if(rw_currentPos != rw_lastPos){
		Serial.print("Right wheel encoder pos: ");
		Serial.print(rw_currentPos);
		Serial.println("");
		rw_lastPos = rw_currentPos;
	}
}


