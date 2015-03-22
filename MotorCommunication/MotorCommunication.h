/*
 * MotorCommunication.cpp
 *
 *  Created on: Feb 12, 2015 by Kellen Carey
 *  Modified for CM9.04 by Adam Stroud and Kellen Carey
 *	Marquette University HEIR Lab 3/22/2015
 */

#ifndef MOTOR_COMMUNICATION_H_

#define MOTOR_COMMUNICATION_H_

#include "Motors.h"

#include <usb_serial.h>

#define P_GOAL_POSITION		30
#define P_MOVING_SPEED		32
#define P_ENABLE			24
#define P_PRESENT_POSITION	36
#define STATUS_RETURN_LEVEL 16

#define NUM_MOTORS	20 //12 in legs, 6 in arms, 2 in neck
#define NUM_BYTES_PER_MOTOR	5 //id, goal_pos_lowbyte, goal_pos_highbyte, speed_lowbyte, speed_highbyte
#define NUM_WORDS_PER_MOTOR	2 //id, goal_pos_word, speed_word


namespace MotorCommunication {
	void init();
	void setMotorPosition(int, double, int, double);
	void setMotorPositionInt(int, int);
	void enableMotor(int);
	int getZeroPose(int);
	void close();
	bool addToSyncwrite(int id, int newData[]);
	bool setSyncwriteStartAddress(int startAddress);
	bool setSyncwriteEachLength(int eachLength);
	bool sendSyncWrite();
	void setInitialPose(int motor, int adjustment);
}

#endif /* MOTOR_COMMUNICATION_H_ */
