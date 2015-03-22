/*
 * Motors.cpp
 *
 *  Created on: Feb 5, 2015
 *      Author: mul8
 */
#include "Motors.h"


double motor_position_array[24];

void Motors::initialize() {
	for (int i = 0; i < 24; i++) {
		motor_position_array[i] = 0.0;
	}
}

double Motors::getMotorPosition(int motor) {
	double result = motor_position_array[motor-1];
	return result;
}

void Motors::setMotorPosition(int motor, double pos) {

	motor_position_array[motor-1] = pos;

}


