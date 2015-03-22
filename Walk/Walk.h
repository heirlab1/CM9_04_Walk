/*
 * Walk.h
 *
 *  Created on: Feb 12, 2015 by Kellen Carey
 *  Modified for CM9.04 by Adam Stroud and Kellen Carey
 *	Marquette University HEIR Lab 3/22/2015
 */

#ifndef WALK_H_
#define WALK_H_

#include "MotorCommunication.h"
#include "Motors.h"
#include <math.h>


#define NUM_LEG_THETAS 4
#define NUM_MOTORS_TO_CHANGE_LEG_LENGTH 3
#define NUM_MOTORS_TO_RAISE_LEG 3

namespace WalkEngine {

class Walk {


public:
	Walk();
	virtual ~Walk();
	void run();
	void turn_left();
	void turn_right();
	void walk_straight();

};

} /* namespace WalkEngine */

#endif /* WALK_H_ */
