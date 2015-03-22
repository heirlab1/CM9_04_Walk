/*
 * Walk.cpp
 *
 *  Created on: Feb 12, 2015 by Kellen Carey
 *  Modified for CM9.04 by Adam Stroud and Kellen Carey
 *	Marquette University HEIR Lab 3/22/2015
 */
 

#include "Walk.h"
#include <HardwareTimer.h>
#include <usb_serial.h>
#include <stdio.h>

HardwareTimer timer(1);

namespace WalkEngine {

#define PI 3.14159265
#define E  2.718
#define LEFT_LEG 1
#define RIGHT_LEG -1

int LEG_CENTER = 32;

int frequency = 12;//25;// 60;
int amplitude = 5;
int samples = 16;//32;// 64;
int height = 26;
double last_clock = 0;
//int fudge_factor = 4;
double fudge_factor = 50;//16
int fudge_2 = 0;
int ankle_sway_percentage = 25;
int hip_multiplier = 2;
int ankle_multiplier = 1;
int straight = 1;
double forward_back_offset = 0.0;
int next_straight = 1;
double next_fudge_factor = 4;

bool testing = true;

double sin_values[16]; //corresponds to number of samples

enum THETAS {LEFT_HIP = 0, RIGHT_HIP = 1, LEFT_ANKLE = 2, RIGHT_ANKLE = 3};
enum RAISE_LEG {L_ONE = 0, THETA_TWO = 1, THETA_ANKLE = 2};

double clock_time = 0.0;

//TODO deal with this return vector.

void getLegThetas(double result[], double amplitude) {

	//	printf("Amplitude: %f\n", amplitude);

	if (amplitude > 0) {
		double theta = PI/2.0 - acos(amplitude/(double)LEG_CENTER);
		result[LEFT_HIP] = theta;
		result[RIGHT_HIP] = -1*theta;
		result[LEFT_ANKLE] = -1*theta;
		result[RIGHT_ANKLE] = theta;
		//		printf("Theta: %f\n", theta);
	}
	else {
		double theta = PI/2.0 - acos(-1*amplitude/(double)LEG_CENTER);
		result[LEFT_HIP] = -1*theta;
		result[RIGHT_HIP] = theta;
		result[LEFT_ANKLE] = theta;
		result[RIGHT_ANKLE] = -1*theta;
		//		printf("Theta: %f\n", theta);
	}


}


//need to deal with this return vector

void raiseLeg(double result[], double height, double leg_length, double theta_one) {
	/* Calculate the values needed to raise a leg straight up while body is on an angle
	 * height is the height to raise the foot vertically
	 * leg_legth is the initial length of the leg
	 * theta_one is the angle at which the leg is in relation to the center of mass
	 *
	 * l_one is the new leg length
	 * theta_two is the (additional) angle for the hip (should be added on to the pre-existing angle)
	 * theta_ankle is the new ankle theta (can be used straight up)
	 */

	double l_one = sqrt(height*height + leg_length*leg_length - 2*height*leg_length*cos(theta_one));
	double theta_two = acos((l_one*l_one + leg_length*leg_length - height*height)/(2*l_one*leg_length));
	double theta_ankle = PI - theta_one - theta_two - PI/2.0;
	result[L_ONE] = l_one;
	result[THETA_TWO] = theta_two;
	result[THETA_ANKLE] = theta_ankle;
}

void setLegLengths(int leg, int legLength) {

	double result[3];

	if (leg == 0) {
	
	//BOTH LEGS

		result[0] = acos(((((double)legLength*(double)legLength) + 70)/(37*(double)legLength)));
		result[1] = PI - acos((614.5-((double)legLength*(double)legLength))/610.5);
		result[2] = acos((((double)legLength*(double)legLength) - 70)/(33*(double)legLength));

		MotorCommunication::setMotorPosition(3, result[0], -1, 1.0/(((double)frequency)/*16*/));
		MotorCommunication::setMotorPosition(9, result[2], -1, 1.0/(((double)frequency)/*16*/));
		MotorCommunication::setMotorPosition(7, result[1], -1, 1.0/(((double)frequency)/*16*/));


		MotorCommunication::setMotorPosition(4, result[0], -1, 1.0/(((double)frequency)/*16*/));
		MotorCommunication::setMotorPosition(10,result[2], -1, 1.0/(((double)frequency)/*16*/));
		MotorCommunication::setMotorPosition(8, result[1], -1, 1.0/(((double)frequency)/*16*/));

	}
	
	else if (leg < 0) {
		// RIGHT LEG

		result[0] = acos(((((double)legLength*(double)legLength) + 70)/(37*(double)legLength)));
		result[1] = PI - acos((614.5-((double)legLength*(double)legLength))/610.5);
		result[2] = acos((((double)legLength*(double)legLength) - 70)/(33*(double)legLength));

		MotorCommunication::setMotorPosition(7, result[1], -1, 1.0/(((double)frequency)/*16*/));
	}
	else {
		// LEFT_LEG

		result[0] = acos(((((double)legLength*(double)legLength) + 70)/(37*(double)legLength)));
		result[1] = PI - acos((614.5-((double)legLength*(double)legLength))/610.5);
		result[2] = acos((((double)legLength*(double)legLength) - 70)/(33*(double)legLength));

		MotorCommunication::setMotorPosition(8, result[1], -1, 1.0/(((double)frequency)/*16*/));
	}

}



double getUnixTime() {
	char mess[20];
	sprintf(mess, "Time = %f\n", clock_time);
	SerialUSB.print(mess); 
	return clock_time;
}


void clockUpdater(void) {
  clock_time += 0.001;
  //SerialUSB.println("clocking"); 
}




Walk::Walk() {
	// TODO Auto-generated constructor stub

}






Walk::~Walk() {
	// TODO Auto-generated destructor stub
}



void Walk::run() {
	//	init_sdl();
		SerialUSB.println("starting walk"); 
	 // Pause the timer while we're configuring it
 	 timer.pause();

 	 // Set up period
	  timer.setPeriod(1000); // in microseconds, 1000 us = 1 ms

  	// Set up an interrupt on channel 1
 	 timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
 	 timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
 	 timer.attachInterrupt(TIMER_CH1, clockUpdater);

 	 // Refresh the timer's count, prescale, and overflow
 	 timer.refresh();

 	// Start the timer counting
  	timer.resume();
  
	double legValues[NUM_LEG_THETAS]; //NUM_LEG_THETAS = 4
	double modified[NUM_MOTORS_TO_RAISE_LEG];
	
	last_clock = getUnixTime();
	int current_sin_index = 0;
	//sin_values.resize(samples);
	
	for (unsigned i = 0; i < samples; i++) {

		sin_values[i] = (sin(2.0*PI*((double)i/samples)));

		//MotorCommunication::sendSyncWrite();
	}

	setLegLengths(0, LEG_CENTER);

// 	double acc_x[samples];
// 	double acc_y[samples];
// 	double acc_z[samples];
// 
// 	double gyro_x[samples];
// 	double gyro_y[samples];
// 	double gyro_z[samples];
// 
// 	double cmps_x[samples];
// 	double cmps_y[samples];
// 	double cmps_z[samples];


	setLegLengths(0, LEG_CENTER);
	MotorCommunication::sendSyncWrite();

	//FIM ME need to substitue another form of unser input to halt the walking.
	
	while (1) {
	SerialUSB.println("walking"); 
		if (testing) {
		SerialUSB.println("testing"); 
			// Check to see if it's been long enough to update the motor positions
			// Taking 20 discrete moments along the sine curve yields the 5/freq.
			// As the freq is on a scale from 0 - 100, with 100 corresponding to 1 Hz.
			double ellapsed_time = getUnixTime()-last_clock;
			if (ellapsed_time > (1.0/(((double)frequency) - 1.0/(4.0*((double)frequency))/*16*/))) {
			
			SerialUSB.println("time to get new motor values"); 
				//		if (wait_key == 'n') {

				// TODO Get values from joystick and print to console
				//				if (Joystick::joystick)

			// 	MotorCommunication::setSyncwriteEachLength(4); //2 bytes for position, 2 bytes for speed
// 				MotorCommunication::setSyncwriteStartAddress(30); //30 is the address for goal position
				
				//resulting leg Thetas will be assigned to legValues inside this function call
				getLegThetas(legValues,(double)(amplitude)*sin_values[current_sin_index]);


// 				acc_x[current_sin_index] = IMU::get(IMU::ACCELEROMETER_X);
// 				acc_y[current_sin_index] = IMU::get(IMU::ACCELEROMETER_Y);
// 				acc_z[current_sin_index] = IMU::get(IMU::ACCELEROMETER_Z);
// 
// 				gyro_x[current_sin_index] = IMU::get(IMU::GYROSCOPE_X);
// 				gyro_y[current_sin_index] = IMU::get(IMU::GYROSCOPE_Y);
// 				gyro_z[current_sin_index] = IMU::get(IMU::GYROSCOPE_Z);
// 
// 				cmps_x[current_sin_index] = IMU::get(IMU::COMPASS_X);
// 				cmps_y[current_sin_index] = IMU::get(IMU::COMPASS_Y);
// 				cmps_z[current_sin_index] = IMU::get(IMU::COMPASS_Z);

				double head_up = Motors::getMotorPosition(24);
				double head_left = Motors::getMotorPosition(23);
				forward_back_offset = 0.0;
				
								//				if (head_left < PI/-6.0) {
				//					Dynamixel::setMotorPosition(13, 0.0, -1, 0.5);
				//					Dynamixel::setMotorPosition(14, 0.0, -1, 0.5);
				//					//					if (current_sin_index == 0) {
				//					//						straight = 2;
				//					//						fudge_factor = 5;
				//					//					}
				//				}
				//				else if (head_left > PI/6.0) {
				//					Dynamixel::setMotorPosition(13, 0.0, -1, 0.5);
				//					Dynamixel::setMotorPosition(14, 0.0, -1, 0.5);
				//					//					if (current_sin_index == 0) {
				//					//						straight = 0;
				//					//						fudge_factor = 5;
				//					//					}
				//				}
				//				else {
				//					if (head_up < PI/-4.0) {
				//						Dynamixel::setMotorPosition(13, PI/-4.0, -1, 0.5);
				//						Dynamixel::setMotorPosition(14, PI/-4.0, -1, 0.5);
				//					} else {
				//						Dynamixel::setMotorPosition(13, PI/-6.0, -1, 0.5);
				//						Dynamixel::setMotorPosition(14, PI/-6.0, -1, 0.5);
				//					}
				//					//					if (current_sin_index == 0) {
				//					//						straight = 1;
				//					//						fudge_factor = 4;
				//					//					}
				//				}

				//				if (head_up < PI/-6.0) {
				//					forward_back_offset = 0.01;
				//				}


				if (current_sin_index == 0) {
				
					straight = next_straight;
					fudge_factor = next_fudge_factor;


				}

				// 10 motors written to if this condition is true: 1, 2, 3, 5, 6, 7, 8, 9, 11, 12. 
				
				if (sin_values[current_sin_index] < -0.85) {
				SerialUSB.println("first half of step"); 
					// Raise the right leg
					raiseLeg(modified, height/10.0, LEG_CENTER, legValues[RIGHT_HIP]);
					MotorCommunication::setMotorPosition(5, legValues[RIGHT_HIP] + 15*modified[THETA_TWO], -1, 1.0/(((double)frequency)));
					MotorCommunication::setMotorPosition(11, (PI/2.0 - modified[THETA_ANKLE])*-1, -1, 1.0/(((double)frequency)/*16*/));
					MotorCommunication::setMotorPosition(6, legValues[LEFT_HIP], -1, 1.0/(((double)frequency)/*16*/));
					MotorCommunication::setMotorPosition(12,legValues[LEFT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));

					// If walking straight
					// motors written to : 3, 9, 1 , 2
					if (straight == 1) {
						MotorCommunication::setMotorPosition(3, -1*sin_values[current_sin_index]/((double)fudge_factor) /* - forward_back_offset */+
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(9, sin_values[current_sin_index]/(4*(double)fudge_factor) +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						MotorCommunication::setMotorPosition(1, 0.0, -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(2, 0.0, -1, 1.0/((double)frequency));
					}
					// If turning Left
					// motors written to : 3, 9, 1, 2
					else if (straight == 0) {
						MotorCommunication::setMotorPosition(3, /*-1*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(9, /*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						MotorCommunication::setMotorPosition(1, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(2, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
					}
					// Turning right
					// motors written to : 3, 9, 1, 2
					else {
						MotorCommunication::setMotorPosition(3, /*-1*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(9, /*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						MotorCommunication::setMotorPosition(1, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(2, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));

					}
					
					//compressing the right
					setLegLengths(RIGHT_LEG, (int)modified[L_ONE]);
					setLegLengths(LEFT_LEG, LEG_CENTER);

				}
				
				
				// 10 motors written to if this condition is true: 1, 2, 4, 5, 6, 7 , 8, 10, 11, 12.  
				
				else if (sin_values[current_sin_index] > 0.85) {
				SerialUSB.println("second half of step"); 
					// Raise the left leg. "modified" is array where results are assigned
					raiseLeg(modified, height/10.0, LEG_CENTER, legValues[RIGHT_HIP]);
					MotorCommunication::setMotorPosition(5, legValues[RIGHT_HIP], -1, 1.0/(((double)frequency)));
					MotorCommunication::setMotorPosition(11, legValues[RIGHT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					MotorCommunication::setMotorPosition(6, legValues[LEFT_HIP] + 15*modified[THETA_TWO], -1, 1.0/(((double)frequency)/*16*/));
					MotorCommunication::setMotorPosition(12,legValues[THETA_ANKLE], -1, 1.0/(((double)frequency)/*16*/));

					// If walking straight
					// motors written to : 4, 10, 1, 2
					if (straight == 1) {
						MotorCommunication::setMotorPosition(4, sin_values[current_sin_index]/((double)fudge_factor) + //forward_back_offset +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(10, -1*sin_values[current_sin_index]/(2*(double)fudge_factor) +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						MotorCommunication::setMotorPosition(1, 0.0, -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(2, 0.0, -1, 1.0/((double)frequency));
					}
					// Turning Left
					// motors written to : 4, 10, 1, 2
					else if (straight == 0) {
						MotorCommunication::setMotorPosition(4, /*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(10, /* -1*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						MotorCommunication::setMotorPosition(1, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(2, sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
					}
					// Turning Right
					// motors written to : 4, 10, 1, 2
					else {
						MotorCommunication::setMotorPosition(4, /*sin_values[current_sin_index]/((double)fudge_factor)*/ -0.1 +
								acos((((modified[L_ONE]*modified[L_ONE]) + 70)/(37*modified[L_ONE]))), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(10, /* -1*sin_values[current_sin_index]/(2*(double)fudge_factor)*/ -0.1 +
								acos(((modified[L_ONE]*modified[L_ONE]) - 70)/(33*modified[L_ONE])), -1, 1.0/((double)frequency));

						MotorCommunication::setMotorPosition(1, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
						MotorCommunication::setMotorPosition(2, -1*sin_values[current_sin_index]/((double)fudge_factor*3.0), -1, 1.0/((double)frequency));
					}
					
					//shorten the leg length to modified[L_ONE]
					setLegLengths(LEFT_LEG, (int)modified[L_ONE]);
					setLegLengths(RIGHT_LEG, LEG_CENTER);
				}
				
				// 6 motors written to if this condition is true: 5, 11, 6, 7, 8 12
				else {
				SerialUSB.println("middle of step"); 
				
					// Perform the normal leg operations
					MotorCommunication::setMotorPosition(5, legValues[RIGHT_HIP], -1, 1.0/(((double)frequency)/*16*/));
					MotorCommunication::setMotorPosition(11, legValues[RIGHT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					MotorCommunication::setMotorPosition(6, legValues[LEFT_HIP], -1, 1.0/(((double)frequency)/*16*/));
					MotorCommunication::setMotorPosition(12,legValues[LEFT_ANKLE], -1, 1.0/(((double)frequency)/*16*/));
					//					MotorCommunication::setMotorPosition(1, 0.0, -1, 1.0/((double)frequency));
					//					MotorCommunication::setMotorPosition(2, 0.0, -1, 1.0/((double)frequency));
					setLegLengths(0, LEG_CENTER);
				}



				current_sin_index = (current_sin_index + 1) % samples;

				if (current_sin_index == samples) {
					current_sin_index = 0;
				}

				// TODO replace this with our embedded version of last_clock
				last_clock = getUnixTime();
			}

		}
		
		//else, if not "testing"
		else {
		SerialUSB.println("not testing"); 
			for (int i = 0; i < 12; i++) {
				//MotorCommunication::setMotorPosition(i+1, 0.0, 100, -1);
			}
		}
		while ((getUnixTime()-last_clock) < (1.0/(((double)frequency)))) {
			// Busy wait
		}
		
		//we could be writing to 6 or 10 motors + vision motors + 
		MotorCommunication::sendSyncWrite();
		SerialUSB.println("sent a syncwrite"); 
	}
}







void Walk::turn_left() {

	next_straight = 0;
	next_fudge_factor = 5;

}




void Walk::turn_right() {

	next_straight = 2;
	next_fudge_factor = 5;

}






void Walk::walk_straight() {

	next_straight = 1;
	//	next_fudge_factor = 5;

}



} /* namespace WalkEngine */
