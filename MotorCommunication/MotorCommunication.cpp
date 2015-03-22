/*
 * MotorCommunication.cpp
 *
 *  Created on: Feb 12, 2015 by Kellen Carey
 *  Modified for CM9.04 by Adam Stroud and Kellen Carey
 *	Marquette University HEIR Lab 3/22/2015
 */

#include "MotorCommunication.h"
#include <Dynamixel.h>
#include <stdio.h>

Dynamixel Dxl(1);


bool new_robot = false;

#define PI 3.14159265


int initial_poses[] = {650, 3436, 1005, 2079,
		2495, 2159, 503, 1548, 2638, 1101,
		1992, 1058
};



int pos[14];
int last_pos[14];

// Variables for the SyncWrite function
int ids[NUM_MOTORS];
int ids_index = 0;

int data[NUM_MOTORS*NUM_BYTES_PER_MOTOR];
int data_index = 0;

int start_address = 0;
int each_length = 0;



void MotorCommunication::init() {
	//dxl_initialize(0, 0);
	Dxl.begin(3);

	if (new_robot) {

pos[0] = 2048;
pos[1] = 2058;
pos[2] = 1891;
pos[3] = 2245;
pos[4] = 1978;
pos[5] = 2008;
pos[6] = 1575;
pos[7] = 2536;
pos[8] = 2649;
pos[9] = 2652;
pos[10] = 1988;
pos[11] = 1968;

	} else {

pos[0] = 628;
pos[1] = 3596;
pos[2] = 1210;
pos[3] = 1884;
pos[4] = 2443;
pos[5] = 2107;
pos[6] = 503;
pos[7] = 1548;
pos[8] = 2588;
pos[9] = 1041;
pos[10] = 1975;
pos[11] = 1058;

	}

	pos[12] = getZeroPose(13); //motors 13 and 14 (shoulder sockets)
	pos[13] = getZeroPose(14);
	
	//TODO just for simulating perfect motor movement
	for (int i = 0; i<12; i++){
		
		last_pos[i] = pos[i];
		
	} 
	
	for (int i = 0; i<12; i++){
		
		Dxl.writeWord(i+1, P_MOVING_SPEED, 5);
		Dxl.writeByte(i+1, STATUS_RETURN_LEVEL, 1);
	}
		
	
	for (int i = 0; i<12; i++){
		Dxl.writeWord(i+1, P_GOAL_POSITION, pos[i]);
		SerialUSB.println(pos[i]); 
		
	} 
}

void MotorCommunication::close() {
	//dxl_terminate();
}

int MotorCommunication::getZeroPose(int motor) {
	if (motor > 0 && motor <= 12) {
		//		return initial_poses[motor-1];
		return pos[motor-1];
	}
	else if (motor == 23) {
		return 2048;
	}
	else if (motor == 24) {
		return 1070;
	}
	else if (motor == 13) {
		return 1000;
	}
	else if (motor == 14) {
		return 2600;
	}
	else {
		return Dxl.readWord(motor, P_PRESENT_POSITION);
	}
}

void MotorCommunication::setInitialPose(int motor, int adjustment) {
	pos[motor-1] += adjustment;
}

void MotorCommunication::setMotorPositionInt(int motor, int position) {
	Dxl.writeWord(motor, P_MOVING_SPEED, 50);
	Dxl.writeWord(motor, P_GOAL_POSITION, position);
}


//THIS IS IMPORTANT!!!!!
//This function takes a motor angle in degrees whihc is a double, and converts it to the appropriate motor position.
//This function aslo adds the motor speed and position to the syncWrite packet.
 
void MotorCommunication::setMotorPosition(int motor, double angle, int speed = -1, double time = -1) {

	// Convert angle to motor positions
	int motor_positions = (int)(angle/(2.0*PI) * 4096.0);
	
	//TODO for simulating perfecto motor movement
	int present_position = last_pos[motor-1]; //
	
// 	int present_position = Dxl.readWord(motor, P_PRESENT_POSITION);
	
	int goal_position = 0;

	int zero_position = getZeroPose(motor);
	
	int newData[4];
	
	switch (motor) {
	case 1:

		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	case 2:

		goal_position = zero_position - motor_positions;
		newData[0] = DXL_LOBYTE(zero_position - motor_positions);
		newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		break;
	case 3:

		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	case 4:

		goal_position = zero_position - motor_positions;
		newData[0] = DXL_LOBYTE(zero_position - motor_positions);
		newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		break;
	case 5:

		goal_position = zero_position - motor_positions;
		newData[0] = DXL_LOBYTE(zero_position - motor_positions);
		newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		break;
	case 6:

		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	case 7:

		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	case 8:

		goal_position = zero_position - motor_positions;
		newData[0] = DXL_LOBYTE(zero_position - motor_positions);
		newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		break;
	case 9:

		if (new_robot) {
			goal_position = zero_position - motor_positions;
			newData[0] = DXL_LOBYTE(zero_position - motor_positions);
			newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		} else {
			goal_position = zero_position + motor_positions;
			newData[0] = DXL_LOBYTE(zero_position + motor_positions);
			newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		}
		break;
	case 10:

		if (new_robot) {
			goal_position = zero_position - motor_positions;
			newData[0] = DXL_LOBYTE(zero_position - motor_positions);
			newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		} else {
			goal_position = zero_position + motor_positions;
			newData[0] = DXL_LOBYTE(zero_position + motor_positions);
			newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		}
		break;
	case 11:

		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	case 12:

		goal_position = zero_position - motor_positions;
		newData[0] = DXL_LOBYTE(zero_position - motor_positions);
		newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		break;
	default:
		Dxl.writeByte(motor, P_ENABLE, true);
		return;
	case 13:
		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	case 14:
		goal_position = zero_position - motor_positions;
		newData[0] = DXL_LOBYTE(zero_position - motor_positions);
		newData[1] = DXL_HIBYTE(zero_position - motor_positions);
		break;
// 	case 15:
// 		break;
// 	case 16:
// 		break;
// 	case 17:
// 		break;
// 	case 18:
// 		break;
	case 23:
		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	case 24:
		goal_position = zero_position + motor_positions;
		newData[0] = DXL_LOBYTE(zero_position + motor_positions);
		newData[1] = DXL_HIBYTE(zero_position + motor_positions);
		break;
	}
	if (speed == -1) {
		// Calculate speed from present position
		if (goal_position < present_position) {
			speed = (int)(((double)present_position - goal_position) / (time * 1.5) * 60.0 / 4096.0 / 0.114);
		}
		else {
			speed =(int) (((double)goal_position - present_position) / (time * 1.5) * 60.0 / 4096.0 / 0.114);
		}
		if (speed > 1023) {
			speed = 1023;
			//printf("\t");
		}
//		printf("Motor %d position set to: %d, speed set to: %d\n",motor, goal_position, speed);
	}
	
	
	//TODO for testing safety
	
// 	speed *=0.2;
	
	newData[2] = DXL_LOBYTE(speed);
	newData[3] = DXL_HIBYTE(speed);

	char mess[50]; 
	sprintf(mess, "home_pos = %i\t\tstart pos= %i\t\tpos = %i\t\tmotor %i\t\tspeed = %i\n", pos[motor-1], present_position, goal_position, motor, speed);
	SerialUSB.print(mess); 
	
// 	Dxl.writeWord(motor, P_MOVING_SPEED, speed);
// 	Dxl.writeWord(motor, P_GOAL_POSITION, goal_position);
	
	newData[0] = DXL_LOBYTE(goal_position);
	newData[1] = DXL_HIBYTE(goal_position);
	newData[2] = DXL_LOBYTE(speed);
	newData[3] = DXL_HIBYTE(speed);
	
	addToSyncwrite(motor, newData);

	Motors::setMotorPosition(motor, angle);
	last_pos[motor-1] = goal_position;
}

void MotorCommunication::enableMotor(int motor) {
	Dxl.writeByte(motor, P_ENABLE, true);
}


bool MotorCommunication::addToSyncwrite(int id, int newData[]) {
	//std::cout << "Adding ID " << id << ": ";
	ids[ids_index] = id;
	for (unsigned i = 0; i < 4; i++) {
		data[data_index+i] = newData[i];
	}
	ids_index++;
	data_index += 4;
	//TODO get rid of these when implementing sycWrite
// 	ids_index = 0;
// 	data_index = 0;
	return true;
}

bool MotorCommunication::setSyncwriteStartAddress(int startAddress) {
	start_address = startAddress;
	return true;
}

bool MotorCommunication::setSyncwriteEachLength(int eachLength) {
	each_length = eachLength;
	return true;
}

bool MotorCommunication::sendSyncWrite() {
	
	//this is the CM9.04 version of a syncWrite
	each_length = 4;
	
	Dxl.initPacket(BROADCAST_ID, INST_SYNC_WRITE);
	Dxl.pushByte(P_GOAL_POSITION);
	Dxl.pushByte(each_length);
	
	//for each motor, we send 5 bytes. 1st is the id number
	for(int i=0; i<ids_index; i++ ) {
		
		Dxl.pushByte(ids[i]);
		//2nd 3rd 4th and 5th are the low and high bytes of goal_pos and speed respectively.
		for(int j = 0; j < each_length; j++){
		
			Dxl.pushByte(data[i * each_length + j]);
		}

	}

// 	for (int i = 0; i < data_index; i++) {
// 		Dxl.pushByte(data[i]);
// 	}
	
	Dxl.flushPacket();
	
	if(!Dxl.getResult()){
   // SerialUSB.println("Comm Fail");
  	}
	
	ids_index = 0;
	
	data_index = 0;

// 
// 	dxl_set_txpacket_id(BROADCAST_ID);//ID of destination
// 	dxl_set_txpacket_instruction(INST_SYNC_WRITE);// packet type
// 	dxl_set_txpacket_parameter(0, start_address);//which data to manipulate. speed, goal pos, ect
// 	dxl_set_txpacket_parameter(1, each_length);//how long the instruction will be. 2 for word, 1 for byte,
// 
// 	for(int i=0; i<idsLength; i++ )
// 	{
// 		// Set the ID number
// 		dxl_set_txpacket_parameter(2+(each_length+1)*i, ids[i]);
// 		//std::cout << "ID " << ids[i] << ": ";
// 		// Set the data values
// 		for (int j = 1; j < each_length+1; j++) {
// 			dxl_set_txpacket_parameter(2+(each_length+1)*i + j, data[(each_length)*i + (j-1)]);
// 			//std::cout << data[(each_length) * i + j-1] << " ";
// 		}
// 
// 	}
// 	//std::cout << "made it here" << std::endl;
// 
// 	dxl_set_txpacket_length((each_length+1)*idsLength+4);//(2+1) for writing a word, (1+1) for writing a byte, 4 is a constant
// 
// 	dxl_txrx_packet();//sends the packet
// 
// 
// 	if (dxl_get_result( ) == COMM_TXSUCCESS) {
// 		return true;
// 	}

	SerialUSB.println("flushed syncwrite packet"); 
	return false;

}
