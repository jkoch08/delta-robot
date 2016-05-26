/*
 * DeltaDriver.c
 *
 * Created: 6/8/2014 2:00:57 PM
 *  Author: Justin
 */ 


#define F_CPU 16000000L

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "serial.h"
#include "dynamixel.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "functions.h"


//Define state machine values
#define WATCH_BEGIN 1
#define WATCH_X 2
#define WATCH_Y 3
#define WATCH_Z 4
#define OLD_DATA 5
#define NEW_DATA 6
#define POSITIVE 7
#define NEGATIVE 8
#define CONFIRM_DATA 9
#define SEND_DATA 10

//positionDataTypes
#define POSITION 11
#define ANGLE 12



/// Control table address
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46

// Default setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define DEFAULT_ID			1

//Declare global variables
int parseState = WATCH_BEGIN;
int dataState = OLD_DATA;
int positionDataType = POSITION;

double xBuffer = 0;
double yBuffer = 0;
double zBuffer = -8.5;
int signState = POSITIVE;


//Declare functions

int parseAll(char dataIn);
void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);


int main(void)
{
	serial_initialize(57600); // USART Initialize
	dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index. i.e., 0 indexing on device number
	dxl_initialize( 1, DEFAULT_BAUDNUM ); // Init #2
	dxl_initialize( 2, DEFAULT_BAUDNUM ); // Init #3
	sei(); // set enable interrupt
	
	_delay_ms(1000);
	
	//unsigned short GoalPos[2] = {0, 1023};
	unsigned short GoalPos[2] = {1301, 3999}; // for EX series
	int index = 0;
	int id = 1;
	int bMoving, wPresentPos;
	int CommStatus;
	
	struct point p;
	double angles[3];
	angles[0] = 180;
	angles[1] = 180;
	angles[2] = 180;
	
	dxl_write_word( 1, P_GOAL_POSITION_L, (int) angles[0]*11.3778 ); //4096./360=11.3778  <--- Ticks per degree
	dxl_write_word( 2, P_GOAL_POSITION_L, (int) angles[1]*11.3778 ); //Command #2
	dxl_write_word( 3, P_GOAL_POSITION_L, (int) angles[2]*11.3778 ); //Command #3
	
	printf("Initialization complete.\n");
	
	
	
	while (1)
	{
		/*
		//DYNAMIXEL DEMO CODE
		// Check moving done
		bMoving = dxl_read_byte( id, P_MOVING );
		CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS )
		{
			if( bMoving == 0 )
			{
				// Change goal position
				if( index == 0 )
				index = 1;
				else
				index = 0;

				// Write goal position
				dxl_write_word( id, P_GOAL_POSITION_L, GoalPos[index] );
			}
			
			PrintErrorCode();
			
			// Read present position
			wPresentPos = dxl_read_word( id, P_PRESENT_POSITION_L );
			printf( "%d   %d\n",GoalPos[index], wPresentPos );
		}
		else
		PrintCommStatus(CommStatus);
		
		//END DYNAMIXEL DEMO CODE
		*/
		
		if(parseAll(getchar()) != 0){
			;//printf("\nError!\n");
			};
		
	if(dataState == NEW_DATA && positionDataType == POSITION){
			p.x = (xBuffer)/1000.;
			p.y = (yBuffer)/1000.;
			p.z = (zBuffer)/1000.;
			
			;//printf("\n\nNew input received\n");
			
			;//printf("\nIn inches:\nX: %f\nY: %f\nZ: %f\n", p.x, p.y, p.z);
			
			if (pointValid(p)){
				;//printf("\nPoint is Valid.\n");
				lookupAngles(INVERSE_TABLE, angles, p);
				
				if (isnan(angles[0])){
					;//printf("No Kinematic Solution.");
				}
				else{
					;//printf("\nProposed position:\n");
					;//printf("X Pos: %f;  Y Pos: %f;  Z Pos: %f\n", p.x, p.y, p.z);
					;//printf("Angle 1: %f;  Angle 2: %f;  Angle 3: %f\n", angles[0], angles[1], angles[2]);
					;//printf("Motor 1: %f;  Motor 2: %f;  Motor 3: %f\n", (int) angles[0]*11.3778, (int) angles[1]*11.3778, (int) angles[2]*11.3778);
					;//printf("Confirm?? (y/n)\n");
					dataState = SEND_DATA;
				}
			
				dataState = CONFIRM_DATA;
			}
			else{
				;//printf("Point is not Valid.\n");
				;//printf("Abort.\n\n");
				dataState = OLD_DATA;
			}

			
			
			
		}
		else if(dataState == NEW_DATA){//receiving angle command
			dxl_write_word( 1, P_GOAL_POSITION_L, (int) xBuffer ); //4096./360=11.3778  <--- Ticks per degree
			dxl_write_word( 2, P_GOAL_POSITION_L, (int) yBuffer ); //Command #2
			dxl_write_word( 3, P_GOAL_POSITION_L, (int) zBuffer ); //Command #3
			dataState = OLD_DATA;
		}
		else if((dataState == SEND_DATA)){
			dxl_write_word( 1, P_GOAL_POSITION_L, (int) angles[0]*11.3778 ); //4096./360=11.3778  <--- Ticks per degree
			dxl_write_word( 2, P_GOAL_POSITION_L, (int) angles[1]*11.3778 ); //Command #2
			dxl_write_word( 3, P_GOAL_POSITION_L, (int) angles[2]*11.3778 ); //Command #3
			;//printf("Command sent!\n\n");
			dataState = OLD_DATA;
		}
		

	}
	return 1;
}

//Begin general function declarations

int parseAll(char dataIn){
	int parseError = 0;
	
	switch (parseState){
		
		case WATCH_BEGIN:
			if(dataIn == '$'){//Position Command
				parseState = WATCH_X;
				xBuffer = 0;
				yBuffer = 0;
				zBuffer = 0;
				signState = POSITIVE;	
				positionDataType = POSITION;
				;//printf("\nReading Packet...\n");
			}
			else if(dataIn == '%'){//Angle Command
				parseState = WATCH_X;
				xBuffer = 0;
				yBuffer = 0;
				zBuffer = 0;
				signState = POSITIVE;
				positionDataType = ANGLE;
				;//printf("\nReading Packet...\n");
			}
			else if((dataIn == 'y') && (dataState == CONFIRM_DATA)){
				dataState = SEND_DATA;
			}
			else if((dataIn == 'n') && (dataState == NEW_DATA)){
				;//printf("\nAbort Send!!\n\n");
				dataState = OLD_DATA;
			}
			else{
				dataState = OLD_DATA;
				//printf("waiting on input\n");
			}
			break;
			
		case WATCH_X:
			if((dataIn == '0') | (dataIn == '1') | (dataIn == '2') | (dataIn == '3') | (dataIn == '4') | (dataIn == '5') | (dataIn == '6') | (dataIn == '7') | (dataIn == '8') | (dataIn == '9')){
				xBuffer = (dataIn - '0') + xBuffer*10;
			}
			else if(dataIn == ','){
				parseState = WATCH_Y;
				if(signState == NEGATIVE)
					xBuffer = -(xBuffer);
				signState = POSITIVE;
				;//printf("Read X: %f thou\n", xBuffer);
			}
			else if(dataIn == '-'){
				signState = NEGATIVE;
			}
			else{
				;//printf("\nBad Data!\n");
				parseState = WATCH_BEGIN;
			}
			break;
			
		case WATCH_Y:
			if((dataIn == '0') | (dataIn == '1') | (dataIn == '2') | (dataIn == '3') | (dataIn == '4') | (dataIn == '5') | (dataIn == '6') | (dataIn == '7') | (dataIn == '8') | (dataIn == '9')){
				yBuffer = (dataIn - '0') + yBuffer*10;
			}
			else if(dataIn == ','){
				parseState = WATCH_Z;
				if(signState == NEGATIVE)
				yBuffer = -(yBuffer);
				signState = POSITIVE;
				;//printf("Read Y: %f thou\n", yBuffer);
			}
			else if(dataIn == '-'){
				signState = NEGATIVE;
			}
			else{
				;//printf("\nBad Data!\n");
				parseState = WATCH_BEGIN;
			}
			break;
			
		case WATCH_Z:
			if((dataIn == '0') | (dataIn == '1') | (dataIn == '2') | (dataIn == '3') | (dataIn == '4') | (dataIn == '5') | (dataIn == '6') | (dataIn == '7') | (dataIn == '8') | (dataIn == '9')){
				zBuffer = (dataIn - '0') + zBuffer*10;
			}
			else if(dataIn == '*'){ //Packet complete!
				if(signState == NEGATIVE)
				zBuffer = -(zBuffer);
				signState = POSITIVE; //reset
				;//printf("Read Z: %f thou\n", zBuffer);
				dataState = NEW_DATA; 
				parseState = WATCH_BEGIN;
				
				
				
			}
			else if(dataIn == '-'){
				signState = NEGATIVE;
			}
			else{
				;//printf("\nbad Data\n");
				parseState = WATCH_BEGIN;
			}
			break;
		case CONFIRM_DATA: //OUTDATED
			if(dataIn == 'y'){
				
			}else{
				
			}
		
	}
	
	return parseError;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
		case COMM_TXFAIL:
		;//printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

		case COMM_TXERROR:
		;//printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

		case COMM_RXFAIL:
		;//printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

		case COMM_RXWAITING:
		;//printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

		case COMM_RXTIMEOUT:
		;//printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

		case COMM_RXCORRUPT:
		;//printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

		default:
		;//printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	;//printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	;//printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	;//printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	;//printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	;//printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	;//printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	;//printf("Instruction code error!\n");
}
