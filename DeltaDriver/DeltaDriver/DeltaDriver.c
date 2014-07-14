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

double xBuffer = 0;
double yBuffer = 0;
double zBuffer = -8.5;
int signState = POSITIVE;


//Declare functions

int parseAll(char dataIn);
void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);


// Declare Kinematic variables and functions

struct point
{
	double x;
	double y;
	double z;
};

int pointValid(struct point p);
void getAngles(double* angles, struct point p);
double getAngle1(struct point p);
double getAngle2(struct point p);
double getAngle3(struct point p);
double removeExtraneous(double* viableOptions);
double modAngle(double angle);
int minInd1(double* lst, int length);
int minInd2(double* lst, int length);

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
			printf("\nError!\n");
			};
		
		if(dataState == NEW_DATA){
			p.x = (xBuffer)/1000.;
			p.y = (yBuffer)/1000.;
			p.z = (zBuffer)/1000.;
			
			printf("\n\nNew input received\n");
			
			printf("\nIn inches:\nX: %f\nY: %f\nZ: %f\n", p.x, p.y, p.z);
			
			if (pointValid(p)){
				printf("\nPoint is Valid.\n");
				getAngles(angles, p);
				
				if (isnan(angles[0])){
					printf("No Kinematic Solution.");
				}
				else{
					printf("\nProposed position:\n");
					printf("X Pos: %f;  Y Pos: %f;  Z Pos: %f\n", p.x, p.y, p.z);
					printf("Angle 1: %f;  Angle 2: %f;  Angle 3: %f\n", angles[0], angles[1], angles[2]);
					printf("Motor 1: %f;  Motor 2: %f;  Motor 3: %f\n", (int) angles[0]*11.3778, (int) angles[1]*11.3778, (int) angles[2]*11.3778);
					printf("Confirm? (y/n)\n");
				}
			
				dataState = CONFIRM_DATA;
			}
			else{
				printf("Point is not Valid.\n");
				printf("Abort.\n\n");
				dataState = OLD_DATA;
			}

			
			
			
		}
		else if((dataState == SEND_DATA)){
			dxl_write_word( 1, P_GOAL_POSITION_L, (int) angles[0]*11.3778 ); //4096./360=11.3778  <--- Ticks per degree
			dxl_write_word( 2, P_GOAL_POSITION_L, (int) angles[1]*11.3778 ); //Command #2
			dxl_write_word( 3, P_GOAL_POSITION_L, (int) angles[2]*11.3778 ); //Command #3
			printf("Command sent!\n\n");
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
			if(dataIn == '$'){
				parseState = WATCH_X;
				xBuffer = 0;
				yBuffer = 0;
				zBuffer = 0;
				signState = POSITIVE;	
				printf("\nReading Packet...\n");
			}
			else if((dataIn == 'y') && (dataState == CONFIRM_DATA)){
				dataState = SEND_DATA;
			}
			else if((dataIn == 'n') && (dataState == NEW_DATA)){
				printf("\nAbort Send!!\n\n");
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
				printf("Read X: %f thou\n", xBuffer);
			}
			else if(dataIn == '-'){
				signState = NEGATIVE;
			}
			else{
				printf("\nBad Data!\n");
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
				printf("Read Y: %f thou\n", yBuffer);
			}
			else if(dataIn == '-'){
				signState = NEGATIVE;
			}
			else{
				printf("\nBad Data!\n");
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
				printf("Read Z: %f thou\n", zBuffer);
				dataState = NEW_DATA; 
				parseState = WATCH_BEGIN;
				
				
				
			}
			else if(dataIn == '-'){
				signState = NEGATIVE;
			}
			else{
				printf("\nbad Data\n");
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
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

		case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

		case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

		case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

		case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

		case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

		default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	printf("Instruction code error!\n");
}





/*
 * Begin functions for kinematic math
 *
 */

/**
 * Returns 1 if the point 'p' is in the workspace; 0 otherwise.
 * Workspace is a cylinder of radius r centered on the z-axis, with a
 * height ranging from z_min to z_max.
 */
 int pointValid(struct point p)
 {
     double r = 6.0;        /* Radius of cylinder in inches. */
     double z_min = -13.0;  /* Lowest height in inches. */
     double z_max = -8.0;   /* Maximum height in inches. */

     return (pow(p.x, 2) + pow(p.y, 2) <= pow(r, 2) &&
             p.z >= z_min &&
             p.z <= z_max);
 }

/**
 * Calculates the required angles for servo actuators 1, 2, and 3 respectively
 * to get to a desired point 'p' (in degrees & in servo coordinates), and stores
 * the result in 'angles'. If no solution exists, sets 'angles[0]', 'angles[1]',
 * and 'angles[2]' equal to NAN.
 */
void getAngles(double* angles, struct point p)
{
    angles[0] = getAngle1(p);
    angles[1] = getAngle2(p);
    angles[2] = getAngle3(p);
    if (isnan(angles[0]) || isnan(angles[1]) || isnan(angles[2]))
    {
        angles[0] = NAN;
        angles[1] = NAN;
        angles[2] = NAN;
    }
}

/**
 * Returns the required angle of servo actuator 1 to get to a desired point 'p'
 * in degrees and servo coordinates. If no solution exists, returns NAN.
 */
double getAngle1(struct point p)
{
    double options [4]; /* Mathematica outputs four possible expressions for the
                           angle. Two are invalid, and one is extraneous. */
    /* GENERATE FOUR POSSIBLE OPTIONS. */
    double arg1 = -(-214 * sqrt(3) + 6 * sqrt(3) * pow(p.x, 2) -
                  pow(p.x, 3) + 2 * sqrt(3) * pow(p.y, 2) + 2 * sqrt(3) *
                  pow(p.z, 2) - p.x * (-83 + pow(p.y, 2) + pow(p.z, 2)) +
                  sqrt(-pow(p.z, 2) * (10249 - 8 * sqrt(3) * pow(p.x, 3)
                  + pow(p.x, 4) + pow(p.y, 4) - 314 * pow(p.z, 2) +
                  pow(p.z, 4) + 2 * pow(p.y, 2) * (-107 + pow(p.z, 2)) -
                  8 * sqrt(3) * p.x * (-157 + pow(p.y, 2) + pow(p.z, 2))
                  + 2 * pow(p.x, 2) * (-133 + pow(p.y, 2) +
                  pow(p.z, 2))))) / (10 * (12 - 4 * sqrt(3) * p.x +
                  pow(p.x, 2) + pow(p.z, 2)));
    double arg2 = -(-214 * sqrt(3) + 6 * sqrt(3) * pow(p.x, 2) -
                  pow(p.x, 3) + 2 * sqrt(3) * pow(p.y, 2) + 2 * sqrt(3) *
                  pow(p.z, 2) - p.x * (-83 + pow(p.y, 2) + pow(p.z, 2)) -
                  sqrt(-pow(p.z, 2) * (10249 - 8 * sqrt(3) * pow(p.x, 3)
                  + pow(p.x, 4) + pow(p.y, 4) - 314 * pow(p.z, 2) +
                  pow(p.z, 4) + 2 * pow(p.y, 2) * (-107 + pow(p.z, 2)) -
                  8 * sqrt(3) * p.x * (-157 + pow(p.y, 2) + pow(p.z, 2))
                  + 2 * pow(p.x, 2) * (-133 + pow(p.y, 2) +
                  pow(p.z, 2))))) / (10 * (12 - 4 * sqrt(3) * p.x +
                  pow(p.x, 2) + pow(p.z, 2)));
    
	if (fabs(arg1) > 1.0)
    {
        if (fabs(arg1 - 1.0) < 0.0001) /* Slightly over 1 (from rounding error) */
		{
			options[0] = 0;
			options[1] = 0;
		}
		else if (fabs(arg1 - (-1.0)) < 0.0001) /* Slightly less than 1 (from rounding error) */
		{
			options[0] = M_PI;
			options[1] = -M_PI;
		}
		else
		{
		    options[0] = NAN;
		    options[1] = NAN;			
		}
    }
    else
    {
        options[0] = acos(arg1);
        options[1] = -acos(arg1);
    }
	
    if (fabs(arg2) > 1.0)
    {
	    if (fabs(arg2 - 1.0) < 0.0001) /* Slightly over 1 (from rounding error) */
	    {
		    options[2] = 0;
		    options[3] = 0;
	    }
	    else if (fabs(arg2 - (-1.0)) < 0.0001) /* Slightly less than 1 (from rounding error) */
	    {
		    options[2] = M_PI;
		    options[3] = -M_PI;
	    }
	    else
	    {
		    options[2] = NAN;
		    options[3] = NAN;
	    }
    }
    else
    {
        options[2] = acos(arg2);
        options[3] = -acos(arg2);
    }
	
    /* REMOVE INVALID SOLUTIONS */
    double values [4]; /* One val for each option; pick 2 smallest. */
    int i; /* Iterate over options. */
    for (i = 0; i < 4; i++)
    {
        if (isnan(options[i]))
            values[i] = DBL_MAX;
        else
            values[i] =
                  fabs(pow(p.x + (4 - 10) / sqrt(3) - 5 * cos(options[i]), 2)
                + pow(p.y, 2)
                + pow(p.z - 5 * sin(options[i]), 2)
                - 144);
        /* Above expression comes from original kinematics equation. */
    }
    /* Find desired indices of 'options'. */
    int ind1 = minInd1(values, sizeof(values) / sizeof(values[0]));
    int ind2 = minInd2(values, sizeof(values) / sizeof(values[0]));
    /* Pick the two closest options to the original equation. */
    double viableOptions [2];
    viableOptions[0] = (ind1 == -1) ? NAN : options[ind1];
    viableOptions[1] = (ind2 == -1) ? NAN : options[ind2];
    /* REMOVE EXTRANEOUS SOLUTION + CONVERT ANGLE*/
    return modAngle(removeExtraneous(viableOptions));
}

/**
 * Returns the required angle of servo actuator 2 to get to a desired point 'p'
 * in degrees and servo coordinates. If no solution exists, returns NAN.
 */
double getAngle2(struct point p)
{
    double options [4]; /* Mathematica outputs four possible expressions for the
                           angle. Two are invalid, and one is extraneous. */
    /* GENERATE FOUR POSSIBLE OPTIONS. */
    double arg1 = -(-428 * sqrt(3) + pow(p.x, 3) - sqrt(3) * pow(p.x, 2) *
                    (-6 + p.y) + 83 * sqrt(3) *  p.y + 10 * sqrt(3) *
                    pow(p.y, 2) - sqrt(3) * pow(p.y, 3) + 4 * sqrt(3) *
                    pow(p.z, 2) - sqrt(3) * p.y * pow(p.z, 2) + p.x * (-83 -
                    12 * p.y + pow(p.y, 2) + pow(p.z, 2)) + 2 *
                    sqrt(-pow(p.z, 2) * (10249 + 4 * sqrt(3) * pow(p.x, 3) +
                    pow(p.x, 4) - 12 * pow(p.y, 3) + pow(p.y, 4) - 314 *
                    pow(p.z, 2) + pow(p.z, 4) - 12 * p.y * (-157 + pow(p.z, 2))
                    + pow(p.y, 2) * (-253 + 2 *
                    pow(p.z, 2)) + pow(p.x, 2) * (-227 - 12 * p.y + 2 *
                    pow(p.y, 2) + 2 * pow(p.z, 2)) + 2 * sqrt(3) * p.x * (-314 +
                    13 * p.y + 2 * pow(p.y, 2) + 2 * pow(p.z, 2)))))/(5 * (48 +
                    pow(p.x, 2) - 2 * sqrt(3) * p.x * (-4 + p.y) - 24 * p.y + 3
                    * pow(p.y, 2) + 4 * pow(p.z, 2)));
    double arg2 = -(-428 * sqrt(3) + pow(p.x, 3) - sqrt(3) * pow(p.x, 2) *
                    (-6 + p.y) + 83 * sqrt(3) *  p.y + 10 * sqrt(3) *
                    pow(p.y, 2) - sqrt(3) * pow(p.y, 3) + 4 * sqrt(3) *
                    pow(p.z, 2) - sqrt(3) * p.y * pow(p.z, 2) + p.x * (-83 -
                    12 * p.y + pow(p.y, 2) + pow(p.z, 2)) - 2 *
                    sqrt(-pow(p.z, 2) * (10249 + 4 * sqrt(3) * pow(p.x, 3) +
                    pow(p.x, 4) - 12 * pow(p.y, 3) + pow(p.y, 4) - 314 *
                    pow(p.z, 2) + pow(p.z, 4) - 12 * p.y * (-157 + pow(p.z, 2))
                    + pow(p.y, 2) * (-253 + 2 *
                    pow(p.z, 2)) + pow(p.x, 2) * (-227 - 12 * p.y + 2 *
                    pow(p.y, 2) + 2 * pow(p.z, 2)) + 2 * sqrt(3) * p.x * (-314 +
                    13 * p.y + 2 * pow(p.y, 2) + 2 * pow(p.z, 2)))))/(5 * (48 +
                    pow(p.x, 2) - 2 * sqrt(3) * p.x * (-4 + p.y) - 24 * p.y + 3
                    * pow(p.y, 2) + 4 * pow(p.z, 2)));

    if (fabs(arg1) > 1.0)
    {
	    if (fabs(arg1 - 1.0) < 0.0001) /* Slightly over 1 (from rounding error) */
	    {
		    options[0] = 0;
		    options[1] = 0;
	    }
	    else if (fabs(arg1 - (-1.0)) < 0.0001) /* Slightly less than 1 (from rounding error) */
	    {
		    options[0] = M_PI;
		    options[1] = -M_PI;
	    }
	    else
	    {
		    options[0] = NAN;
		    options[1] = NAN;
	    }
    }
    else
    {
	    options[0] = acos(arg1);
	    options[1] = -acos(arg1);
    }
    
    if (fabs(arg2) > 1.0)
    {
	    if (fabs(arg2 - 1.0) < 0.0001) /* Slightly over 1 (from rounding error) */
	    {
		    options[2] = 0;
		    options[3] = 0;
	    }
	    else if (fabs(arg2 - (-1.0)) < 0.0001) /* Slightly less than 1 (from rounding error) */
	    {
		    options[2] = M_PI;
		    options[3] = -M_PI;
	    }
	    else
	    {
		    options[2] = NAN;
		    options[3] = NAN;
	    }
    }
    else
    {
	    options[2] = acos(arg2);
	    options[3] = -acos(arg2);
    }
    
    /* REMOVE INVALID SOLUTIONS */
    double values [4]; /* One val for each option; pick 2 smallest. */
    int i; /* Iterate over options. */
    for (i = 0; i < 4; i++)
    {
        if (isnan(options[i]))
            values[i] = DBL_MAX;
        else
            values[i] = fabs(pow(p.x + (10 - 4) / (2 * sqrt(3)) + 5 *
                        cos(options[i]) / 2.0, 2) + pow(p.y + (4 - 10 - 5 *
                        sqrt(3) * cos(options[i])) / 2.0, 2) + pow(p.z - 5 *
                        sin(options[i]), 2) - 144);
        /* Above expression comes from original kinematics equation. */
    }
    /* Find desired indices of 'options'. */
    int ind1 = minInd1(values, sizeof(values) / sizeof(values[0]));
    int ind2 = minInd2(values, sizeof(values) / sizeof(values[0]));
    /* Pick the two closest options to the original equation. */
    double viableOptions [2];
    viableOptions[0] = (ind1 == -1) ? NAN : options[ind1];
    viableOptions[1] = (ind2 == -1) ? NAN : options[ind2];
    /* REMOVE EXTRANEOUS SOLUTION + CONVERT ANGLE*/
    return modAngle(removeExtraneous(viableOptions));
}

/**
 * Returns the required angle of servo actuator 3 to get to a desired point 'p'
 * in degrees and servo coordinates. If no solution exists, returns NAN.
 */
double getAngle3(struct point p)
{
    double options [4]; /* Mathematica outputs four possible expressions for the
                           angle. Two are invalid, and one is extraneous. */
    /* GENERATE FOUR POSSIBLE OPTIONS. */
    double arg1 = -(-428 * sqrt(3) + pow(p.x, 3) - 83 * sqrt(3) * p.y +
                   10 * sqrt(3) * pow(p.y, 2) + sqrt(3) * pow(p.y, 3) +
                   sqrt(3) * pow(p.x, 2) * (6 + p.y) + 4 * sqrt(3) *
                   pow(p.z, 2) + sqrt(3) * p.y * pow(p.z, 2) + p.x * (-83 +
                   12 * p.y + pow(p.y, 2) + pow(p.z, 2)) + 2 * sqrt(
                   -pow(p.z, 2) * (10249 + 4 * sqrt(3) * pow(p.x, 3) +
                   pow(p.x, 4) + 12 * pow(p.y, 3) + pow(p.y, 4) -
                   314 * pow(p.z, 2) + pow(p.z, 4) + 12 * p.y * (-157 +
                   pow(p.z, 2)) + pow(p.y, 2) * (-253 + 2 * pow(p.z, 2)) +
                   2 * sqrt(3) * p.x * (-314 - 13 * p.y + 2 * pow(p.y, 2) +
                   2 * pow(p.z, 2)) + pow(p.x, 2) * (-227 + 12 * p.y + 2 *
                   pow(p.y, 2) + 2 * pow(p.z, 2)))))/(5 * (48 + pow(p.x, 2) +
                   24 * p.y + 3 * pow(p.y, 2) + 2 * sqrt(3) * p.x * (4 + p.y)
                   + 4 * pow(p.z, 2)));
    double arg2 = -(-428 * sqrt(3) + pow(p.x, 3) - 83 * sqrt(3) * p.y +
                   10 * sqrt(3) * pow(p.y, 2) + sqrt(3) * pow(p.y, 3) +
                   sqrt(3) * pow(p.x, 2) * (6 + p.y) + 4 * sqrt(3) *
                   pow(p.z, 2) + sqrt(3) * p.y * pow(p.z, 2) + p.x * (-83 +
                   12 * p.y + pow(p.y, 2) + pow(p.z, 2)) - 2 * sqrt(
                   -pow(p.z, 2) * (10249 + 4 * sqrt(3) * pow(p.x, 3) +
                   pow(p.x, 4) + 12 * pow(p.y, 3) + pow(p.y, 4) -
                   314 * pow(p.z, 2) + pow(p.z, 4) + 12 * p.y * (-157 +
                   pow(p.z, 2)) + pow(p.y, 2) * (-253 + 2 * pow(p.z, 2)) +
                   2 * sqrt(3) * p.x * (-314 - 13 * p.y + 2 * pow(p.y, 2) +
                   2 * pow(p.z, 2)) + pow(p.x, 2) * (-227 + 12 * p.y + 2 *
                   pow(p.y, 2) + 2 * pow(p.z, 2)))))/(5 * (48 + pow(p.x, 2) +
                   24 * p.y + 3 * pow(p.y, 2) + 2 * sqrt(3) * p.x * (4 + p.y)
                   + 4 * pow(p.z, 2)));

    if (fabs(arg1) > 1.0)
    {
	    if (fabs(arg1 - 1.0) < 0.0001) /* Slightly over 1 (from rounding error) */
	    {
		    options[0] = 0;
		    options[1] = 0;
	    }
	    else if (fabs(arg1 - (-1.0)) < 0.0001) /* Slightly less than 1 (from rounding error) */
	    {
		    options[0] = M_PI;
		    options[1] = -M_PI;
	    }
	    else
	    {
		    options[0] = NAN;
		    options[1] = NAN;
	    }
    }
    else
    {
	    options[0] = acos(arg1);
	    options[1] = -acos(arg1);
    }
    
    if (fabs(arg2) > 1.0)
    {
	    if (fabs(arg2 - 1.0) < 0.0001) /* Slightly over 1 (from rounding error) */
	    {
		    options[2] = 0;
		    options[3] = 0;
	    }
	    else if (fabs(arg2 - (-1.0)) < 0.0001) /* Slightly less than 1 (from rounding error) */
	    {
		    options[2] = M_PI;
		    options[3] = -M_PI;
	    }
	    else
	    {
		    options[2] = NAN;
		    options[3] = NAN;
	    }
    }
    else
    {
	    options[2] = acos(arg2);
	    options[3] = -acos(arg2);
    }
    
        /* REMOVE INVALID SOLUTIONS */
    double values [4]; /* One val for each option; pick 2 smallest. */
    int i; /* Iterate over options. */
    for (i = 0; i < 4; i++)
    {
        if (isnan(options[i]))
            values[i] = DBL_MAX;
        else
            values[i] = fabs(pow(p.x + (10 - 4) / (2 * sqrt(3)) + 5 *
                        cos(options[i]) / 2.0, 2) + pow(p.y + (10 - 4 + 5 *
                        sqrt(3) * cos(options[i])) / 2.0, 2) + pow(p.z - 5 *
                        sin(options[i]), 2) - 144);
        /* Above expression comes from original kinematics equation. */
    }
    /* Find desired indices of 'options'. */
    int ind1 = minInd1(values, sizeof(values) / sizeof(values[0]));
    int ind2 = minInd2(values, sizeof(values) / sizeof(values[0]));
    /* Pick the two closest options to the original equation. */
    double viableOptions [2];
    viableOptions[0] = (ind1 == -1) ? NAN : options[ind1];
    viableOptions[1] = (ind2 == -1) ? NAN : options[ind2];
    /* REMOVE EXTRANEOUS SOLUTION + CONVERT ANGLE*/
    return modAngle(removeExtraneous(viableOptions));
}

/**
 * Returns the desired angle solution from a double* of two viable options.
 * Returns NAN if viable options are both NAN.
 */
double removeExtraneous(double* viableOptions)
{
    if (!isnan(viableOptions[0]) && !isnan(viableOptions[1]))
    {
        double option1;
        double option2;

        /* Convert [0, 2pi] to [-pi, pi]. */
        if (viableOptions[0] > M_PI)
            option1 = viableOptions[0] - 2 * M_PI;
        else
            option1 = viableOptions[0];

        if (viableOptions[1] > M_PI)
            option2 = viableOptions[1] - 2 * M_PI;
        else
            option2 = viableOptions[1];
        /* Return angle with smallest absolute value. */
        if (fabs(option1) <= fabs(option2))
            return viableOptions[0];
        else
            return viableOptions[1];
    }
    else if (!isnan(viableOptions[0]) && isnan(viableOptions[1]))
        return viableOptions[0];
    else if (isnan(viableOptions[0]) && !isnan(viableOptions[1]))
        return viableOptions[1];
    else
        return NAN; /* Both options are NAN. */
}

/**
 * Converts an angle from kinematics coordinates (in radians from -pi to pi) to
 * servo coordinates (in degrees from 0 to 360).
 */
double modAngle(double angle)
{
    double modAngle = 180.0 - 180.0 * angle / M_PI;
    if (modAngle < 0)
        return modAngle + 360.0;
    else
        return modAngle;
}

/**
 * Returns the index of the smallest double in an array 'lst' of doubles of
 * length 'length'. Returns -1 for empty list or list with minimum value
 * DBL_MAX.
 */
int minInd1(double* lst, int length)
{
    double minimum = DBL_MAX;
    int minInd1 = 0;
    int i;
    for (i = 0; i < length; i++)
    {
        if (lst[i] < minimum)
        {
            minimum = lst[i];
            minInd1 = i;
        }
    }
    if (fabs(minimum - DBL_MAX) < 1)
        return -1;
    else
        return minInd1;
}

/**
 * Returns the index of the second smallest double in an array 'lst' of doubles
 * of length 'length'. Returns -1 for list of size 0 or 1, or list with second
 * smallest value DBL_MAX.
 */
int minInd2(double* lst, int length)
{
    double minimum = DBL_MAX;
    double minimum2 = DBL_MAX; /* Second smallest minimum */
    int minInd2 = 0;
    int i;
    for (i = 0; i < length; i++)
    {
        if (lst[i] < minimum)
            minimum = lst[i];
        else if (lst[i] < minimum2)
        {
            minimum2 = lst[i];
            minInd2 = i;
        }
    }
    if (fabs(minimum2 - DBL_MAX) < 1)
        return -1;
    else
        return minInd2;
}