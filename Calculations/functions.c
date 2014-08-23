#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "functions.h"

/**
 * Implements kinematics functions for a delta robot.
 */

/**
 * Returns 1 if the point 'p' is in the workspace; 0 otherwise.
 * Workspace is a cylinder of radius R_MIN centered on the z-axis, with a
 * height ranging from Z_MIN to Z_MAX. Note: This does not check that the pen
 * does not run into the table or clamps.
 */
 int pointValid(struct point p)
 {
     return (pow(p.x, 2) + pow(p.y, 2) <= pow(R_MAX, 2) &&
             p.z >= Z_MIN &&
             p.z <= Z_MAX);
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
        if (fabs(arg1 - 1.0) < 0.0001) /* Slightly over 1 (rounding error) */
		{
			options[0] = 0;
			options[1] = 0;
		}
		else if (fabs(arg1 - (-1.0)) < 0.0001) /* Slightly < 1 (round error) */
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
	    if (fabs(arg2 - 1.0) < 0.0001) /* Slightly over 1 ( rounding error) */
	    {
		    options[2] = 0;
		    options[3] = 0;
	    }
	    else if (fabs(arg2 - (-1.0)) < 0.0001) /* Slightly < 1 (round error) */
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
	    if (fabs(arg1 - 1.0) < 0.0001) /* Slightly over 1 (rounding error) */
	    {
		    options[0] = 0;
		    options[1] = 0;
	    }
	    else if (fabs(arg1 - (-1.0)) < 0.0001) /* Slightly < 1 (round error) */
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
	    if (fabs(arg2 - 1.0) < 0.0001) /* Slightly over 1 (rounding error) */
	    {
		    options[2] = 0;
		    options[3] = 0;
	    }
	    else if (fabs(arg2 - (-1.0)) < 0.0001) /* Slightly < 1 (round error) */
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
	    if (fabs(arg1 - 1.0) < 0.0001) /* Slightly over 1 (rounding error) */
	    {
		    options[0] = 0;
		    options[1] = 0;
	    }
	    else if (fabs(arg1 - (-1.0)) < 0.0001) /* Slightly < 1 (round error) */
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
	    if (fabs(arg2 - 1.0) < 0.0001) /* Slightly over 1 (rounding error) */
	    {
		    options[2] = 0;
		    options[3] = 0;
	    }
	    else if (fabs(arg2 - (-1.0)) < 0.0001) /* Slightly < 1 (round error) */
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

/**
 * Initializes the values of the empty table 'inverseTable' with the inverse
 * kinematics values attained by iterating x, y, z from *_TABLE_MIN to
 * *_TABLE_MAX by *_TABLE_RES. Entries are length 3 arrays correspoding to
 * [angle1, angle2, angle3]. Points with no kinematic solution receive angle
 * values [0, 0, 0].
 */
void generateInverseTable(double inverseTable[X_TABLE_DIM]
                          [Y_TABLE_DIM][Z_TABLE_DIM][3])
{
    struct point p;
    int xIndex;
    int yIndex;
    int zIndex;

    for (xIndex = 0; xIndex < X_TABLE_DIM; xIndex++)
    {
        for (yIndex = 0; yIndex < Y_TABLE_DIM; yIndex++)
        {
            for (zIndex = 0; zIndex < Z_TABLE_DIM; zIndex++)
            {
                p.x = X_TABLE_MIN + X_TABLE_RES * xIndex;
                p.y = Y_TABLE_MIN + Y_TABLE_RES * yIndex;
                p.z = Z_TABLE_MIN + Z_TABLE_RES * zIndex;
                getAngles(inverseTable[xIndex][yIndex][zIndex], p);
                /* Change to [0, 0, 0] if no solution. */
                if (isnan(inverseTable[xIndex][yIndex][zIndex][0]))
                {
                    inverseTable[xIndex][yIndex][zIndex][0] = 0.0;
                    inverseTable[xIndex][yIndex][zIndex][1] = 0.0;
                    inverseTable[xIndex][yIndex][zIndex][2] = 0.0;
                }
            }
        }
    }
}

/**
 * Writes the table 'inverseTable' in a format that enables its initialization
 * in the C programming language. File name is 'inverse_table.txt'.
 */
void writeInverseTable(double inverseTable[X_TABLE_DIM]
                       [Y_TABLE_DIM][Z_TABLE_DIM][3])
{
    FILE *f = fopen("inverse_table.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }

    /** Generate lines to be printed. */
    int xIndex;
    int yIndex;
    int zIndex;

    fprintf(f, "{\n");
    for (xIndex = 0; xIndex < X_TABLE_DIM; xIndex++)
    {
        fprintf(f, "    {\n");
        for (yIndex = 0; yIndex < Y_TABLE_DIM; yIndex++)
        {
            fprintf(f, "        {\n");
            for (zIndex = 0; zIndex < Z_TABLE_DIM; zIndex++)
            {
                fprintf(f, "            {%.6f, %.6f, %.6f}",
                       inverseTable[xIndex][yIndex][zIndex][0],
                       inverseTable[xIndex][yIndex][zIndex][1],
                       inverseTable[xIndex][yIndex][zIndex][2]);
                /* Add a comma unless this is the last entry. */
                if (zIndex != Z_TABLE_DIM - 1)
                    fprintf(f, ",");
                fprintf(f, "\n");
            }
            fprintf(f, "        }");
            /* Add a comma unless this is the last entry. */
            if (yIndex != Y_TABLE_DIM - 1)
                fprintf(f, ",");
            fprintf(f, "\n");
        }
        fprintf(f, "    }");
        /* Add a comma unless this is the last entry. */
        if (yIndex != Y_TABLE_DIM - 1)
            fprintf(f, ",");
        fprintf(f, "\n");
    }
    fprintf(f, "}\n");

    fclose(f);
}

/**
 * Determines the required angles for servo actuators 1, 2, and 3 respectively
 * to get to a desired point 'p' (in degrees & in servo coordinates) by
 * interpolating from values in 'inverseTable', and stores the result in
 * 'angles'. If no solution exists, sets 'angles[0]', 'angles[1]', and
 * 'angles[2]' equal to NAN.
 */
void lookupAngles(double inverseTable[X_TABLE_DIM][Y_TABLE_DIM][Z_TABLE_DIM][3],
                  double* angles, struct point p)
{
    int DEBUG = 0; /* Results in print statements for debugging. */
    /* First, check boundaries of table. */
    if (!(p.x >= X_TABLE_MIN &&
          p.x <= X_TABLE_MAX &&
          p.y >= Y_TABLE_MIN &&
          p.y <= Y_TABLE_MAX &&
          p.z >= Z_TABLE_MIN &&
          p.z <= Z_TABLE_MAX))
    {
        angles[0] = NAN;
        angles[1] = NAN;
        angles[2] = NAN;
        return;
    }

    /* Second, find surrounding indices corresponding to point 'p'.
       'xInd, yInd, and zInd represent the indices correspoinding to the lattice
       point just before 'p'. */
    int xInd = pointToIndex('x', p.x);
    int yInd = pointToIndex('y', p.y);
    int zInd = pointToIndex('z', p.z);

    /* Third, get the proportion of the distance between adjacent lattice points
       on each of the three axes. */
    double xProp = (p.x - indexToPoint('x', xInd)) / X_TABLE_RES;
    double yProp = (p.y - indexToPoint('y', yInd)) / Y_TABLE_RES;
    double zProp = (p.z - indexToPoint('z', zInd)) / Z_TABLE_RES;

    if (DEBUG == 1)
        printf("xProp: %.4f, yProp: %.4f, zProp: %.4f\n",
               xProp, yProp, zProp);

    /* Fourth, lookup the angles corresponding to the 8 points
       p0 = (xInd, yInd, zInd)
       p1 = (xInd, yInd, zInd + 1)
       p2 = (xInd, yInd + 1, zInd)
       p3 = (xInd, yInd + 1, zInd + 1)
       p4 = (xInd + 1, yInd, zInd)
       p5 = (xInd + 1, yInd, zInd + 1)
       p6 = (xInd + 1, yInd + 1, zInd)
       p7 = (xInd + 1, yInd + 1, zInd + 1)
       Note: 'api' is the set of angles corresponding to point 'i'.
    */
    double ap0[3] = {inverseTable[xInd][yInd][zInd][0],
                     inverseTable[xInd][yInd][zInd][1],
                     inverseTable[xInd][yInd][zInd][2]};
    double ap1[3] = {inverseTable[xInd][yInd][zInd + 1][0],
                     inverseTable[xInd][yInd][zInd + 1][1],
                     inverseTable[xInd][yInd][zInd + 1][2]};
    double ap2[3] = {inverseTable[xInd][yInd + 1][zInd][0],
                     inverseTable[xInd][yInd + 1][zInd][1],
                     inverseTable[xInd][yInd + 1][zInd][2]};
    double ap3[3] = {inverseTable[xInd][yInd + 1][zInd + 1][0],
                     inverseTable[xInd][yInd + 1][zInd + 1][1],
                     inverseTable[xInd][yInd + 1][zInd + 1][2]};
    double ap4[3] = {inverseTable[xInd + 1][yInd][zInd][0],
                     inverseTable[xInd + 1][yInd][zInd][1],
                     inverseTable[xInd + 1][yInd][zInd][2]};
    double ap5[3] = {inverseTable[xInd + 1][yInd][zInd + 1][0],
                     inverseTable[xInd + 1][yInd][zInd + 1][1],
                     inverseTable[xInd + 1][yInd][zInd + 1][2]};
    double ap6[3] = {inverseTable[xInd + 1][yInd + 1][zInd][0],
                     inverseTable[xInd + 1][yInd + 1][zInd][1],
                     inverseTable[xInd + 1][yInd + 1][zInd][2]};
    double ap7[3] = {inverseTable[xInd + 1][yInd + 1][zInd + 1][0],
                     inverseTable[xInd + 1][yInd + 1][zInd + 1][1],
                     inverseTable[xInd + 1][yInd + 1][zInd + 1][2]};
    if (DEBUG == 1)
    {
        printf("\n");
        printf("X: %.3f, %.3f\n",
               indexToPoint('x', xInd), indexToPoint('x', xInd + 1));
        printf("Y: %.3f, %.3f\n",
               indexToPoint('y', yInd), indexToPoint('y', yInd + 1));
        printf("Z: %.3f, %.3f\n",
               indexToPoint('z', zInd), indexToPoint('z', zInd + 1));

        printf("\n");
        printAngles("ap0", ap0);
        printAngles("ap1", ap1);
        printAngles("ap2", ap2);
        printAngles("ap3", ap3);
        printAngles("ap4", ap4);
        printAngles("ap5", ap5);
        printAngles("ap6", ap6);
        printAngles("ap7", ap7);
    }

    /* Check that all points have a kinematic solution. Invalid angles will be
       represented by NAN if being read from memory, or 0 if being read from
       file. */
    int valid = 1;
    int i;
    /* Ideally should iterate i = 0, 1, 2, but can same time by only checking
       i = 0 case. */
    for (i = 0; i < 1; i++)
    {
        if (isnan(ap0[i]) || isnan(ap1[i]) || isnan(ap2[i]) || isnan(ap3[i]) ||
            isnan(ap4[i]) || isnan(ap5[i]) || isnan(ap6[i]) || isnan(ap7[i]))
        {
            valid = 0;
            break;
        }
        if (abs(ap0[i]) < 0.01 || abs(ap1[i]) < 0.01 || abs(ap2[i]) < 0.01 ||
            abs(ap3[i]) < 0.01 || abs(ap4[i]) < 0.01 || abs(ap5[i]) < 0.01 ||
            abs(ap6[i]) < 0.01 || abs(ap7[i]) < 0.01)
        {
            valid = 0;
            break;
        }
    }
    if (valid == 0)
    {
        angles[0] = NAN;
        angles[1] = NAN;
        angles[2] = NAN;
        return;
    }

    /* Fifth, collapse the x-axis about point 'p', and approximate by
       interpolation the four points
       p10 - average of p0 and p4
       p11 - average of p1 and p5
       p12 - average of p2 and p6
       p13 - average of p3 and p7 */
    double ap10[3] = {(1 - xProp) * ap0[0] + xProp * ap4[0],
                      (1 - xProp) * ap0[1] + xProp * ap4[1],
                      (1 - xProp) * ap0[2] + xProp * ap4[2]};
    double ap11[3] = {(1 - xProp) * ap1[0] + xProp * ap5[0],
                      (1 - xProp) * ap1[1] + xProp * ap5[1],
                      (1 - xProp) * ap1[2] + xProp * ap5[2]};
    double ap12[3] = {(1 - xProp) * ap2[0] + xProp * ap6[0],
                      (1 - xProp) * ap2[1] + xProp * ap6[1],
                      (1 - xProp) * ap2[2] + xProp * ap6[2]};
    double ap13[3] = {(1 - xProp) * ap3[0] + xProp * ap7[0],
                      (1 - xProp) * ap3[1] + xProp * ap7[1],
                      (1 - xProp) * ap3[2] + xProp * ap7[2]};
    if (DEBUG == 1)
    {
        printf("\n");
        printAngles("ap10", ap10);
        printAngles("ap11", ap11);
        printAngles("ap12", ap12);
        printAngles("ap13", ap13);
    }

    /* Sixth, collapse the y-axis about point 'p', and approximate by
       interpolation the two points
       p20 - average of p10 and p12
       p21 - average of p11 and p13 */
    double ap20[3] = {(1 - yProp) * ap10[0] + yProp * ap12[0],
                      (1 - yProp) * ap10[1] + yProp * ap12[1],
                      (1 - yProp) * ap10[2] + yProp * ap12[2]};
    double ap21[3] = {(1 - yProp) * ap11[0] + yProp * ap13[0],
                      (1 - yProp) * ap11[1] + yProp * ap13[1],
                      (1 - yProp) * ap11[2] + yProp * ap13[2]};

    if (DEBUG == 1)
    {
        printf("\n");
        printAngles("ap20", ap20);
        printAngles("ap21", ap21);
    }

    /* Seventh, collapse the z-axis about point 'p', and approximate by
       interpolation the point 'p' (average of p20 and p21). Store the result in
       'angles'. */
    angles[0] = (1 - zProp) * ap20[0] + zProp * ap21[0];
    angles[1] = (1 - zProp) * ap20[1] + zProp * ap21[1];
    angles[2] = (1 - zProp) * ap20[2] + zProp * ap21[2];

    if (DEBUG == 1)
    {
        printf("\n");
        printAngles("OUT", angles);
        printf("\n");
    }
}

/**
 * Returns the coordinate on axis 'axis' ('x', 'y', or 'z') corresponding to
 * index 'index' from the lookup table.
 */
double indexToPoint(char axis, int index)
{
    if (axis == 'x')
        return X_TABLE_MIN + X_TABLE_RES * index;
    else if (axis == 'y')
        return Y_TABLE_MIN + Y_TABLE_RES * index;
    else if (axis == 'z')
        return Z_TABLE_MIN + Z_TABLE_RES * index;
    else
    {
        fprintf(stderr, "'axis' must be 'x', 'y', or 'z'.");
        return NAN;
    }
}

/**
 * Returns the index in the lookup table corresponding to the point 'point' on
 * the correspoding 'axis' ('x', 'y', or 'z').
 */
int pointToIndex(char axis, double point)
{
    if (axis == 'x')
        return (int) ((point - X_TABLE_MIN ) / X_TABLE_RES);
    else if (axis == 'y')
        return (int) ((point - Y_TABLE_MIN ) / Y_TABLE_RES);
    else if (axis == 'z')
        return (int) ((point - Z_TABLE_MIN ) / Z_TABLE_RES);
    else
    {
        fprintf(stderr, "'axis' must be 'x', 'y', or 'z'.");
        return -1;
    }
}

/**
 * Prints a set of 'angles' for easy viewing and debugging. The string
 * 'indicator' will precede the angle values.
 */
 void printAngles(char indicator[], double angles[3])
 {
    printf("%s - Angle 1: %.2f;   Angle 2: %.2f;  Angle 3: %.2f;\n",
           indicator, angles[0], angles[1], angles[2]);
 }

/**
 * Tests various functions.
 */
int main(void)
{
//    /** Generating Inverse Kinematics lookup table. */
//    double inverseTable[X_TABLE_DIM][Y_TABLE_DIM][Z_TABLE_DIM][3];
//    /* Fill in elements of table. */
//    generateInverseTable(inverseTable);
//    /* Write inverse table to file. */
//    writeInverseTable(inverseTable);

    /** Test Individual point. */

    struct point p;
    int i;
    for (i = 0; i < 1; i++)
    {
        p.x = 6;
        p.y = 0;
        p.z = -9.0;

        printf("Point Evaluated: (%.3f, %.3f, %.3f)\n", p.x, p.y, p.z);

        if (pointValid(p))
            printf("Point is Valid.\n");
        else
           printf("Point is not Valid.\n");

        double anglesExact[3];
        double anglesApprox[3];
        getAngles(anglesExact, p);
        lookupAngles(INVERSE_TABLE, anglesApprox, p);
        if (isnan(anglesApprox[0]))
            printf("No Kinematic Solution.");
        else
        {
            printAngles("EXACT ", anglesExact);
            printAngles("APPROX", anglesApprox);
        }
    }
    return 0;
}
