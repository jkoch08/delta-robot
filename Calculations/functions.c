#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/**
 * Implements kinematics functions for a delta robot.
 */



/**
 * Represents a point in 3D.
 */
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

/**
 * Tests various functions.
 */
int main(void)
{
   struct point p;
    p.x = 0;
    p.y = 0;
    p.z = -8.5064;

    if (pointValid(p))
        printf("Point is Valid.\n");
    else
        printf("Point is not Valid.\n");

    double angles[3];
    getAngles(angles, p);
    if (isnan(angles[0]))
        printf("No Kinematic Solution.");
    else
        printf("Angle 1: %f;   Angle 2: %f;  Angle 3: %f;\n",
               angles[0], angles[1], angles[2]);
    return 0;
}
