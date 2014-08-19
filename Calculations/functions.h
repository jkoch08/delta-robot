/**
 * Implements kinematics functions for a delta robot.
 */

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

/**
 * GLOBAL CONSTANTS
 */

/** WORKSPACE. */
#define R_MAX         6.00    /* Max radius of cylinder in inches. */
#define Z_MIN       -13.00    /* Lowest height in inches. */
#define Z_MAX        -8.00    /* Maximum height in inches. */
#define PEN_DIST      1.50    /* Portrusion of pen below bottom face of bottom
                                 triangle in inches. Must be >= 1.50, or Z_MIN
                                 limit will prevent pen from touching paper. */

/** INVERSE LOOKUP TABLE SPECS. */
#define X_TABLE_MIN   -6.00   /* Minimum x-value in lookup table. */
#define X_TABLE_MAX    6.00   /* Maximum x-value in lookup table. */
#define X_TABLE_RES    0.25  /* x-axis resolution in lookup table. */

#define Y_TABLE_MIN   -6.00   /* Minimum y-value in lookup table. */
#define Y_TABLE_MAX    6.00   /* Maximum y-value in lookup table. */
#define Y_TABLE_RES    0.25  /* y-axis resolution in lookup table. */

#define Z_TABLE_MIN  -13.00   /* Minimum z-value in lookup table. */
#define Z_TABLE_MAX   -8.00   /* Maximum z-value in lookup table. */
#define Z_TABLE_RES    0.25   /* z-axis resolution in lookup table. */

/* xyz Dimensions. */
#define X_TABLE_DIM  (int) ((X_TABLE_MAX - X_TABLE_MIN) / X_TABLE_RES + 1)
#define Y_TABLE_DIM  (int) ((Y_TABLE_MAX - Y_TABLE_MIN) / Y_TABLE_RES + 1)
#define Z_TABLE_DIM  (int) ((Z_TABLE_MAX - Z_TABLE_MIN) / Z_TABLE_RES + 1)

/** MANUAL INVERSE LOOKUP TABLE GENERATION. */
//double INVERSE_TABLE [X_TABLE_DIM] [Y_TABLE_DIM][Z_TABLE_DIM][3] =


/**
 * Represents a point in 3D.
 */
struct point
{
    double x;
    double y;
    double z;
};

/**
 * Functions.
 */
int pointValid(struct point p);
void getAngles(double* angles, struct point p);
double getAngle1(struct point p);
double getAngle2(struct point p);
double getAngle3(struct point p);
double removeExtraneous(double* viableOptions);
double modAngle(double angle);
int minInd1(double* lst, int length);
int minInd2(double* lst, int length);
void generateInverseTable(double inverseTable[X_TABLE_DIM]
                          [Y_TABLE_DIM][Z_TABLE_DIM][3]);
void writeInverseTable(double inverseTable[X_TABLE_DIM]
                       [Y_TABLE_DIM][Z_TABLE_DIM][3]);
void lookupAngles(double inverseTable[X_TABLE_DIM][Y_TABLE_DIM][Z_TABLE_DIM][3],
                  double* angles, struct point p);

#endif /* FUNCTIONS_H */
