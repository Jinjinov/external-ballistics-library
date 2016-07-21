// GNU Ballistics Library
// Originally created by Derek Yates
// Now available free under the GNU GPL

#ifndef _BALLISTICS_
#define _BALLISTICS_

#define BCOMP_MAXRANGE 50000
#define GRAVITY (-32.194)
#define M_PI (3.1415926535897)

#define DegtoMOA(deg) ((deg)*60)  // Converts degrees to minutes of angle
#define DegtoRad(deg) ((deg)*0.01745329251994329576923690768489)	// Converts degrees to radians
#define MOAtoDeg(moa) ((moa)*0.01666666666666666666666666666667) // Converts minutes of angle to degrees
#define MOAtoRad(moa) ((moa)*0.00029088820866572159615394846141477) // Converts minutes of angle to radians
#define RadtoDeg(rad) ((rad)*57.295779513082320876798154814105) // Converts radians to degrees
#define RadtoMOA(rad) ((rad)*3437.7467707849392526078892888463) // Converts radians to MOA

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>

enum EDragFunction { G1=1, G2, G3, G4, G5, G6, G7, G8 };

struct ProjectilePath
{
   double Range; // range, in yards.
   double Path; // projectile path, in inches, relative to the line of sight.
   double MOA; // an estimated elevation correction for achieving a zero at this range - this is useful for "click charts" and the like.
   double Time;// the projectile's time of flight to this range.
   double Windage; // the windage correction in inches required to achieve zero at this range.
   double WindageMOA; // an approximate windage correction in MOA to achieve a zero at this range.
   double Velocity; // the projectile's total velocity (Vector product of Vx and Vy)
   double Vx; // the velocity of the projectile in the bore direction.
   double Vy; // the velocity of the projectile perpendicular to the bore direction.
};

// For very steep shooting angles, Vx can actually become what you would think of as Vy relative to the ground, 
// because Vx is referencing the bore's axis.  All computations are carried out relative to the bore's axis, and
// have very little to do with the ground's orientation.

class Projectile
{
public:
   double vx;
   double vy;

   double x;
   double y;

   int ProjectilePathCount;
   ProjectilePath* aProjectilePath;

   double t;
   double Gx;
   double Gy;
   EDragFunction DragFunction;
   double DragCoefficient;
   double MuzzleVelocity;
   double SightHeightOverBore;

   Projectile(double DragCoefficient, EDragFunction DragFunction, double MuzzleVelocity, double SightHeightOverBore)
   {
      this->DragCoefficient = DragCoefficient;
      this->DragFunction = DragFunction;
      this->MuzzleVelocity = MuzzleVelocity;
      this->SightHeightOverBore = SightHeightOverBore;

      aProjectilePath = new ProjectilePath[BCOMP_MAXRANGE];
   }
   ~Projectile()
   {
      delete[] aProjectilePath;
   }

   void Initialize(double BoreAngle, double SightToBoreAngle);
   int Update(double &dt, double headwind_mile_hr, double crosswind_mile_hr);

   // A function to calculate ballistic retardation values based on standard drag functions.
   double DragRetardationVelocity(double Velocity);
   /* Arguments:
         DragFunction:  G1, G2, G3, G4, G5, G6, G7, or G8.  All are enumerated above.
         DragCoefficient:  The coefficient of drag for the projectile for the given drag function.
         Velocity:  The Velocity of the projectile.
      
      Return Value: 
         The function returns the projectile drag retardation velocity, in ft/s per second.
         
   */

   double CalculateFactorRH(double Temperature, double Pressure, double RelativeHumidity);
   double CalculateFactorP(double Pressure);
   double CalculateFactorT(double Temperature, double Altitude);
   double CalculateFactorA(double Altitude);

   // A function to correct a "standard" Drag Coefficient for differing atmospheric conditions.
   // Returns the corrected drag coefficient for supplied drag coefficient and atmospheric conditions.
   double DragCoefficientAtmosphericCorrection(double Altitude_feet, double Barometer_hg, double Temperature_f, double RelativeHumidity);
   /* Arguments:
         DragCoefficient:  The coefficient of drag for a given projectile.
         Altitude:  The altitude above sea level in feet.  Standard altitude is 0 feet above sea level.
         Barometer:  The barometric pressure in inches of mercury (in Hg).
                  This is not "absolute" pressure, it is the "standardized" pressure reported in the papers and news.
                  Standard pressure is 29.53 in Hg.
         Temperature:  The temperature in Fahrenheit.  Standard temperature is 59 degrees.
         RelativeHumidity:  The relative humidity fraction.  Ranges from 0.00 to 1.00, with 0.50 being 50% relative humidity.
                        Standard humidity is 78%

      Return Value:
         The function returns a ballistic coefficient, corrected for the supplied atmospheric conditions.
   */


   // A function to compute the windage deflection for a given crosswind speed,
   // given flight time in a vacuum, and given flight time in real life.
   // Returns the windage correction needed in inches.
   double WindageCorrection(double WindSpeed_mile_hr, double Velocity_feet_sec, double range_feet, double time);
   /* Arguments:
         WindSpeed:  The wind velocity in mi/hr.
         Velocity:  The initial velocity of the projectile (muzzle velocity).
         range:  The range at which you wish to determine windage, in feet.
         time:  The time it has taken the projectile to traverse the range x, in seconds.
      
      Return Value:
         Returns the amount of windage correction, in inches, required to achieve zero on a target at the given range.
         
   */

   // bore = the hollow part inside a gun barrel or other tube.

   // A function to determine the bore angle needed to achieve a target zero at Range yards
   // (at standard conditions and on level ground.)
   double CalculateSightToBoreAngle(double ZeroRange_yard, double yIntercept_inch); 
   /*  Arguments: 
         DragFunction:  The drag function to use (G1, G2, G3, G5, G6, G7, G8)
         DragCoefficient:  The coefficient of drag for the projectile, for the supplied drag function.
         Velocity:  The initial velocity of the projectile, in feet/s
         SightHeight:  The height of the sighting system above the bore centerline, in inches. 
                    Most scopes fall in the 1.6 to 2.0 inch range.
         ZeroRange:  The range in yards, at which you wish the projectile to intersect yIntercept.
         yIntercept:  The height, in inches, you wish for the projectile to be when it crosses ZeroRange yards.
                  This is usually 0 for a target zero, but could be any number.  For example if you wish
                  to sight your rifle in 1.5" high at 100 yards, then you would set yIntercept to 1.5, and ZeroRange to 100
                  
      Return Value:
         Returns the angle of the bore relative to the sighting system, in degrees.
   */
};

// Functions to resolve any wind / angle combination into headwind and crosswind components.
double HeadWindVelocity(double WindSpeed_mile_hr, double WindAngle);
double CrossWindVelocity(double WindSpeed_mile_hr, double WindAngle);
/* Arguments:
      WindSpeed:  The wind velocity, in mi/hr.
      WindAngle:  The angle from which the wind is coming, in degrees.
               0 degrees is from straight ahead
               90 degrees is from right to left
               180 degrees is from directly behind
               270 or -90 degrees is from left to right.
   
   Return value:
      Returns the headwind or crosswind velocity component, in mi/hr.
*/

// bore = the hollow part inside a gun barrel or other tube.

// A function to generate a ballistic solution table in 1 yard increments, up to BCOMP_MAXRANGE.
int SolveAll(double WindSpeed_mile_hr, double WindAngle, Projectile& projectile);
/* Arguments:
      DragFunction:  The drag function you wish to use for the solution (G1, G2, G3, G5, G6, G7, or G8)
      DragCoefficient:  The coefficient of drag for the projectile you wish to model.
      Vi:  The projectile initial velocity.
      SightHeight:  The height of the sighting system above the bore centerline.  
                  Most scopes are in the 1.5"-2.0" range.
      BoreAngle:  The uphill or downhill shooting angle, in degrees.  Usually 0, but can be anything from
                  90 (directly up), to -90 (directly down).
      SightToBoreAngle:  The angle of the sighting system relative to the bore, in degrees.  This can be easily computed
               using the CalculateSightToBoreAngle() function documented above.
      WindSpeed:  The wind velocity, in mi/hr
      WindAngle:  The angle at which the wind is approaching from, in degrees.
               0 degrees is a straight headwind
               90 degrees is from right to left
               180 degrees is a straight tailwind
               -90 or 270 degrees is from left to right.
      Solution:	A pointer provided for accessing the solution after it has been generated.
               Memory for this pointer is allocated in the function, so the user does not need
               to worry about it.  This solution can be passed to the retrieval functions to get
               useful data from the solution.
      Return Value:
               This function returns an integer representing the maximum valid range of the
               solution.  This also indicates the maximum number of rows in the solution matrix,
               and should not be exceeded in order to avoid a memory segmentation fault.
*/

#endif
