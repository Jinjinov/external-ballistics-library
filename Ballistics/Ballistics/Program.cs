// GNU Ballistics Library
// Originally created by Derek Yates
// Now available free under the GNU GPL

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Ballistics
{
    struct Projectile
    {
        public double Range; // range, in yards.
        public double Path; // projectile path, in inches, relative to the line of sight.
        public double MOA; // an estimated elevation correction for achieving a zero at this range - this is useful for "click charts" and the like.
        public double Time;// the projectile's time of flight to this range.
        public double Windage; // the windage correction in inches required to achieve zero at this range.
        public double WindageMOA; // an approximate windage correction in MOA to achieve a zero at this range.
        public double Velocity; // the projectile's total velocity (Vector product of Vx and Vy)
        public double Vx; // the velocity of the projectile in the bore direction.
        public double Vy; // the velocity of the projectile perpendicular to the bore direction.
    }

    // For very steep shooting angles, Vx can actually become what you would think of as Vy relative to the ground, 
    // because Vx is referencing the bore's axis.  All computations are carried out relative to the bore's axis, and
    // have very little to do with the ground's orientation.

    class Program
    {
        static int __BCOMP_MAXRANGE__ = 50001;
        static int size = 0;

        static double GRAVITY = -32.194;
        static double M_PI = 3.1415926535897;

        enum DragFunction { G1=1,G2,G3,G4,G5,G6,G7,G8 };

        static List<Projectile> ProjectileYards = new List<Projectile>();

        static void Main(string[] args)
        {
            double ballisticCoefficient = 0.465; // The ballistic coefficient for the projectile.
            double initialVelocity = 2650; // Initial velocity, in ft/s
            double sightHeightOverBore = 1.6; // The Sight height over bore, in inches.
            double hillAngle = 0; // The shooting angle (uphill / downhill), in degrees.
            double zeroRange = 200; // The zero range of the rifle, in yards.
            double windspeed = 0; // The wind speed in miles per hour.
            double windangle = 0; // The wind angle (0=headwind, 90=right to left, 180=tailwind, 270/-90=left to right)
            double yIntercept = 0; // The height, in inches, you wish for the projectile to be when it crosses ZeroRange yards.

            double Altitude = 0; // The altitude above sea level in feet.  Standard altitude is 0 feet above sea level.
            double Barometer = 29.59; // The barometric pressure in inches of mercury (in Hg).
            double Temperature = 59; // The temperature in Fahrenheit.  Standard temperature is 59 degrees.
            double RelativeHumidity = 0.78; // The relative humidity fraction.  Ranges from 0.00 to 1.00, with 0.50 being 50% relative humidity.

            // If we wish to use the weather correction features, we need to 
            // Correct the BC for any weather conditions.  If we want standard conditions,
            // then we can just leave this commented out.
            ballisticCoefficient = DragCoefficientAtmosphericCorrection(ballisticCoefficient, Altitude, Barometer, Temperature, RelativeHumidity);

            // First find the angle of the bore relative to the sighting system.
            // We call this the "zero angle", since it is the angle required to 
            // achieve a zero at a particular yardage.  This value isn't very useful
            // to us, but is required for making a full ballistic solution.
            // It is left here to allow for zero-ing at altitudes (bc) different from the
            // final solution, or to allow for zero's other than 0" (ex: 3" high at 100 yards)
            double boreAngle = BoreAngleNeededToAchieveZero(DragFunction.G1, ballisticCoefficient, initialVelocity, sightHeightOverBore, zeroRange, yIntercept);

            // Now we have everything needed to generate a full solution.
            // So we do.  The solution is stored in the pointer "sln" passed as the last argument.
            // k has the number of yards the solution is valid for, also the number of rows in the solution.
            int rowCount = SolveAll(DragFunction.G1, ballisticCoefficient, initialVelocity, sightHeightOverBore, hillAngle, boreAngle, windspeed, windangle);

            // Now print a simple chart of X / Y trajectory spaced at 10yd increments
            int s = 0;
            for (s = 0; s <= 100; s++)
            {
                Console.WriteLine("X: {0,20:0.0000} Y: {1,20:0.0000} t: {2,20:0.0000}", ProjectileYards[s * 10].Range, ProjectileYards[s * 10].Path, ProjectileYards[s * 10].Time);
            }

            Console.ReadKey();
        }

        // A function to calculate ballistic retardation values based on standard drag functions.
        /* Arguments:
		        DragFunction:  G1, G2, G3, G4, G5, G6, G7, or G8.  All are enumerated above.
		        DragCoefficient:  The coefficient of drag for the projectile for the given drag function.
		        Vi:  The Velocity of the projectile.
        	
	        Return Value: 
		        The function returns the projectile drag retardation velocity, in ft/s per second.
        */
        static double DragRetardationVelocity(DragFunction DragFunction, double DragCoefficient, double Velocity_feet_sec)
        {
            double vp = Velocity_feet_sec;
            double val = -1;
            double A = -1;
            double M = -1;

            switch (DragFunction)
            {
                case DragFunction.G1:
                    if (vp > 4230) { A = 1.477404177730177e-04; M = 1.9565; }
                    else if (vp > 3680) { A = 1.920339268755614e-04; M = 1.925; }
                    else if (vp > 3450) { A = 2.894751026819746e-04; M = 1.875; }
                    else if (vp > 3295) { A = 4.349905111115636e-04; M = 1.825; }
                    else if (vp > 3130) { A = 6.520421871892662e-04; M = 1.775; }
                    else if (vp > 2960) { A = 9.748073694078696e-04; M = 1.725; }
                    else if (vp > 2830) { A = 1.453721560187286e-03; M = 1.675; }
                    else if (vp > 2680) { A = 2.162887202930376e-03; M = 1.625; }
                    else if (vp > 2460) { A = 3.209559783129881e-03; M = 1.575; }
                    else if (vp > 2225) { A = 3.904368218691249e-03; M = 1.55; }
                    else if (vp > 2015) { A = 3.222942271262336e-03; M = 1.575; }
                    else if (vp > 1890) { A = 2.203329542297809e-03; M = 1.625; }
                    else if (vp > 1810) { A = 1.511001028891904e-03; M = 1.675; }
                    else if (vp > 1730) { A = 8.609957592468259e-04; M = 1.75; }
                    else if (vp > 1595) { A = 4.086146797305117e-04; M = 1.85; }
                    else if (vp > 1520) { A = 1.954473210037398e-04; M = 1.95; }
                    else if (vp > 1420) { A = 5.431896266462351e-05; M = 2.125; }
                    else if (vp > 1360) { A = 8.847742581674416e-06; M = 2.375; }
                    else if (vp > 1315) { A = 1.456922328720298e-06; M = 2.625; }
                    else if (vp > 1280) { A = 2.419485191895565e-07; M = 2.875; }
                    else if (vp > 1220) { A = 1.657956321067612e-08; M = 3.25; }
                    else if (vp > 1185) { A = 4.745469537157371e-10; M = 3.75; }
                    else if (vp > 1150) { A = 1.379746590025088e-11; M = 4.25; }
                    else if (vp > 1100) { A = 4.070157961147882e-13; M = 4.75; }
                    else if (vp > 1060) { A = 2.938236954847331e-14; M = 5.125; }
                    else if (vp > 1025) { A = 1.228597370774746e-14; M = 5.25; }
                    else if (vp > 980) { A = 2.916938264100495e-14; M = 5.125; }
                    else if (vp > 945) { A = 3.855099424807451e-13; M = 4.75; }
                    else if (vp > 905) { A = 1.185097045689854e-11; M = 4.25; }
                    else if (vp > 860) { A = 3.566129470974951e-10; M = 3.75; }
                    else if (vp > 810) { A = 1.045513263966272e-08; M = 3.25; }
                    else if (vp > 780) { A = 1.291159200846216e-07; M = 2.875; }
                    else if (vp > 750) { A = 6.824429329105383e-07; M = 2.625; }
                    else if (vp > 700) { A = 3.569169672385163e-06; M = 2.375; }
                    else if (vp > 640) { A = 1.839015095899579e-05; M = 2.125; }
                    else if (vp > 600) { A = 5.71117468873424e-05; M = 1.950; }
                    else if (vp > 550) { A = 9.226557091973427e-05; M = 1.875; }
                    else if (vp > 250) { A = 9.337991957131389e-05; M = 1.875; }
                    else if (vp > 100) { A = 7.225247327590413e-05; M = 1.925; }
                    else if (vp > 65) { A = 5.792684957074546e-05; M = 1.975; }
                    else if (vp > 0) { A = 5.206214107320588e-05; M = 2.000; }
                    break;

                case DragFunction.G2:
                    if (vp > 1674) { A = .0079470052136733; M = 1.36999902851493; }
                    else if (vp > 1172) { A = 1.00419763721974e-03; M = 1.65392237010294; }
                    else if (vp > 1060) { A = 7.15571228255369e-23; M = 7.91913562392361; }
                    else if (vp > 949) { A = 1.39589807205091e-10; M = 3.81439537623717; }
                    else if (vp > 670) { A = 2.34364342818625e-04; M = 1.71869536324748; }
                    else if (vp > 335) { A = 1.77962438921838e-04; M = 1.76877550388679; }
                    else if (vp > 0) { A = 5.18033561289704e-05; M = 1.98160270524632; }
                    break;

                case DragFunction.G5:
                    if (vp > 1730) { A = 7.24854775171929e-03; M = 1.41538574492812; }
                    else if (vp > 1228) { A = 3.50563361516117e-05; M = 2.13077307854948; }
                    else if (vp > 1116) { A = 1.84029481181151e-13; M = 4.81927320350395; }
                    else if (vp > 1004) { A = 1.34713064017409e-22; M = 7.8100555281422; }
                    else if (vp > 837) { A = 1.03965974081168e-07; M = 2.84204791809926; }
                    else if (vp > 335) { A = 1.09301593869823e-04; M = 1.81096361579504; }
                    else if (vp > 0) { A = 3.51963178524273e-05; M = 2.00477856801111; }
                    break;

                case DragFunction.G6:
                    if (vp > 3236) { A = 0.0455384883480781; M = 1.15997674041274; }
                    else if (vp > 2065) { A = 7.167261849653769e-02; M = 1.10704436538885; }
                    else if (vp > 1311) { A = 1.66676386084348e-03; M = 1.60085100195952; }
                    else if (vp > 1144) { A = 1.01482730119215e-07; M = 2.9569674731838; }
                    else if (vp > 1004) { A = 4.31542773103552e-18; M = 6.34106317069757; }
                    else if (vp > 670) { A = 2.04835650496866e-05; M = 2.11688446325998; }
                    else if (vp > 0) { A = 7.50912466084823e-05; M = 1.92031057847052; }
                    break;

                case DragFunction.G7:
                    if (vp > 4200) { A = 1.29081656775919e-09; M = 3.24121295355962; }
                    else if (vp > 3000) { A = 0.0171422231434847; M = 1.27907168025204; }
                    else if (vp > 1470) { A = 2.33355948302505e-03; M = 1.52693913274526; }
                    else if (vp > 1260) { A = 7.97592111627665e-04; M = 1.67688974440324; }
                    else if (vp > 1110) { A = 5.71086414289273e-12; M = 4.3212826264889; }
                    else if (vp > 960) { A = 3.02865108244904e-17; M = 5.99074203776707; }
                    else if (vp > 670) { A = 7.52285155782535e-06; M = 2.1738019851075; }
                    else if (vp > 540) { A = 1.31766281225189e-05; M = 2.08774690257991; }
                    else if (vp > 0) { A = 1.34504843776525e-05; M = 2.08702306738884; }
                    break;

                case DragFunction.G8:
                    if (vp > 3571) { A = .0112263766252305; M = 1.33207346655961; }
                    else if (vp > 1841) { A = .0167252613732636; M = 1.28662041261785; }
                    else if (vp > 1120) { A = 2.20172456619625e-03; M = 1.55636358091189; }
                    else if (vp > 1088) { A = 2.0538037167098e-16; M = 5.80410776994789; }
                    else if (vp > 976) { A = 5.92182174254121e-12; M = 4.29275576134191; }
                    else if (vp > 0) { A = 4.3917343795117e-05; M = 1.99978116283334; }
                    break;

                default:
                    break;

            }

            if (A != -1 && M != -1 && vp > 0 && vp < 10000)
            {
                val = A * Math.Pow(vp, M) / DragCoefficient;
                return val;
            }
            else
            {
                return -1;
            }
        }

        static double calcFR(double Temperature, double Pressure, double RelativeHumidity)
        {
            double VPw = 4e-6 * Math.Pow(Temperature, 3) - 0.0004 * Math.Pow(Temperature, 2) + 0.0234 * Temperature - 0.2517;
            double FRH = 0.995 * (Pressure / (Pressure - (0.3783) * (RelativeHumidity) * VPw));
            return FRH;
        }

        static double calcFP(double Pressure)
        {
            double Pstd = 29.53; // in-hg
            double FP = (Pressure - Pstd) / (Pstd);
            return FP;
        }

        static double calcFT(double Temperature, double Altitude)
        {
            double Tstd = -0.0036 * Altitude + 59;
            double FT = (Temperature - Tstd) / (459.6 + Tstd);
            return FT;
        }

        static double calcFA(double Altitude)
        {
            double fa = -4e-15 * Math.Pow(Altitude, 3) + 4e-10 * Math.Pow(Altitude, 2) - 3e-5 * Altitude + 1;
            return (1 / fa);
        }

        // A function to correct a "standard" Drag Coefficient for differing atmospheric conditions.
        // Returns the corrected drag coefficient for supplied drag coefficient and atmospheric conditions.
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
        static double DragCoefficientAtmosphericCorrection(double DragCoefficient, double Altitude_feet, double Barometer_hg, double Temperature_f, double RelativeHumidity)
        {
            double FA = calcFA(Altitude_feet);
            double FT = calcFT(Temperature_f, Altitude_feet);
            double FR = calcFR(Temperature_f, Barometer_hg, RelativeHumidity);
            double FP = calcFP(Barometer_hg);

            // Calculate the atmospheric correction factor
            double CD = (FA * (1 + FT - FP) * FR);
            return DragCoefficient * CD;
        }

        // A function to compute the windage deflection for a given crosswind speed,
        // given flight time in a vacuum, and given flight time in real life.
        // Returns the windage correction needed in inches.
        /* Arguments:
		        WindSpeed:  The wind velocity in mi/hr.
		        Vi:  The initial velocity of the projectile (muzzle velocity).
		        x:  The range at which you wish to determine windage, in feet.
		        t:  The time it has taken the projectile to traverse the range x, in seconds.
        	
	        Return Value:
		        Returns the amount of windage correction, in inches, required to achieve zero on a target at the given range.
        		
        */
        static double WindageCorrectionRequiredToAchieveZero(double WindSpeed_mile_hr, double Velocity_feet_sec, double range_feet, double time)
        {
            double Vw = WindSpeed_mile_hr * 17.60; // Convert to inches per second.
            return (Vw * (time - range_feet / Velocity_feet_sec));
        }

        // Functions to resolve any wind / angle combination into headwind and crosswind components.
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
        static double HeadWindVelocity(double WindSpeed_mile_hr, double WindAngle)
        {
            double Wangle = DegtoRad(WindAngle);
            return Math.Cos(Wangle) * WindSpeed_mile_hr;
        }
        static double CrossWindVelocity(double WindSpeed_mile_hr, double WindAngle)
        {
            double Wangle = DegtoRad(WindAngle);
            return Math.Sin(Wangle) * WindSpeed_mile_hr;
        }

        // bore = the hollow part inside a gun barrel or other tube.

        // A function to determine the bore angle needed to achieve a target zero at Range yards
        // (at standard conditions and on level ground.)
        /*  Arguments: 
		        DragFunction:  The drag function to use (G1, G2, G3, G5, G6, G7, G8)
		        DragCoefficient:  The coefficient of drag for the projectile, for the supplied drag function.
		        Vi:  The initial velocity of the projectile, in feet/s
		        SightHeight:  The height of the sighting system above the bore centerline, in inches. 
					          Most scopes fall in the 1.6 to 2.0 inch range.
		        ZeroRange:  The range in yards, at which you wish the projectile to intersect yIntercept.
		        yIntercept:  The height, in inches, you wish for the projectile to be when it crosses ZeroRange yards.
					        This is usually 0 for a target zero, but could be any number.  For example if you wish
					        to sight your rifle in 1.5" high at 100 yards, then you would set yIntercept to 1.5, and ZeroRange to 100
        					
	        Return Value:
		        Returns the angle of the bore relative to the sighting system, in degrees.
        */
        static double BoreAngleNeededToAchieveZero(DragFunction DragFunction, double DragCoefficient, double Velocity_feet_sec, double SightHeight_inch, double ZeroRange_yard, double yIntercept_inch)
        {
            // Numerical Integration variables
            double t = 0;
            double dt = 1 / Velocity_feet_sec; // The solution accuracy generally doesn't suffer if its within a foot for each second of time.
            double y = 0; /*-SightHeight/12;*/
            double x = 0;
            double da; // The change in the bore angle used to iterate in on the correct zero angle.

            // State variables for each integration loop.
            double v = 0, vx = 0, vy = 0; // velocity
            double vx1 = 0, vy1 = 0; // Last frame's velocity, used for computing average velocity.
            double dv = 0, dvx = 0, dvy = 0; // acceleration
            double Gx = 0, Gy = 0; // Gravitational acceleration

            double angle = 0; // The actual angle of the bore.

            // Start with a very coarse angular change, to quickly solve even large launch angle problems.
            da = DegtoRad(14);

            // The general idea here is to start at 0 degrees elevation, and increase the elevation by 14 degrees
            // until we are above the correct elevation.  Then reduce the angular change by half, and begin reducing
            // the angle.  Once we are again below the correct angle, reduce the angular change by half again, and go
            // back up.  This allows for a fast successive approximation of the correct elevation, usually within less
            // than 20 iterations.
            for (angle = 0; ; angle = angle + da)
            {
                vy = Velocity_feet_sec * Math.Sin(angle);
                vx = Velocity_feet_sec * Math.Cos(angle);
                Gx = GRAVITY * Math.Sin(angle);
                Gy = GRAVITY * Math.Cos(angle);

                for (t = 0, x = 0, y = 0 - SightHeight_inch / 12; x <= ZeroRange_yard * 3; t = t + dt)
                {
                    vy1 = vy;
                    vx1 = vx;

                    v = Math.Sqrt(vx * vx + vy * vy);
                    dt = 1 / v;

                    // Compute acceleration using the drag function retardation	
                    dv = DragRetardationVelocity(DragFunction, DragCoefficient, v);
                    dvx = -(vx / v) * dv;
                    dvy = -(vy / v) * dv;

                    // Compute velocity, including the resolved gravity vectors.	
                    vx += dt * dvx + dt * Gx;
                    vy += dt * dvy + dt * Gy;

                    x += dt * (vx + vx1) / 2;
                    y += dt * (vy + vy1) / 2;

                    // Break early to save CPU time if we won't find a solution.
                    if (vy < 0 && y < yIntercept_inch)
                    {
                        break;
                    }
                    if (vy > 3 * vx)
                    {
                        break;
                    }
                }

                if (y > yIntercept_inch && da > 0)
                {
                    da = -da / 2;
                }

                if (y < yIntercept_inch && da < 0)
                {
                    da = -da / 2;
                }

                if (Math.Abs(da) < MOAtoRad(0.01))
                    break; // If our accuracy is sufficient, we can stop approximating.
                if (angle > DegtoRad(45))
                    break; // If we exceed the 45 degree launch angle, then the projectile just won't get there, so we stop trying.
            }

            return RadtoDeg(angle); // Convert to degrees for return value.
        }

        // bore = the hollow part inside a gun barrel or other tube.

        // A function to generate a ballistic solution table in 1 yard increments, up to __BCOMP_MAXRANGE__.
        /* Arguments:
		        DragFunction:  The drag function you wish to use for the solution (G1, G2, G3, G5, G6, G7, or G8)
		        DragCoefficient:  The coefficient of drag for the projectile you wish to model.
		        Vi:  The projectile initial velocity.
		        SightHeight:  The height of the sighting system above the bore centerline.  
						        Most scopes are in the 1.5"-2.0" range.
		        ShootingAngle:  The uphill or downhill shooting angle, in degrees.  Usually 0, but can be anything from
						        90 (directly up), to -90 (directly down).
		        ZeroAngle:  The angle of the sighting system relative to the bore, in degrees.  This can be easily computed
					        using the ZeroAngle() function documented above.
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
        static int SolveAll(DragFunction DragFunction, double DragCoefficient, double Velocity_feet_sec, double SightHeight_inch, double HillAngle, double BoreAngle, double WindSpeed_mile_hr, double WindAngle)
        {
            // seconds
            double t = 0;
            double dt = 0;
            // feet per second
            double v = 0;
            double vx = 0, vx1 = 0, vy = 0, vy1 = 0;
            // acceleration
            double dv = 0, dvx = 0, dvy = 0;
            // feet
            double x = 0, y = 0;

            // miles per hour
            double headwind_mile_hr = HeadWindVelocity(WindSpeed_mile_hr, WindAngle);
            double crosswind_mile_hr = CrossWindVelocity(WindSpeed_mile_hr, WindAngle);

            // feet per second
            double Gy = GRAVITY * Math.Cos(DegtoRad((HillAngle + BoreAngle)));
            double Gx = GRAVITY * Math.Sin(DegtoRad((HillAngle + BoreAngle)));

            vx = Velocity_feet_sec * Math.Cos(DegtoRad(BoreAngle));
            vy = Velocity_feet_sec * Math.Sin(DegtoRad(BoreAngle));

            y = -SightHeight_inch / 12;

            int n = 0;
            for (t = 0; ; t = t + dt)
            {
                // feet per second
                vx1 = vx;
                vy1 = vy;

                v = Math.Sqrt(vx * vx + vy * vy);

                // Compute acceleration using the drag function retardation	
                dv = DragRetardationVelocity(DragFunction, DragCoefficient, v + headwind_mile_hr * 5280.0 / 3600.0);
                dvx = -(vx / v) * dv;
                dvy = -(vy / v) * dv;

                // Compute velocity, including the resolved gravity vectors.	
                vx += dt * dvx + dt * Gx;
                vy += dt * dvy + dt * Gy;

                if (x / 3 >= n)
                {
                    Projectile projectile = new Projectile();
                    projectile.Range = x / 3;							                     // Range in yards
                    projectile.Path = y * 12;							                     // Path in inches
                    projectile.MOA = -RadtoMOA(Math.Atan(y / x));			                 // Correction in MOA
                    projectile.Time = t + dt;							                     // Time in s
                    projectile.Windage = WindageCorrectionRequiredToAchieveZero(crosswind_mile_hr, Velocity_feet_sec, x, t + dt);  // Windage in inches
                    projectile.WindageMOA = RadtoMOA(Math.Atan(projectile.Windage / (12 * x))); // Windage in MOA
                    projectile.Velocity = v;								                 // Velocity (combined)
                    projectile.Vx = vx;								                         // Velocity (x)
                    projectile.Vy = vy;								                         // Velocity (y)
                    ProjectileYards.Add(projectile);
                    n++;
                }

                // Compute position based on average velocity.
                x += dt * (vx + vx1) / 2;
                y += dt * (vy + vy1) / 2;

                dt = 0.5 / v;

                if (Math.Abs(vy) > Math.Abs(3 * vx))
                    break;
                if (n >= __BCOMP_MAXRANGE__ + 1)
                    break;
            }

            size = n;

            return n;
        }

        // Angular conversion functions to make things a little easier:

        // Converts degrees to minutes of angle
        static double DegtoMOA(double deg)
        {
            return deg * 60;
        }

        // Converts degrees to radians
        static double DegtoRad(double deg)
        {
            return deg * M_PI / 180;
        }

        // Converts minutes of angle to degrees
        static double MOAtoDeg(double moa)
        {
            return moa / 60;
        }

        // Converts minutes of angle to radians
        static double MOAtoRad(double moa)
        {
            return moa / 60 * M_PI / 180;
        }

        // Converts radians to degrees
        static double RadtoDeg(double rad)
        {
            return rad * 180 / M_PI;
        }

        // Converts radiants to minutes of angle
        static double RadtoMOA(double rad)
        {
            return rad * 60 * 180 / M_PI;
        }
    }
}
