#include "ballistics.h"

int main()
{
   EDragFunction DragFunction = G1;
   double initialVelocity = 2650; // Initial velocity, in ft/s
   double ballisticCoefficient = 0.465; // The ballistic coefficient for the projectile.
   double sightHeightOverBore = 1.6; // The Sight height over bore, in inches.

   Projectile projectile(ballisticCoefficient,DragFunction,initialVelocity,sightHeightOverBore);

   double Altitude = 0; // The altitude above sea level in feet.  Standard altitude is 0 feet above sea level.
   double Barometer = 29.59; // The barometric pressure in inches of mercury (in Hg).
   double Temperature = 59; // The temperature in Fahrenheit.  Standard temperature is 59 degrees.
   double RelativeHumidity = 0.78; // The relative humidity fraction.  Ranges from 0.00 to 1.00, with 0.50 being 50% relative humidity.
   // If we wish to use the weather correction features, we need to 
   // Correct the BC for any weather conditions.  If we want standard conditions,
   // then we can just leave this commented out.
   ballisticCoefficient = projectile.DragCoefficientAtmosphericCorrection(Altitude, Barometer, Temperature, RelativeHumidity);

   double zeroRange = 200; // The zero range of the rifle, in yards.
   double yIntercept = 0; // The height, in inches, you wish for the projectile to be when it crosses ZeroRange yards.
   // First find the angle of the bore relative to the sighting system.
   // We call this the "zero angle", since it is the angle required to 
   // achieve a zero at a particular yardage.  This value isn't very useful
   // to us, but is required for making a full ballistic solution.
   // It is left here to allow for zero-ing at altitudes (bc) different from the
   // final solution, or to allow for zero's other than 0" (ex: 3" high at 100 yards)
   double sightToBoreAngle = projectile.CalculateSightToBoreAngle(zeroRange,yIntercept);

   double boreAngle = 0; // The shooting angle (uphill / downhill), in degrees.
   projectile.Initialize(boreAngle,sightToBoreAngle);

   // Now we have everything needed to generate a full solution.
   // So we do.  The solution is stored in the pointer "sln" passed as the last argument.
   // k has the number of yards the solution is valid for, also the number of rows in the solution.
   
   double windspeed = 0; // The wind speed in miles per hour.
   double windangle = 0; // The wind angle (0=headwind, 90=right to left, 180=tailwind, 270/-90=left to right)
   int rowCount = SolveAll(windspeed,windangle,projectile);

   // Now print a simple chart of X / Y trajectory spaced at 10yd increments
   for (int s = 0; s <= 100; s++)
   {
      printf("\nX: %.0f     Y: %.2f		t: %.2f", projectile.aProjectilePath[s*10].Range, projectile.aProjectilePath[s*10].Path, projectile.aProjectilePath[s*10].Time);
   }

   getch();

   return 0;
}

double Projectile::CalculateFactorRH(double Temperature, double Pressure, double RelativeHumidity)
{
   double VPw = 4e-6 * pow(Temperature,3) - 0.0004 * pow(Temperature,2) + 0.0234 * Temperature - 0.2517;
   double FRH = 0.995 * (Pressure / (Pressure-(0.3783) * (RelativeHumidity) * VPw));
   return FRH;
}

double Projectile::CalculateFactorP(double Pressure)
{
   double Pstd = 29.53; // in-hg
   double FP = (Pressure-Pstd) / (Pstd);
   return FP;
}

double Projectile::CalculateFactorT(double Temperature, double Altitude)
{
   double Tstd = -0.0036 * Altitude + 59;
   double FT = (Temperature-Tstd) / (459.6 + Tstd);
   return FT;
}

double Projectile::CalculateFactorA(double Altitude)
{
   double fa = -4e-15 * pow(Altitude,3) + 4e-10 * pow(Altitude,2) - 3e-5 * Altitude + 1;
   return (1/fa);
}

double Projectile::DragCoefficientAtmosphericCorrection(double Altitude_feet, double Barometer_hg, double Temperature_f, double RelativeHumidity)
{
   double FA = CalculateFactorA(Altitude_feet);
   double FT = CalculateFactorT(Temperature_f, Altitude_feet);
   double FR = CalculateFactorRH(Temperature_f, Barometer_hg, RelativeHumidity);
   double FP = CalculateFactorP(Barometer_hg);

   // Calculate the atmospheric correction factor
   double CD = (FA*(1+FT-FP)*FR);
   DragCoefficient *= CD;
   return DragCoefficient;
}

double Projectile::CalculateSightToBoreAngle(double ZeroRange_yard, double yIntercept_inch)
{
   // Numerical Integration variables
   double t = 0;
   double dt = 1/MuzzleVelocity; // The solution accuracy generally doesn't suffer if its within a foot for each second of time.
   double y = 0; /*-SightHeight/12;*/
   double x = 0;
   double da; // The change in the bore angle used to iterate in on the correct zero angle.

   // State variables for each integration loop.
   double v = 0;
   double vx = 0;
   double vy = 0; // velocity
   double vx_last = 0;
   double vy_last = 0; // Last frame's velocity, used for computing average velocity.
   double dv = 0;
   double dvx = 0;
   double dvy = 0; // acceleration
   double Gx = 0;
   double Gy = 0; // Gravitational acceleration

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
      vy = MuzzleVelocity * sin(angle);
      vx = MuzzleVelocity * cos(angle);
      Gx = GRAVITY * sin(angle);
      Gy = GRAVITY * cos(angle);

      for (t = 0, x = 0, y = 0-SightHeightOverBore/12; x <= ZeroRange_yard*3; t = t + dt)
      {
         vy_last = vy;
         vx_last = vx;

         v = sqrt(vx*vx+vy*vy);
         dt = 1/v;

         // Compute acceleration using the drag function retardation	
         dv = DragRetardationVelocity(v);
         dvx = -(vx/v) * dv;
         dvy = -(vy/v) * dv;

         // Compute velocity, including the resolved gravity vectors.	
         vx += dt * dvx + dt * Gx;
         vy += dt * dvy + dt * Gy;

         x += dt * (vx+vx_last)/2;
         y += dt * (vy+vy_last)/2;

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
         da = -da/2;
      }

      if (y < yIntercept_inch && da < 0)
      {
         da = -da/2;
      }

      if (fabs(da) < MOAtoRad(0.01))
         break; // If our accuracy is sufficient, we can stop approximating.
      if (angle > DegtoRad(45))
         break; // If we exceed the 45 degree launch angle, then the projectile just won't get there, so we stop trying.
   }

   return RadtoDeg(angle); // Convert to degrees for return value.
}

void Projectile::Initialize(double BoreAngle, double SightToBoreAngle)
{
   this->Gy = GRAVITY * cos(DegtoRad((BoreAngle + SightToBoreAngle)));
   this->Gx = GRAVITY * sin(DegtoRad((BoreAngle + SightToBoreAngle)));

   this->vx = MuzzleVelocity * cos(DegtoRad(SightToBoreAngle));
   this->vy = MuzzleVelocity * sin(DegtoRad(SightToBoreAngle));

   this->x = 0;
   this->y = -SightHeightOverBore/12;

   this->ProjectilePathCount = 0;
   this->t = 0;
}

int Projectile::Update(double &dt, double headwind_mile_hr, double crosswind_mile_hr)
{
   // feet per second
   double vx_last = vx;
   double vy_last = vy;

   double v = sqrt(vx*vx+vy*vy);

   // Compute acceleration using the drag function retardation	
   double dv = DragRetardationVelocity(v + headwind_mile_hr*5280.0/3600.0);
   double dvx = -(vx/v) * dv;
   double dvy = -(vy/v) * dv;

   // Compute velocity, including the resolved gravity vectors.	
   vx += dt * dvx + dt * Gx;
   vy += dt * dvy + dt * Gy;

   if (x/3>=ProjectilePathCount)
   {
      aProjectilePath[ProjectilePathCount].Range = x/3;                 // Range in yards
      aProjectilePath[ProjectilePathCount].Path = y*12;                 // Path in inches
      aProjectilePath[ProjectilePathCount].MOA = -RadtoMOA(atan(y/x));  // Correction in MOA
      aProjectilePath[ProjectilePathCount].Time = t+dt;                 // Time in s
      aProjectilePath[ProjectilePathCount].Windage = WindageCorrection(crosswind_mile_hr,MuzzleVelocity,x,t+dt);  // Windage in inches
      aProjectilePath[ProjectilePathCount].WindageMOA = RadtoMOA(atan(aProjectilePath[ProjectilePathCount].Windage/(12*x)));  // Windage in MOA
      aProjectilePath[ProjectilePathCount].Velocity = v;                // Velocity (combined)
      aProjectilePath[ProjectilePathCount].Vx = vx;                     // Velocity (x)
      aProjectilePath[ProjectilePathCount].Vy = vy;                     // Velocity (y)
      ProjectilePathCount++;
   }

   // Compute position based on average velocity.
   x += dt * (vx+vx_last)/2;
   y += dt * (vy+vy_last)/2;

   dt = 0.5 / v;

   if (fabs(vy) > fabs(3*vx))
      return 0;

   if (ProjectilePathCount >= BCOMP_MAXRANGE)
      return 0;

   return 1;
}

int SolveAll(double WindSpeed_mile_hr, double WindAngle, Projectile& projectile)
{
      double headwind_mile_hr = HeadWindVelocity(WindSpeed_mile_hr, WindAngle);
      double crosswind_mile_hr = CrossWindVelocity(WindSpeed_mile_hr, WindAngle);

      for (double dt = 0; ; projectile.t += dt)
      {
         if( !projectile.Update( dt, headwind_mile_hr, crosswind_mile_hr ) )
            break;
      }

      return projectile.ProjectilePathCount;
}

double Projectile::WindageCorrection(double WindSpeed_mile_hr, double Velocity_feet_sec, double range_feet, double time)
{
   double Vw = WindSpeed_mile_hr * 17.60; // Convert to inches per second.
   return (Vw * (time - range_feet/Velocity_feet_sec));
}

// Headwind is positive at WindAngle=0
double HeadWindVelocity(double WindSpeed_mile_hr, double WindAngle)
{
   double Wangle = DegtoRad(WindAngle);
   return (cos(Wangle) * WindSpeed_mile_hr);
}

// Positive is from Shooter's Right to Left (Wind from 90 degree)
double CrossWindVelocity(double WindSpeed_mile_hr, double WindAngle)
{
   double Wangle = DegtoRad(WindAngle);
   return (sin(Wangle) * WindSpeed_mile_hr);
}

double Projectile::DragRetardationVelocity(double Velocity)
{
   double vp = Velocity;	
   double val = -1;
   double A = -1;
   double M = -1;

   switch(DragFunction)
   {
   case G1:
      if (vp > 4230) { A = 1.477404177730177e-04; M = 1.9565; }
      else if (vp> 3680) { A = 1.920339268755614e-04; M = 1.925 ; }
      else if (vp> 3450) { A = 2.894751026819746e-04; M = 1.875 ; }
      else if (vp> 3295) { A = 4.349905111115636e-04; M = 1.825 ; }
      else if (vp> 3130) { A = 6.520421871892662e-04; M = 1.775 ; }
      else if (vp> 2960) { A = 9.748073694078696e-04; M = 1.725 ; }
      else if (vp> 2830) { A = 1.453721560187286e-03; M = 1.675 ; }
      else if (vp> 2680) { A = 2.162887202930376e-03; M = 1.625 ; }
      else if (vp> 2460) { A = 3.209559783129881e-03; M = 1.575 ; }
      else if (vp> 2225) { A = 3.904368218691249e-03; M = 1.55  ; }
      else if (vp> 2015) { A = 3.222942271262336e-03; M = 1.575 ; }
      else if (vp> 1890) { A = 2.203329542297809e-03; M = 1.625 ; }
      else if (vp> 1810) { A = 1.511001028891904e-03; M = 1.675 ; }
      else if (vp> 1730) { A = 8.609957592468259e-04; M = 1.75  ; }
      else if (vp> 1595) { A = 4.086146797305117e-04; M = 1.85  ; }
      else if (vp> 1520) { A = 1.954473210037398e-04; M = 1.95  ; }
      else if (vp> 1420) { A = 5.431896266462351e-05; M = 2.125 ; }
      else if (vp> 1360) { A = 8.847742581674416e-06; M = 2.375 ; }
      else if (vp> 1315) { A = 1.456922328720298e-06; M = 2.625 ; }
      else if (vp> 1280) { A = 2.419485191895565e-07; M = 2.875 ; }
      else if (vp> 1220) { A = 1.657956321067612e-08; M = 3.25  ; }
      else if (vp> 1185) { A = 4.745469537157371e-10; M = 3.75  ; }
      else if (vp> 1150) { A = 1.379746590025088e-11; M = 4.25  ; }
      else if (vp> 1100) { A = 4.070157961147882e-13; M = 4.75  ; }
      else if (vp> 1060) { A = 2.938236954847331e-14; M = 5.125 ; }
      else if (vp> 1025) { A = 1.228597370774746e-14; M = 5.25  ; }
      else if (vp>  980) { A = 2.916938264100495e-14; M = 5.125 ; }
      else if (vp>  945) { A = 3.855099424807451e-13; M = 4.75  ; }
      else if (vp>  905) { A = 1.185097045689854e-11; M = 4.25  ; }
      else if (vp>  860) { A = 3.566129470974951e-10; M = 3.75  ; }
      else if (vp>  810) { A = 1.045513263966272e-08; M = 3.25  ; }
      else if (vp>  780) { A = 1.291159200846216e-07; M = 2.875 ; }
      else if (vp>  750) { A = 6.824429329105383e-07; M = 2.625 ; }
      else if (vp>  700) { A = 3.569169672385163e-06; M = 2.375 ; }
      else if (vp>  640) { A = 1.839015095899579e-05; M = 2.125 ; }
      else if (vp>  600) { A = 5.71117468873424e-05 ; M = 1.950 ; }
      else if (vp>  550) { A = 9.226557091973427e-05; M = 1.875 ; }
      else if (vp>  250) { A = 9.337991957131389e-05; M = 1.875 ; }
      else if (vp>  100) { A = 7.225247327590413e-05; M = 1.925 ; }
      else if (vp>   65) { A = 5.792684957074546e-05; M = 1.975 ; }
      else if (vp>    0) { A = 5.206214107320588e-05; M = 2.000 ; }
      break;

   case G2:
      if (vp> 1674 ) { A = .0079470052136733   ;  M = 1.36999902851493; }
      else if (vp> 1172 ) { A = 1.00419763721974e-03;  M = 1.65392237010294; }
      else if (vp> 1060 ) { A = 7.15571228255369e-23;  M = 7.91913562392361; }
      else if (vp>  949 ) { A = 1.39589807205091e-10;  M = 3.81439537623717; }
      else if (vp>  670 ) { A = 2.34364342818625e-04;  M = 1.71869536324748; }
      else if (vp>  335 ) { A = 1.77962438921838e-04;  M = 1.76877550388679; }
      else if (vp>    0 ) { A = 5.18033561289704e-05;  M = 1.98160270524632; }
      break;

   case G5:
      if (vp> 1730 ){ A = 7.24854775171929e-03; M = 1.41538574492812; }
      else if (vp> 1228 ){ A = 3.50563361516117e-05; M = 2.13077307854948; }
      else if (vp> 1116 ){ A = 1.84029481181151e-13; M = 4.81927320350395; }
      else if (vp> 1004 ){ A = 1.34713064017409e-22; M = 7.8100555281422 ; }
      else if (vp>  837 ){ A = 1.03965974081168e-07; M = 2.84204791809926; }
      else if (vp>  335 ){ A = 1.09301593869823e-04; M = 1.81096361579504; }
      else if (vp>    0 ){ A = 3.51963178524273e-05; M = 2.00477856801111; }	
      break;

   case G6:
      if (vp> 3236 ) { A = 0.0455384883480781   ; M = 1.15997674041274; }
      else if (vp> 2065 ) { A = 7.167261849653769e-02; M = 1.10704436538885; }
      else if (vp> 1311 ) { A = 1.66676386084348e-03 ; M = 1.60085100195952; }
      else if (vp> 1144 ) { A = 1.01482730119215e-07 ; M = 2.9569674731838 ; }
      else if (vp> 1004 ) { A = 4.31542773103552e-18 ; M = 6.34106317069757; }
      else if (vp>  670 ) { A = 2.04835650496866e-05 ; M = 2.11688446325998; }
      else if (vp>    0 ) { A = 7.50912466084823e-05 ; M = 1.92031057847052; }
      break;

   case G7:
      if (vp> 4200 ) { A = 1.29081656775919e-09; M = 3.24121295355962; }
      else if (vp> 3000 ) { A = 0.0171422231434847  ; M = 1.27907168025204; }
      else if (vp> 1470 ) { A = 2.33355948302505e-03; M = 1.52693913274526; }
      else if (vp> 1260 ) { A = 7.97592111627665e-04; M = 1.67688974440324; }
      else if (vp> 1110 ) { A = 5.71086414289273e-12; M = 4.3212826264889 ; }
      else if (vp>  960 ) { A = 3.02865108244904e-17; M = 5.99074203776707; }
      else if (vp>  670 ) { A = 7.52285155782535e-06; M = 2.1738019851075 ; }
      else if (vp>  540 ) { A = 1.31766281225189e-05; M = 2.08774690257991; }
      else if (vp>    0 ) { A = 1.34504843776525e-05; M = 2.08702306738884; }
      break;

   case G8:
      if (vp> 3571 ) { A = .0112263766252305   ; M = 1.33207346655961; }
      else if (vp> 1841 ) { A = .0167252613732636   ; M = 1.28662041261785; }
      else if (vp> 1120 ) { A = 2.20172456619625e-03; M = 1.55636358091189; }
      else if (vp> 1088 ) { A = 2.0538037167098e-16 ; M = 5.80410776994789; }
      else if (vp>  976 ) { A = 5.92182174254121e-12; M = 4.29275576134191; }
      else if (vp>    0 ) { A = 4.3917343795117e-05 ; M = 1.99978116283334; }
      break;

   default:
      break;

   }

   if (A != -1 && M != -1 && vp > 0 && vp < 10000)
   {
      val = A * pow(vp,M) / DragCoefficient;
      return val;
   }
   else
   {
      return -1;
   }
}
