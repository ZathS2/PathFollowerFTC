package org.firstinspires.ftc.teamcode.util.pathfollower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.interfaces.MotionProfile;

public class TrapezoidalMotionProfile implements MotionProfile
{
    double MAX_ACCEL;
    double MAX_VEL;

    Telemetry telemetry;

    public TrapezoidalMotionProfile(double MAX_ACCEL, double MAX_VEL)
    {
        this.MAX_ACCEL = MAX_ACCEL;
        this.MAX_VEL = MAX_VEL;
    }

    public TrapezoidalMotionProfile(double MAX_ACCEL, double MAX_VEL, Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    public double[] calculate(double distance, double elapsedTime)
    {
        double[] profileValues = new double[3];

        double acceldt = MAX_VEL / MAX_ACCEL;

        double halfDistance = distance / 2;
        double accelDistance = 0.5 * MAX_ACCEL * Math.pow(acceldt,2);

        if (accelDistance > halfDistance)
        {
            acceldt = Math.sqrt(halfDistance / (0.5 * MAX_ACCEL));
        }

        accelDistance = 0.5 * MAX_ACCEL * Math.pow(acceldt,2);

        double newMAX_VEL = MAX_ACCEL * acceldt;

        double dAcceldt = acceldt;

        double cruiseDistance = distance - 2 * accelDistance;
        double cruisedt = cruiseDistance / newMAX_VEL;
        double dAccelTime = acceldt + cruisedt;

        double totaldt  = acceldt + cruisedt + dAcceldt;
        if (elapsedTime > totaldt)
        {
            profileValues[0] = distance;
            profileValues[1] = 0.0;
            profileValues[2] = 0.0;
            return profileValues;
        }

        if (elapsedTime < acceldt)
        {
            profileValues[0] = 0.5 * MAX_ACCEL * Math.pow(elapsedTime,2);
            profileValues[1] = MAX_ACCEL * elapsedTime;
            profileValues[2] = MAX_ACCEL;


            return profileValues;
        } else if (elapsedTime < dAccelTime) {
            accelDistance = 0.5 * MAX_ACCEL * Math.pow(acceldt,2);
            double cruiseCurrentdt = elapsedTime - acceldt;

            profileValues[0] = accelDistance + newMAX_VEL * cruiseCurrentdt;
            profileValues[1] = newMAX_VEL;
            profileValues[2] = 0.0;


            return profileValues;
        } else {
            accelDistance = 0.5 * MAX_ACCEL * Math.pow(acceldt,2);
            cruiseDistance = newMAX_VEL * cruisedt;
            dAccelTime = elapsedTime - dAccelTime;

            profileValues[0] = accelDistance + cruiseDistance + newMAX_VEL * dAccelTime - 0.5 * MAX_ACCEL * Math.pow(dAccelTime,2);
            profileValues[1] = newMAX_VEL - MAX_ACCEL * dAccelTime;
            profileValues[2] = -MAX_ACCEL;


            return profileValues;
        }
    }

    public double getRuntime(double distance)
    {
        double acceldt = MAX_VEL / MAX_ACCEL;

        double halfDistance = distance / 2;
        double accelDistance = 0.5 * MAX_ACCEL * Math.pow(acceldt,2);

        if (accelDistance < halfDistance)
        {
            acceldt = Math.sqrt(halfDistance / (0.5 * MAX_ACCEL));
        }

        double cruiseDistance = distance - 2 * accelDistance;
        double cruisedt = cruiseDistance / MAX_VEL;

        return 2 * acceldt + cruisedt;
    }
}
