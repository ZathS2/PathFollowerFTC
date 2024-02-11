package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class MecannumWheelKinematics
{
    /*
        __  ----------------------  __
     0 |  | |                    | |  | 1
       |__| |                    | |__|
            |                    |
            |          ^         |
            |          |         |
            |                    |
        __  |                    |  __
       |  | |                    | |  |
     3 |__| |____________________| |__| 2

     */

    public static double[] ForwardKinematics(double[] wheelVelocities, double angularVelocity)
    {
        // 0 - Para frente, 1 - Para o lado, 2 - angular
        double[] robotVelocity = new double[3];

        double vFL = getLinearVelocity(wheelVelocities[0]);
        double vFR = getLinearVelocity(wheelVelocities[1]);
        double vBR = getLinearVelocity(wheelVelocities[2]);
        double vBL = getLinearVelocity(wheelVelocities[3]);

        double vForward = (vFR + vFL + vBR + vBL) / 4;
        double vStrafe = (vBL + vFR - vFL - vBR) / 4;

        robotVelocity[0] = vForward;
        robotVelocity[1] = vStrafe;
        robotVelocity[2] = angularVelocity;

        return robotVelocity;
    }

    public double[] InverseKinematics(double[] robotVelocity)
    {
        double[] wheelVelocities = new double[4];

        double vFL = robotVelocity[0] - robotVelocity[1] - DriveConstants.ROBOT_WIDTH * robotVelocity[2];
        double vBL = robotVelocity[0] + robotVelocity[1] - DriveConstants.ROBOT_WIDTH * robotVelocity[2];
        double vBR = robotVelocity[0] - robotVelocity[1] + DriveConstants.ROBOT_WIDTH * robotVelocity[2];
        double vFR = robotVelocity[0] + robotVelocity[1] + DriveConstants.ROBOT_WIDTH * robotVelocity[2];

        wheelVelocities[0] = getAngularVelocity(vFL);
        wheelVelocities[1] = getAngularVelocity(vFR);
        wheelVelocities[2] = getAngularVelocity(vBR);
        wheelVelocities[3] = getAngularVelocity(vBL);

        return wheelVelocities;
    }


    static double getLinearVelocity(double angularVelocity)
    {
        return angularVelocity * DriveConstants.WHEEL_RADIUS;
    }

    double getAngularVelocity(double linearVelocity)
    {
        return linearVelocity / DriveConstants.WHEEL_RADIUS;
    }



}
