package org.firstinspires.ftc.teamcode.util.pathfollower;

import org.firstinspires.ftc.teamcode.util.controllers.FeedforwardCoefficients;
import org.firstinspires.ftc.teamcode.util.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

public class MecannumDrive
{
    public static Vector2d fieldToRobotFrame(Vector2d fieldFrame, double angle)
    {
        return new Vector2d(fieldFrame.x * Math.cos(angle) + fieldFrame.y * Math.sin(angle),
                fieldFrame.y * Math.cos(angle) - fieldFrame.x * Math.sin(angle));
    }

    public static double[] RobotCharacterization(double[] wheelVelocities, double[] wheelAccels, FeedforwardController ffController)
    {
        double[] wheelsPower = new double[4];
        for (int i = 0; i < wheelsPower.length; i++)
        {
            wheelsPower[i] = ffController.calculate(wheelVelocities[i], wheelAccels[i]);
        }

        return  wheelsPower;
    }
}
