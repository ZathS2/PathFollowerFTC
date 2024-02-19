package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

public class Localizer
{
    MecannumWheelKinematics mecannumWheelKinematics = new MecannumWheelKinematics();

    Pose2d currentPos = new Pose2d();

    ElapsedTime timeSinceLastFrame = new ElapsedTime();

    IMU imu;

    public Localizer(IMU imu)
    {
        this.imu = imu;
        timeSinceLastFrame.reset();
    }

    public void update(double[] wheelVelocities)
    {
        double[] robotVelocity = MecannumWheelKinematics.ForwardKinematics(wheelVelocities, 0);///imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);

        double robotDeltaX = robotVelocity[0];
        double robotDeltaY = robotVelocity[1];
        double robotDeltaTheta = robotVelocity[2];

        double unRotated_dX;
        double unRotated_dY;

        if (robotDeltaTheta < 0.001)
        {
            // Série de taylor
            unRotated_dX = (-Math.pow(robotDeltaTheta,2) * robotDeltaX + 6 * robotDeltaX - 3 * robotDeltaTheta * robotDeltaY) / 6;
            unRotated_dY = (3 * robotDeltaTheta * robotDeltaX - Math.pow(robotDeltaTheta,2) * robotDeltaY + 6 * robotDeltaY) / 6;
        } else {
            unRotated_dX = (-robotDeltaY + robotDeltaY * Math.cos(robotDeltaTheta) + robotDeltaX * Math.sin(robotDeltaTheta)) / robotDeltaTheta;
            unRotated_dY = (-robotDeltaX + robotDeltaX * Math.cos(robotDeltaTheta) + robotDeltaY * Math.sin(robotDeltaTheta)) / robotDeltaTheta;
        }

        double fieldDeltaX = unRotated_dX * Math.cos(currentPos.angle) - unRotated_dY * Math.sin(currentPos.angle);
        double fieldDeltaY = unRotated_dY * Math.cos(currentPos.angle) + unRotated_dX * Math.sin(currentPos.angle);

        currentPos.x += fieldDeltaX;
        currentPos.y += fieldDeltaY;
        currentPos.angle += robotDeltaTheta;

        timeSinceLastFrame.reset();
    }

    public Pose2d getCurrentPos()
    {
        if (currentPos == null)
            return new Pose2d();

        return currentPos;
    }

}
