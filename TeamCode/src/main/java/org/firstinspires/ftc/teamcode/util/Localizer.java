package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecannumDriveHandler;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

public class Localizer
{
    MecannumWheelKinematics mecannumWheelKinematics = new MecannumWheelKinematics();

    Pose2d currentPos = new Pose2d();

    ElapsedTime timeSinceLastFrame = new ElapsedTime();

    IMU imu;

    MecannumDriveHandler drive;

    public Localizer(IMU imu, MecannumDriveHandler drive)
    {
        this.imu = imu;
        this.drive = drive;
        timeSinceLastFrame.reset();
    }

    public void update()
    {
        drive.getWheelPositions();
        drive.getWheelVelocities();
        drive.getAngularPos();

        double[] wheelDeltas = new double[] {0,0,0,0};

        for (int i = 0 ; i < wheelDeltas.length; i++)
        {
            double deltaTick = drive.lastWheelPositions.get(1)[i] - drive.lastWheelPositions.get(0)[i];

            wheelDeltas[i] = drive.encoderTicksToRadians(deltaTick);
        }

        double deltaAngle = drive.lastAngularPos.get(1) - drive.lastAngularPos.get(0);

        double[] robotVelocity = MecannumWheelKinematics.ForwardKinematics(wheelDeltas, deltaAngle);

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
    }

    public Pose2d getCurrentPos()
    {
        if (currentPos == null)
            return new Pose2d();

        return currentPos;
    }

}
