package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveConstants
{
    public static double ROBOT_WIDTH = 37.0; //CM
    public static double WHEEL_RADIUS = 9.6; //CM
    public static double GEAR_RATIO = 1.0 / 20.0;
    public static double TICKS_PER_REV = 28.0;

    public static double MAX_VEL = 40; // CM/S
    public static double MAX_ACCEL = 40; // CM/S²
    public static double MAX_ANGULAR_VEL = Math.toRadians(180); // Deg/s
    public static double MAX_ANGULAR_ACCEL = Math.toRadians(180); // Deg/s²

    public static IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );
}
