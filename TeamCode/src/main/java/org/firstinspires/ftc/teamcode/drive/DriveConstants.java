package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveConstants
{
    public static double ROBOT_WIDTH = 37.0; //CM
    public static double WHEEL_RADIUS = 9.6; //CM
    public static double GEAR_RATIO = 1.0 / 20.0;
    public static double TICKS_PER_REV = 28.0;

    public static IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );
}
