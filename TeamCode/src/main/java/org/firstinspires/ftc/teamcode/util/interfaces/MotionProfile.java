package org.firstinspires.ftc.teamcode.util.interfaces;

public interface MotionProfile
{
    double[] calculate(double distance, double elapsedTime);

    double getRuntime(double distance);
}
