package org.firstinspires.ftc.teamcode.util.geometry;

public class Pose2d
{
    public double x;
    public double y;
    public double angle;

    public Pose2d()
    {
        x = 0;
        y = 0;
        angle = 0;
    }

    public Pose2d(double x, double y, double angle)
    {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

}
