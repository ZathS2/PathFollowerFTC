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

    public Pose2d(Vector2d pos, double angle)
    {
        this.x = pos.x;
        this.y = pos.y;
        this.angle = angle;
    }

    public void rotate(double angle)
    {
        x = x * Math.cos(angle) - y * Math.sin(angle);
        y = y * Math.cos(angle) + x * Math.sin(angle);
    }

    public static double angleDistance(Pose2d start, Pose2d end)
    {
        return end.angle - start.angle;
    }

}
