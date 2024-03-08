package org.firstinspires.ftc.teamcode.util.geometry;

public class Vector2d
{
    public double x;
    public double y;

    public Vector2d(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public static double distance(Vector2d initialPoint, Vector2d endPoint)
    {
        return Math.sqrt(Math.pow(endPoint.x - initialPoint.x,2) + Math.pow(endPoint.y - initialPoint.y,2));
    }
}
