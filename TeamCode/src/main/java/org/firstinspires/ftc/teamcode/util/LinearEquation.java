package org.firstinspires.ftc.teamcode.util;

public class LinearEquation
{
    double a;
    double b;

    public LinearEquation(double a, double b)
    {
        this.a = a;
        this.b = b;
    }

    public double getY(double x)
    {
        return x * a + b;
    }
}
