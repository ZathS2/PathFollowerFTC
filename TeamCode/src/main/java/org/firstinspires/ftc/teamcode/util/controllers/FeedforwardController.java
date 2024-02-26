package org.firstinspires.ftc.teamcode.util.controllers;

public class FeedforwardController
{
    FeedforwardCoefficients ffCoefficients;
    public FeedforwardController(FeedforwardCoefficients ffCoefficients)
    {
        this.ffCoefficients = ffCoefficients;
    }

    public double calculate(double vSetPoint, double aSetPoint)
    {
        return ffCoefficients.kv * vSetPoint + ffCoefficients.ka * aSetPoint + ffCoefficients.ks;
    }
}
