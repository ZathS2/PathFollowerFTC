package org.firstinspires.ftc.teamcode.util.controllers;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDController
{
    PIDCoefficients pidCoefficients;

    double P = 0.0;
    double I = 0.0;
    double D = 0.0;
    double lastError = 0.0;
    public PIDController(PIDCoefficients pidCoefficients)
    {
        this.pidCoefficients = pidCoefficients;
    }

    public double calculate(double reference, double setPoint)
    {
        double error = setPoint - reference;


        //P
        P = error * pidCoefficients.p;

        //I
        I += error * pidCoefficients.i;

        //D
        D = (error - lastError) * pidCoefficients.d;

        lastError = error;

        return P + I + D;
    }

    public void resetController()
    {
        I = 0;
        lastError = 0;
    }
}
