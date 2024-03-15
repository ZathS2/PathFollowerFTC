package org.firstinspires.ftc.teamcode.util;

public class NanoTimer
{
    long initTime;

    public NanoTimer()
    {
        resetTimer();
    }

    public long getNanoTime()
    {
        return System.nanoTime();
    }

    public void resetTimer()
    {
        initTime = getNanoTime();
    }

    public double getElapsedTimeSeconds()
    {
        return (getNanoTime() - initTime) / 1e9;
    }

}
