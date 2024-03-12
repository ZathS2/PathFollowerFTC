package org.firstinspires.ftc.teamcode.util.interfaces;

import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

import java.util.Vector;

public interface PathSegment
{
    // x = valores de t
    // y = valores de distancia

    Vector2d[] buildLookupTable();
    double findTByDistance(double distance);
    Vector2d findPointOnPath(double t);
    double getLength();
    Vector2d calculate1stDerivative(double t);
}
