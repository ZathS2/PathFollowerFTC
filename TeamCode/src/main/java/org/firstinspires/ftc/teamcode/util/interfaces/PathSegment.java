package org.firstinspires.ftc.teamcode.util.interfaces;

import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

public interface PathSegment
{
    // primeiro array é de valores de t, segundo de valores de d

    double[][] buildLookupTable();
    double findTByDistance(double distance);
    Vector2d findPointOnPath(double t);


    double getLength();
}
