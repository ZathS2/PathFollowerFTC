package org.firstinspires.ftc.teamcode.util.PathSegments;

import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.interfaces.PathSegment;

public class Line implements PathSegment
{

    Vector2d startPoint;
    Vector2d endPoint;

    double length;

    public Line(Vector2d startPoint,Vector2d endPoint)
    {
        this.startPoint = startPoint;
        this.endPoint = endPoint;

        length = Vector2d.distance(startPoint, endPoint);

    }
    public double[][] buildLookupTable()
    {
        double samples = 100.0;
        double dt = 1.0 / samples;

        double t = 0;

        double[][] lookupTable = new double[(int)samples][(int)samples];

        for (int i = 0; i < (int)samples; i++)
        {
            lookupTable[i][0] = Vector2d.distance(startPoint, findPointOnPath(t));
            t += dt;
        }

        return lookupTable;
    }

    public Vector2d findPointOnPath(double t)
    {
        double a = endPoint.x - startPoint.x;
        double b = endPoint.y - startPoint.y;

        return new Vector2d(a * t + startPoint.x,b * t + startPoint.y);
    }

    public double findTByDistance(double distance) {
        return 0;
    }

    public double getLength()
    {
        return length;
    }
}
