package org.firstinspires.ftc.teamcode.util.PathSegments;

import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.interfaces.PathSegment;

import java.util.Vector;

public class Line implements PathSegment
{

    Vector2d startPoint;
    Vector2d endPoint;

    double length;

    Vector2d[] lookupTable;

    public Line(Vector2d startPoint,Vector2d endPoint)
    {
        this.startPoint = startPoint;
        this.endPoint = endPoint;

        length = Vector2d.distance(startPoint, endPoint);
        lookupTable = buildLookupTable();
    }
    public Vector2d[] buildLookupTable()
    {
        double samples = 100.0;
        double dt = 1.0 / samples;

        double t = 0;

        Vector2d[] lookupTable = new Vector2d[(int)samples];

        for (int i = 0; i < (int)samples; i++)
        {
            lookupTable[i] = new Vector2d(t, Vector2d.distance(startPoint, findPointOnPath(t)));

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

    public double findTByDistance(double distance)
    {
        if (distance >= 0 && distance <= length)
        {
            for (int i = 0; i < lookupTable.length - 1; i++)
            {
                if (distance >= lookupTable[i].y && distance <= lookupTable[i + 1].y)
                {
                    // código para estimar valor de t
                    // calcular slope

                    double m = (lookupTable[i + 1].y - lookupTable[i].y) / (lookupTable[i + 1].x - lookupTable[i].x);

                    // calcular t, usando a formula:
                    // ((d - d1) / m) + t1 = t

                    double estimateT = ((distance - lookupTable[i].y) / m) + lookupTable[i].x;

                    return estimateT;
                }
            }
        }

        return distance / length;
    }

    public double getLength()
    {
        return length;
    }
}
