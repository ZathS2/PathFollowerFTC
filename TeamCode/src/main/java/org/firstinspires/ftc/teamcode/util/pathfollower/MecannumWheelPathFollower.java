package org.firstinspires.ftc.teamcode.util.pathfollower;

import org.firstinspires.ftc.teamcode.util.controllers.FeedforwardCoefficients;
import org.firstinspires.ftc.teamcode.util.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.util.interfaces.PathFollower;

public class MecannumWheelPathFollower implements PathFollower
{
    FeedforwardCoefficients ffCoefficients = new FeedforwardCoefficients(0,0,0);
    FeedforwardController ffController = new FeedforwardController(ffCoefficients);


    public void update()
    {

    }

    public double robotCharacterization(double velocity, double accel)
    {
        return ffController.calculate(velocity, accel);
    }
}
