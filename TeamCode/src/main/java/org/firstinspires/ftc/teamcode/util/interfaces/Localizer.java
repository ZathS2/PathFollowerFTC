package org.firstinspires.ftc.teamcode.util.interfaces;

import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

public interface Localizer
{
    void update();

    Pose2d getCurrentPos();
}
