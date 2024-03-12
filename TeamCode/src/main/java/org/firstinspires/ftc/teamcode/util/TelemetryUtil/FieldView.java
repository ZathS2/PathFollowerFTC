package org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;

public class FieldView
{
    TelemetryPacket packet;
    FtcDashboard dashboard;

    public FieldView(TelemetryPacket packet, FtcDashboard dashboard)
    {
        this.packet = packet;
        this.dashboard = dashboard;
    }


    public void drawOnField(Pose2d drawPose)
    {

        packet = new TelemetryPacket();

        Vector2d positionInches = new Vector2d(drawPose.x / 2.54, drawPose.y / 2.54);
        double robotDimensionsInches = DriveConstants.ROBOT_WIDTH / 2.54;
        packet.fieldOverlay()
                .strokeRect(positionInches.x, positionInches.y, robotDimensionsInches, robotDimensionsInches);

        dashboard.sendTelemetryPacket(packet);

    }

}