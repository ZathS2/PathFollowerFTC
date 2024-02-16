package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecannumDriveHandler;

@Autonomous
public class StraightTest extends LinearOpMode
{

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTel = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException
    {
        MecannumDriveHandler drive = new MecannumDriveHandler(hardwareMap);

        waitForStart();

        drive.startRun();

        while (!isStopRequested())
        {
            drive.update();

            dashTel.addLine("x: " + drive.getCurrentPos().x);
            dashTel.addLine("y: " + drive.getCurrentPos().y);
            dashTel.addLine("angle: " + drive.getCurrentPos().angle);
            dashTel.update();
        }
    }
}
