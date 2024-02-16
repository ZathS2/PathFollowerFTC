package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecannumDriveHandler;

@Autonomous
public class StraightTest extends LinearOpMode
{
    MecannumDriveHandler drive = new MecannumDriveHandler(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        drive.startRun();

        while (!isStopRequested())
        {
            drive.update();

            telemetry.addLine("x: " + drive.getCurrentPos().x);
            telemetry.addLine("y: " + drive.getCurrentPos().y);
            telemetry.addLine("angle: " + drive.getCurrentPos().angle);
            telemetry.update();
        }
    }
}
