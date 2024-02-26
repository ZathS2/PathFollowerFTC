package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.pathfollower.TrapezoidalMotionProfile;

@TeleOp
public class MotionProfileTest extends LinearOpMode
{
    TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(20,20);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTel = dashboard.getTelemetry();

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException
    {
        dashTel.addData("pos",  0.0);
        dashTel.addData("vel", 0.0);
        dashTel.addData("accel",0.0);
        dashTel.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive())
        {
            double[] motionProfileValues = motionProfile.calculate(100, timer.time());

            dashTel.addData("pos",motionProfileValues[0]);
            dashTel.addData("vel", motionProfileValues[1]);
            dashTel.addData("accel", motionProfileValues[2]);
            dashTel.update();
        }
    }
}
