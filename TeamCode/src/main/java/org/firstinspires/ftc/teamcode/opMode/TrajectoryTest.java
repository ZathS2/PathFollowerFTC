package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PathSegments.Line;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil.FieldView;
import org.firstinspires.ftc.teamcode.util.Trajectory;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.interfaces.PathSegment;

@TeleOp
public class TrajectoryTest extends LinearOpMode
{
    PathSegment[] pathSegments = new PathSegment[1];
    Trajectory trajectory;

    TelemetryPacket packet;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    FieldView fieldView = new FieldView(packet, dashboard);
    @Override
    public void runOpMode() throws InterruptedException
    {
        pathSegments[0] = new Line(new Vector2d(0,0), new Vector2d(50,100));
        trajectory = new Trajectory(pathSegments, new Pose2d(0,0,0));

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (opModeIsActive())
        {
            Pose2d trajPose = trajectory.calculatePoseOnTrajectory(elapsedTime.time());
            telemetry.addLine("x: " + trajPose.x);
            telemetry.addLine("y: " + trajPose.y);
            telemetry.addLine("angle: " + trajPose.angle);

            fieldView.drawOnField(trajPose);

            telemetry.update();
        }

    }
}
