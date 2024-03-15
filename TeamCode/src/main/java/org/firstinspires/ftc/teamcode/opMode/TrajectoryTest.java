package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecannumDriveHandler;
import org.firstinspires.ftc.teamcode.util.PathSegments.Line;
import org.firstinspires.ftc.teamcode.util.Trajectory;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.interfaces.PathSegment;
import org.firstinspires.ftc.teamcode.util.pathfollower.MecannumWheelPathFollower;

@TeleOp
public class TrajectoryTest extends LinearOpMode
{
    PathSegment[] pathSegments = new PathSegment[1];
    Trajectory trajectory;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MecannumDriveHandler drive;
    MecannumWheelPathFollower pathFollower;
    @Override
    public void runOpMode() throws InterruptedException
    {
        pathSegments[0] = new Line(new Vector2d(0,0), new Vector2d(0,100));
        trajectory = new Trajectory(pathSegments, new Pose2d(0,0,0), dashboard);

        drive = new MecannumDriveHandler(hardwareMap, telemetry);
        pathFollower = new MecannumWheelPathFollower(drive);

        pathFollower.followTrajectory(trajectory);


        waitForStart();

        while (opModeIsActive())
        {

            //telemetry.addLine("x: " + trajPose.x);
            //telemetry.addLine("y: " + trajPose.y);
            //telemetry.addLine("angle: " + trajPose.angle);
            pathFollower.update();



            telemetry.update();
        }

    }
}
