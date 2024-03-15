package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil.FieldView;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.interfaces.MotionProfile;
import org.firstinspires.ftc.teamcode.util.interfaces.PathSegment;
import org.firstinspires.ftc.teamcode.util.pathfollower.TrapezoidalMotionProfile;


public class Trajectory
{
    PathSegment[] pathSegments;
    MotionProfile[] motionProfiles;

    Pose2d initialPose;

    // salva o tempo final de cada segmento
    double[] trajectoryRuntime;

    double[] motionProfileValues;

    double directionAngle;

    FtcDashboard dashboard;

    FieldView fieldView;
    public Trajectory(PathSegment[] pathSegments, Pose2d initialPose, FtcDashboard dashboard)
    {
        this.initialPose = initialPose;
        this.pathSegments = pathSegments;

        motionProfiles = new MotionProfile[pathSegments.length];

        trajectoryRuntime = new double[pathSegments.length];

        for (int i = 0; i < pathSegments.length; i++) {
            motionProfiles[i] = new TrapezoidalMotionProfile(DriveConstants.MAX_ACCEL, DriveConstants.MAX_VEL);

            if (i == 0) {
                trajectoryRuntime[i] = motionProfiles[i].getRuntime(pathSegments[i].getLength());
                continue;
            }

            trajectoryRuntime[i] = trajectoryRuntime[i - 1] + motionProfiles[i].getRuntime(pathSegments[i].getLength());
        }

        this.dashboard = dashboard;
        fieldView = new FieldView(new TelemetryPacket(), dashboard);
    }

    public Pose2d calculatePoseOnTrajectory(double elapsedTime)
    {
        int pathIndex = 0;
        // Descobrir em qual segmento está
        for (int i = 0; i < trajectoryRuntime.length; i++)
        {
            if (elapsedTime <= trajectoryRuntime[i])
            {
                pathIndex = i;
                break;
            }
        }

        double pathInitialTime = pathIndex == 0 ? 0 : trajectoryRuntime[pathIndex - 1];

        // Calcular valores do motionProfile
        motionProfileValues = motionProfiles[pathIndex].calculate(pathSegments[pathIndex].getLength(), elapsedTime - pathInitialTime);

        // Distancia para tempo
        double t = pathSegments[pathIndex].findTByDistance(motionProfileValues[0]);

        // Posição no caminho
        Vector2d currentPosition = pathSegments[pathIndex].findPointOnPath(t);

        // Direção do movimento

        Vector2d directionVector = pathSegments[pathIndex].calculate1stDerivative(t);
        directionAngle = Math.atan2(directionVector.y, directionVector.x);

        // Lidar com ângulo
        double angle = 0;

        Pose2d currentPose = new Pose2d(currentPosition, angle);

        drawTrajectory(currentPose);

        return currentPose;
    }

    public void drawTrajectory(Pose2d pos)
    {
        fieldView.drawRobotOnField(pos);
    }

    public double getTrajectoryRuntime()
    {
        return trajectoryRuntime[trajectoryRuntime.length - 1];
    }
    public double[] getCurrentMotionProfileValues() { return motionProfileValues; }
    public double getDirectionAngle() {return  directionAngle; }
}
