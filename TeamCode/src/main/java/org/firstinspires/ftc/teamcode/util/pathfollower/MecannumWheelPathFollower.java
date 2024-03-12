package org.firstinspires.ftc.teamcode.util.pathfollower;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecannumDriveHandler;
import org.firstinspires.ftc.teamcode.util.MecannumWheelKinematics;
import org.firstinspires.ftc.teamcode.util.Trajectory;
import org.firstinspires.ftc.teamcode.util.controllers.FeedforwardCoefficients;
import org.firstinspires.ftc.teamcode.util.controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.util.interfaces.Localizer;
import org.firstinspires.ftc.teamcode.util.interfaces.PathFollower;

public class MecannumWheelPathFollower implements PathFollower
{
    FeedforwardCoefficients ffCoefficients = new FeedforwardCoefficients(0,0,0);
    FeedforwardController ffController = new FeedforwardController(ffCoefficients);

    Localizer localizer;

    Trajectory currentTrajectory;
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    MecannumDriveHandler drive;
    public MecannumWheelPathFollower(Localizer localizer, MecannumDriveHandler drive)
    {
        this.localizer = localizer;
        this.drive = drive;
    }
    public void update()
    {
        if (currentTrajectory == null)
            return;

        Pose2d currentPose = localizer.getCurrentPos();

        // calcular ponto na trajetória
        Pose2d targetPose = currentTrajectory.calculatePoseOnTrajectory(elapsedTime.time());

        // pega valores de motionProfile
        double[] motionProfileValues = currentTrajectory.getCurrentMotionProfileValues();

        // pega direção da velocidade
        double directionAngle = currentTrajectory.getDirectionAngle();

        Vector2d fieldRelativeVelocity = new Vector2d(motionProfileValues[1] * Math.cos(directionAngle),
                motionProfileValues[1] * Math.sin(directionAngle));

        Vector2d robotRelativeVelocity = MecannumDrive.fieldToRobotFrame(fieldRelativeVelocity, currentPose.angle);

        // Lidar com ângulo

        // Traduzir velocidade do robo para rodas
        double[] wheelVelocities = MecannumWheelKinematics.InverseKinematics(
                new double[] {robotRelativeVelocity.x, robotRelativeVelocity.y, 0});

        Vector2d fieldRelativeAccel = new Vector2d(motionProfileValues[2] * Math.cos(directionAngle),
                motionProfileValues[2] * Math.sin(directionAngle));
        Vector2d robotRelativeAccel = MecannumDrive.fieldToRobotFrame(fieldRelativeAccel, currentPose.angle);

        // Traduzir aceleração do robo para rodas
        double[] wheelAccels = MecannumWheelKinematics.InverseKinematics(
                new double[] {robotRelativeAccel.x, robotRelativeAccel.y, 0});

        // Calcular potência em cada roda
        double[] wheelPowers = MecannumDrive.RobotCharacterization(wheelVelocities, wheelAccels, ffController);

        // enviar potência para rodas
        drive.setPower(wheelPowers);


    }

    public void followTrajectory(Trajectory trajectory)
    {
        currentTrajectory = trajectory;
        elapsedTime.reset();
    }

}
