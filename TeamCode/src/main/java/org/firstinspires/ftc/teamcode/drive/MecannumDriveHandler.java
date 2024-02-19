package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

import java.util.ArrayList;

public class MecannumDriveHandler
{
    DcMotorEx lBD;
    DcMotorEx lFD;
    DcMotorEx rBD;
    DcMotorEx rFD;

    DcMotorEx[] motors = new DcMotorEx[4];

    public ArrayList<int[]> lastWheelPositions; // Ticks
    public ArrayList<double[]> lastWheelVels; // Ticks/s
    public ArrayList<Double> lastAngularPos;

    IMU imu;
    Localizer localizer;

    ElapsedTime runTimer = new ElapsedTime();
    boolean isRunning = false;


    public MecannumDriveHandler(HardwareMap hardwareMap)
    {
        // Inicializar motores
        lBD = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        lFD = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rBD = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rFD = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        imu = hardwareMap.get(IMU.class, "imu1");

        lBD.setDirection(DcMotorSimple.Direction.FORWARD);
        lFD.setDirection(DcMotorSimple.Direction.FORWARD);
        rBD.setDirection(DcMotorSimple.Direction.REVERSE);
        rFD.setDirection(DcMotorSimple.Direction.REVERSE);

        lBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors[0] = lFD;
        motors[1] = rFD;
        motors[2] = rBD;
        motors[3] = lBD;


        // IMU
        imu.initialize(DriveConstants.imuParameters);
        imu.resetYaw();

        // localizer
        localizer = new Localizer(imu, this);

        lastWheelPositions = new ArrayList<>();
        lastWheelVels = new ArrayList<>();
        lastAngularPos = new ArrayList<>();


        lastAngularPos.add(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        lastWheelPositions.add(new int[] {0,0,0,0});

    }

    public void update()
    {
        if (isRunning && runTimer.time() < 2)
        {
            setPower(new double[] {-0.2,0.2,0.2,-0.2});
        } else {
            setPower(new double[] {0.0,0.0,0.0,0.0});
        }

        localizer.update();
    }
    public void startRun()
    {
        isRunning = true;
        runTimer.reset();
    }

    public void setPower(double[] power)
    {
        for (int i = 0; i < 4; i++)
            motors[i].setPower(power[i]);

    }

    public int[] getWheelPositions()
    {
        // manter um máximo de 2 elementos
        if (lastWheelPositions.size() > 2)
        {
            lastWheelPositions.remove(0);
        }

        int[] wheelPositions = new int[4];
        for (int i = 0; i < wheelPositions.length; i++)
        {
            int position = motors[i].getCurrentPosition();
            wheelPositions[i] = position;
        }

        lastWheelPositions.add(wheelPositions);
        return wheelPositions;
    }

    public double[] getWheelVelocities()
    {
        // manter um máximo de 2 elementos
        if (lastWheelVels.size() > 2)
        {
            lastWheelVels.remove(0);
        }

        double[] wheelVelocities = new double[4];

        for (int i = 0; i < wheelVelocities.length; i++)
        {
            double velocity = motors[i].getVelocity();
            wheelVelocities[i] = velocity;
        }

        lastWheelVels.add(wheelVelocities);

        return wheelVelocities;
    }

    public double getAngularPos()
    {
        // Manter máximo de 2 elementos
        if (lastAngularPos.size() > 2)
            lastAngularPos.remove(0);

        double angularPos = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastAngularPos.add(normalizeAngle(angularPos));

        return angularPos;
    }


    public Pose2d getCurrentPos()
    {
        return localizer.getCurrentPos();
    }

    public double encoderTicksToRadians(double ticks)
    {
        return (Math.PI * ticks) / (TICKS_PER_REV / GEAR_RATIO);
    }

    double normalizeAngle(double angle)
    {
        double fMod = angle - ((int) Math.floor(angle / Math.PI)) * Math.PI;
        if (angle < 0)
            fMod += Math.PI;
        return fMod;
    }


}
