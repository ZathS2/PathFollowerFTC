package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

public class MecannumDriveHandler
{
    DcMotorEx lBD;
    DcMotorEx lFD;
    DcMotorEx rBD;
    DcMotorEx rFD;

    DcMotorEx[] motors = new DcMotorEx[4];

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

        // localizer
        localizer = new Localizer(imu);

    }

    public void update()
    {
        if (isRunning && runTimer.time() < 2)
        {
            setPower(0.5);
        } else {
            setPower(0.0);
        }

        double[] wheelVelocities = new double[] {lFD.getVelocity(AngleUnit.RADIANS), rFD.getVelocity(AngleUnit.RADIANS),
                                    rBD.getVelocity(AngleUnit.RADIANS), lBD.getVelocity(AngleUnit.RADIANS)};

        localizer.update(wheelVelocities);
    }
    public void startRun()
    {
        isRunning = true;
        runTimer.reset();
    }

    public void setPower(double power)
    {
        for (int i = 0; i < motors.length; i++)
            motors[i].setPower(power);
    }

    public Pose2d getCurrentPos()
    {
        return localizer.getCurrentPos();
    }

}
