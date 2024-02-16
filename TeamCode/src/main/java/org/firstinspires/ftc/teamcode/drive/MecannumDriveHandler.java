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

    IMU imu;
    Localizer localizer;

    boolean startRunning = false;

    ElapsedTime timer = new ElapsedTime();

    public MecannumDriveHandler(HardwareMap hardwareMap)
    {
        // Inicializar motores
        lBD = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        lFD = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rBD = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rFD = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        //imu = hardwareMap.get(IMU.class, "imu1");

        lBD.setDirection(DcMotorSimple.Direction.REVERSE);
        lFD.setDirection(DcMotorSimple.Direction.REVERSE);
        rBD.setDirection(DcMotorSimple.Direction.FORWARD);
        rFD.setDirection(DcMotorSimple.Direction.FORWARD);

        lBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU
        //imu.initialize(DriveConstants.imuParameters);

        // localizer
        localizer = new Localizer(null);

    }

    public void update()
    {
        double[] wheelVelocities = new double[] {lBD.getVelocity(AngleUnit.RADIANS), lFD.getVelocity(AngleUnit.RADIANS),
                                    rBD.getVelocity(AngleUnit.RADIANS), rFD.getVelocity(AngleUnit.RADIANS)};

        localizer.update(wheelVelocities);

        // DAQUI PRA BAIXO É SÓ PRA TESTE RETIRAR JUNTO DAS VARIAVEIS DE TIMER E START RUNNING

        if (timer.time() < 5 && startRunning)
        {
            lBD.setPower(0.5);
            lFD.setPower(0.5);
            rBD.setPower(0.5);
            rFD.setPower(0.5);
        } else {
            lBD.setPower(0);
            lFD.setPower(0);
            rBD.setPower(0);
            rFD.setPower(0);
        }
    }

    public void startRunning()
    {
        startRunning = true;
        timer.reset();
    }

    public Pose2d getCurrentPos()
    {
        return localizer.getCurrentPos();
    }

}
