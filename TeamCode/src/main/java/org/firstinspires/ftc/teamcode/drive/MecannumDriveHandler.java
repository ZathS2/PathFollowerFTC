package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Localizer;

public class MecannumDriveHandler
{
    DcMotorEx lBD;
    DcMotorEx lFD;
    DcMotorEx rBD;
    DcMotorEx rFD;

    IMU imu;
    Localizer localizer;

    public MecannumDriveHandler(HardwareMap hardwareMap)
    {
        // Inicializar motores
        lBD = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        lFD = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rBD = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rFD = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        lBD.setDirection(DcMotorSimple.Direction.REVERSE);
        lFD.setDirection(DcMotorSimple.Direction.REVERSE);
        rBD.setDirection(DcMotorSimple.Direction.FORWARD);
        rFD.setDirection(DcMotorSimple.Direction.FORWARD);

        lBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU
        imu.initialize(DriveConstants.imuParameters);

        // localizer
        localizer = new Localizer(imu);

    }

    public void update()
    {
        double[] wheelVelocities = new double[] {lBD.getVelocity(AngleUnit.RADIANS), lFD.getVelocity(AngleUnit.RADIANS),
                                    rBD.getVelocity(AngleUnit.RADIANS), rFD.getVelocity(AngleUnit.RADIANS)};

        localizer.update(wheelVelocities);
    }



}
