package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecannumDriveHandler;

@TeleOp
public class LocalizationTest extends LinearOpMode
{

    MecannumDriveHandler drive;
    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new MecannumDriveHandler(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            double[] motorPowers = new double[] {leftFrontPower, rightFrontPower, rightBackPower, leftBackPower};
            drive.setPower(motorPowers);

            telemetry.addLine("x: " + drive.getCurrentPos().x);
            telemetry.addLine("y: " + drive.getCurrentPos().y);
            telemetry.addLine("angle: " + Math.toDegrees(drive.getCurrentPos().angle));
            telemetry.update();

            drive.update();
        }
    }
}
