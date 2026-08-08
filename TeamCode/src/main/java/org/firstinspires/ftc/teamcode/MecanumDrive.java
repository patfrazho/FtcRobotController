package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum Drive TeleOp for goBILDA Mecanum Kit.
 * Left Stick: Translation (Forward, Reverse, Strafe)
 * Right Stick: Rotation (Turning)
 */
@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Declare motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right_drive");

        // goBILDA Mecanum motors on the left side are usually reversed to drive forward
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: Brake on zero power for more precise stopping
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized - Ready to Drive");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Y is negative on joysticks because up is forward
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Powers", "FL:%.2f, FR:%.2f, BL:%.2f, BR:%.2f", 
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}
