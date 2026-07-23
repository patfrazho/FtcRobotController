package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Arcade Drive Right Stick", group = "Drive")
public class ArcadeDriveRightStick extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Maximum drive power
    private static final double MAX_POWER = 0.70;

    @Override
    public void runOpMode() {

        // Hardware Mapping
        leftMotor = hardwareMap.get(DcMotor.class, "left motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right motor");

        // Reverse the left motor so both motors move the robot forward
        // when positive power is applied.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Use encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake when joystick returns to center
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Right stick controls everything
            double drive = -gamepad1.right_stick_y;   // Forward/Reverse
            double turn  =  gamepad1.right_stick_x;   // Left/Right

            // Arcade drive mixing
            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Clip to requested maximum power
            leftPower  = Range.clip(leftPower,  -MAX_POWER, MAX_POWER);
            rightPower = Range.clip(rightPower, -MAX_POWER, MAX_POWER);

            // Send power to motors
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Telemetry
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
            telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}