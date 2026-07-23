package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Professional Arcade Drive implementation using the right stick.
 * Features: Power normalization, input deadzones, and encoder-based velocity control.
 */
@TeleOp(name = "Arcade Drive Right Stick", group = "Drive")
public class ArcadeDriveRightStick extends LinearOpMode {

    // Hardware Configuration Constants
    private static final String LEFT_MOTOR_NAME = "left motor";
    private static final String RIGHT_MOTOR_NAME = "right motor";

    // Driving Constants
    private static final double MAX_POWER = 0.70;
    private static final double DEADZONE = 0.05;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {

        // 1. Hardware Mapping
        leftMotor = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME);
        rightMotor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME);

        // 2. Motor Configuration
        // Adjust directions so positive power moves the robot forward
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Use RUN_USING_ENCODER for consistent speed regulation
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure the robot stops immediately when power is zero
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized - Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 3. Input Capture & Deadzone Logic
            double drive = -gamepad1.right_stick_y;   // Forward/Reverse
            double turn  =  gamepad1.right_stick_x;   // Left/Right

            if (Math.abs(drive) < DEADZONE) drive = 0;
            if (Math.abs(turn) < DEADZONE) turn = 0;

            // 4. Arcade Drive Mixing
            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // 5. Power Normalization
            // This ensures the turning ratio is preserved even if we hit MAX_POWER
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > MAX_POWER) {
                leftPower  = (leftPower / max) * MAX_POWER;
                rightPower = (rightPower / max) * MAX_POWER;
            }

            // 6. Send power to motors
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // 7. Telemetry for Debugging and Driver Feedback
            telemetry.addData("Status", "Running");
            telemetry.addData("Inputs", "Drive: %.2f, Turn: %.2f", drive, turn);
            telemetry.addData("Powers", "Left: %.2f, Right: %.2f", leftPower, rightPower);
            telemetry.addData("Encoders", "L: %d, R: %d", 
                leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
