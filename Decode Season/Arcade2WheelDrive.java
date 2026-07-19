package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Arcade Drive 2 Wheel", group = "Drive")

public class Arcade2WheelDrive extends OpMode {

    // Declare motors
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    private DcMotor launcher;
    private DcMotor feeder;
    private CRServo feederservo;
    
    @Override
    public void init() {
        // Map motors
        leftmotor = hardwareMap.get(DcMotor.class, "left motor");
        rightmotor = hardwareMap.get(DcMotor.class, "right motor");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        feeder = hardwareMap.get(DcMotor.class,"feeder");
        feederservo = hardwareMap.get(CRServo.class, "feederservo");
        
        // Otional set initial position This is only if its positional not continuous
        //feederservo.setPosition(0.0);       // Range 0.0 - 1.0
        
        // Reverse one motor so robot drives straight
        leftmotor.setDirection(DcMotor.Direction.FORWARD);
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        
        // Brake when sticks are released
        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Read joystick values
        double drive = -gamepad1.left_stick_y;  // Forward/back
        double turn  =  gamepad1.right_stick_x; // Turning

        // Arcade drive math
        double leftPower  = drive + turn;
        double rightPower = drive - turn;

        // declare variables
        double launchSpeed = 0.8;
        double feederSpeed = 0.6;
        double feederServoSpeed = 1.0;
        
        // Optional slow mode (hold right bumper)
        if (gamepad1.right_bumper) {
            leftPower  *= 0.4;
            rightPower *= 0.4;
        }

        // Clip motor powers
        leftPower  = Range.clip(leftPower,  -0.8, 0.8);
        rightPower = Range.clip(rightPower, -0.8, 0.8);
        feederSpeed = Range.clip(feederSpeed, -1.0,1.0);
        feederServoSpeed = Range.clip(feederServoSpeed, -1.0, 1.0);
        launchSpeed = Range.clip(launchSpeed, -1.0, 1.0);

        if (gamepad1.dpad_right) {
            feederservo.setPower(feederServoSpeed);
        }
        
        if (gamepad1.dpad_left) {
            feederservo.setPower(-feederServoSpeed);
        }
        
        if (gamepad1.dpad_down) {
            feederservo.setPower(0.0);
        }
        
        // Send power to motors
        leftmotor.setPower(leftPower);
        rightmotor.setPower(rightPower);

        // Telemetry
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Feeder", feederSpeed);
        telemetry.update();
        
        // turn on launch on
        if (gamepad1.y) {
            launcher.setPower(launchSpeed);
        }
        
        // turn off launch motor
        if (gamepad1.a) {
            launcher.setPower(0);
        }
        
        // turn on feeder on
        if (gamepad1.b) {
            feeder.setPower(feederSpeed);
        }
        
        // turn off feeder
        if (gamepad1.x) {
            feeder.setPower(0);
        }
    }
}



