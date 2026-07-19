package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "indubitbly67", group = "Drive")


public class indubitbly67 extends OpMode {

    //declare motors
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    private DcMotor launcher;
    private DcMotor feeder;
    private Servo feederservo;
    
    
    @Override
    public void init() {
    //map motor 
    leftmotor = hardwareMap.get(DcMotor.class, "left motor");
    rightmotor = hardwareMap.get(DcMotor.class, "right motor");
    launcher = hardwareMap.get(DcMotor.class, "launcher");
    feederservo = hardwareMap.get(Servo.class, "feederservo");
    
    
    //Brake when sticks are released
    leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    // set encoder mode 
    leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    // change launcher direction 
    launcher.setDirection(DcMotor.Direction.REVERSE);
    
    // set servo position
    feederservo.setPosition(0.0);
    
    }
    
    @Override
    public void loop() {
        //read joysticks values
        double T545 = gamepad1.right_stick_y;
        double t1229 = gamepad1.left_stick_x;
        
        //Declare variables
        double leftpower = -(T545 - t1229);
        double rightpower = T545 + t1229;
        double launchpower = 0.65;
        
        // clip motor power 
        leftpower = Range.clip(leftpower, -0.6, 0.6);
        rightpower = Range.clip(rightpower, -0.6, 0.6);
        
        // display power and stick values
        telemetry.addData("right stick", T545);
        telemetry.addData("left stick", t1229);
        telemetry.addData("Left Power",leftpower);
        telemetry.addData("right Power", rightpower);
        telemetry.update();
        
        leftmotor.setPower(leftpower);
        rightmotor.setPower(rightpower);
            
        if (gamepad1.y) {
            launcher.setPower(launchpower);
        }
        
        if (gamepad1.a) {
            launcher.setPower(0);
        }
        
        // Servo control
        if(gamepad1.dpad_up) {
            feederservo.setPosition(0.7);
        }
        
        if(gamepad1.dpad_down) {
            feederservo.setPosition(0.0);
        }
    }

}




