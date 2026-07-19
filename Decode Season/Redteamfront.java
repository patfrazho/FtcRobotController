package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class Redteamfront extends LinearOpMode {

  DcMotor rightmotor;
  DcMotor leftmotor;
  private DcMotor launcher;
  IMU imu;
  private Servo feederservo;

  // define variables
  final int ticksPerRevolution = 28;                                        // HD hex motor REV-41-1291
  final int gearRatio = 20;                                                 // planetary gear ratio
  final double wheelDiameter = 3.594;                                       // 90 mm grip wheel
  final int ticksPerWheelRotation = ticksPerRevolution * gearRatio;         // ticks per revolution after planetary gears
  final double wheelCircumference = wheelDiameter * Math.PI;                // wheel circumference in inch
  final double ticksPerInch = ticksPerWheelRotation / wheelCircumference;   // encoder ticks per linear inch
  
  @Override
  public void runOpMode() {
  // get hardware map for drive motors
  rightmotor = hardwareMap.get(DcMotor.class, "right motor");
  leftmotor = hardwareMap.get(DcMotor.class, "left motor");
  launcher = hardwareMap.get(DcMotor.class, "launcher");
  feederservo = hardwareMap.get(Servo.class, "feederservo");

  // turn off auto clear on driver hub telemetry
  telemetry.setAutoClear(false);
    
  // define IMU to control turning angles
  imu = hardwareMap.get(IMU.class, "imu");
  RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
  RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    
  RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    
  // initialize imu
  try {
      imu.initialize(new IMU.Parameters(orientationOnRobot));
      telemetry.addLine("IMU initialized");
    } catch (Exception e) {
      telemetry.addLine("IMU initialization failed");
      telemetry.addData("Error", e.getMessage());
    }
    telemetry.addData("IMU calibrated",imu.getRobotYawPitchRollAngles());
    telemetry.update();
    
    // reset yaw to zero
    imu.resetYaw();
    
    // set servo position
    feederservo.setPosition(0.0);

    // wait for start button to be pressed
    waitForStart();

    
    // drive program goes here
    driveToDistance(43,0.61);
    turnToYaw(185,0.34);
    launcher.setPower(0.6);
    sleep(2500);
    
    // first shot
    feederservo.setPosition(0.6);
    sleep(300);
    feederservo.setPosition(0.0);
    sleep(2500);
    
    // second shot
    feederservo.setPosition(0.5);
    sleep(300);
    feederservo.setPosition(0.0);
    sleep(2500);

    // third shot
    feederservo.setPosition(0.6);
    sleep(300);
    feederservo.setPosition(0.0);
    sleep(1000);
 
    launcher.setPower(0.0);

    turnToYaw(-40,0.2);
    driveToDistance(68,0.45);
    
}

  // methods
  private void initMotor() {
    // reset encoders
    rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // set zero power behavior
    rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    
    // set motor mode run using encoder
    rightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    telemetry.addData("Init Motor","Run Using Encoder");
    telemetry.update();
    
    // set motor direction to reverse on right side
    rightmotor.setDirection(DcMotor.Direction.REVERSE);
    leftmotor.setDirection(DcMotor.Direction.FORWARD);
    launcher.setDirection(DcMotor.Direction.REVERSE);

  }
  
  private void driveToDistance(double targetInches, double maxPower) {
    // define variables
    double kP = 0.02;         // power per inch
    double minPower = 0.15;   // min power to overcome static friction
    double tolerance = 0.5;   // inches of distance tolerance

    // initialize motors
    initMotor();
    
    // run motors until desired distance is achieved
    while (opModeIsActive()) {
      // define variables
      double currentInches = getAverageEncoderInches();
      double error = targetInches - currentInches;

      // check if robot is at desired distance
      if (Math.abs(error) <= tolerance) {
        // break out of loop
        break;
      }

      // set variable power, faster for longer distances and slower at closer distances
      double drivePower = kP * error;

      // get power without going below the minimum power, set sign of power for direction
      drivePower = Math.copySign(
        Math.max(minPower, Math.abs(drivePower)),
        drivePower
      );

      // make sure that drivePower is between the min and max power range
      drivePower = Range.clip(drivePower, -maxPower, maxPower);

      //setDrivePower(drivePower, drivePower);
      rightmotor.setPower(drivePower);
      leftmotor.setPower(drivePower);
    }

    // stop motors
    stopDrive();
  }

  private double getAverageEncoderInches() {
    // average the two motor encoders
    double avgTicks = (leftmotor.getCurrentPosition() + rightmotor.getCurrentPosition()) / 2.0;
    return avgTicks / ticksPerInch;
  }

  private void stopDrive() {
    // stop motors
    leftmotor.setPower(0);
    rightmotor.setPower(0);
  }

  private void turnToYaw(double targetYaw, double maxPower) {
    // define variables
    double tolerance = 2.0;   // degrees of tolerance for yaw
    double kP = 0.01;         // power per degree
    double minPower = 0.15;   // min power to overcome static friction
    
    // initialize motors
    initMotor();

    // run motors until desired yaw is achieved
    while (opModeIsActive()) {
        // define varibles
        double currentYaw = getYaw();
        double error = normalizeAngle(targetYaw - currentYaw);

        // check if yaw is achieved
        if (Math.abs(error) <= tolerance) {
            // break out of loop
            break;
        }

        // set variable power, faster for greater angles and slower at closer angles
        double turnPower = kP * error;
        
        // get power without going below the minimum power, set sign of power for direction 
        turnPower = Math.copySign(
                Math.max(minPower, Math.abs(turnPower)),
                turnPower);
        
        // make sure that drivePower is between the min and max power range
        turnPower = Range.clip(turnPower, -maxPower, maxPower);

        //setDrivePower
        rightmotor.setPower(turnPower);
        leftmotor.setPower(-turnPower);
    }

    // stop motors
    stopDrive();
  }
  
  private double getYaw() {
    // get current yaw
    YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
    return angles.getYaw(AngleUnit.DEGREES);
    }

  private double normalizeAngle(double angle) {
    // normalize angle to make sure its between -180 and 180
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
  }
}
  






