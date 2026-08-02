package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

/**
 * Autonomous OpMode that searches for an AprilTag and aligns the robot to it.
 * The robot rotates until the AprilTag bearing is between -1.0 and 1.0 degrees.
 */
@Autonomous(name = "Arcade Drive Autonomous", group = "Drive")
public class ArcadeDriveAutonomous extends LinearOpMode {

    // Hardware Configuration Constants
    private static final String LEFT_MOTOR_NAME = "left motor";
    private static final String RIGHT_MOTOR_NAME = "right motor";

    // Driving Constants
    private static final double MAX_AUTO_SPEED = 0.30; // Slower for precision
    private static final double TURN_P_GAIN = 0.02;     // Proportional gain for turning
    private static final double DRIVE_P_GAIN = 0.03;    // Proportional gain for driving
    private static final double BEARING_THRESHOLD = 1.0;
    private static final double TARGET_DISTANCE = 12.0; // Target distance in inches
    private static final double DISTANCE_THRESHOLD = 0.5;
    private static final double SEARCH_SPEED = 0.08;    // Slow rotation speed for searching

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private WebcamName webcam;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // 1. Hardware Mapping
        leftMotor = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME);
        rightMotor = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME);

        // 2. Motor Configuration
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Camera Initialization
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Max sensitivity (1.0 = highest, 3.0 = default)
        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized - Ready for Autonomous Alignment");
        telemetry.update();

        waitForStart();
        
        // Ensure camera is actually streaming before moving
        while (opModeIsActive() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Status", "Waiting for camera...");
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
            sleep(20);
        }

        boolean aligned = false;

        while (opModeIsActive() && !aligned) {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection targetTag = null;

            // Diagnostic: Count any detections (even those without metadata)
            int rawCount = currentDetections.size();

            // Find the first valid detection with metadata
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    targetTag = detection;
                    break;
                }
            }

            if (targetTag != null) {
                double bearing = targetTag.ftcPose.bearing;
                double range = targetTag.ftcPose.range;
                
                telemetry.addLine(String.format(Locale.US, "AprilTag Detected: ID %d, Bearing: %.2f deg", targetTag.id, bearing));
                telemetry.addLine(String.format(Locale.US, "Range: %.2f inches", range));

                // Calculate error
                double rangeError = range - TARGET_DISTANCE;

                if (Math.abs(bearing) <= BEARING_THRESHOLD && Math.abs(rangeError) <= DISTANCE_THRESHOLD) {
                    // Within threshold - stop
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    aligned = true;
                    telemetry.addLine("STATUS: TARGET REACHED!");
                } else {
                    // Drive and Turn towards the tag
                    double drive = rangeError * DRIVE_P_GAIN;
                    double turn  = bearing * TURN_P_GAIN;

                    // Clip to max speed
                    drive = com.qualcomm.robotcore.util.Range.clip(drive, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn  = com.qualcomm.robotcore.util.Range.clip(turn, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    // Combine drive and turn
                    double leftPower  = drive + turn;
                    double rightPower = drive - turn;

                    // Send power to motors
                    leftMotor.setPower(leftPower);
                    rightMotor.setPower(rightPower);
                    
                    telemetry.addData("Status", "Approaching...");
                }
            } else if (rawCount > 0) {
                // Tag is seen but metadata is missing - STOP to get a clear frame
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.addData("Status", "Tag seen! Identifying...");
            } else {
                // No tag detected - search by rotating slowly
                leftMotor.setPower(SEARCH_SPEED);
                rightMotor.setPower(-SEARCH_SPEED);
                telemetry.addData("Status", "Searching for AprilTag...");
                telemetry.addData("Camera State", visionPortal.getCameraState());
            }

            telemetry.update();
            sleep(10); // Small delay to avoid hammering the CPU
        }

        telemetry.addLine("Autonomous Complete");
        telemetry.update();

        // Clean up camera resources
        visionPortal.close();
    }
}
