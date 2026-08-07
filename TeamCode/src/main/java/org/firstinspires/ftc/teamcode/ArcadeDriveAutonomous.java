package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Professional Autonomous OpMode for AprilTag alignment.
 * Uses manual exposure control to eliminate motion blur during search.
 */
@Autonomous(name = "Arcade Drive Autonomous", group = "Drive")
public class ArcadeDriveAutonomous extends LinearOpMode {

    // Hardware Configuration Constants
    private static final String LEFT_MOTOR_NAME = "left motor";
    private static final String RIGHT_MOTOR_NAME = "right motor";

    // Driving Constants
    private static final double MAX_AUTO_SPEED = 0.35; 
    private static final double TURN_P_GAIN = 0.025;    
    private static final double DRIVE_P_GAIN = 0.035;   
    private static final double BEARING_THRESHOLD = 5.0;
    private static final double TARGET_DISTANCE = 12.0; 
    private static final double DISTANCE_THRESHOLD = 1.0;
    private static final double SEARCH_SPEED = 0.15;    // Higher to overcome motor friction

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

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Camera Initialization
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized - Setting Camera Controls...");
        telemetry.update();

        // 3. Manual Exposure Setup (Crucial for eliminating blur)
        setManualExposure(6, 250); 

        waitForStart();

        boolean aligned = false;
        AprilTagDetection lastDetection = null;

        while (opModeIsActive() && !aligned) {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection targetTag = null;

            // Search for recognized tags
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    targetTag = detection;
                    lastDetection = detection;
                    break;
                }
            }

            if (targetTag != null) {
                // TAG FOUND - Drive to Target
                double bearing = targetTag.ftcPose.bearing;
                double range = targetTag.ftcPose.range;
                double rangeError = range - TARGET_DISTANCE;

                telemetry.addLine(String.format(Locale.US, "Target ID %d: Bearing %.1f, Range %.1f", 
                        targetTag.id, bearing, range));

                //stopMotors();
                //sleep(10000);

                if (Math.abs(bearing) <= BEARING_THRESHOLD && Math.abs(rangeError) <= DISTANCE_THRESHOLD) {
                    stopMotors();
                    aligned = true;
                    telemetry.addLine("STATUS: TARGET REACHED!");
                } else {
                    double drive = Range.clip(rangeError * DRIVE_P_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double turn  = Range.clip(bearing * TURN_P_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    leftMotor.setPower(drive + turn);
                    rightMotor.setPower(drive - turn);
                    telemetry.addData("Status", "Aligning to Tag...");
                }
            } else if (currentDetections.size() > 0) {
                // UNKNOWN TAG - Stop and wait for identification
                stopMotors();
                telemetry.addLine("Tag Detected - Identifying...");
            } else {
                // SEARCHING - Rotate slowly
                leftMotor.setPower(SEARCH_SPEED);
                rightMotor.setPower(-SEARCH_SPEED);
                telemetry.addData("Status", "Searching (Scan Mode)...");
            }

            telemetry.update();
            sleep(20);
        }

        // Final Result Display
        while (opModeIsActive()) {
            telemetry.addLine("Autonomous Complete - Results:");
            if (lastDetection != null) {
                telemetry.addLine(String.format(Locale.US, "Final Tag ID: %d", lastDetection.id));
                telemetry.addLine(String.format(Locale.US, "Final Bearing: %.2f deg", lastDetection.ftcPose.bearing));
                telemetry.addLine(String.format(Locale.US, "Final Range: %.2f in", lastDetection.ftcPose.range));
            }
            telemetry.update();
            idle();
        }

        visionPortal.close();
    }

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /*
     Sets camera to manual exposure mode to eliminate motion blur.
     This is the most effective way to improve AprilTag detection while moving.
    */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        // Wait for camera to open
        while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
