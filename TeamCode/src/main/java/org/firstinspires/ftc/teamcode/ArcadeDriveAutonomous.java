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
    private static final double MAX_AUTO_SPEED = 0.30; 
    private static final double TURN_P_GAIN = 0.02;     
    private static final double DRIVE_P_GAIN = 0.03;    
    private static final double BEARING_THRESHOLD = 1.0;
    private static final double TARGET_DISTANCE = 12.0; 
    private static final double DISTANCE_THRESHOLD = 0.5;
    private static final double SEARCH_SPEED = 0.06;    // Very slow for better detection

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

        // Use RUN_WITHOUT_ENCODER to match working TeleOp
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Camera Initialization
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // Use default decimation (3.0) for higher FPS
        aprilTag.setDecimation(3);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized - Ready for Autonomous Alignment");
        telemetry.update();

        waitForStart();
        
        // Wait for camera to be ready
        while (opModeIsActive() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Status", "Waiting for camera...");
            telemetry.update();
            sleep(20);
        }

        boolean aligned = false;

        while (opModeIsActive() && !aligned) {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection targetTag = null;

            int rawCount = currentDetections.size();
            telemetry.addData("# Tags Seen", rawCount);

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
                
                telemetry.addLine(String.format(Locale.US, "Target Found: ID %d", targetTag.id));
                telemetry.addLine(String.format(Locale.US, "Bearing: %.2f deg, Range: %.2f in", bearing, range));

                double rangeError = range - TARGET_DISTANCE;

                if (Math.abs(bearing) <= BEARING_THRESHOLD && Math.abs(rangeError) <= DISTANCE_THRESHOLD) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    aligned = true;
                    telemetry.addLine("STATUS: TARGET REACHED!");
                } else {
                    double drive = rangeError * DRIVE_P_GAIN;
                    double turn  = bearing * TURN_P_GAIN;

                    drive = Range.clip(drive, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn  = Range.clip(turn, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    leftMotor.setPower(drive + turn);
                    rightMotor.setPower(drive - turn);
                    telemetry.addData("Status", "Approaching...");
                }
            } else {
                // No recognized tag - search slowly
                leftMotor.setPower(SEARCH_SPEED);
                rightMotor.setPower(-SEARCH_SPEED);
                telemetry.addData("Status", "Searching...");
                if (rawCount > 0) {
                    telemetry.addLine("Tag detected but ID not in CenterStage library!");
                }
            }

            telemetry.update();
            sleep(10);
        }

        telemetry.addLine("Autonomous Complete");
        telemetry.update();
        visionPortal.close();
        sleep(5000);
    }
}
