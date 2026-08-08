package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Autonomous OpMode that searches for a cluster of yellow balls and aligns the robot to them.
 */
@Autonomous(name = "Arcade Ball Search Autonomous", group = "Drive")
public class ArcadeBallSearchAutonomous extends LinearOpMode {

    // Hardware Configuration Constants
    private static final String LEFT_MOTOR_NAME = "left motor";
    private static final String RIGHT_MOTOR_NAME = "right motor";

    // Driving Constants
    private static final double MAX_AUTO_SPEED = 0.35; 
    private static final double TURN_P_GAIN = 0.005;    // Adjusted for pixel-based error
    private static final double DRIVE_P_GAIN = 0.002;   // Adjusted for area-based error
    private static final double BEARING_THRESHOLD = 10.0; // Pixels from center
    private static final double TARGET_BLOB_AREA = 5000.0; // Proxy for distance
    private static final double AREA_THRESHOLD = 500.0;
    private static final double SEARCH_SPEED = 0.15;

    private static final int IMAGE_WIDTH = 320;
    private static final int IMAGE_HEIGHT = 240;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private WebcamName webcam;
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor colorLocator;

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

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(IMAGE_WIDTH, IMAGE_HEIGHT))
                .build();

        telemetry.addLine("Initialized - Setting Camera Controls...");
        telemetry.update();

        // 3. Manual Exposure Setup
        setManualExposure(6, 250); 

        waitForStart();

        boolean aligned = false;

        while (opModeIsActive() && !aligned) {

            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            
            // Filter out small noise blobs
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    100, 20000, blobs);

            ColorBlobLocatorProcessor.Blob targetBlob = null;
            if (!blobs.isEmpty()) {
                // The list is sorted by area by default, so index 0 is the largest
                targetBlob = blobs.get(0);
            }

            if (targetBlob != null) {
                // BLOB FOUND - Drive to Target
                double centerX = targetBlob.getBoxFit().center.x;
                double area = targetBlob.getContourArea();
                
                // Bearing error in pixels from center
                double bearingError = centerX - (IMAGE_WIDTH / 2.0);
                // Area error (proxy for distance)
                double areaError = TARGET_BLOB_AREA - area;

                telemetry.addLine("Yellow Ball Cluster Detected!");
                telemetry.addData("Center X", "%.1f", centerX);
                telemetry.addData("Area", "%.1f", area);

                if (Math.abs(bearingError) <= BEARING_THRESHOLD && Math.abs(areaError) <= AREA_THRESHOLD) {
                    stopMotors();
                    aligned = true;
                    telemetry.addLine("STATUS: TARGET REACHED!");
                } else {
                    // Turn to align (bearingError > 0 means blob is right, need positive turn)
                    double turn = Range.clip(bearingError * TURN_P_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    // Drive to align distance (areaError > 0 means too far, need positive drive)
                    double drive = Range.clip(areaError * DRIVE_P_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    leftMotor.setPower(drive + turn);
                    rightMotor.setPower(drive - turn);
                    telemetry.addData("Status", "Aligning to Cluster...");
                }
            } else {
                // SEARCHING - Rotate slowly
                leftMotor.setPower(SEARCH_SPEED);
                rightMotor.setPower(-SEARCH_SPEED);
                telemetry.addData("Status", "Searching for Yellow Balls...");
            }

            telemetry.update();
            sleep(20);
        }

        // Final Result Display
        while (opModeIsActive()) {
            telemetry.addLine("Autonomous Complete - Cluster Found");
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
