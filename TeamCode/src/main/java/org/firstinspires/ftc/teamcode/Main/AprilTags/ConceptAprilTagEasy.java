/* Copyright (c) 2023 FIRST. All rights reserved.
 * License and disclaimer notice
 */

package org.firstinspires.ftc.teamcode.Main.AprilTags; // Package location

// FTC OpMode imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Camera and vision imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal; // Manages camera streaming
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // Stores tag info
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor; // Detects tags

import java.util.List; // For holding multiple tag detections

// Declare this OpMode for TeleOp
@TeleOp(name = "Concept: AprilTag Easy", group = "Concept")
public class ConceptAprilTagEasy extends LinearOpMode {

    private static final boolean USE_WEBCAM = true; // true = external webcam, false = phone camera
    private AprilTagProcessor aprilTag; // AprilTag processor object
    private VisionPortal visionPortal; // Manages camera streaming and processing

    @Override
    public void runOpMode() {

        initAprilTag(); // Initialize AprilTag processor and camera

        // Show instructions on Driver Station
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart(); // Wait for the START button

        // Main loop while OpMode is active
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag(); // Show tag detections on telemetry
                telemetry.update(); // Push telemetry to Driver Station

                // Allow pausing/resuming camera streaming using D-pad
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming(); // Stop camera to save CPU
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming(); // Resume camera
                }

                sleep(20); // Small delay to prevent CPU overload
            }
        }

        visionPortal.close(); // Release camera resources
    }

    /**
     * Initialize AprilTag processor and camera
     */
    private void initAprilTag() {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults(); // Create processor with default settings

        if (USE_WEBCAM) {
            // Use external webcam
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            // Use phone camera
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    /**
     * Display telemetry information for detected AprilTags
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections(); // Get current tag detections
        telemetry.addData("# AprilTags Detected", currentDetections.size()); // Show how many tags detected

        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                // Tag is recognized in Tag Library
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                // Unknown tag, only show ID and center
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y));
            }
        }

        // Key to help read telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
