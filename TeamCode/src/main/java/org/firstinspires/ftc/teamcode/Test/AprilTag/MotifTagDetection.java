package org.firstinspires.ftc.teamcode.Test.AprilTag;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Motif Detector", group = "Concept")
public class MotifTagDetection extends LinearOpMode {

    private static final boolean USE_WEBCAM = true; // Choose camera

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean cameraActive = false; // Track if camera is streaming
    private boolean motifDetected = false; // Track if a motif has been detected

    @Override
    public void runOpMode() {

        // Initialize AprilTag processor and camera (camera off initially)
        initAprilTag();

        telemetry.addLine("Press 'A' to start camera and detect tags");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Start camera streaming when 'A' is pressed
                if (!cameraActive && gamepad1.a) {
                    visionPortal.resumeStreaming(); // Start camera
                    cameraActive = true;
                    telemetry.addLine("Camera started! Detecting tags...");
                    telemetry.update();
                }

                // Only check detections if camera is active and motif not detected yet
                if (cameraActive && !motifDetected) {

                    List<AprilTagDetection> detections = aprilTag.getDetections();
                    telemetry.addData("# Tags Detected", detections.size());

                    for (AprilTagDetection detection : detections) {
                        String tagInfo = "";
                        boolean isMotif = false;

                        // Alliance tags (optional)
                        if (detection.id == 20) tagInfo = "BLUE BOX ALLIANCE";
                        else if (detection.id == 24) tagInfo = "RED BOX ALLIANCE";
                            // Motif randomization tags
                        else if (detection.id == 23) {
                            tagInfo = "PPG MOTIF";
                            isMotif = true;
                        } else if (detection.id == 22) {
                            tagInfo = "PGP MOTIF";
                            isMotif = true;
                        } else if (detection.id == 21) {
                            tagInfo = "GPP MOTIF";
                            isMotif = true;
                        } else tagInfo = "Unknown Tag";

                        telemetry.addLine(String.format("ID %d: %s", detection.id, tagInfo));

                        // If this is a Motif tag, perform actions
                        if (isMotif) {
                            motifDetected = true; // Prevent further motif detection

                            // --- PLACEHOLDER: Perform actions based on Motif ---
                            switch (detection.id) {
                                case 23: // PPG
                                    telemetry.addLine("Performing PPG actions...");
                                    // TODO: Insert PPG-specific robot actions here
                                    break;
                                case 22: // PGP
                                    telemetry.addLine("Performing PGP actions...");
                                    // TODO: Insert PGP-specific robot actions here
                                    break;
                                case 21: // GPP
                                    telemetry.addLine("Performing GPP actions...");
                                    // TODO: Insert GPP-specific robot actions here
                                    break;
                            }

                            telemetry.update();

                            // Break out of the loop after detecting the first motif
                            break;
                        }
                    }

                    telemetry.update();
                }

                sleep(20); // small delay to save CPU
            }
        }

        // Close camera when OpMode stops
        visionPortal.close();
    }

    private void initAprilTag() {
        // Create AprilTag processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Initialize VisionPortal but do not start streaming yet
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

        visionPortal.stopStreaming(); // Camera off initially
    }
}
