/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Main.AprilTags;

// Import necessary FTC classes for OpModes
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Import FTC vision and camera classes
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// Import List for holding multiple detections
import java.util.List;

/*
 * Example OpMode demonstrating the basics of AprilTag detection using the VisionPortal.
 * Shows tag IDs, positions, rotations, and allows toggling camera streaming.
 */
@Disabled // Remove this to show the OpMode on Driver Station
@TeleOp(name = "Concept: AprilTag Easy", group = "Concept")
public class ConceptAprilTagEasy extends LinearOpMode {

    // Flag to select camera type: true = external webcam, false = phone camera
    private static final boolean USE_WEBCAM = true;

    // Processor object for AprilTag detection
    private AprilTagProcessor aprilTag;

    // Vision portal object to manage camera streaming and processing
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Initialize the AprilTag processor and camera
        initAprilTag();

        // Send instructions to Driver Station before start
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        // Wait until the START button is pressed
        waitForStart();

        // Run while the OpMode is active
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Update telemetry with AprilTag detections
                telemetryAprilTag();

                // Push the telemetry data to Driver Station screen
                telemetry.update();

                // Allow user to stop or resume camera streaming using dpad
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming(); // Save CPU resources
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming(); // Resume camera stream
                }

                // Short sleep to prevent CPU overload
                sleep(20);
            }
        }

        // Close the vision portal when done to free resources
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the AprilTag processor and camera portal.
     */
    private void initAprilTag() {

        // Create AprilTag processor using default settings
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal depending on camera choice
        if (USE_WEBCAM) {
            // External webcam
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            // Phone camera
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end initAprilTag()

    /**
     * Adds telemetry information about detected AprilTags.
     */
    private void telemetryAprilTag() {

        // Get the list of currently detected tags
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Show number of tags detected
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Loop through each detection and display info
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Tag is recognized in TagLibrary, show full pose info
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                // Tag is unknown, show only its ID and center coordinates
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for loop

        // Add key for understanding telemetry output
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end telemetryAprilTag()

}   // end class
