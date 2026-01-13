package org.firstinspires.ftc.teamcode.Test.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Test.Subsystems.Vision;

@TeleOp(name = "Vision Test OpMode")
public class VisionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize the subsystem
        // This runs the "initVision" logic and starts the dashboard stream
        Vision vision = new Vision(hardwareMap);

        telemetry.addData("Status", "Camera Initialized");
        telemetry.update();

        // 2. Wait for the driver to press START
        waitForStart();

        while (opModeIsActive()) {
            // Get the X position from our subsystem
            double tagX = vision.getTagX();

            if (tagX != -1) {
                telemetry.addData("Tag Status", "TARGET DETECTED");
                telemetry.addData("Tag X Center", "%.2f pixels", tagX);

                // Calculate distance from center (assuming 640x480 resolution)
                double error = 320 - tagX;
                telemetry.addData("Error from Center", "%.2f pixels", error);
            } else {
                telemetry.addData("Tag Status", "No Tag Seen");
            }

            telemetry.update();
        }

        // 3. Clean up (Very important to free the camera hardware)
        vision.close();
    }
}