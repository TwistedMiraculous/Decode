package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShooterCam {

    private ShooterSubsystem shooter;
    private ConfigLED leds;

    // Adjustable RPM Limits
    public double minRPM = 2000.0;
    public double maxRPM = 5000.0;

    // Distance limits (Inches) - adjust based on your field testing
    public double minDistance = 24.0; // 2 feet
    public double maxDistance = 120.0; // 10 feet

    private double lastTargetRPM = 0;

    public void init(HardwareMap hwMap) {
        shooter = new ShooterSubsystem();
        leds = new ConfigLED();
        shooter.init(hwMap);
        leds.init(hwMap);
    }

    /**
     * Calculates RPM based on AprilTag distance and updates shooter
     */
    public void autoAdjustRPM(AprilTagDetection detection) {
        if (detection != null && detection.ftcPose != null) {
            double distance = detection.ftcPose.range;

            // Map distance to RPM: (Current - Min) / (Max - Min)
            double scale = (distance - minDistance) / (maxDistance - minDistance);

            // Constrain scale between 0 and 1
            scale = Math.max(0, Math.min(1, scale));

            // Calculate target RPM
            lastTargetRPM = minRPM + (scale * (maxRPM - minRPM));

            shooter.setRPM(lastTargetRPM);
        } else {
            // If no tag is seen, you might want to stop or stay at a default
            // shooter.stop();
        }
    }

    public void updateFeedback(Telemetry telemetry, AprilTagDetection detection) {
        double currentRPM = shooter.getCurrentRPM();
        boolean ready = shooter.isAtTargetRPM(lastTargetRPM);

        // LED Feedback
        if (ready) {
            leds.setGreenLed(true);
            leds.setRedLed(false);
        } else if (currentRPM > 200) {
            leds.setGreenLed(false);
            leds.setRedLed(true);
        } else {
            leds.setGreenLed(false);
            leds.setRedLed(false);
        }

        // Telemetry
        telemetry.addData("Tag Distance", detection != null ? detection.ftcPose.range : "NO TAG");
        telemetry.addData("Target RPM", Math.round(lastTargetRPM));
        telemetry.addData("Current RPM", Math.round(currentRPM));
        telemetry.addData("Status", ready ? "READY" : "ADJUSTING...");
    }

    public void stopShooter() {
        shooter.stop();
        lastTargetRPM = 0;
    }
}