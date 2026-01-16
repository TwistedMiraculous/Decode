package org.firstinspires.ftc.teamcode.Test.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private ShooterSubsystem shooter;
    private ConfigLED leds;
    public static final double TARGET_RPM = 3000.0;

    public void init(HardwareMap hwMap) {
        shooter = new ShooterSubsystem();
        leds = new ConfigLED();
        shooter.init(hwMap);
        leds.init(hwMap);
    }

    /**
     * Updates LEDs and sends data to the Driver Station
     */
    public void updateFeedback(Telemetry telemetry) {
        double currentRPM = shooter.getCurrentRPM();
        boolean ready = shooter.isAtTargetRPM(TARGET_RPM);

        // LED Logic
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

        // Telemetry Data
        telemetry.addData("Shooter Status", ready ? "READY TO FIRE" : "SPOOLING...");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", Math.round(currentRPM));
        telemetry.addData("RPM Error", Math.round(TARGET_RPM - currentRPM));
    }

    public void runShooter() { shooter.setRPM(TARGET_RPM); }
    public void stopShooter() { shooter.stop(); }
}