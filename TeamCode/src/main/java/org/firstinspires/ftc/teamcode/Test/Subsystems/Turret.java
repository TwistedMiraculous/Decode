package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {
    private DcMotor turretMotor;

    // PID Gains
    public static double kP = 0.0006; //0.0006
    public static double kI = 0.00000001; //0.00000001
    public static double kD = 0.003; //0.003

    // Search Settings
    public static double SEARCH_SPEED = 0.12;
    private int searchDirection = 1;

    // Soft Limits
    public static int LEFT_LIMIT = -290;
    public static int RIGHT_LIMIT = 250;

    private double integralSum = 0;
    private double lastError = 0;
    private final double TARGET_X = 320;

    // --- NEW: Directional Memory ---
    private boolean wasTargetVisible = false;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void trackTarget(double currentX) {
        int currentPos = turretMotor.getCurrentPosition();

        // --- SEARCH MODE LOGIC ---
        if (currentX == -1) {
            // If this is the FIRST frame we lost the tag,
            // the searchDirection is already set by the last known error (see below)

            if (currentPos >= RIGHT_LIMIT) {
                searchDirection = -1; // Hit right wall, go left
            } else if (currentPos <= LEFT_LIMIT) {
                searchDirection = 1;  // Hit left wall, go right
            }

            turretMotor.setPower(SEARCH_SPEED * searchDirection);
            integralSum = 0; // Clear PID memory
            wasTargetVisible = false; // Update state
            return;
        }

        // --- TRACKING MODE (PID) ---
        double error = currentX - TARGET_X;

        // DIRECTIONAL MEMORY LOGIC:
        // If the tag is to the left of center, error is negative.
        // If the tag is to the right of center, error is positive.
        // We set the searchDirection based on where the tag is RIGHT NOW.
        if (error > 0) {
            searchDirection = 1;  // Tag is to the right, search right if lost
        } else {
            searchDirection = -1; // Tag is to the left, search left if lost
        }

        // Small deadzone to prevent "jitter" when locking
        if (Math.abs(error) < 2) {
            error = 0;
            integralSum = 0;
        }

        integralSum += error;
        double derivative = error - lastError;
        double power = (error * kP) + (integralSum * kI) + (derivative * kD);
        lastError = error;
        wasTargetVisible = true;

        // Apply Soft Limits
        if (currentPos <= LEFT_LIMIT && power < 0) power = 0;
        else if (currentPos >= RIGHT_LIMIT && power > 0) power = 0;

        turretMotor.setPower(Range.clip(power, -0.25, 0.25));
    }

    public int getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}