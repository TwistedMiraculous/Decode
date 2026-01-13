package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {
    private DcMotor turretMotor;

    public static double kP = 0.0003;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // --- SOFT LIMIT SETTINGS ---
    // You will need to find these numbers by spinning your turret manually
    public static int LEFT_LIMIT = -1000;  // Example: 1000 ticks to the left
    public static int RIGHT_LIMIT = 1000;  // Example: 1000 ticks to the right

    private double integralSum = 0;
    private double lastError = 0;
    private final double TARGET_X = 320;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoder so "0" is wherever the turret is when you turn it on
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void trackTarget(double currentX) {
        int currentPos = turretMotor.getCurrentPosition();

        // 1. STOP IF NO TAG
        if (currentX == -1) {
            turretMotor.setPower(0);
            return;
        }

        // 2. CALCULATE PID POWER
        double error = currentX - TARGET_X;
        integralSum += error;
        double derivative = error - lastError;
        double power = (error * kP) + (integralSum * kI) + (derivative * kD);
        lastError = error;

        // 3. APPLY SOFT LIMITS
        // If we are at the left limit and trying to go further left, stop.
        if (currentPos <= LEFT_LIMIT && power < 0) {
            power = 0;
        }
        // If we are at the right limit and trying to go further right, stop.
        else if (currentPos >= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        // 4. SET MOTOR POWER
        turretMotor.setPower(Range.clip(power, -0.1, 0.1));
    }

    public int getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}