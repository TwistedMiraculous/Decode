package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {
    private DcMotorEx shooterMotor;

    // Constants for GoBILDA 6000 RPM (1:1 ratio)
    private static final double TICKS_PER_REV = 28.0;
    private static final double TOLERANCE_RPM = 100.0;

    public void init(HardwareMap hwMap) {
        shooterMotor = hwMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setRPM(double targetRPM) {
        // Formula: (RPM * TicksPerRev) / 60 seconds
        double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;
        shooterMotor.setVelocity(ticksPerSec);
    }

    public double getCurrentRPM() {
        double ticksPerSec = shooterMotor.getVelocity();
        return (ticksPerSec * 60.0) / TICKS_PER_REV;
    }

    public boolean isAtTargetRPM(double targetRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) < TOLERANCE_RPM && targetRPM > 0;
    }

    public void stop() {
        shooterMotor.setPower(0);
    }
}