package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {
    private DcMotor turretMotor;

    public static double kP = 0.0009;
    public static double kI = 0.00000;
    public static double kD = 0.000001;

    // --- SOFT LIMIT SETTINGS ---

    public static int LEFT_LIMIT = -1000;
    public static int RIGHT_LIMIT = 1000;

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


        if (currentX == -1) {
            turretMotor.setPower(0);
            return;
        }


        double error = currentX - TARGET_X;
        integralSum += error;
        double derivative = error - lastError;
        double power = (error * kP) + (integralSum * kI) + (derivative * kD);
        lastError = error;


        if (currentPos <= LEFT_LIMIT && power < 0) {
            power = 0;
        }

        else if (currentPos >= RIGHT_LIMIT && power > 0) {
            power = 0;
        }


        turretMotor.setPower(Range.clip(power, -0.2, 0.2));
    }

    public int getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}