package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class IMUTurret {
    private DcMotor turretMotor;
    private IMU imu;
    private Vision vision; // The turret now owns the Vision subsystem

    // PID Gains
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.001;

    // Search & IMU Settings
    public static double FIELD_TARGET_ANGLE = 20.0;

    // Hardware Constants
    private static final double TICKS_PER_REV = 537.7;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

    // Limits (The 180 degree restriction)
    public static double LIMIT_DEGREES = 180.0;

    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime;

    public IMUTurret(HardwareMap hardwareMap) {
        // Initialize Hardware
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize Vision internally
        vision = new Vision(hardwareMap);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastTime = System.nanoTime();
    }

    /**
     * Main update loop for the turret.
     * Call this once per loop in your OpMode.
     */
    public void update() {
        // 1. Sync camera exposure settings from Dashboard
        vision.updateDashboardExposure();

        // 2. Get sensor data
        double visionX = vision.getTagX();
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentTurretAngle = turretMotor.getCurrentPosition() * DEGREES_PER_TICK;

        double targetAngle;

        // 3. Sensor Fusion Logic
        if (visionX != -1) {
            // VISION MODE: If tag is seen, calculate target based on pixels
            // A common scaler is 0.125 degrees per pixel for 640x480
            double pixelError = 320 - visionX;
            targetAngle = currentTurretAngle + (pixelError * 0.125);
        } else {
            // IMU MODE: If tag is lost, stay pointed at the field target (e.g. 20 degrees)
            targetAngle = FIELD_TARGET_ANGLE - robotYaw;
        }

        // 4. Safety: Apply the 180 degree restriction
        targetAngle = Range.clip(targetAngle, -LIMIT_DEGREES, LIMIT_DEGREES);

        // 5. PID Calculation
        double error = targetAngle - currentTurretAngle;

        long now = System.nanoTime();
        double deltaTime = (now - lastTime) / 1e9;
        lastTime = now;

        // Anti-windup for Integral
        if (Math.abs(error) < 1.0 || Math.abs(error) > 20.0) {
            integralSum = 0;
        } else {
            integralSum += error * deltaTime;
        }

        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        double power = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Final motor output
        turretMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public double getVisionX() {
        return vision.getTagX();
    }

    public void stop() {
        turretMotor.setPower(0);
        vision.close();
    }
}