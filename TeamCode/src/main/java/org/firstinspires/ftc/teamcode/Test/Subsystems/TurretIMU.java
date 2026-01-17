package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@Config
@TeleOp(name = "Turret PID + IMU")
public class TurretIMU extends LinearOpMode {

    // ===== Dashboard Tunables =====
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double MAX_POWER = 1.0;

    // ===== Hardware =====
    private DcMotor turretMotor;
    private IMU imu;

    // ===== PID State =====
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime;

    // ===== Encoder conversion =====
    private static final double TICKS_PER_REV = 537.7;
    private static final double GEAR_RATIO = 1.0;
    public static double DESIRED_FIELD_YAW = 20.0;

    private static final double DEGREES_PER_TICK =
            360.0 / (TICKS_PER_REV * GEAR_RATIO);

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Optional: brake so turret holds position
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastTime = System.nanoTime();

        waitForStart();

        while (opModeIsActive()) {

            // --- IMU yaw ---
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double robotYaw = orientation.getYaw(AngleUnit.DEGREES);

            // Target: always face field yaw = 20
            double targetTurretAngle = DESIRED_FIELD_YAW - robotYaw;


            // Current turret angle
            double currentTurretAngle =
                    turretMotor.getCurrentPosition() * DEGREES_PER_TICK;

            // PID error
            double error = targetTurretAngle - currentTurretAngle;

            // Time step
            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            double output =
                    (kP * error) +
                            (kI * integralSum) +
                            (kD * derivative);

            output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));
            turretMotor.setPower(output);

            // Telemetry (very useful)
            telemetry.addData("Robot Yaw", robotYaw);
            telemetry.addData("Target Turret", targetTurretAngle);
            telemetry.addData("Current Turret", currentTurretAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Power", output);
            telemetry.update();
        }
    }
}
