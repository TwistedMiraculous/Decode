package org.firstinspires.ftc.teamcode.Test.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Test.Subsystems.AprilTagCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Disabled
@Config
@TeleOp
public class TurretTrackerPID extends LinearOpMode {

    DcMotor turretMotor;
    AprilTagCamera tagCamera;

    // ===== Dashboard PD settings =====
    public static double kP = 0.07;       // proportional gain
    public static double kD = 0.01;       // derivative gain
    public static double MAX_POWER = 0.25;
    public static double MIN_POWER = 0.07;
    public static double YAW_FILTER = 0.9; // smooth noisy yaw
    public static double DEADZONE_YAW = 0.8; // degrees deadzone to stop motor

    double lastError = 0;
    double filteredYaw = 0;
    long lastTime;

    @Override
    public void runOpMode() {

        // Initialize motor
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize camera

        tagCamera = new AprilTagCamera(hardwareMap, true);

        // Start camera stream for Dashboard
        FtcDashboard.getInstance().startCameraStream(tagCamera.getCamera(), 30);

        telemetry.addLine("Turret PD tracking (Yaw 0, deadzone applied)");
        telemetry.update();
        waitForStart();

        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            AprilTagDetection tag = tagCamera.getBestDetection();

            if (tag != null && tag.metadata != null) {

                // Smooth the noisy yaw reading
                filteredYaw = YAW_FILTER * tag.ftcPose.yaw + (1 - YAW_FILTER) * filteredYaw;

                // Error toward yaw = 0
                double error = 0 - filteredYaw;

                double power = 0;

                // Only move if outside deadzone
                if (Math.abs(error) > DEADZONE_YAW) {

                    long now = System.currentTimeMillis();
                    double dt = (now - lastTime) / 1000.0;
                    lastTime = now;

                    // PD control
                    double derivative = (error - lastError) / dt;
                    lastError = error;

                    power = kP * error + kD * derivative;

                    // Clamp power
                    if (power > MAX_POWER) power = MAX_POWER;
                    if (power < -MAX_POWER) power = -MAX_POWER;

                    // Optional: apply min power only if error is significant
                    if (Math.abs(power) < MIN_POWER)
                        power = Math.signum(power) * MIN_POWER;
                } else {
                    power = 0; // stop motor inside deadzone
                    lastError = 0;
                }

                turretMotor.setPower(power);

                telemetry.addData("Yaw", filteredYaw);
                telemetry.addData("Error", error);
                telemetry.addData("Power", power);

            } else {
                turretMotor.setPower(0);
                lastError = 0;
                telemetry.addLine("No tag detected");
            }

            telemetry.update();
        }

        tagCamera.close();
    }
}
