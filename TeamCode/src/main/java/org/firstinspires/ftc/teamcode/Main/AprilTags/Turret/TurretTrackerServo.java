package org.firstinspires.ftc.teamcode.Main.AprilTags.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp
public class TurretTrackerServo extends LinearOpMode {

    Servo turretServo;
    AprilTagCamera tagCamera;

    // ===== Dashboard PD settings =====
    public static double kP = 0.004;        // proportional gain
    public static double kD = 0.002;        // derivative gain (damping)
    public static double MAX_SPEED = 0.02;  // max change per loop
    public static double YAW_FILTER = 0.9;  // smooth noisy yaw
    public static double DEADZONE_YAW = 0.8; // degrees deadzone

    // Servo mapping
    public static double SERVO_MIN = 0;     // min servo position
    public static double SERVO_MAX = 1;     // max servo position
    public static double SERVO_CENTER = 0.5; // servo position for yaw = 0

    double filteredYaw = 0;
    double lastError = 0;

    @Override
    public void runOpMode() {

        // Initialize servo
        turretServo = hardwareMap.get(Servo.class, "turret");

        // Initialize camera
        tagCamera = new AprilTagCamera(hardwareMap, true);

        telemetry.addLine("Turret tracking with servo (PD, no overshoot)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            AprilTagDetection tag = tagCamera.getBestDetection();

            if (tag != null && tag.metadata != null) {

                // Smooth the noisy yaw reading
                filteredYaw = YAW_FILTER * tag.ftcPose.yaw + (1 - YAW_FILTER) * filteredYaw;

                // Error toward yaw = 0
                double error = 0 - filteredYaw;

                // Only move if outside deadzone
                if (Math.abs(error) > DEADZONE_YAW) {

                    // PD control for servo movement
                    double derivative = error - lastError;
                    lastError = error;

                    double targetOffset = kP * error + kD * derivative;

                    // Map to servo position
                    double targetPos = SERVO_CENTER + targetOffset;

                    // Clamp to servo range
                    targetPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, targetPos));

                    // Smooth movement: move by max MAX_SPEED per loop
                    double currentPos = turretServo.getPosition();
                    double distance = targetPos - currentPos;

                    if (Math.abs(distance) > 0.001) { // small deadzone to prevent jitter
                        double move = Math.signum(distance) * Math.min(Math.abs(distance), MAX_SPEED);
                        turretServo.setPosition(currentPos + move);
                    }

                }

                telemetry.addData("Yaw", filteredYaw);
                telemetry.addData("Error", error);
                telemetry.addData("Servo Pos", turretServo.getPosition());

            } else {
                telemetry.addLine("No tag detected");
            }

            telemetry.update();
        }

        tagCamera.close();
    }
}
