package org.firstinspires.ftc.teamcode.Main.AprilTags.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class TurretLocalization extends LinearOpMode {

    DcMotor turretMotor;
    AprilTagCamera tagCamera;

    // Motor / Encoder constants
    static final double TICKS_PER_REV = 537.7;  // GoBILDA 312 RPM
    static final double GEAR_RATIO = 1.0;       // 1:1 if direct drive
    static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    int HOME_TICKS = 0;
    double maxPower = 0.4;
    double minPower = 0.05;
    double deadzoneYaw = 2.0;    // ±2 degrees deadzone
    int deadzoneTicks = 5;

    @Override
    public void runOpMode() {

        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize camera / AprilTag
        tagCamera = new AprilTagCamera(hardwareMap, true);

        telemetry.addLine("Ready - turret will track tag globally");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            AprilTagDetection tag = tagCamera.getBestDetection();

            if (tag != null && tag.metadata != null) {

                double yaw = tag.ftcPose.yaw;

                // Round yaw to nearest 0.5 degree
                double roundedYaw = Math.round(yaw * 2.0) / 2.0;

                // Current turret angle (global) in degrees
                double turretAngle = (turretMotor.getCurrentPosition() - HOME_TICKS) / TICKS_PER_DEGREE;

                // Global target angle for turret
                double globalTarget = turretAngle + roundedYaw;

                // Stop motor if yaw is within deadzone
                if (Math.abs(roundedYaw) < deadzoneYaw) {
                    turretMotor.setPower(0);
                } else {
                    // Convert global target to encoder ticks
                    int targetTicks = HOME_TICKS + (int)(globalTarget * TICKS_PER_DEGREE);

                    // Only update target if movement is significant
                    if (Math.abs(targetTicks - turretMotor.getTargetPosition()) > deadzoneTicks) {
                        turretMotor.setTargetPosition(targetTicks);
                    }

                    // Smooth power scaling
                    double power = Math.abs(roundedYaw) / 30.0 * maxPower;
                    if (power < minPower) power = minPower;
                    if (power > maxPower) power = maxPower;

                    turretMotor.setPower(power);
                }

                // Telemetry
                telemetry.addData("Yaw (deg)", roundedYaw);
                telemetry.addData("Turret Angle", turretAngle);
                telemetry.addData("Global Target", globalTarget);
                telemetry.addData("Target Ticks", turretMotor.getTargetPosition());
                telemetry.addData("Current Ticks", turretMotor.getCurrentPosition());
                telemetry.addData("Motor Power", turretMotor.getPower());

            } else {
                turretMotor.setPower(0);
                telemetry.addLine("No tag detected");
            }

            telemetry.update();
            sleep(20);
        }

        tagCamera.close();
    }
}
