package org.firstinspires.ftc.teamcode.Main.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "RPM", group = "Test")
public class RPM extends LinearOpMode {

    private DcMotorEx shooterLeft;

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        // --- Motor setup ---
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- PIDF tuning
        // You can tweak these to stabilize speed
        shooterLeft.setVelocityPIDFCoefficients(100, 0, 20, 12);

        // --- Shooter target speed ---
        double targetRPM = 3400;    // desired shooter RPM
        double gearRatio = 1.0;     // change if your wheel is geared
        double ticksPerRev = 28 * gearRatio; // GoBILDA motor
        double targetTPS = targetRPM * ticksPerRev / 60.0; // ticks per second

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.a) {
                shooterLeft.setVelocity(targetTPS);
            } else {
                shooterLeft.setPower(0);
            }

            // --- Telemetry ---
            telemetry.addData("Shooter Velocity (TPS)", shooterLeft.getVelocity());
            telemetry.addData("Target Velocity (TPS)", targetTPS);
            telemetry.update();
        }

    }
}
