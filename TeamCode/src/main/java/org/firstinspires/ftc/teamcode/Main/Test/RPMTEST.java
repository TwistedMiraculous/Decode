package org.firstinspires.ftc.teamcode.Main.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RPMTEST", group = "Test")
public class RPMTEST extends LinearOpMode {
    private boolean yWasPressed = false;
    private DcMotorEx shooterLeft;
    boolean shooterOn = false;

    private Servo helper1, helper2;
    private boolean rightBumperWasPressed = false;
    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        helper1 = hardwareMap.get(Servo.class, "helper1");
        helper2 = hardwareMap.get(Servo.class, "helper2");

        // --- Motor setup ---
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- PIDF tuning
        // You can tweak these to stabilize speed
        shooterLeft.setVelocityPIDFCoefficients(100, 0, 20, 12);

        // --- Shooter target speed ---
        double targetRPM = 3400;   // desired shooter RPM
        double gearRatio = 1.0;     // change if your wheel is geared
        double ticksPerRev = 28 * gearRatio; // GoBILDA motor
        double targetTPS = targetRPM * ticksPerRev / 60.0; // ticks per second

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.y && !yWasPressed) {
                shooterOn = !shooterOn;   // flip state
                if (shooterOn) {
                    shooterLeft.setVelocity(targetTPS);  // turn ON
                } else {
                    shooterLeft.setPower(0);             // turn OFF
                }
            }
            yWasPressed = gamepad1.y;



            helperControl();
            // --- Telemetry ---
            telemetry.addData("Shooter Velocity (TPS)", shooterLeft.getVelocity());
            telemetry.addData("Target Velocity (TPS)", targetTPS);
            telemetry.update();
        }


    }
    private void helperControl() {
        boolean downPressed = gamepad1.dpad_down || gamepad2.dpad_down;



        boolean rightBumperPressed = gamepad1.right_bumper;

        if (rightBumperPressed && !rightBumperWasPressed) {
            helper2.setPosition(0.1);
            sleep(400);
            helper2.setPosition(0.0);
        }

        rightBumperWasPressed = rightBumperPressed;
    }
}
