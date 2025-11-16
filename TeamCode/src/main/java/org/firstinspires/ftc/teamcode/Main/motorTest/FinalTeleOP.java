package org.firstinspires.ftc.teamcode.Main.motorTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Final TeleOp", group = "Concept")
public class FinalTeleOP extends LinearOpMode {

    private DcMotor shooterLeft;
    private DcMotor intake;

    private CRServo pusher1, pusher2;
    private Servo helper1, helper2;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {

        // Drivetrain
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Shooter + intake
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        intake      = hardwareMap.get(DcMotor.class, "intake");

        // Servos
        pusher1 = hardwareMap.get(CRServo.class, "pusher1");
        pusher2 = hardwareMap.get(CRServo.class, "pusher2");
        helper1 = hardwareMap.get(Servo.class, "helper1");
        helper2 = hardwareMap.get(Servo.class, "helper2");

        // Motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher2.setDirection(CRServo.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Robot Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive();
            intakeControl();
            shooterControl();
            pusherControl();
            helperControl();

            telemetry.addData("Shooter Power", shooterLeft.getPower());
            telemetry.addData("Helper1", helper1.getPosition());
            telemetry.addData("Helper2", helper2.getPosition());
            telemetry.update();
        }
    }

    /** MECANUM DRIVE */
    private void drive() {
        double y  = -gamepad1.left_stick_y;
        double x  = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    /** SHOOTER CONTROL */
    private void shooterControl() {
        if (gamepad1.right_trigger > 0.1) {
            shooterLeft.setPower(1);       // full speed
        } else {
            shooterLeft.setPower(0);       // stop
        }
    }

    /** INTAKE CONTROL — FIXED LOGIC */
    private void intakeControl() {
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    /** PUSHER CR SERVOS */
    private void pusherControl() {
        if (gamepad1.right_bumper) {
            pusher1.setPower(-1);
            pusher2.setPower(-1);
        } else if (gamepad1.left_bumper) {
            pusher1.setPower(1);
            pusher2.setPower(1);
        } else {
            pusher1.setPower(0);
            pusher2.setPower(0);
        }
    }

    /** HELPER SERVOS — FIXED INDEPENDENT CONTROL */
    private void helperControl() {

        // Helper 1 on button A
        if (gamepad1.a) {
            helper1.setPosition(0.2);
        } else {
            helper1.setPosition(1.0);
        }

        // Helper 2 on D-pad Up
        if (gamepad1.dpad_up) {
            helper2.setPosition(0.1);
        } else {
            helper2.setPosition(0.0);
        }
    }
}

