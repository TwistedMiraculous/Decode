package org.firstinspires.ftc.teamcode.Main.FInalTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentric extends LinearOpMode {

    // Motors
    private DcMotor shooterLeft;
    private DcMotor intake;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // Servos
    private CRServo pusher1, pusher2;
    private Servo helper1, helper2;

    // Shooter toggle variables

    private double shooterPower = 0.0;      // current motor power
    private boolean yWasPressed = false;    // detect rising edge
    private boolean bWasPressed = false;    // detect rising edge

    // Intake toggle variables
    private boolean intakeOn = false;
    private boolean leftTriggerPreviouslyPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Configure drive motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor  = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor= hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure shooter and intake
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        intake      = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configure servos
        pusher1 = hardwareMap.get(CRServo.class, "pusher1");
        pusher2 = hardwareMap.get(CRServo.class, "pusher2");
        pusher2.setDirection(CRServo.Direction.REVERSE);
        helper1 = hardwareMap.get(Servo.class, "helper1");
        helper2 = hardwareMap.get(Servo.class, "helper2");

        // Configure IMU
        IMU imu = hardwareMap.get(IMU.class, "pinpoint");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        telemetry.addLine("Robot Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Field-centric drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
                telemetry.addLine("Yaw reset!");
                telemetry.update();
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1; // compensate strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            backRightMotor.setPower((rotY + rotX - rx) / denominator);


            shooterToggleControl();
            shooterSpeedControl();  // Gamepad2 overrides only while pressed
            intakeControl();
            pusherControl();
            helperControl();


            telemetry.addData("Motor Power", shooterLeft.getPower());
            telemetry.update();
        }
    }



    private void shooterToggleControl() {
        // Y button toggle
        if (gamepad1.y && !yWasPressed) {
            if (shooterPower == 0.7) {
                shooterPower = 0; // turn off if already 1.0
            } else {
                shooterPower = 0.7; // turn on
            }
        }

        // B button toggle
        if (gamepad1.b && !bWasPressed) {
            if (shooterPower == 0.5 ) {
                shooterPower = 0.0; // turn off if already 0.9
            } else {
                shooterPower = 0.5; // turn on
            }
        }

        // Update previous button states
        yWasPressed = gamepad1.y;
        bWasPressed = gamepad1.b;

        // Set motor power
        shooterLeft.setPower(shooterPower);
    }


    // Shooter Gamepad2 speed control (override while held)
    private void shooterSpeedControl() {
        double power = -1; // -1 means no override
        if (gamepad2.dpad_left)  power = 0.6;
        else if (gamepad2.dpad_up)    power = 0.65;
        else if (gamepad2.dpad_right) power = 0.7;
        else if (gamepad2.dpad_down)  power = 0.75;
        else if (gamepad2.x)           power = 0.8;
        else if (gamepad2.y)           power = 0.85;
        else if (gamepad2.b)           power = 0.9;
        else if (gamepad2.a)           power = 1.0;

        if (power != -1) shooterLeft.setPower(power); // only set if pressed
    }


    private void intakeControl() {
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
        if (leftTriggerPressed && !leftTriggerPreviouslyPressed) intakeOn = !intakeOn;

        if (intakeOn) intake.setPower(1);
        else if (gamepad1.left_bumper) intake.setPower(-1);
        else intake.setPower(0);

        leftTriggerPreviouslyPressed = leftTriggerPressed;
    }


    private void pusherControl() {
        if (gamepad1.dpad_left) {
            pusher1.setPower(-1);
            pusher2.setPower(-1);
        }
        else if (gamepad1.dpad_right) {
            pusher1.setPower(1);
            pusher2.setPower(1);
        }
        else {
            pusher1.setPower(0);
            pusher2.setPower(0);
        }

        if (gamepad2.left_bumper) {
            pusher1.setPower(-1);
            pusher2.setPower(-1);
        }
        else if (gamepad2.right_bumper) helper1.setPosition(0.2);
        else { helper1.setPosition(1.0); }
    }


    private void helperControl() {
        if (gamepad1.a) helper1.setPosition(0.2);
        else helper1.setPosition(1.0);

        if (gamepad1.right_bumper) helper2.setPosition(0.1);
        else helper2.setPosition(0.0);
    }
}
