package org.firstinspires.ftc.teamcode.Main.FInalTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main.Test.ConfigLED;

@TeleOp
public class FieldCentric extends LinearOpMode {

    // ---------------- SHOOTER CONSTANTS ----------------
    private final double targetRPMHigh = 3900;
    private final double targetRPMLow = 3400;
    private final double targetRPM2Low = 3000;
    private final double gearRatio = 1.0;
    private final double ticksPerRev = 28 * gearRatio;   // GoBILDA motor at output shaft

    private double targetTPSHigh = 0;
    private double targetTPSLow2 = 0;
    private double targetTPSLow = 0;

    // ---------------- Hardware ----------------
    private DcMotorEx shooterLeft;
    private DcMotor intake;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private CRServo pusher1, pusher2;
    private Servo helper1, helper2;

    // ---------------- Toggles ----------------
    private boolean shooterOn = false;
    private boolean yWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;

    private boolean intakeOn = false;
    private boolean leftTriggerPreviouslyPressed = false;

    private boolean slowMode = false;
    private boolean backWasPressed = false;
    private boolean rightBumperWasPressed = false;

    // ---------------- PIDF Coefficients ----------------
    private final double P = 60;
    private final double I = 0;
    private final double D = 6;
    private final double FHigh = 5.0;   // Feedforward for high RPM target
    private final double FLow  = 5.8;   // Feedforward for low RPM target (more to overcome inertia)
    ConfigLED bench = new ConfigLED();
    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- DRIVE MOTORS ----------------
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor  = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor= hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");


        // Initialize LEDs
        bench.init(hardwareMap);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- SHOOTER + INTAKE ----------------
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        intake      = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Pre-calculate TPS targets
        targetTPSHigh = targetRPMHigh * ticksPerRev / 60.0;
        targetTPSLow  = targetRPMLow  * ticksPerRev / 60.0;
        targetTPSLow2  = targetRPM2Low  * ticksPerRev / 60.0;

        // ---------------- SERVOS ----------------
        pusher1 = hardwareMap.get(CRServo.class, "pusher1");
        pusher2 = hardwareMap.get(CRServo.class, "pusher2");
        pusher2.setDirection(CRServo.Direction.REVERSE);

        helper1 = hardwareMap.get(Servo.class, "helper1");
        helper2 = hardwareMap.get(Servo.class, "helper2");

        // ---------------- IMU ----------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));

        telemetry.addLine("Robot Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ---------------- FIELD CENTRIC DRIVE ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            // Slow mode toggle
            if (gamepad1.back && !backWasPressed) {
                slowMode = !slowMode;
            }

            backWasPressed = gamepad1.back;

            // LED indicator
            if (slowMode) {
                bench.setRedLed(false);
                bench.setGreenLed(true);
            } else {
                bench.setRedLed(true);
                bench.setGreenLed(false);
            }


            double speedMultiplier = slowMode ? 0.5 : 1.0;

            // Drive power
            frontLeftMotor.setPower(speedMultiplier * ((rotY + rotX + rx) / denominator));
            backLeftMotor.setPower(speedMultiplier * ((rotY - rotX + rx) / denominator));
            frontRightMotor.setPower(speedMultiplier * ((rotY - rotX - rx) / denominator));
            backRightMotor.setPower(speedMultiplier * ((rotY + rotX - rx) / denominator));

            // ---------------- CALL HANDLERS ----------------
            intakeControl();
            pusherControl();
            helperControl();
            shooterToggleControl();
            shooterGamepad2Control();

            // ---------------- LIVE SHOOTER SPEED ----------------
            double currentTPS = shooterLeft.getVelocity();     // ticks per second
            double currentRPM = (currentTPS / ticksPerRev) * 60.0;

            telemetry.addData("Shooter On?", shooterOn);
            telemetry.addData("Current TPS", currentTPS);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Target RPM High", targetRPMHigh);
            telemetry.addData("Target RPM Low", targetRPMLow);
            telemetry.update();
        }
    }

    // ---------------- INTAKE ----------------
    private void intakeControl() {
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        if (leftTriggerPressed && !leftTriggerPreviouslyPressed) intakeOn = !intakeOn;

        if (intakeOn) intake.setPower(1);
        else if (gamepad1.left_bumper || gamepad2.left_bumper) intake.setPower(-1);
        else intake.setPower(0);

        leftTriggerPreviouslyPressed = leftTriggerPressed;
    }

    // ---------------- PUSHER ----------------
    private void pusherControl() {
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            pusher1.setPower(-1);
            pusher2.setPower(-1);
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            pusher1.setPower(1);
            pusher2.setPower(1);
        } else {
            pusher1.setPower(0);
            pusher2.setPower(0);
        }
    }

    // ---------------- HELPERS ----------------
    private void helperControl() {
        if (gamepad1.a || gamepad1.dpad_down || gamepad2.dpad_down) {
            helper1.setPosition(0.2);
        } else {
            helper1.setPosition(1.0);
        }

        boolean rb = gamepad1.right_bumper;
        if (rb && !rightBumperWasPressed) {
            helper2.setPosition(0.1);
            sleep(400);
            helper2.setPosition(0.0);
        }
        rightBumperWasPressed = rb;
    }

    // ---------------- SHOOTER TOGGLE (Y / B) ----------------
    private void shooterToggleControl() {

        // HIGH RPM (Y)
        if (gamepad1.y && !yWasPressed) {
            shooterOn = !shooterOn;

            if (shooterOn) {
                shooterLeft.setVelocityPIDFCoefficients(P, I, D, FHigh);
                shooterLeft.setVelocity(targetTPSHigh);
            } else shooterLeft.setPower(0);
        }

        // MID RPM (B)
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn;

            if (shooterOn) {
                shooterLeft.setVelocityPIDFCoefficients(P, I, D, FLow);
                shooterLeft.setVelocity(targetTPSLow);
            } else shooterLeft.setPower(0);
        }

        // LOW RPM (X)
        if (gamepad1.x && !xWasPressed) {
            shooterOn = !shooterOn;

            if (shooterOn) {
                shooterLeft.setVelocityPIDFCoefficients(P, I, D, FLow);
                shooterLeft.setVelocity(targetTPSLow2);
            } else shooterLeft.setPower(0);
        }

        yWasPressed = gamepad1.y;
        bWasPressed = gamepad1.b;
        xWasPressed = gamepad1.x;
    }

    // ---------------- SHOOTER GAMEPAD 2 OVERRIDE ----------------
    private void shooterGamepad2Control() {
        if (gamepad2.x) shooterLeft.setVelocity(targetTPSLow2);
        if (gamepad2.y) shooterLeft.setVelocity(targetTPSLow);
        if (gamepad2.a) shooterLeft.setVelocity(targetTPSHigh);
        if (gamepad2.b) shooterLeft.setVelocity(targetTPSHigh + 200); // slight boost
    }

}
