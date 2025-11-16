package org.firstinspires.ftc.teamcode.Main.Helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class FieldCentric extends LinearOpMode {

    private DcMotor shooterLeft;
    private DcMotor intake;

    private CRServo pusher1, pusher2;
    private Servo helper1, helper2;
    private boolean intakeOn = false;
    private boolean leftTriggerPreviouslyPressed = false;

    @Override

    public void runOpMode() throws InterruptedException {
        // Config Drive
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");



        // Shooter + intake
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        intake      = hardwareMap.get(DcMotor.class, "intake");

        // Servos
        pusher1 = hardwareMap.get(CRServo.class, "pusher1");
        pusher2 = hardwareMap.get(CRServo.class, "pusher2");
        helper1 = hardwareMap.get(Servo.class, "helper1");
        helper2 = hardwareMap.get(Servo.class, "helper2");


        // Reverse the right side motors. This may be wrong for your setup.

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher2.setDirection(CRServo.Direction.REVERSE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        telemetry.addLine("Robot Ready!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();

            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


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

    /** Ball Launcher */
    private void shooterControl() {


        double speed1 = 0.6;
        double speed2 = 0.65;
        double speed3 = 0.7;
        double speed4 = 0.75;
        double speed5 = 0.8;
        double speed6 = 8.5;
        double speed7 = 0.9;
        double speed8 = 1.0;


        if (gamepad1.right_trigger > 0.1) {
            shooterLeft.setPower(0.7);       // full speed

        } else {
            shooterLeft.setPower(0);       // stop
        }



        if (gamepad2.dpad_left) {
            shooterLeft.setPower(speed1);
        }
        else if (gamepad2.dpad_up) {
            shooterLeft.setPower(speed2);
        }
        else if (gamepad2.dpad_right) {
            shooterLeft.setPower(speed3);
        }
        else if (gamepad2.dpad_down) {
            shooterLeft.setPower(speed4);
        }
        else if (gamepad2.x) {
            shooterLeft.setPower(speed5);
        }
        else if (gamepad2.y) {
            shooterLeft.setPower(speed6);
        }
            else if (gamepad2.b) {
                shooterLeft.setPower(speed7);
            }
            else if (gamepad2.a) {
                shooterLeft.setPower(speed8);
        }else {
            shooterLeft.setPower(0);       // stop
        }



    }

    /** Intake */
    private void intakeControl() {

        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
        if (leftTriggerPressed && !leftTriggerPreviouslyPressed) {
            intakeOn = !intakeOn;
        }
        if (intakeOn) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        leftTriggerPreviouslyPressed = leftTriggerPressed;




    }


    /** Intake: CR Servos to push ball inwards after intake */
    private void pusherControl() {
        if (gamepad1.dpad_left) {
            pusher1.setPower(-1);
            pusher2.setPower(-1);
        } else if (gamepad1.dpad_right) {
            pusher1.setPower(1);
            pusher2.setPower(1);
        } else {
            pusher1.setPower(0);
            pusher2.setPower(0);
        }

        if (gamepad2.left_bumper) {
            pusher1.setPower(-1);
            pusher2.setPower(-1);
        } else if (gamepad2.right_bumper) {
            helper1.setPosition(0.2);
        } else {
            pusher1.setPower(0);
            pusher2.setPower(0);
            helper1.setPosition(1.0);
        }
    }

    /** Third Leg Servo (help for intake - helper1) & Kick Up Servo (kicks ball for launch)*/
    private void helperControl() {

        // Helper 1 on button A
        if (gamepad1.a) {
            helper1.setPosition(0.2);
        } else {
            helper1.setPosition(1.0);
        }

        // Helper 2 on D-pad Up
        if (gamepad1.right_bumper) {
            helper2.setPosition(0.1);
        } else {
            helper2.setPosition(0.0);
        }
    }
}