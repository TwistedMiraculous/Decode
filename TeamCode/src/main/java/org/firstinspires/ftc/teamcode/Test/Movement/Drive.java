package org.firstinspires.ftc.teamcode.Test.Movement;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "Drive Only", group = "Concept")
public class Drive extends LinearOpMode {


    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {


        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Robot Ready!");
        telemetry.update();

        // Wait until PLAY is pressed
        waitForStart();

        // Run until STOP is pressed
        while (opModeIsActive()) {

            // Left stick Y controls forward and backward
            double y = -gamepad1.left_stick_y;

            // Left stick X controls left and right strafing
            double x = gamepad1.left_stick_x;

            // Right stick X controls turning left and right
            double rx = gamepad1.right_stick_x;

            // Set each motor power directly
            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;

            // Send power to the motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Show motor power on the driver station
            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Front Right", frontRightPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Back Right", backRightPower);
            telemetry.update();
        }
    }
}
