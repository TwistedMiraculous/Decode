package org.firstinspires.ftc.teamcode.Main.motorTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Red Park", group="Linear Opmode")
public class RedAuto extends LinearOpMode {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Constants for encoders
    private static final double COUNTS_PER_MOTOR_REV = 537.6; // Rev HD Motor
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Track width in encoder ticks for a full 360-degree turn
    private static final double TRACK_WIDTH_TICKS = 3554.539012095659;

    // Optional: turn correction factor for fine tuning
    private static final double TURN_CORRECTION = 0.98;

    @Override
    public void runOpMode() throws InterruptedException {

        // Map hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Example movements
        moveForward(6);   // Move forward 12 inches
        sleep(500);
        strafeRight(12);    // Strafe right 6 inches
        sleep(500);

    }

    // ----------------- Movement Methods (inches) -----------------
    private void moveForward(double inches) { moveInches(inches, inches, inches, inches); }
    private void moveBackward(double inches) { moveInches(-inches, -inches, -inches, -inches); }
    private void strafeRight(double inches) { moveInches(inches, -inches, -inches, inches); }
    private void strafeLeft(double inches) { moveInches(-inches, inches, inches, -inches); }

    private void moveInches(double flInches, double frInches, double blInches, double brInches) {
        int flTicks = (int)(flInches * COUNTS_PER_INCH);
        int frTicks = (int)(frInches * COUNTS_PER_INCH);
        int blTicks = (int)(blInches * COUNTS_PER_INCH);
        int brTicks = (int)(brInches * COUNTS_PER_INCH);
        moveTicks(flTicks, frTicks, blTicks, brTicks, 0.5);
    }

    // ----------------- Turn Methods (degrees) -----------------
    private void turnRight(double degrees) {
        int ticks = (int)(degrees / 360.0 * TRACK_WIDTH_TICKS * TURN_CORRECTION);
        moveTicks(ticks, -ticks, ticks, -ticks, 0.35);
    }

    private void turnLeft(double degrees) {
        int ticks = (int)(degrees / 360.0 * TRACK_WIDTH_TICKS * TURN_CORRECTION);
        moveTicks(-ticks, ticks, -ticks, ticks, 0.35);
    }

    // ----------------- Core Method (ticks) -----------------
    private void moveTicks(int flTicks, int frTicks, int blTicks, int brTicks, double power) {
        frontLeft.setTargetPosition(flTicks + frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(frTicks + frontRight.getCurrentPosition());
        backLeft.setTargetPosition(blTicks + backLeft.getCurrentPosition());
        backRight.setTargetPosition(brTicks + backRight.getCurrentPosition());

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while(opModeIsActive() &&
                frontLeft.isBusy() &&
                frontRight.isBusy() &&
                backLeft.isBusy() &&
                backRight.isBusy()) {
            // wait until all motors reach their target
        }

        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
