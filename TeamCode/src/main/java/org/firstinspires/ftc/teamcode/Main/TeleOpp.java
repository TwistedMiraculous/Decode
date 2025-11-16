package org.firstinspires.ftc.teamcode.Main;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp (name = "Test1", group = "StarterBottest")
public class TeleOpp extends OpMode {

    final double FEED_TIME_SECONDS = 0.20;
    final double LAUNCHER_TARGET_VELOCITY = 95;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    double leftPower;
    double rightPower;

    final double TICKS_PER_REV = 28; // Adjust if using a different motor

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Launcher control
        if (gamepad1.y) {
            launcher.setPower(0.85);
        } else if (gamepad1.b) {
            launcher.setPower(0);
        }

        // Feeder control
        if (gamepad1.right_bumper) {
            leftFeeder.setPower(-1);
            rightFeeder.setPower(-1);
        } else if (gamepad1.dpad_up) {
            leftFeeder.setPower(1);
            rightFeeder.setPower(1);
        } else {
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }

        // Calculate launcher RPM
        double launcherRPM = (launcher.getVelocity() / TICKS_PER_REV) * 60;

        // Telemetry
        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Launcher RPM", launcherRPM);
        telemetry.update();
    }

    @Override
    public void stop() {}

    void arcadeDrive(double forward, double rotate) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
