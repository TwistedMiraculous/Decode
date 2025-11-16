package org.firstinspires.ftc.teamcode.Main;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
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
@TeleOp(name = "ATest2 ", group = "StarterBottest")
public class Test2 extends OpMode {

    public static double FEED_TIME_SECONDS = 0.20;
    public static double LAUNCHER_MIN_VELOCITY = 1075;
    public static double kP = 300;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 10;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    ElapsedTime feederTimer = new ElapsedTime();

    double leftPower;
    double rightPower;
    final double TICKS_PER_REV = 28;

    // Dashboard instance
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void init() {
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
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Dashboard
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive
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

        // Update PIDF dynamically
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        // Calculate launcher RPM
        double launcherRPM = (launcher.getVelocity() / TICKS_PER_REV) * 60;

        // ----- Dashboard Telemetry -----
        packet.put("Launcher Power", launcher.getPower());
        packet.put("Launcher RPM", launcherRPM);
        packet.put("Left Feeder Power", leftFeeder.getPower());
        packet.put("Right Feeder Power", rightFeeder.getPower());
        packet.put("Left Drive Power", leftPower);
        packet.put("Right Drive Power", rightPower);

        // Field overlay for visualization
        Canvas field = packet.fieldOverlay();
        field.setStroke("#FF0000"); // red
        field.strokeCircle(leftPower*10, rightPower*10, 2); // simple example visualization

        // Send packet to Dashboard
        dashboard.sendTelemetryPacket(packet);
    }

    void arcadeDrive(double forward, double rotate) {
        double y = forward;
        double x = 0;
        double rx = rotate;

        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        leftPower = (frontLeftPower + backLeftPower) / 2.0;
        rightPower = (frontRightPower + backRightPower) / 2.0;
    }
}
