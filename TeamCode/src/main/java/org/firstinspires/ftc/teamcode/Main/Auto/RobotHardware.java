package org.firstinspires.ftc.teamcode.Main.Auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotHardware {

    // ---------------- Hardware ----------------
    public DcMotorEx shooterLeft;
    public DcMotor intake;
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public CRServo pusher1, pusher2;
    public Servo helper1, helper2;
    public IMU imu;

    // ---------------- Shooter Constants ----------------
    public final double gearRatio = 1.0;
    public final double ticksPerRev = 28 * gearRatio;

    public double targetRPMHigh = 3775;
    public double targetRPMLow  = 3400;

    public double targetRPMNeg = -3000;
    public double targetTPSNeg = 0;
    public double targetTPSHigh = 0;
    public double targetTPSLow  = 0;

    // ---------------- PIDF Coefficients ----------------
    public final double P = 60;
    public final double I = 0;
    public final double D = 6;
    public final double FHigh = 5.0;
    public final double FLow  = 5.8;

    // ---------------- State ----------------
    public boolean shooterOn = false;

    public void init(HardwareMap hwMap) {

        // Drive Motors
        frontLeftMotor = hwMap.dcMotor.get("frontLeft");
        backLeftMotor  = hwMap.dcMotor.get("backLeft");
        frontRightMotor= hwMap.dcMotor.get("frontRight");
        backRightMotor = hwMap.dcMotor.get("backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter + Intake
        shooterLeft = hwMap.get(DcMotorEx.class, "shooterLeft");
        intake      = hwMap.get(DcMotor.class, "intake");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetTPSHigh = targetRPMHigh * ticksPerRev / 60.0;
        targetTPSLow  = targetRPMLow  * ticksPerRev / 60.0;
        targetTPSNeg = targetRPMNeg * ticksPerRev / 60.0;

        // Servos
        pusher1 = hwMap.get(CRServo.class, "pusher1");
        pusher2 = hwMap.get(CRServo.class, "pusher2");
        pusher2.setDirection(CRServo.Direction.REVERSE);

        helper1 = hwMap.get(Servo.class, "helper1");
        helper2 = hwMap.get(Servo.class, "helper2");

        // IMU
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));
    }

    // ---------------- Shooter Functions ----------------
    public void setShooterHighRPM() {
        shooterLeft.setVelocityPIDFCoefficients(P, I, D, FHigh);
        shooterLeft.setVelocity(targetTPSHigh);
        shooterOn = true;
    }

    public void setShooterLowRPM() {
        shooterLeft.setVelocityPIDFCoefficients(P, I, D, FLow);
        shooterLeft.setVelocity(targetTPSLow);
        shooterOn = true;
    }

    public void setShooterNegRPM() {
        shooterLeft.setVelocityPIDFCoefficients(P, I, D, FLow);
        shooterLeft.setVelocity(targetTPSNeg);
        shooterOn = true;
    }

    public void stopShooter() {
        shooterLeft.setVelocity(0);
        shooterOn = false;
    }

    // ---------------- Intake Functions ----------------
    public void intakeForward1() {
        intake.setPower(0.5);
    }

    public void intakeForward2() {
        intake.setPower(1.0);
    }

    public void intakeReverse() {
        intake.setPower(-1);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    // ---------------- Helper Functions ----------------
    public void moveHelper1(double pos) {
        helper1.setPosition(pos);
    }

    public void moveHelper2(double pos) {
        helper2.setPosition(pos);
    }

    // ---------------- Pusher Functions ----------------
    public void pusherLeft() {
        pusher1.setPower(-1);
        pusher2.setPower(-1);
    }

    public void pusherRight() {
        pusher1.setPower(0.4);
        pusher2.setPower(0.4);
    }

    public void pusherStop() {
        pusher1.setPower(0);
        pusher2.setPower(0);
    }
}
