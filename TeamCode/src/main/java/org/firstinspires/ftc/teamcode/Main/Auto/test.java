package org.firstinspires.ftc.teamcode.Main.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Disabled
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public final class test extends LinearOpMode {

    // Action Vars
    public DcMotor shooterLeft;
    private DcMotor intake;
    private CRServo pusher1, pusher2;
    private Servo helper2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Start at the center of the field
        Pose2d beginPose = new Pose2d(0, 0, 0);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            // Init Hardware
            shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
            intake = hardwareMap.get(DcMotor.class, "intake");
            pusher1 = hardwareMap.get(CRServo.class, "pusher1");
            pusher2 = hardwareMap.get(CRServo.class, "pusher2");
            pusher2.setDirection(CRServo.Direction.REVERSE);
            helper2 = hardwareMap.get(Servo.class, "helper2");

            waitForStart();

            // Run your square path
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToX(30)
                            .turn(Math.toRadians(90))
                            .lineToY(30)
                            .turn(Math.toRadians(90))
                            .lineToX(0)
                            .turn(Math.toRadians(90))
                            .lineToY(0)
                            .turn(Math.toRadians(90))
                            .build()
            );

        } else {
            throw new RuntimeException();
        }
    }

    private void intakeOn() { intake.setPower(1); }
    private void intakeOff() { intake.setPower(0); }

    private void pushersForward() {
        pusher1.setPower(1);
        pusher2.setPower(1);
    }

    private void pushersStop() {
        pusher1.setPower(0);
        pusher2.setPower(0);
    }

    private void helper2Up() { helper2.setPosition(0.1); }
    private void helper2Down() { helper2.setPosition(0.0); }

    private void shooterOn(double power) { shooterLeft.setPower(power); }
    private void shooterOff() { shooterLeft.setPower(0); }

}
