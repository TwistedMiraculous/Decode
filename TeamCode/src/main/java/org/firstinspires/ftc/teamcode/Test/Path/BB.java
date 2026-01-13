package org.firstinspires.ftc.teamcode.Test.Path;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Test.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public final class BB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, 12, 0);
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        if (!MecanumDrive.class.isAssignableFrom(TuningOpModes.DRIVE_CLASS)) {
            throw new RuntimeException("Drive class mismatch");
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        Pose2d lastPose = beginPose;

        // First segment: strafe and turn
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-50, 12))
                        .turn(Math.toRadians(20))
                        .build()
        );

        lastPose = new Pose2d(-50, 12, Math.toRadians(20));

        // Start shooter
        robot.setShooterHighRPM();
        sleep(2400);
        robot.moveHelper2(0.0);
        sleep(600); // changed from 1500 -> 600
        robot.moveHelper2(0.1);
        sleep(500);
        robot.moveHelper2(0.0);
        sleep(600);
        robot.moveHelper2(0.1);
        sleep(500);
        robot.moveHelper2(0.0);
        robot.intakeForward1();
        robot.pusherLeft();
        sleep(1000);
        robot.moveHelper2(0.1);
        sleep(500);
        robot.setShooterNegRPM();
        robot.moveHelper2(0.0);
        sleep(250);
        robot.intakeStop();
        robot.pusherStop();
        robot.stopShooter();

        sleep(1000); // 2700 -> 1000

        // Second segment: follow the rest of the path
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .turn(Math.toRadians(-110))
                        .strafeToConstantHeading(new Vector2d(-43, 12))
                        .build()
        );

        lastPose = new Pose2d(-43, 12, Math.toRadians(-90));

        robot.intakeForward1();
        robot.pusherLeft();
        sleep(1500); // 2500 -> 1500


        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-43, 64))
                        .build()
        );
        lastPose = new Pose2d(-43, 57, Math.toRadians(-90));

        sleep(500);

        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-43, 58))
                        .build()
        );


        lastPose = new Pose2d(-43, 62, Math.toRadians(-90));

        sleep(1400);
        robot.pusherStop();
        robot.intakeStop();

        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-40, 12))
                        .strafeToConstantHeading(new Vector2d(-50, 12))
                        .turn(Math.toRadians(105))
                        .build()
        );
        lastPose = new Pose2d(-43, 62, Math.toRadians(-90));


        // Start shooter
        robot.pusherRight();
        robot.setShooterHighRPM();
        sleep(2000);
        robot.moveHelper2(0.0);
        sleep(10);
        robot.moveHelper2(0.1);
        sleep(600);
        robot.moveHelper2(0.0);
        sleep(600);
        robot.moveHelper2(0.1);
        sleep(600);
        robot.moveHelper2(0.0);
        robot.intakeForward1();
        robot.pusherLeft();
        sleep(1000);
        robot.moveHelper2(0.1);
        sleep(600);
        robot.stopShooter();
        robot.pusherStop();
        robot.moveHelper2(0.0);
        robot.intakeStop();
    }
}