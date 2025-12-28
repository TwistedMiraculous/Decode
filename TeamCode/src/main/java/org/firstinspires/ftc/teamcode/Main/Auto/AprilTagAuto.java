package org.firstinspires.ftc.teamcode.Main.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Main.Auto.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "BB + AprilTag Dynamic Box")
public final class AprilTagAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final int TAG_BOX = 21; // Box location tag

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-63, 12, 0);
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Initialize AprilTag vision
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

        telemetry.addLine("Looking for box AprilTag (ID 21)...");
        telemetry.update();

        waitForStart();

        Pose2d lastPose = beginPose;

        // --- First segment: initial movement and turn
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-50, 12))
                        .turn(Math.toRadians(20))
                        .build()
        );
        lastPose = new Pose2d(-50, 12, Math.toRadians(20));

        // Shooter sequence
        robot.setShooterHighRPM();
        sleep(2400);
        robot.moveHelper2(0.0);
        sleep(600);
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
        sleep(1000);

        // --- Second segment: move toward scoring area
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .turn(Math.toRadians(-110))
                        .strafeToConstantHeading(new Vector2d(-43, 12))
                        .build()
        );
        lastPose = new Pose2d(-43, 12, Math.toRadians(-90));

        robot.intakeForward1();
        robot.pusherLeft();
        sleep(1500);

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

        // --- Dynamic Box Tag Check ---
        List<AprilTagDetection> boxDetections = aprilTag.getDetections();
        boolean boxDetected = false;
        for (AprilTagDetection detection : boxDetections) {
            if (detection.id == TAG_BOX) {
                telemetry.addData("Box tag detected!", detection.id);
                telemetry.update();
                boxDetected = true;
                break;
            }
        }

        // Adjust final strafe depending on box
        if (boxDetected) {
            Actions.runBlocking(
                    drive.actionBuilder(lastPose)
                            .strafeToConstantHeading(new Vector2d(-43, 70)) // correct to box
                            .build()
            );
            lastPose = new Pose2d(-43, 70, Math.toRadians(-90));
        }

        // Final movement & shooting sequence
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-40, 12))
                        .strafeToConstantHeading(new Vector2d(-50, 12))
                        .turn(Math.toRadians(105))
                        .build()
        );

        lastPose = new Pose2d(-43, 62, Math.toRadians(-90));

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

        // Close camera
        visionPortal.close();
    }
}
