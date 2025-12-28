package org.firstinspires.ftc.teamcode.Main.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Main.Auto.RobotHardware;
import org.firstinspires.ftc.teamcode.Main.Auto.AprilTagPoseEstimator;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 * RB_Auto_Corrected
 * -----------------
 * Autonomous routine that:
 * 1) Drives using Road Runner + Pinpoint
 * 2) Shoots preloads
 * 3) Uses AprilTags mid-auto to correct pose drift
 * 4) Continues the rest of the path from the corrected pose
 */
@Autonomous(name = "RB_Auto_Corrected")
public final class RB extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------------------------------------------------------
        // INITIAL SETUP
        // ---------------------------------------------------------------------

        // Starting pose on the FTC field (inches, radians)
        Pose2d beginPose = new Pose2d(-63, 12, 0);

        // Initialize robot hardware (motors, servos, shooter, intake, etc.)
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        // Safety check to make sure the correct drive class is being used
        if (!MecanumDrive.class.isAssignableFrom(TuningOpModes.DRIVE_CLASS)) {
            throw new RuntimeException("Drive class mismatch");
        }

        // Initialize Road Runner mecanum drive with starting pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Initialize AprilTag pose estimator using the webcam
        AprilTagPoseEstimator aprilTagEstimator =
                new AprilTagPoseEstimator(hardwareMap, "Webcam 1");

        waitForStart();
        if (isStopRequested()) return;

        // Keeps track of the robot's pose between action sequences
        Pose2d lastPose = beginPose;

        // ---------------------------------------------------------------------
        // 1. FIRST SEGMENT: INITIAL MOVE + SHOOTING
        // ---------------------------------------------------------------------

        // Drive forward and turn toward the shooting angle
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-50, 12))
                        .turn(Math.toRadians(20))
                        .build()
        );

        // Update pose using Road Runner + AprilTag-enhanced localizer
        lastPose = drive.localizer.getPose();


        // Spin up shooter
        robot.setShooterHighRPM();
        sleep(2400); // Allow shooter to reach target RPM

        // Feed rings
        robot.moveHelper2(0.0);
        robot.moveHelper2(0.1);
        sleep(600);

        // Stop shooter after firing
        robot.stopShooter();

        sleep(1000);

        // ---------------------------------------------------------------------
        // 2. SECOND SEGMENT: MOVE TO TAG VIEWING POSITION
        // ---------------------------------------------------------------------

        // Turn and strafe to a location where AprilTag 21 should be visible
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .turn(Math.toRadians(-110))
                        .strafeToConstantHeading(new Vector2d(-43, 12))
                        .build()
        );

        // Pose estimate before correction (pure odometry)
        lastPose = drive.localizer.getPose();


        // Intake / mechanism actions while stationary
        robot.intakeForward1();
        robot.pusherLeft();
        sleep(1500);

        // ---------------------------------------------------------------------
        // 3. APRILTAG RELOCALIZATION (POSE CORRECTION)
        // ---------------------------------------------------------------------

        telemetry.addLine("Attempting AprilTag Relocalization (Tag 21)...");
        telemetry.update();

        // Get corrected field pose from AprilTag detections
        Pose2d correctedPose = aprilTagEstimator.getRobotFieldPose();

        if (correctedPose != null) {
            // Override Road Runner / Pinpoint pose with AprilTag-based pose
            lastPose = drive.localizer.getPose();


            telemetry.addData(
                    "Correction Success",
                    "New Pose: (%.2f, %.2f, %.2f)",
                    correctedPose.position.x,
                    correctedPose.position.y,
                    Math.toDegrees(correctedPose.heading.log())
            );

            // Use corrected pose for future trajectories
            lastPose = correctedPose;
        } else {
            // Fall back to odometry if no reliable tag was seen
            telemetry.addLine(
                    "Correction Failed: No reliable tags found. Continuing with Pinpoint."
            );
            lastPose = drive.localizer.getPose();

        }

        telemetry.update();
        sleep(200);

        // Turn off camera stream once correction is complete
        aprilTagEstimator.stopStreaming();

        // ---------------------------------------------------------------------
        // 4. THIRD SEGMENT: LONG STRAFE USING CORRECTED POSE
        // ---------------------------------------------------------------------

        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-43, 64))
                        .build()
        );

        lastPose = drive.localizer.getPose();

        sleep(500);

        // Minor adjustment
        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-43, 58))
                        .build()
        );

        lastPose = drive.localizer.getPose();


        // ---------------------------------------------------------------------
        // 5. FINAL PARKING SEQUENCE
        // ---------------------------------------------------------------------

        Actions.runBlocking(
                drive.actionBuilder(lastPose)
                        .strafeToConstantHeading(new Vector2d(-40, 12))
                        .strafeToConstantHeading(new Vector2d(-50, 12))
                        .turn(Math.toRadians(105))
                        .build()
        );

        // Stop intake and finalize auto
        robot.intakeStop();
    }
}
