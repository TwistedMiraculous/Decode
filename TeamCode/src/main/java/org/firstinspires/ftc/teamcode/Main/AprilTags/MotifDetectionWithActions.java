package org.firstinspires.ftc.teamcode.Main.AprilTags;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous(name = "Motif Auto Only", group = "Autonomous")
public class MotifDetectionWithActions extends LinearOpMode {

    // Use webcam for AprilTag detection
    private static final boolean USE_WEBCAM = true;

    // AprilTag processor and camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Motif detection flags
    private boolean motifDetected = false;  // True once a motif tag is detected
    private int motifID = -1;               // Stores the detected motif ID

    // Robot hardware
    private MecanumDrive drive;   // Road Runner drive
    private DcMotor shooterLeft;
    private DcMotor intake;
    private CRServo pusher1, pusher2;
    private Servo helper2;

    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pusher1 = hardwareMap.get(CRServo.class, "pusher1");
        pusher2 = hardwareMap.get(CRServo.class, "pusher2");
        pusher2.setDirection(CRServo.Direction.REVERSE);  // Reverse one pusher so both spin same way
        helper2 = hardwareMap.get(Servo.class, "helper2");

        // --- Initialize AprilTag processor and camera ---
        initAprilTag();

        // --- Initialize Road Runner drive ---
        Pose2d startPose = new Pose2d(0, 0, 0);  // Starting position on the field
        drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Starting camera for motif detection...");
        telemetry.update();

        // Start camera streaming immediately for autonomous
        visionPortal.resumeStreaming();

        waitForStart();

        // --- Detect motif tag only once ---
        while (opModeIsActive() && !motifDetected) {
            List<AprilTagDetection> detections = aprilTag.getDetections(); // Get current tag detections
            for (AprilTagDetection detection : detections) {

                // Check if the detected tag is one of the motifs (21, 22, 23)
                if (detection.id == 23 || detection.id == 22 || detection.id == 21) {
                    motifDetected = true;  // Stop detecting after first motif
                    motifID = detection.id; // Store detected motif ID

                    // Map the motif ID to a name for telemetry
                    String motifName = "";
                    switch (motifID) {
                        case 23: motifName = "PPG"; break;
                        case 22: motifName = "PGP"; break;
                        case 21: motifName = "GPP"; break;
                    }

                    telemetry.addLine("Motif detected: " + motifName);
                    telemetry.update();
                    break; // Exit the loop since motif is detected
                }
            }
        }

        // Stop camera streaming to save CPU
        visionPortal.stopStreaming();

        // --- Execute odometry path based on detected motif ---
        if (motifDetected) {
            switch (motifID) {
                case 23: // PPG motif path
                    telemetry.addLine("Running PPG path");
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))  // Start at initial pose
                                    .lineToX(30)  // Move to X=30
                                    .lineToY(30)  // Move to Y=30
                                    .build()
                    );
                    break;

                case 22: // PGP motif path
                    telemetry.addLine("Running PGP path");
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                    .lineToX(15)
                                    .lineToY(45)
                                    .build()
                    );
                    break;

                case 21: // GPP motif path
                    telemetry.addLine("Running GPP path");
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                    .lineToX(40)
                                    .lineToY(10)
                                    .build()
                    );
                    break;
            }
        }

        telemetry.addLine("Autonomous finished");
        telemetry.update();
    }

    /**
     * Initialize the AprilTag processor and the webcam
     */
    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults(); // Create processor with default parameters

        // Only using webcam; pass hardwareMap webcam name
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        // Ensure camera is off before starting auto
        visionPortal.stopStreaming();
    }

    // --- Helper methods for robot hardware ---

    private void intakeOn() {
        intake.setPower(1);
    }
    private void intakeOff() {
        intake.setPower(0);
    }
    private void pushersForward() {
        pusher1.setPower(1); pusher2.setPower(1);
    }
    private void pushersStop() {
        pusher1.setPower(0); pusher2.setPower(0);
    }
    private void helper2Up() {
        helper2.setPosition(0.1);
    }
    private void helper2Down() {
        helper2.setPosition(0.0);
    }
    private void shooterOn(double power) {
        shooterLeft.setPower(power);
    }
    private void shooterOff() {
        shooterLeft.setPower(0);
    }
}
