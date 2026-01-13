package org.firstinspires.ftc.teamcode.Test.AprilTag;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

/**
 * AprilTagPoseEstimator
 * ---------------------
 * Uses FTC AprilTag detections to compute a corrected Road Runner Pose2d
 * for the robot on the field.
 *
 * This class is designed to:
 * - Run vision continuously in the background
 * - Be called mid-auto to re-localize the robot
 * - Override odometry (Pinpoint / encoders) when drift occurs
 */
public class AprilTagPoseEstimator {

    // FTC AprilTag processor (detects tags and gives camera-relative pose)
    private AprilTagProcessor aprilTagProcessor;

    // Vision portal that manages the camera stream
    private VisionPortal visionPortal;

    // =====================================================================
    // CAMERA → ROBOT TRANSFORM (USER MUST MEASURE THESE)
    // =====================================================================
    // Road Runner axes:
    // +X = forward, +Y = left, +Heading = CCW

    // Camera position relative to robot center (in inches)
    private static final double CAMERA_X_ROBOT = 5.0; // forward of robot center
    private static final double CAMERA_Y_ROBOT = 0.0; // left of robot center
    private static final double CAMERA_H_ROBOT = 0.0; // camera rotation (radians)

    // Transform from robot → camera
    private final Pose2d txCameraRobot =
            new Pose2d(CAMERA_X_ROBOT, CAMERA_Y_ROBOT, CAMERA_H_ROBOT);

    // =====================================================================
    // APRILTAG FIELD MAP (MUST MATCH OFFICIAL FIELD COORDINATES)
    // =====================================================================

    // Only Tag 21 is used here for correction
    private static final TagMetadata[] TAG_MAP = {
            // Replace with official FTC field coordinates
            new TagMetadata(21, new Pose2d(72.0, 36.0, Math.toRadians(0)))
    };

    /** Helper container for storing known tag poses on the field */
    private static class TagMetadata {
        final int id;
        final Pose2d fieldPose; // Pose of the tag in FIELD coordinates

        TagMetadata(int id, Pose2d fieldPose) {
            this.id = id;
            this.fieldPose = fieldPose;
        }
    }

    // =====================================================================
    // CONSTRUCTOR
    // =====================================================================

    public AprilTagPoseEstimator(HardwareMap hardwareMap, String webcamName) {

        // Build AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                // For best accuracy, add .setLensIntrinsics(...)
                .build();

        // Build vision portal and attach camera
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    // =====================================================================
    // MAIN POSE ESTIMATION METHOD
    // =====================================================================

    /**
     * Computes the robot's FIELD pose using visible AprilTags.
     *
     * @return Pose2d of robot in field coordinates, or null if no valid tags
     */
    public Pose2d getRobotFieldPose() {

        // Get all current detections
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            return null;
        }

        // Accumulators for averaging multiple detections
        double sumX = 0.0;
        double sumY = 0.0;
        double sumHeading = 0.0;
        int validDetections = 0;

        for (AprilTagDetection detection : detections) {

            // Only accept Tag 21 within reasonable distance
            if (detection.id != 21 || detection.ftcPose.range > 70) {
                continue;
            }

            // Look up known field pose of this tag
            Pose2d tagFieldPose = null;
            for (TagMetadata metadata : TAG_MAP) {
                if (metadata.id == detection.id) {
                    tagFieldPose = metadata.fieldPose;
                    break;
                }
            }

            if (tagFieldPose == null) continue;

            // -------------------------------------------------------------
            // STEP 1: Convert FTC camera-relative pose → Road Runner pose
            // -------------------------------------------------------------
            // FTC axes:
            //   X = right, Y = forward, yaw = CCW
            // RR axes:
            //   X = forward, Y = left, heading = CCW

            Vector2d tagRelCam =
                    new Vector2d(detection.ftcPose.y, -detection.ftcPose.x);

            Rotation2d tagHeadingRelCam =
                    Rotation2d.fromDouble(Math.toRadians(detection.ftcPose.yaw));

            Pose2d tagPoseRelCam =
                    new Pose2d(tagRelCam, tagHeadingRelCam);

            // -------------------------------------------------------------
            // STEP 2: Compute robot pose in FIELD coordinates
            // -------------------------------------------------------------
            // txRobotField = txTagField * txRobotTag
            // txRobotTag   = txRobotCamera * txCameraTag^-1

            Pose2d txRobotCamera = txCameraRobot.inverse();
            Pose2d txRobotTag = txRobotCamera.times(tagPoseRelCam.inverse());
            Pose2d robotFieldPose = tagFieldPose.times(txRobotTag);

            // Accumulate for averaging
            sumX += robotFieldPose.position.x;
            sumY += robotFieldPose.position.y;
            sumHeading += robotFieldPose.heading.log();

            validDetections++;
        }

        // Return averaged pose if at least one valid tag was used
        if (validDetections > 0) {
            return new Pose2d(
                    sumX / validDetections,
                    sumY / validDetections,
                    sumHeading / validDetections
            );
        }

        return null;
    }

    // =====================================================================
    // CLEANUP
    // =====================================================================

    /** Stops the camera stream once vision is no longer needed */
    public void stopStreaming() {
        if (visionPortal != null &&
                visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopStreaming();
        }
    }
}
