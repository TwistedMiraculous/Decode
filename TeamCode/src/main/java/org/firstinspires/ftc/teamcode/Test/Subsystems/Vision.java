package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class Vision {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public Vision(HardwareMap hardwareMap) {
        // 1. Define your 100mm tag (100mm = 3.937 inches)
        // This is crucial for accurate distance and centering
        AprilTagLibrary myTagLibrary = new AprilTagLibrary.Builder()

                //Todo: Change Tag ID to the box ID
                .addTag(19, "My100mmTag", 3.937, DistanceUnit.INCH)
                .build();

        // 2. Create the AprilTag logic with your custom library
// These values are standard for a Logitech C920 at 640x480
// fx, fy, cx, cy
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(myTagLibrary)
                .setLensIntrinsics(622.548, 622.548, 319.5, 239.5) // ADD THIS LINE
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();;

        // 3. Create the Engine (VisionPortal)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        // 4. Send the feed to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    /**
     * Returns the horizontal center of the first detected tag.
     * At 640x480 resolution, 320 is the exact center.
     */
    public double getTagX() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Only return data if we have a valid detection
            if (detection.metadata != null) {
                return detection.center.x;
            }
        }
        return -1; // No tag found
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}