package org.firstinspires.ftc.teamcode.Main.AprilTags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled
@TeleOp(name = "AprilTag ID 21 Tracker", group = "Vision")
public class Localization extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final double MIN_POWER = 0.1;
    private static final double MAX_POWER = 0.7;
    private static final double STOP_DISTANCE = 6.0;   // inches
    private static final double MAX_DISTANCE = 36.0;   // inches

    private static final double FIELD_SIZE = 72.0;     // half of 12x12ft field
    private static final int TARGET_ID = 21;           // Focus on GPP tag

    private DcMotor motorForward;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        motorForward = hardwareMap.get(DcMotor.class, "motor");
        motorForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 30);

        telemetry.addData(">", "Press START to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean foundTag21 = false;

            // Look for Tag ID 21
            for (AprilTagDetection detection : detections) {
                if (detection.id == TARGET_ID && detection.metadata != null) {
                    foundTag21 = true;

                    double distance = detection.ftcPose.range;
                    double x = detection.ftcPose.x;
                    double y = detection.ftcPose.y;
                    double heading = detection.ftcPose.yaw;

                    double power = calculatePower(distance);
                    motorForward.setPower(power);

                    packet.put("Tracking Tag ID", TARGET_ID);
                    packet.put("Distance (inch)", distance);
                    packet.put("X (inch)", x);
                    packet.put("Y (inch)", y);
                    packet.put("Yaw (deg)", heading);
                    packet.put("Motor Power", power);

                    drawFieldLayout(field);

                    // Draw robot position relative to field
                    field.setStroke("red");
                    field.fillCircle(FIELD_SIZE + x, FIELD_SIZE + y, 3);

                    telemetry.addData("Tracking Tag ID", TARGET_ID);
                    telemetry.addData("Distance", "%.1f in", distance);
                    telemetry.addData("Power", "%.2f", power);
                    telemetry.addData("X", "%.1f", x);
                    telemetry.addData("Y", "%.1f", y);
                    telemetry.addData("Yaw", "%.1f", heading);
                    break; // Stop after finding the target tag
                }
            }

            // If Tag ID 21 not visible
            if (!foundTag21) {
                motorForward.setPower(0);
                telemetry.addLine("Tag ID 21 not visible");
                packet.put("Tag ID 21 Visible", false);
                drawFieldLayout(field);
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        visionPortal.close();
    }

    private double calculatePower(double distance) {
        if (distance <= STOP_DISTANCE) return 0.0;
        double scale = (distance - STOP_DISTANCE) / (MAX_DISTANCE - STOP_DISTANCE);
        scale = Math.min(Math.max(scale, 0.0), 1.0);
        return MIN_POWER + (MAX_POWER - MIN_POWER) * scale;
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    /** Draws field outline and known tag positions. */
    private void drawFieldLayout(Canvas field) {
        field.setStrokeWidth(1);
        field.setStroke("gray");
        field.strokeRect(0, 0, FIELD_SIZE * 2, FIELD_SIZE * 2);

        // Field tags (approx. 2025–26 layout)
        drawTag(field, 36, 36, "blue", "Blue Box", 20);
        drawTag(field, 72, 36, "gold", "GPP", 21);
        drawTag(field, 108, 36, "green", "PGP", 22);
        drawTag(field, 72, 108, "purple", "PPG", 23);
        drawTag(field, 108, 108, "red", "Red Box", 24);
    }

    private void drawTag(Canvas field, double x, double y, String color, String label, int id) {
        field.setStroke(color);
        field.strokeCircle(x, y, 3);
        // Note: FTC Dashboard Canvas does not support text, so we skip label drawing here.
        // You can display tag labels using telemetry or packet.put() instead.
    
    }
}
