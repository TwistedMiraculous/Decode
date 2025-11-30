package org.firstinspires.ftc.teamcode.Main.motorTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@TeleOp(name = "Servo Test", group = "Concept")
public class ServoTest extends LinearOpMode {

    private Servo servo;
    private CRServo crServo;

    @Override
    public void runOpMode() {
        // Initialize servos
        servo = hardwareMap.get(Servo.class, "servo");
        crServo = hardwareMap.get(CRServo.class, "crServo");

        telemetry.addLine("Servo Test Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Control CRServo with button A and stop with X
            if (gamepad1.a) {
                crServo.setPower(1); // spin forward
            } else if (gamepad1.x) {
                crServo.setPower(-1); // spin backward
            } else {
                crServo.setPower(0); // stop
            }

            // Control regular servo with button B
            if (gamepad1.b) {
                servo.setPosition(0.5); // move to mid position
            } else if (gamepad1.y) {
                servo.setPosition(1.0); // move to max position
            }

            telemetry.addData("CRServo Power", crServo.getPower());
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
