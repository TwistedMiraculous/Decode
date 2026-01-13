package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PS4 Rumble Test", group="Tests")
public class Rumble extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            handleFaceButtons();
            handleDpad();
            handleBumpers();
            handleTriggers();
            handleStickButtons();
            handleStickMovement();

            sleep(20); // Small delay to avoid spamming rumble commands
        }
    }

    private void handleFaceButtons() {
        if (gamepad1.cross) {
            gamepad1.rumble(1.0, 1.0, 500); // Both motors full power
        }
        if (gamepad1.circle) {
            gamepad1.rumble(1.0, 0.0, 300); // Left only
        }
        if (gamepad1.square) {
            gamepad1.rumble(0.0, 1.0, 300); // Right only
        }
        if (gamepad1.triangle) {
            gamepad1.rumble(0.5, 0.5, 1000); // Both half power
        }
    }

    private void handleDpad() {
        if (gamepad1.dpad_up) {
            gamepad1.rumble(1.0, 0.5, 200);
        }
        if (gamepad1.dpad_down) {
            gamepad1.rumble(0.5, 1.0, 200);
        }
        if (gamepad1.dpad_left) {
            gamepad1.rumble(0.8, 0.2, 200);
        }
        if (gamepad1.dpad_right) {
            gamepad1.rumble(0.2, 0.8, 200);
        }
    }

    private void handleBumpers() {
        if (gamepad1.left_bumper) {
            gamepad1.rumble(0.8, 0.2, 400);
        }
        if (gamepad1.right_bumper) {
            gamepad1.rumble(0.2, 0.8, 400);
        }
    }

    private void handleTriggers() {
        if (gamepad1.left_trigger > 0.1) {
            float intensity = gamepad1.left_trigger;
            gamepad1.rumble(intensity, intensity, 200);
        }
        if (gamepad1.right_trigger > 0.1) {
            float intensity = gamepad1.right_trigger;
            gamepad1.rumble(intensity, intensity, 200);
        }
    }

    private void handleStickButtons() {
        if (gamepad1.left_stick_button) {
            gamepad1.rumble(1.0, 0.5, 300);
        }
        if (gamepad1.right_stick_button) {
            gamepad1.rumble(0.5, 1.0, 300);
        }
    }

    private void handleStickMovement() {
        // Left stick movement
        if (Math.abs(gamepad1.left_stick_x) > 0.5 || Math.abs(gamepad1.left_stick_y) > 0.5) {
            float magnitude = (float) Math.min(1.0, (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y)) / 2);
            gamepad1.rumble(magnitude, magnitude, 100);
        }
        // Right stick movement
        if (Math.abs(gamepad1.right_stick_x) > 0.5 || Math.abs(gamepad1.right_stick_y) > 0.5) {
            float magnitude = (float) Math.min(1.0, (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y)) / 2);
            gamepad1.rumble(magnitude, magnitude, 100);
        }
    }
}
