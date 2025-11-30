package org.firstinspires.ftc.teamcode.Main.motorTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
@TeleOp(name = "Motor Test", group = "Concept")

public class motor extends LinearOpMode {
    DcMotor motor;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {


        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.x) {

                    motor.setPower(1);

                } else if (gamepad1.b) {

                    motor.setPower(-1);

                }
                else if (gamepad1.a){

                    motor.setPower(0.5);
                }
                else {

                    motor.setPower(0);

                }
            }
        }
    }
}
