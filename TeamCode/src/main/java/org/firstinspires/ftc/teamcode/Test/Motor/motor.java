package org.firstinspires.ftc.teamcode.Test.Motor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class motor extends LinearOpMode {
    DcMotor motor;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {


        motor = hardwareMap.get(DcMotor.class, "motorTurret");
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.x) {

                    motor.setPower(1);

                } else if (gamepad1.y) {

                    motor.setPower(-1);

                }
                else if (gamepad1.a){

                    motor.setPower(0.5);
                }
                else if (gamepad1.b) {

                    motor.setPower(0);

                }
            }
        }
    }
}
