package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DriveAndArm;


@TeleOp
public class LiftMotorTest extends OpMode {
    DriveAndArm drive = new DriveAndArm();

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
            drive.setMotorSpeed(0.5);
        } else {
            drive.setMotorSpeed(0.0);
        }
        telemetry.addData("Motor rotations", drive.getMotorRotations());
        telemetry.addData("Gamepad 1 B", gamepad1.b);
    }

}


