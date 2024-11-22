package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.PotentiometerTest;

@TeleOp()
public class PotTestOpMode extends OpMode {
    PotentiometerTest drive = new PotentiometerTest();
    int state;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double motorTowersdgf = gamepad2.right_stick_y;

        telemetry.addData("Pot Angle", drive.getPotAngle());
        telemetry.addData("Right stick x", gamepad2.right_stick_x);
        telemetry.addData("Right stick y", gamepad2.right_stick_y);
        telemetry.addData("A button", gamepad2.a);
        telemetry.addData("B button", gamepad2.b);
        telemetry.addData("X button", gamepad2.x);
        telemetry.addData("Y button", gamepad2.y);

        drive.setRotateSpeed(motorTowersdgf);

        if (drive.getPotAngle() > 90) {
            state = 3;
        }
    }
}
