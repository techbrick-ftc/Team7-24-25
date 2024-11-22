package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ArmTest;

@TeleOp()
public class ArmTestOpMode extends OpMode {
    ArmTest drive = new ArmTest();
    int state;
    //boolean dPadDownAlreadyPressed = false;
    //boolean toggleWristServoDirection = false;
    //boolean getWristServoDirection = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double slider = gamepad2.left_stick_y;
        double armRotate = gamepad2.right_stick_y;
        boolean gp2xAlreadyPressed = false;
        boolean gp2yAlreadyPressed = false;
        boolean clawOpen = false;
        boolean wristUp = false;
        //boolean dPadDownAlreadyPressed = false;

        drive.setSliderSpeed(slider);
        drive.setRotateSpeed(armRotate);

        /*private void checkClawOpen() {
            telemetry.addData("Claw open", clawOpen);
            if(gamepad2.y && !gp2yAlreadyPressed) clawOpen = !clawOpen;
            gp2yAlreadyPressed = gamepad2.y;

            if (clawOpen) drive.setClawServoPosition(drive.clawPositionOpen);
            else drive.setClawServoPosition(drive.clawPositionClosed);
        }

        private void checkWristUp() {
            telemetry.addData("Wrist Position", wristUp);
            if(gamepad2.x && !gp2xAlreadyPressed) wristUp = !wristUp;
            gp2xAlreadyPressed = gamepad2.x;

            if (wristUp) drive.setRightWristServoPosition(drive.wristPositionMid);
            else drive.setRightWristServoPosition(drive.wristPositionDown);
        }*/

        /*if (gamepad2.dpad_down && !dPadDownAlreadyPressed) getWristServoDirection = !getWristServoDirection; {
            dPadDownAlreadyPressed = gamepad2.dpad_down;
            drive.toggleWristServoDirection();
        }*/

        telemetry.addData("Left stick x", gamepad2.left_stick_x);
        telemetry.addData("Left stick y", gamepad2.left_stick_y);
        telemetry.addData("Right stick x", gamepad2.right_stick_x);
        telemetry.addData("Right stick y", gamepad2.right_stick_y);
        telemetry.addData("A button", gamepad2.a);
        telemetry.addData("B button", gamepad2.b);
        telemetry.addData("X button", gamepad2.x);
        telemetry.addData("Y button", gamepad2.y);
        telemetry.addData("Dpad down", gamepad2.dpad_down);
        telemetry.addData("Left bumper", gamepad2.left_bumper);
        telemetry.addData("Right bumper", gamepad2.right_bumper);
        telemetry.addData("Wrist up", wristUp);
        telemetry.addData("Claw open", clawOpen);
        telemetry.addData("Right wrist servo Position", drive.getRightWristServoPosition());
        telemetry.addData("Left wrist servo Position", drive.getLeftWristServoPosition());
        telemetry.addData("Right wrist servo Direction", drive.getRightWristServoDirection());
        telemetry.addData("Left wrist servo Direction", drive.getLeftWristServoDirection());
        telemetry.addData("Pot Angle", drive.getArmPotAngle());

        /*if (drive.getArmPotAngle() > 90) {
            state = 3;
        }*/
    }
}