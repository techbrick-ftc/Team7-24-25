package org.firstinspires.ftc.teamcode.automode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.FieldCentricOmniBot;
import org.firstinspires.ftc.teamcode.mechanisms.DriveAndArm;

import java.util.concurrent.TimeUnit;

@Autonomous()
public class AutoTestDrive extends OpMode {
    FieldCentricOmniBot drive = new FieldCentricOmniBot();
    int state;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void start() {
        drive.setDrive(1, 0, 0);
            try {
                    TimeUnit.SECONDS.sleep(1);
            } catch (InterruptedException e) {
        throw new RuntimeException(e);
            }
                    drive.setDrive(0, 0, 0);
            try {
                    TimeUnit.SECONDS.sleep(2);
            } catch (InterruptedException e) {
        throw new RuntimeException(e);
            }
                    drive.setClawServoPosition(0.83);
            try {
                    TimeUnit.SECONDS.sleep(1);
            } catch (InterruptedException e) {
        throw new RuntimeException(e);
            }
                    drive.setClawServoPosition(0);
            try {
                    TimeUnit.SECONDS.sleep(1);
            } catch (InterruptedException e) {
        throw new RuntimeException(e);
            }
                    drive.setClawServoPosition(0.5);
    }

    @Override
    public void loop() {

    }


}
