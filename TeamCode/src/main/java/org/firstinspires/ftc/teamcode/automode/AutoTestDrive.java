package org.firstinspires.ftc.teamcode.automode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.FieldCentricOmniBot;
import org.firstinspires.ftc.teamcode.mechanisms.DriveAndArm;

import java.util.concurrent.TimeUnit;

@Autonomous()
public class AutoTestDrive extends LinearOpMode {
    FieldCentricOmniBot drive = new FieldCentricOmniBot();
    int state;



    @Override
    public void runOpMode() throws InterruptedException  {
        //Initializes Hardware map
        drive.init(hardwareMap);
        //Starts driving for one second
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




}
