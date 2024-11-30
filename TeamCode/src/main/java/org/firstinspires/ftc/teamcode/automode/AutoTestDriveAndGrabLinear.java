package org.firstinspires.ftc.teamcode.automode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.DriveAndArm2;

import java.util.concurrent.TimeUnit;

@Autonomous()
public class AutoTestDriveAndGrabLinear extends LinearOpMode {
    DriveAndArm2 drive = new DriveAndArm2();
    int state;
    double currentPosition = drive.sliderPot2.getVoltage();


    @Override
    public void runOpMode() throws InterruptedException  {
        drive.setSliderPosition( 1, currentPosition);
        drive.setClawServoPosition(0.83);
        telemetry.addData("Autonomous", "Finished");
    }




}
