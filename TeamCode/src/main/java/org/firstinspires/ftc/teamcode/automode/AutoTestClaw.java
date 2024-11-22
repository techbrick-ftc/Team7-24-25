package org.firstinspires.ftc.teamcode.automode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AutoConfig;

import java.util.concurrent.TimeUnit;

@Autonomous()
public class AutoTestClaw extends OpMode {
    AutoConfig drive = new AutoConfig();
    int state;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void start() {
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        if (state == 0)  {
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
            telemetry.addData("Autonomous", "Finished");
    }
}}
