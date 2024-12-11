package org.firstinspires.ftc.teamcode.automode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.mechanisms.AutoConfig;

@Autonomous()
public class AutoParkRight extends OpMode{
    AutoConfig autodrive = new AutoConfig();
    int state;
    int sliderPosition;
    String stateOp;
    boolean robotCentric = false;
    //boolean slowMode = false;
    double forward = 0.0;
    double right = 0.0;
    double rotate = 0.0;
    double timeSec = 0.0;
    double sliderPower;
    double currentPosition;

    @Override
    public void init () {
        autodrive.init(hardwareMap);
        robotCentric = false;
        autodrive.setClawServoPosition(autodrive.clawPositionClosed);
    }

    @Override
    public void start() {
        state = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("Auto State: ",state);
        autodrive.setClawServoPosition(autodrive.clawPositionClosed);
        autodrive.setRightWristServoPosition(autodrive.wristPositionMid);
        switch (state) {
            case 0:
                forward = 0.0;
                right = 1.0;
                rotate = 0.0;
                timeSec = 0.75;
                break;
        }    }   }