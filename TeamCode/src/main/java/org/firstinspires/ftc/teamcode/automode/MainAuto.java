package org.firstinspires.ftc.teamcode.automode; // armRotate position start 2.144, slider position clip specimen 2.336

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.mechanisms.AutoConfig;

@Autonomous()
public class MainAuto extends OpMode{
    AutoConfig autodrive = new AutoConfig();
    private DcMotorEx armRotate;
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
    double [] out;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init () {
        autodrive.init(hardwareMap);
        robotCentric = false;
    }

    @Override
    public void start() {
        autodrive.liftMotor.setPower(1);timeSec = 2.3;
        if (runtime.seconds() > timeSec) {
            autodrive.liftMotor.setPower(0);
        }// for now
        autodrive.setClawServoPosition(autodrive.clawPositionClosed); // for now
        autodrive.setRightWristServoPosition(autodrive.wristPositionMid); // for now
        state = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("Auto State: ",state);
        autodrive.setClawServoPosition(autodrive.clawPositionClosed);
        autodrive.setRightWristServoPosition(autodrive.wristPositionMid);
        runtime.reset();
        autodrive.liftMotor.setPower(-1);timeSec = 2.5;
        if (runtime.seconds() > timeSec) {
            autodrive.liftMotor.setPower(0);
        }
        autodrive.armSlider.setPower(1);timeSec = 1;

        switch (state) {
            case 0:
                forward = 32.0; right = 0.0; rotate = 0.0;timeSec = 2.5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                //out = autodrive.encoderDrive(forward,right,rotate,timeSec);
                out = autodrive.encoderDriveCalibrated( forward,right,rotate);
                telemetry.addData("output from encoderDrive=",out);
                telemetry.update();
                state = 10;
                break;
            case 1:
                forward = 32.0; right = 12.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                //autodrive.encoderDrive(forward,right,rotate,timeSec);
                out = autodrive.encoderDriveCalibrated( forward,right,rotate);
                state = 10;//state += 1;
                break;
            case 2:
                currentPosition = autodrive.sliderPot2.getVoltage();
                sliderPosition = 3;
                stateOp = "--> Mover slider forward to position "+sliderPosition;
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.addData("Current Slider Position: ",currentPosition);
                sliderPower = autodrive.setSliderPosition(sliderPosition, currentPosition);
                telemetry.addData("Slider Power: ",sliderPower);
                telemetry.update();
                if (sliderPower == 0) {
                    state += 1;
                }
                break;
            case 3:
                currentPosition = autodrive.sliderPot2.getVoltage();
                sliderPosition = 0;
                stateOp = "--> Retracting slider to position "+sliderPosition;
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.addData("Current Slider Position: ",currentPosition);
                sliderPower = autodrive.setSliderPosition(sliderPosition, currentPosition);
                telemetry.addData("Slider Power: ",sliderPower);
                telemetry.update();
                if (sliderPower == 0) {
                    state += 1;
                }
                break;
            case 4:
                forward = -1.0; right = 0.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                autodrive.encoderDrive(forward,right,rotate,timeSec);
                state += 1;
                break;
            case 5:
                forward = 0.0; right = -1.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                autodrive.encoderDrive(forward,right,rotate,timeSec);
                state += 1;
                break;
            case 6:
                forward = 1.0; right = 0.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                autodrive.encoderDrive(forward,right,rotate,timeSec);
                state += 1;
                break;
            case 7:
                forward = 0.0; right = -1.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                autodrive.encoderDrive(forward,right,rotate,timeSec);
                state += 1;
                break;
            case 8:
                forward = -1.0; right = 0.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                autodrive.encoderDrive(forward,right,rotate,timeSec);
                state += 1;
                break;
            case 10:
                telemetry.addData("output ticksPerRotation=",out[0]);
                telemetry.addData("output ticksPerInch=",out[1]);
                telemetry.addData("output ticksPerDeg=",out[2]);
                telemetry.addData("output forwardTicks=",out[3]);
                telemetry.addData("output rightTicks=",out[4]);
                telemetry.addData("output rotateTicks=",out[5]);
                telemetry.addData("output leftFrontTicks=",out[6]);
                telemetry.addData("output leftBackTicks=",out[7]);
                telemetry.addData("output rightFrontTicks=",out[8]);
                telemetry.addData("output rightBackTicks=",out[9]);
                telemetry.addData("current leftFrontTicks=",out[10]);
                telemetry.addData("current leftBackTicks=",out[11]);
                telemetry.addData("current rightFrontTicks=",out[12]);
                telemetry.addData("current rightBackTicks=",out[13]);                telemetry.update();
                break;

        }
    }
}