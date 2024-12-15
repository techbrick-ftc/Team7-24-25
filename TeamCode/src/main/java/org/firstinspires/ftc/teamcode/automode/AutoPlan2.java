package org.firstinspires.ftc.teamcode.automode; // armRotate position start 2.144, slider position clip specimen 2.336

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.AutoConfig;

@Autonomous()
public class AutoPlan2 extends OpMode{
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
    public double armPositionAutoSpecimen = 2.128;

    @Override
    public void init () {
        autodrive.init(hardwareMap);
        robotCentric = false;
    }

    @Override
    public void start() {
        //autodrive.liftMotor.setPower(1);timeSec = 2.3;
        //if (runtime.seconds() > timeSec) {
            //autodrive.liftMotor.setPower(0);
        //}// for now
        autodrive.setClawServoPosition(autodrive.clawPositionClosed); // for now
        autodrive.setRightWristServoPosition(autodrive.wristPositionMid); // for now
        state = -1;
    }

    @Override
    public void loop() {
        telemetry.addData("Auto State: ",state);
        autodrive.setClawServoPosition(autodrive.clawPositionClosed);
        autodrive.setRightWristServoPosition(autodrive.wristPositionAuto2);
        //runtime.reset();
        //utodrive.liftMotor.setPower(-1);timeSec = 1;
        //if (runtime.seconds() > timeSec) {
        //    autodrive.liftMotor.setPower(0);
        //}
        //autodrive.armSlider.setPower(1);timeSec = 1;

        switch (state) {
            case -1:
                runtime.reset();
                autodrive.liftMotor.setPower(-1);timeSec = 2.2;
                while (runtime.seconds() < timeSec) {
                    autodrive.liftMotor.setPower(0);
                }
                autodrive.liftMotor.setPower(0);
                state =0;
                break;
            case 0:
                forward = 0.0; right = 0.0; rotate = 0.0;timeSec = 2.5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                //out = autodrive.encoderDrive(forward,right,rotate,timeSec);
                out = autodrive.encoderDriveCalibrated( forward,right,rotate);
                telemetry.addData("output from encoderDrive=",out);
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() < timeSec) {
                    currentPosition = autodrive.armPot0.getVoltage();
                    autodrive.setArmPosition(4, currentPosition);
                }
                state = 1;
                break;
            case 1:
                forward = 0.0; right = 0.0; rotate = 0.0;timeSec = 1.0;
                runtime.reset();
                autodrive.setSliderSpeed(-1);
                //autodrive.armSlider.setPower(0);
                while (runtime.seconds() < timeSec) {
                    //autodrive.setSliderSpeed(-1);
                    //autodrive.setSliderSpeed(-0.1);
                    //autodrive.armSlider.setPower(-1);
                    telemetry.addData("Auto State: %d ",state);
                    telemetry.addData("time=",runtime.seconds());
                    telemetry.update();
                }
                if (runtime.seconds() > timeSec) {
                     autodrive.setSliderSpeed(0);
                    //autodrive.liftMotor.setPower(0);
                }
                autodrive.setSliderSpeed(0);
                //autodrive.armSlider.setPower(0);
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                //out = autodrive.encoderDrive(forward,right,rotate,timeSec);
                out = autodrive.encoderDriveCalibrated( forward,right,rotate);
                telemetry.addData("output from encoderDrive=",out);
                telemetry.update();
                runtime.reset();
                state = 2;
                break;
            case 2:
                forward = 0.0; right = 0.0; rotate = 0.0;timeSec = 1;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                //out = autodrive.encoderDrive(forward,right,rotate,timeSec);
                out = autodrive.encoderDriveCalibrated( forward,right,rotate);
                telemetry.addData("output from encoderDrive=",out);
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() < timeSec) {
                    currentPosition = autodrive.armPot0.getVoltage();
                    autodrive.setArmPosition(1, currentPosition);
                }
                state = 3;
                break;
            case 3:
                forward = 30.5; right = 0.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                //autodrive.encoderDrive(forward,right,rotate,timeSec);
                out = autodrive.encoderDriveCalibrated( forward,right,rotate);
                state = 4;
                break;
            case 4:
                forward = -20.0; right = 0.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                autodrive.encoderDrive(forward,right,rotate,timeSec);
                state = 10;
                break;
            case 5:
                forward = 0.0; right = -30.0; rotate = 0.0;timeSec = 5;
                stateOp = "--> "+autodrive.driveByTimeToString(forward,right,rotate,timeSec);
                telemetry.addData("Auto State: %d "+stateOp,state);
                telemetry.update();
                //autodrive.setDriveByTime(forward,right,rotate,timeSec);
                //try {
                //    wait(28);
                //} catch (InterruptedException e) {
                //    throw new RuntimeException(e);
                //}
                autodrive.encoderDrive(forward,right,rotate,timeSec);
                state = 6;
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