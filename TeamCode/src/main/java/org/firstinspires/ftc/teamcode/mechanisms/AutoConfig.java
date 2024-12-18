package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoConfig {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private IMU imu;
    public TouchSensor touchSensor;
    public DcMotorEx armRotate;
    public DcMotorEx armSlider;
    public DcMotor liftMotor;
    private Servo clawServo;
    private Servo rightWristServo;
    private Servo leftWristServo;
    public AnalogInput armPot0;
    private double ticksPerRotation;
    //private double ticksPerRotation1;
    //private double ticksPerRotation2;
    //private double ticksPerRotation3;
    public double clawPositionOpen = 0.45;//0.83;    // This is actually a close position
    public double clawPositionClosed = 0.83;//0.45;  // This is open position
    public double wristPositionMid = 0.5;
    public double wristPositionDown = 1;
    public double wristPositionAuto2 = 0.7;
    public double armPositionAutoSpecimen = 2.128;
    public ElapsedTime runtime = new ElapsedTime();
    public AnalogInput sliderPot2;
    /*public int armPosition0 = 0;
    public int armPosition1 = -625;
    public int armPosition2 = -1250;
    public int armPosition3 = -1875;
    public int armPosition4 = -2500;
    public int sliderPosition0 = 0;
    public int sliderPosition1 = 500;
    public int sliderPosition2 = 1000;
    public int sliderPosition3 = 1500;
    public int sliderPosition4 = 2000;
    public int armVel = -100;
    public int slideVel = -100;*/

    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        imu = hardwareMap.get(IMU.class, "imu");
        armRotate = hardwareMap.get(DcMotorEx.class, "armRotate");
        armSlider = hardwareMap.get(DcMotorEx.class, "armSlider");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        touchSensor = hardwareMap.touchSensor.get("touch_sensor");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        armPot0 = hardwareMap.get(AnalogInput.class, "armPot0");
        sliderPot2 = hardwareMap.get(AnalogInput.class, "sliderPot2");


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        touchSensor.getConnectionInfo();

        /*ticksPerRotation1 = armRotate.getMotorType().getTicksPerRev();
        ticksPerRotation2 = armSlider.getMotorType().getTicksPerRev();
        ticksPerRotation3 = liftMotor.getMotorType().getTicksPerRev();*/

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    /*public void setArmPosition(int armPosition){
        armRotate.setTargetPosition(armPosition);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setVelocity(armVel);
    }*/

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

    }

    public void setPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        double largest = 1;
        largest = Math.max(largest, Math.abs(leftFrontPower));
        largest = Math.max(largest, Math.abs(leftBackPower));
        largest = Math.max(largest, Math.abs(rightFrontPower));
        largest = Math.max(largest, Math.abs(rightBackPower));

        leftFrontDrive.setPower(leftFrontPower / largest);
        leftBackDrive.setPower(leftBackPower / largest);
        rightFrontDrive.setPower(rightFrontPower / largest);
        rightBackDrive.setPower(rightBackPower / largest);
    }

    public double[] encoderDriveCalibrated(double forwardInch, double rightInch, double rotateDeg){
        double forwardInchNew, rightInchNew,rotateDegNew,timeNew;
        double rightInchCalibrated1=-1.02*(11+1.0/8.0),rightForwardCalibrated1=(-1.0/2.0)*10,rightTime1=2.5;
        double rightInchCalibrated2=(-12.75*0.8),rightForwardCalibrated2=7.0/8.0,rightTime2=2.5;
        double rightTime = (rightInch > 0.0) ? rightTime1 : rightTime2;
        double rightInchCalibrated = (rightInch < 0.0) ? rightInchCalibrated1 : rightInchCalibrated2;
        double rightForwardCalibrated = (rightInch < 0.0) ? rightForwardCalibrated1 : rightForwardCalibrated2;

        double forwardInchCalibrated=16*0.97,forwardTime=2.5;
        rightInchNew = -rightInch;
        forwardInchNew = forwardInch;
        rotateDegNew = rotateDeg;
        timeNew = Math.abs(rightTime*rightInch/rightInchCalibrated)+ Math.abs(forwardTime*forwardInch/forwardInchCalibrated);
        double [] out;
        out = encoderDrive(forwardInchNew,rightInchNew,rotateDegNew,timeNew);
        return out;
    }

    public double[] encoderDrive(double forwardInch, double rightInch, double rotateDeg,double timeout) {
        double[] out = new double[14];
        ticksPerRotation = (int) (leftFrontDrive.getMotorType().getTicksPerRev()+
                leftBackDrive.getMotorType().getTicksPerRev()+
                rightFrontDrive.getMotorType().getTicksPerRev()+
                rightBackDrive.getMotorType().getTicksPerRev())/4.0;
        out[0] = (double) ticksPerRotation;

        double wheelDiameterInch = 3.78; //3.78 in (96 mm) or 4.09 in (104 mm)
        double wheelDistanceFromCenter = 7.00;
        double ticksPerInch = ticksPerRotation/(wheelDiameterInch*Math.PI);
        double ticksPerDeg  = ticksPerInch*(wheelDistanceFromCenter*2.0*Math.PI)/360.0;
        double driveSpeed = 0.3;
        if (Math.abs(rightInch) > 24) {
            driveSpeed = 0.3;
        }

        double rightTicksPerInch = 36.14742015;  //overriding the wheel ticksPerInch
        double rightForwardOffsetPerInch = 0.0;  //31.62899263/12.0;
        double forwardTicksPerInch = 36.14742015;  //overriding the wheel ticksPerInch
        double forwardRightOffsetPerInch = 0.0;

        out[1] = (double) ticksPerInch;
        out[2] = (double) ticksPerDeg;

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int forwardTicks = (int) (ticksPerInch*forwardInch);// + rightForwardOffsetPerInch*rightInch);
        int rotateTicks = (int)(ticksPerDeg*rotateDeg);
        int rightTicks = (int)(ticksPerInch*rightInch);

        out[3] = (double) forwardTicks;
        out[4] = (double) rightTicks;
        out[5] = (double) rotateTicks;

        int leftFrontTicks = forwardTicks + rotateTicks - rightTicks; //forward + right + rotate;
        int leftBackTicks = forwardTicks - rotateTicks + rightTicks; //forward - right + rotate;
        int rightFrontTicks = forwardTicks + rotateTicks + rightTicks; //forward - right - rotate;
        int rightBackTicks = forwardTicks - rotateTicks - rightTicks; //forward + right - rotate;

        out[6] = (double) leftFrontTicks;
        out[7] = (double) leftBackTicks;
        out[8] = (double) rightFrontTicks;
        out[9] = (double) rightBackTicks;

        leftFrontDrive.setTargetPosition(leftFrontTicks);
        leftBackDrive.setTargetPosition(leftBackTicks);
        rightFrontDrive.setTargetPosition(rightFrontTicks);
        rightBackDrive.setTargetPosition(rightBackTicks);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftFrontDrive.setPower(Math.abs(driveSpeed));
        rightFrontDrive.setPower(Math.abs(driveSpeed));
        leftBackDrive.setPower(Math.abs(driveSpeed));
        rightBackDrive.setPower(Math.abs(driveSpeed));
        while ((runtime.seconds() < timeout) &&
                (leftFrontDrive.isBusy() && leftBackDrive.isBusy()) && (rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

        }
        out[10] = leftFrontDrive.getCurrentPosition();
        out[11] = leftBackDrive.getCurrentPosition();
        out[12] = rightFrontDrive.getCurrentPosition();
        out[13] = rightBackDrive.getCurrentPosition();
        setDrive(0.0, 0.0, 0.0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return out;
    }

    public void setDrive(double forward, double right, double rotate) {
        double frontLeftPower = forward - rotate - right; //forward + right + rotate;
        double backLeftPower = forward - rotate + right; //forward - right + rotate;
        double frontRightPower = forward + rotate + right; //forward - right - rotate;
        double backRightPower = forward + rotate - right; //forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public String driveByTimeToString(double forward, double right, double rotate,double time) {
        String forwardMessage = (forward > 0.0) ? " Moving Forward " : " Moving Backward ";
        String rightMessage = (right > 0.0) ? " Moving Right " : " Moving Left ";
        String rotateMessage = (rotate > 0.0) ? " Rotate Right " : " Rotate Left ";
        String timeMessage = String.format(" (%2.1f Seconds)", new Object[]{time});
        forwardMessage = (forward == 0.0) ? "" : forwardMessage;
        rightMessage = (right == 0.0) ? "" : rightMessage;
        rotateMessage = (rotate == 0.0) ? "" : rotateMessage;
        timeMessage = (time == 0.0) ? "" : timeMessage;
        return forwardMessage+rightMessage+rotateMessage+timeMessage;
    }

    public void setDriveByTime(double forward, double right, double rotate,double time) {
        setDrive(forward, right, rotate);
        runtime.reset();
        while (runtime.seconds() < time) {
            //telemetry.addData("setDriveByTime: Elapsed Time: %4.1f Seconds", runtime.seconds());
            //telemetry.update();
        }
        setDrive(0.0, 0.0, 0.0); //stop the motors
    }

    public void setMotorSpeed(double speed) {
        armRotate.setPower(speed);
        armSlider.setPower(speed);
    }

    public void setSliderSpeed(double speed) {
        double armSliderSpeedMax = 1;
        armSlider.setPower(speed * armSliderSpeedMax);
    }

    public void setRotateSpeed(double speed) {
        double armRotateSpeedMax = 0.25;
        armRotate.setPower(speed * armRotateSpeedMax);
    }

    public double setArmPosition(int armPosition, double currentPosition) {
       /* if (drive.armPot0.getVoltage() > 2.15) { // arm down limit (~0 deg)
        }
        if (drive.armPot0.getVoltage() < 2.105) { // arm stright up limit (~80 deg)
        }*/
        double armPositionMin = 2.17; // arm down
        double armPositionMax = 2.095; // arm up
        double maxLimit = Math.max(armPositionMin,armPositionMax);
        double minLimit = Math.min(armPositionMin,armPositionMax);
        double maxPower = 1.0;
        double maxRate  = 1.0;
        double delta = armPositionMax-armPositionMin;
        double targetArmPosition = armPositionMin;
        //if (armPosition == 0) targetArmPosition = armPositionMin;
        if (armPosition == 1) targetArmPosition = 2.13; // start plan 1
        if (armPosition == 2) targetArmPosition = 2.136; // end plan 1
        if (armPosition == 3) targetArmPosition = 2.1585; // specimen grab plan 3
        if (armPosition == 4) targetArmPosition = 2.129; // plan 2
        if (armPosition == 5) targetArmPosition = 2.148; // after grab plan 3
        if (armPosition == 6) targetArmPosition = 2.123; // clip plan 3
        maxPower = (currentPosition > targetArmPosition) ? 1.0 : 1.0;
        maxRate  = (currentPosition > targetArmPosition) ? 0.2 : 0.1;
        // first number power for moving arm up (need more power)
        // second number power for moving arm down (need less power) <-- gravity assist
        double armPower =-maxPower*Math.tanh((currentPosition-targetArmPosition)*50/delta);
        double positionError = Math.abs((currentPosition-targetArmPosition)*100/delta);
        if (positionError < 2){
            armRotate.setPower(0.0);
            return(0.0);
        }
        if ((currentPosition < maxLimit) && (currentPosition > minLimit)) {
            //armRotate.setVelocity(maxRate);
            armRotate.setPower(armPower);
        }
        return (armPower);
    }

    public double setSliderPosition(int sliderPosition, double currentPosition) {
        double sliderPositionMin = 2.4; // slider all the way in
        double sliderPositionMax = 1.2; // high bucket limit (fully extended)
        double maxLimit = Math.max(sliderPositionMin, sliderPositionMax);
        double minLimit = Math.min(sliderPositionMin, sliderPositionMax);
        double maxPower = 0.10;
        double maxRate = 1;
        double delta = sliderPositionMax-sliderPositionMin;
        double targetSliderPosition = sliderPositionMin;
        if (sliderPosition == 0) targetSliderPosition = sliderPositionMin;
        if (sliderPosition == 1) targetSliderPosition = 1.85;//armPositionMin + 0.25*delta;
        if (sliderPosition == 2) targetSliderPosition = 1.2;//armPositionMin + 0.50*delta;
        if (sliderPosition == 3) targetSliderPosition = 2.0;
        if (sliderPosition == 4) targetSliderPosition = 2.;
        maxPower = (currentPosition > targetSliderPosition) ? 1 : 1;
        maxRate  = (currentPosition > targetSliderPosition) ? 0.2 : 0.1;
        double positionError = Math.abs((currentPosition-targetSliderPosition)*100/delta);
        // first number power for moving arm up (need more power)
        // second number power for moving arm down (need less power) <-- gravity assist
        double sliderPower = maxPower*Math.tanh((currentPosition-targetSliderPosition)*50/delta);
        if (positionError < 1){
            armSlider.setPower(0.0);
            return(0.0);
        }
        if ((currentPosition < maxLimit) && (currentPosition > minLimit)) {
            //armSlider.setVelocity(maxRate);
            armSlider.setPower(sliderPower);
        }
        //else {
        //    sliderPower = 0.0;
        //    armSlider.setPower(sliderPower);
        //}
        return (sliderPower);
    }

    public double getMotorRotations() {
        return armRotate.getCurrentPosition() / ticksPerRotation;
    }
    public void setClawServoPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setRightWristServoPosition(double position) {
        rightWristServo.setPosition(position);
    }

    public void setLeftWristServoPosition(double position) {
        leftWristServo.setPosition(position);
    }

    public double getRightWristServoPosition() {
        return rightWristServo.getPosition();
    }

    public double getLeftWristServoPosition() {
        return leftWristServo.getPosition();
    }

    public Servo.Direction getRightWristServoDirection() {
        return rightWristServo.getDirection();
    }

    public Servo.Direction getLeftWristServoDirection() {
        return leftWristServo.getDirection();
    }
    public double getPotAngle() {
        return Range.scale(armPot0.getVoltage(), 0, armPot0.getMaxVoltage(), 0, 90);
    }

    public void armSlider(int currentPosition) {

    }
}