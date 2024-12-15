package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveAndArmFinal {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private IMU imu;
    public DcMotor armRotate;
    public DcMotor armSlider;
    public DcMotor liftMotor;
    private Servo clawServo;
    private Servo rightWristServo;
    private Servo leftWristServo;
    public AnalogInput armPot0;
    public AnalogInput armPot1;
    public AnalogInput sliderPot2;
    public AnalogInput sliderPot3;
    public DigitalChannel liftLimitSwitch;
    private double ticksPerRotation;
    //private double ticksPerRotation1;
    //private double ticksPerRotation2;
    //private double ticksPerRotation3;
    public double clawPositionOpen = 0.83;
    public double clawPositionClosed = 0.45;
    public double wristPositionMid = 0.5;
    public double wristPositionDown = 1;
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
        //imu = hardwareMap.get(IMU.class, "imu");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armSlider = hardwareMap.get(DcMotor.class, "armSlider");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        armPot0 = hardwareMap.get(AnalogInput.class, "armPot0");
        armPot1 = hardwareMap.get(AnalogInput.class, "armPot1");
        sliderPot2 = hardwareMap.get(AnalogInput.class, "sliderPot2");
        sliderPot3 = hardwareMap.get(AnalogInput.class, "sliderPot3");
        liftLimitSwitch = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch");

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
        armRotate.setDirection(DcMotor.Direction.REVERSE);
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /*ticksPerRotation1 = armRotate.getMotorType().getTicksPerRev();
        ticksPerRotation2 = armSlider.getMotorType().getTicksPerRev();
        ticksPerRotation3 = liftMotor.getMotorType().getTicksPerRev();*/
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                //new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double setArmPosition(int armPosition, double currentPosition) {
       /* if (drive.armPot0.getVoltage() > 2.15) { // arm down limit (~0 deg)
        }
        if (drive.armPot0.getVoltage() < 2.105) { // arm stright up limit (~80 deg)
        }*/
        double armPositionMin = 2.17;  // arm down
        double armPositionMax = 2.095;  // arm up
        double maxLimit = Math.max(armPositionMin,armPositionMax);
        double minLimit = Math.min(armPositionMin,armPositionMax);
        double maxPower = 1.0;
        double maxRate  = 1.0;
        double delta = armPositionMax-armPositionMin;
        double targetArmPosition = armPositionMin;
        //if (armPosition == 0) targetArmPosition = armPositionMin;
        if (armPosition == 1) targetArmPosition = 2.15;//armPositionMin + 0.25*delta;
        if (armPosition == 2) targetArmPosition = 2.13;//armPositionMin + 0.50*delta;
        if (armPosition == 3) targetArmPosition = 2.12;//armPositionMin + 0.50*delta;
        if (armPosition == 4) targetArmPosition = 2.095;//armPositionMin + 0.75*delta;
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

    public double setSliderPosition(int sliderPosition, double currentPosition){
        double sliderPositionMin = 2.4; // slider all the way in
        double sliderPositionMax = 1.2; // high bucket limit (fully extended)
        double maxLimit = Math.max(sliderPositionMin,sliderPositionMax);
        double minLimit = Math.min(sliderPositionMin,sliderPositionMax);
        double maxPower = 0.10;
        double maxRate  = 1;
        double delta = sliderPositionMax-sliderPositionMin;
        double targetSliderPosition = sliderPositionMin;
        //if (sliderPosition == 0) targetSliderPosition = SliderPositionMin;
        if (sliderPosition == 1) targetSliderPosition = 1.85;//armPositionMin + 0.25*delta;
        if (sliderPosition == 2) targetSliderPosition = 1.2;//armPositionMin + 0.50*delta;
        //if (sliderPosition == 3) targetSliderPosition = sliderPositionMin + 0.50*delta;
        //if (sliderPosition == 4) targetSliderPosition = sliderPositionMin + 0.75*delta;
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
        return (sliderPower);
    }


    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

    }

    public void setLiftMotorSpeed (double speed) {
        liftMotor.setPower(speed);
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


    public void setDrive(double forward, double right, double rotate) {
        double frontLeftPower = forward + rotate - right; //forward + right + rotate;
        double backLeftPower = forward - rotate + right; //forward - right + rotate;
        double frontRightPower = forward + rotate + right; //forward - right - rotate;
        double backRightPower = forward - rotate - right; //forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void setMotorSpeed(double speed) {
        armRotate.setPower(speed);
        armSlider.setPower(speed);
    }

    public double setSliderSpeed(double speed) {
        double armSliderSpeedMax = 1;
        armSlider.setPower(speed * armSliderSpeedMax);
        return speed * armSliderSpeedMax;
    }

    public double setRotateSpeed(double speed) {
        double armRotateSpeedMax = 0.10;
        armRotateSpeedMax = (speed > 0.0) ? 0.5 : 0.2;
        armRotate.setPower(speed * armRotateSpeedMax);
        return speed * armRotateSpeedMax;
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

    public double getArmPotAngle0() {
        return Range.scale(armPot0.getVoltage(), 0, armPot0.getMaxVoltage(), 0, 270);
    }

    public double getArmPotAngle1() {
        return Range.scale(armPot1.getVoltage(), 0, armPot1.getMaxVoltage(), 0, 270);
    }

    public double getSliderPotAngle2() {
        return Range.scale(sliderPot2.getVoltage(), 0, sliderPot2.getMaxVoltage(), 0, 270);
    }

    public double getSliderPotAngle3() {
        return Range.scale(sliderPot3.getVoltage(), 0, sliderPot3.getMaxVoltage(), 0, 270);
    }
}