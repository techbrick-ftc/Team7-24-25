package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class ArmTest {
    private DcMotor armRotate;
    private DcMotor armSlider;
    private double ticksPerRotation;
    private Servo clawServo;
    private Servo rightWristServo;
    private Servo leftWristServo;
    public AnalogInput armPot;

    public double clawPositionOpen = 0.83;
    public double clawPositionClosed = 0.45;
    public double wristPositionMid = 0.5;
    public double wristPositionDown = 1;

    public void init(HardwareMap hardwareMap) {
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armSlider = hardwareMap.get(DcMotor.class, "armSlider");
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        ticksPerRotation = armRotate.getMotorType().getTicksPerRev();
        ticksPerRotation = armSlider.getMotorType().getTicksPerRev();
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        armPot = hardwareMap.get(AnalogInput.class, "armPot");
    }

    public void setMotorSpeed(double speed){
        armRotate.setPower(speed);
        armSlider.setPower(speed);
    }

    public void setSliderSpeed(double speed){
        armSlider.setPower(speed);
    }

    public void setRotateSpeed(double speed){
        armRotate.setPower(speed);
    }

    public double getMotorRotations() {
        return armRotate.getCurrentPosition()/ticksPerRotation;
        //return armSlider.getCurrentPosition()/ticksPerRotation;
    }

    public void setRightWristServoPosition(double position) {
        rightWristServo.setPosition(position);
    }

    public void setClawServoPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setLeftWristServoPosition(double position) {
        leftWristServo.setPosition(position);
    }

    public double getRightWristServoPosition() {
        return rightWristServo.getPosition();
    }

    public double getLeftWristServoPosition() { return leftWristServo.getPosition(); }

    public Servo.Direction getRightWristServoDirection() {
        return rightWristServo.getDirection();
    }

    public Servo.Direction getLeftWristServoDirection() {
        return leftWristServo.getDirection();
    }

    public double getArmPotAngle() {
        return Range.scale(armPot.getVoltage(), 0, armPot.getMaxVoltage(), 0, 90);
    }
}