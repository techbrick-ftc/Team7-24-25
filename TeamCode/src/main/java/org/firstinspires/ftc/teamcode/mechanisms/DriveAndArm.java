package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveAndArm {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private IMU imu;
    private DcMotor armRotate;
    private DcMotor armSlider;
    private Servo rightClawServo;
    private Servo clawServo;
    private Servo leftClawServo;
    private DcMotor liftMotor;
    private double ticksPerRotation;



    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        imu = hardwareMap.get(IMU.class, "imu");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armSlider = hardwareMap.get(DcMotor.class, "armSlider");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        ticksPerRotation = armRotate.getMotorType().getTicksPerRev();
        ticksPerRotation = armSlider.getMotorType().getTicksPerRev();

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);

    }


    public void setClawServoPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setLeftClawServoPosition(double position) {
        leftClawServo.setPosition(position);
    }

    public void setRightClawServoPosition(double position) {
        rightClawServo.setPosition(position);
    }

    public void setPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        double largest = 1.0;
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
        double frontLeftPower = forward - rotate - right; //forward + right + rotate;
        double backLeftPower = forward - rotate + right; //forward - right + rotate;
        double frontRightPower = forward + rotate + right; //forward - right - rotate;
        double backRightPower = forward + rotate - right; //forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void setMotorSpeed(double speed) {
        armRotate.setPower(speed);
        armSlider.setPower(speed);
    }

    public void setSliderSpeed(double speed) {
        armSlider.setPower(speed);
    }

    public void setRotateSpeed(double speed) {
        armRotate.setPower(speed);
    }


    public double getMotorRotations() {
        return armRotate.getCurrentPosition() / ticksPerRotation;
    }
}