package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoConfig {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private IMU imu;
    public DcMotorEx armRotate;
    public DcMotor armSlider;
    public DcMotor liftMotor;
    private Servo clawServo;
    private Servo rightWristServo;
    private Servo leftWristServo;
    public AnalogInput armPot;
    private double ticksPerRotation;
    //private double ticksPerRotation1;
    //private double ticksPerRotation2;
    //private double ticksPerRotation3;
    public double clawPositionOpen = 0.83;
    public double clawPositionClosed = 0.45;
    public double wristPositionMid = 0.5;
    public double wristPositionDown = 1;
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
        armSlider = hardwareMap.get(DcMotor.class, "armSlider");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        armPot = hardwareMap.get(AnalogInput.class, "armPot");


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

    public void encoderDrive(double forwardInch, double rightInch, double rotateDeg,double timeout) {
        ticksPerRotation = (leftFrontDrive.getMotorType().getTicksPerRev()+
                leftBackDrive.getMotorType().getTicksPerRev()+
                rightFrontDrive.getMotorType().getTicksPerRev()+
                rightBackDrive.getMotorType().getTicksPerRev())/4.0;

        double wheelDiameterInch = 3.78; //3.78 in (96 mm) or 4.09 in (104 mm)
        double wheelDistanceFromCenter = 8.00;
        double ticksPerInch = ticksPerRotation/(wheelDiameterInch*Math.PI);
        double ticksPerDeg  = ticksPerInch*(wheelDistanceFromCenter*2.0*Math.PI)/360.0;
        double driveSpeed = 0.5;

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int forwardTicks = (int)(ticksPerInch*forwardInch);
        int rotateTicks = (int)(ticksPerDeg*rotateDeg);
        int rightTicks = (int)(ticksPerInch*rightInch);

        int leftFrontTicks = forwardTicks + rotateTicks - rightTicks; //forward + right + rotate;
        int leftBackTicks = forwardTicks - rotateTicks + rightTicks; //forward - right + rotate;
        int rightFrontTicks = forwardTicks + rotateTicks + rightTicks; //forward - right - rotate;
        int rightBackTicks = forwardTicks - rotateTicks - rightTicks; //forward + right - rotate;

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
        setDrive(0.0, 0.0, 0.0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public double setSliderPosition(int sliderPosition, double currentPosition) {
        double sliderPositionMin = 2.4; // slider all the way in
        double sliderPositionMax = 1.2; // high bucket limit (fully extended)
        double maxLimit = Math.max(sliderPositionMin, sliderPositionMax);
        double minLimit = Math.min(sliderPositionMin, sliderPositionMax);
        double maxPower = 0.10;
        double maxRate = 1;
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
        return Range.scale(armPot.getVoltage(), 0, armPot.getMaxVoltage(), 0, 90);
    }
}