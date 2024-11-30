package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.mechanisms.DriveAndArmFinal;

@TeleOp()
public class MainTeleOp  extends OpMode {
    DriveAndArmFinal drive = new DriveAndArmFinal();
    boolean gp1yAlreadyPressed = false;
    boolean gp1aAlreadyPressed = false;
    boolean gp2xAlreadyPressed = false;
    boolean gp2yAlreadyPressed = false;
    boolean gp1dPadUpAlreadyPressed = false;
    boolean gp1dPadDownAlreadyPressed = false;
    boolean gp1dPadLeftAlreadyPressed = false;
    boolean gp2dPadUpAlreadyPressed = false;
    boolean gp2dPadDownAlreadyPressed = false;
    boolean gp2dPadLeftAlreadyPressed = false;
    boolean clawOpen = false;
    boolean wristUp = false;
    boolean armManual = true;
    int armPosition = 0;
    double armPower = 0;
    boolean sliderManual = true;
    double sliderPower = 0;
    int sliderPosition = 0;
    boolean robotCentric = false;
    boolean slowMode = false;
    IMU imu;
    public DcMotor liftMotor;
    public AnalogInput armPot0;
    public AnalogInput sliderPot2;
    public DcMotorEx armRotate;
    public DcMotorEx armSlider;

    public void init() {

        drive.init(hardwareMap);
        robotCentric = false;

        imu = hardwareMap.get(IMU.class, "imu");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        //RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        //imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        //used for omni bot because control hub (imu) is at -45 to Y axis and + 45 to X axis
        /*IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                0,
                                +90,
                                0,
                                0  // acquisitionTime, not used
                        )
                )
        );

        imu.initialize(myIMUparameters);*/
        /*drive.armRotate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        drive.armSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.armRotate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        drive.armSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
    }

    private void checkArmPosition() {
        telemetry.addData("Arm position", armPosition);
        int maxPosition = 4;
        int minPosition = 0;
        double currentPosition = drive.armPot0.getVoltage();
        if(gamepad1.dpad_up && !gp1dPadUpAlreadyPressed) {
            armPosition += 1;
            //drive.setArmPosition(armPosition);
            //drive.armRotate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        gp1dPadUpAlreadyPressed = gamepad1.dpad_up;

        if(gamepad1.dpad_down && !gp1dPadDownAlreadyPressed) {
            armPosition -= 1;
            //drive.setArmPosition(armPosition);
            //drive.armRotate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
        gp1dPadDownAlreadyPressed = gamepad1.dpad_down;

        if (armPosition > maxPosition) armPosition = maxPosition;
        if (armPosition < minPosition) armPosition = minPosition;

        armPower = drive.setArmPosition(armPosition, currentPosition);
        //telemetry.addData("Arm Power-2", armPower);
        /*
        if (armPosition == 0) drive.setArmPosition(drive.armPosition0);
        if (armPosition == 1) drive.setArmPosition(drive.armPosition1);
        if (armPosition == 2) drive.setArmPosition(drive.armPosition2);
        if (armPosition == 3) drive.setArmPosition(drive.armPosition3);
        if (armPosition == 4) drive.setArmPosition(drive.armPosition4);
        */
    }


    private void checkSliderPosition() {
        telemetry.addData("Slider position", sliderPosition);
        int maxPosition = 2;
        int minPosition = 0;
        double currentPosition = drive.sliderPot2.getVoltage();
        if(gamepad2.dpad_up && !gp2dPadUpAlreadyPressed) {
            sliderPosition += 1;
        }
        gp2dPadUpAlreadyPressed = gamepad2.dpad_up;

        if(gamepad2.dpad_down && !gp2dPadDownAlreadyPressed) {
            sliderPosition -= 1;
        }
        gp2dPadDownAlreadyPressed = gamepad2.dpad_down;

        if (sliderPosition > maxPosition) sliderPosition = maxPosition;
        if (sliderPosition < minPosition) sliderPosition = minPosition;
        sliderPower = drive.setSliderPosition(sliderPosition, currentPosition);
        /*if (sliderPosition == 0) drive.setArmPosition(drive.sliderPosition0);
        if (sliderPosition == 1) drive.setArmPosition(drive.sliderPosition1);
        if (sliderPosition == 2) drive.setArmPosition(drive.sliderPosition2);
        if (sliderPosition == 3) drive.setArmPosition(drive.sliderPosition3);
        if (sliderPosition == 4) drive.setArmPosition(drive.sliderPosition4);*/
    }

    private void checkArmMode() {
        telemetry.addData("Arm Mode: Manual", armManual);
        if (gamepad1.dpad_left && !gp1dPadLeftAlreadyPressed)  armManual = !armManual;
        gp1dPadLeftAlreadyPressed = gamepad1.dpad_left;
    }

    private void checkSliderMode() {
        telemetry.addData("Slider Mode: Manual", sliderManual);
        if (gamepad2.dpad_left && !gp2dPadLeftAlreadyPressed)  sliderManual = !sliderManual;
        gp2dPadLeftAlreadyPressed = gamepad2.dpad_left;
    }

    private void checkDriveMode() {
        telemetry.addData("Robot centric", robotCentric);
        if(gamepad1.y && !gp1yAlreadyPressed) robotCentric = !robotCentric;
        gp1yAlreadyPressed = gamepad1.y;
    }

    private void checkClawOpen() {
        telemetry.addData("Claw open", clawOpen);
        if(gamepad2.y && !gp2yAlreadyPressed) clawOpen = !clawOpen;
        gp2yAlreadyPressed = gamepad2.y;

        if (clawOpen) drive.setClawServoPosition(drive.clawPositionOpen);
        else drive.setClawServoPosition(drive.clawPositionClosed);
    }

    private void checkWristUp() {
        telemetry.addData("Wrist Position", wristUp);
        if(gamepad2.x && !gp2xAlreadyPressed) wristUp = !wristUp;
        gp2xAlreadyPressed = gamepad2.x;

        if (wristUp) drive.setRightWristServoPosition(drive.wristPositionMid);
        else drive.setRightWristServoPosition(drive.wristPositionDown);
    }

    private void robotCentricDrive(double forward, double right, double rotate) {
        double slowFactor;
        slowFactor = (slowMode) ? 0.2 : 1.0;
        drive.setDrive(slowFactor*forward, slowFactor*right, slowFactor*rotate);
    }

    private void fieldCentricDrive(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        //rotate angle
        theta = AngleUnit.normalizeRadians(theta + robotAngle);

        //convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        double slowFactor;
        slowFactor = (slowMode) ? 0.2 : 1.0;
        drive.setDrive(slowFactor*newForward, slowFactor*newRight, slowFactor*rotate);
        //drive.setDrive(newForward, newRight, rotate);
    }

    private void checkSlowMode() {
        telemetry.addData("Slow mode", slowMode);
        if (gamepad1.a && !gp1aAlreadyPressed)  slowMode = !slowMode;
        gp1aAlreadyPressed = gamepad1.a;
    }

    public void lengthLimitCommands() {
        double currentArmPosition = armRotate.getCurrentPosition();
        if ((currentArmPosition >= 2.134) && (armSlider.getCurrentPosition() >= 1.173)) {
            drive.setArmPosition(1, currentArmPosition);
            armRotate.setPower(0.0);
            armSlider.setPower(0.0);
        }
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double slider = gamepad2.left_stick_y;
        double armRotate = -gamepad2.right_stick_y;

        if (gamepad2.left_bumper) { // go down
            liftMotor.setPower(1);
        }

        else if (gamepad2.right_bumper) { // lift up
            liftMotor.setPower(-1);
        }

        else {
            liftMotor.setPower(0);
        }

        checkSliderMode();
        if (sliderManual) {
            sliderPower = drive.setSliderSpeed(slider);
        }
        else
            checkSliderPosition();

        checkArmMode();//armManual = armRotate != 0.0;
        if (armManual) {
            armPower = drive.setRotateSpeed(armRotate);
        }
        else
            checkArmPosition();


        checkDriveMode();
        checkWristUp();
        checkClawOpen();
        checkSlowMode();
        //checkArmPosition();
        //checkSliderPosition();
        telemetry.addData("Heading", drive.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Right stick x gp 1", gamepad1.right_stick_x);
        telemetry.addData("A button gp 2", gamepad2.a);
        /*telemetry.addData("Left stick x gp 1", gamepad1.left_stick_x);
        telemetry.addData("Left stick x gp2", gamepad2.left_stick_x);
        telemetry.addData("Left stick y gp1", gamepad1.left_stick_y);
        telemetry.addData("Left stick y gp2", gamepad2.left_stick_y);
        telemetry.addData("Left trigger gp2", gamepad2.left_trigger);*/
        telemetry.addData("Robot centric", robotCentric);
        telemetry.addData("Wrist up", wristUp);
        telemetry.addData("Claw open", clawOpen);
        telemetry.addData("Arm position", armPosition);
        telemetry.addData("Arm Power", armPower);
        //telemetry.addData("Arm rotate", drive.armRotate.getCurrentPosition());
        //telemetry.addData("Slider", drive.armSlider.getCurrentPosition());
        //telemetry.addData("Slider position", sliderPosition);
        telemetry.addData("Right wrist servo Position", drive.getRightWristServoPosition());
        telemetry.addData("Left wrist servo Position", drive.getLeftWristServoPosition());
        telemetry.addData("Right wrist servo Direction", drive.getRightWristServoDirection());
        telemetry.addData("Left wrist servo Direction", drive.getLeftWristServoDirection());
        telemetry.addData("Arm pot 0 Angle", drive.armPot0.getVoltage());
        //telemetry.addData("Arm pot 1 Angle", drive.armPot1.getVoltage());
        telemetry.addData("Slider pot Angle 2", drive.sliderPot2.getVoltage());
        //telemetry.addData("Slider pot Angle 3", drive.sliderPot3.getVoltage());

        if (robotCentric) {
            robotCentricDrive(forward, right, rotate);
        }
        else {
            fieldCentricDrive(forward, right, rotate);
        }
/*
        if (drive.armPot0.getVoltage() > 2.168) { // arm down limit (~0 deg)
        }
        if (drive.armPot0.getVoltage() < 2.095) { // arm stright up limit (~80 deg)
        }
        if (drive.sliderPot2.getVoltage() > 2.442) { // slider all the way in
        }
        if (drive.sliderPot2.getVoltage() < 1.2) { // high bucket limit (fully extended)
        }
 */
    }
}