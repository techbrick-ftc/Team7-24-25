package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class PotentiometerTest {
    private DcMotor motorTowersdgf;
    public AnalogInput armPot;
    private double ticksPerRotation;

    public void init(HardwareMap hardwareMap) {
        armPot = hardwareMap.get(AnalogInput.class, "armPot");
        motorTowersdgf = hardwareMap.get(DcMotor.class, "motorTowersdgf");
        motorTowersdgf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTowersdgf.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTowersdgf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ticksPerRotation = motorTowersdgf.getMotorType().getTicksPerRev();
    }

    public void setMotorSpeed(double speed) {
        motorTowersdgf.setPower(speed);
    }

    public void setRotateSpeed(double speed){
        motorTowersdgf.setPower(speed);
    }

    public double getMotorRotations() {
        return motorTowersdgf.getCurrentPosition() / ticksPerRotation;
    }

    public double getPotAngle() {
        return Range.scale(armPot.getVoltage(), 0, armPot.getMaxVoltage(), 0, 90);
    }
}
