package org.firstinspires.ftc.teamcode.mechanisms;

// import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LeftLiftOmniBotMotor {
    public DcMotor motorTowersdgf;
    private double ticksPerRotation;
    public AnalogInput armPot;

    public void init(HardwareMap hardwareMap) {
        motorTowersdgf = hardwareMap.get(DcMotor.class, "motorTowersdgf");
        motorTowersdgf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTowersdgf.setDirection(DcMotorSimple.Direction.REVERSE);
        ticksPerRotation = motorTowersdgf.getMotorType().getTicksPerRev();
        armPot = hardwareMap.get(AnalogInput.class, "armPot");
    }

    public void setMotorSpeed(double speed){
        motorTowersdgf.setPower(speed);
       // telemetry.addData("Motor Speed =", speed);
    }

    public double getMotorRotations(){
        return motorTowersdgf.getCurrentPosition()/ticksPerRotation;
    }
//    public void setPowers(double v, double v1) {
//
//    }
}
