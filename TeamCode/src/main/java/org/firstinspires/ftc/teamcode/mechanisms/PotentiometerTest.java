package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class PotentiometerTest {
    public AnalogInput armPot;

    public void init(HardwareMap hardwareMap) {
        armPot = hardwareMap.get(AnalogInput.class, "armPot");
    }

    public double getPotAngle() {
        return Range.scale(armPot.getVoltage(), 0, armPot.getMaxVoltage(), 0, 270);
    }
}
