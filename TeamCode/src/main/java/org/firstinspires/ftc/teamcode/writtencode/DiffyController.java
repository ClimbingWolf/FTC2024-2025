package org.firstinspires.ftc.teamcode.writtencode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class DiffyController {
    public CRServo left;
    public CRServo right;

    public AnalogInput rightEncoder;
    public AnalogInput leftEncoder;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;
    public PIDFController pidf = new PIDFController(kP, kI, kD, kF);


    public double gearRatio;
    public DiffyController(CRServo left, CRServo right, double gearRatio, AnalogInput rightEncoder, AnalogInput leftEncoder){
        this.left = left;
        this.right = right;
        this.gearRatio = gearRatio;
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
    }

    public void setPitchAndRollAngleDeg(double pitchDeg, double rollDeg){
        double pitchReal = pitchDeg/360;
        double rollReal = rollDeg/360;
        double currentPosRight = rightEncoder.getVoltage();
        double currentPosLeft = leftEncoder.getVoltage();
        //double angle = currentPosRight - currentPosLeft;
        double outputLeft = pidf.calculate(currentPosLeft, pitchReal + rollReal);
        double outputRight = pidf.calculate(currentPosRight, pitchReal - rollReal);
        left.setPower(outputLeft);
        right.setPower(outputRight);
    }

}
