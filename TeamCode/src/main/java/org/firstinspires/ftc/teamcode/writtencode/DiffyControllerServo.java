package org.firstinspires.ftc.teamcode.writtencode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class DiffyControllerServo {
    public Servo left;
    public Servo right;


    public double gearRatio;
    public DiffyControllerServo(Servo left, Servo right, double gearRatio){
        this.gearRatio = gearRatio;
        this.left = left;
        this.right = right;
    }

    public void setPitchAndRollAngleDeg(double pitchDeg, double rollDeg){
        double pitchReal = pitchDeg/355;
        double rollReal = rollDeg/355;
        //double currentPosRight = right.getPosition();
        //double currentPosLeft = left.getPosition();
        left.setPosition(pitchReal + rollReal);
        right.setPosition(pitchReal - rollReal);
    }

}
