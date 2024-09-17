package org.firstinspires.ftc.teamcode.writtencode;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoArmController {
    public Servo topRot;
    public Servo bottomRot;
    public Servo clawServo;
    public double legLength;
    public double defaultHeight;
    public double topRotOffset;
    public double bottomRotOffset;
    public final double maxServoDegrees = 255;

    public final double clawCloseDegrees = 0;
    public final double clawOpenDegrees = 255;
    public ServoArmController(Servo topRot, Servo bottomRot, double legLength, double defaultHeight, double topRotOffset, double bottomRotOffset, Servo clawServo){
        this.topRot = topRot;
        this.bottomRot = bottomRot;
        this.legLength = legLength;
        this.defaultHeight = defaultHeight;
        this.topRotOffset = topRotOffset;
        this.bottomRotOffset = bottomRotOffset;
        this.clawServo = clawServo;
    }

    public double radians2Servo(double angleInRad){
        return angleInRad/Math.toRadians(maxServoDegrees);
    }
    public void moveToPos(double x, double y){
        double leg3 = Math.sqrt(x*x + y*y);
        double theta2 = 2*Math.acos(0.5 * leg3 * legLength);
        double theta1 = (Math.PI-theta2)/2 + Math.atan(y/x);
        topRot.setPosition(radians2Servo(theta2));
        bottomRot.setPosition(radians2Servo(theta1));
    }

    public void close(){
        clawServo.setPosition(clawCloseDegrees);
    }
    public void open(){
        clawServo.setPosition(clawOpenDegrees);
    }
}
