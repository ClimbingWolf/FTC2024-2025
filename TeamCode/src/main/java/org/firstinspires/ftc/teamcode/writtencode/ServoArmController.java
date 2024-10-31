package org.firstinspires.ftc.teamcode.writtencode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ServoArmController {

    public Servo topRot;
    public FtcDashboard dash = FtcDashboard.getInstance();

    public TelemetryPacket packet = new TelemetryPacket();

    public static double piover4constantidk = Math.PI/4;
    public Servo bottomRot;
    public Servo clawServo;
    public double legLength;
    public double defaultHeight;
    public double topRotOffset;
    public double bottomRotOffset;
    public static double maxServoDegreesBottom = 320;

    public double theta1;
    public double theta2;

    public static double maxServoDegreesTop = 320;

    public static double clawCloseDegrees = 0;
    public static double clawOpenDegrees = 255;

    public double leg3;
    public ServoArmController(Servo topRot, Servo backRot, double legLength, double defaultHeight, double topRotOffset, double bottomRotOffset, Servo clawServo){
        this.topRot = topRot;
        this.bottomRot = backRot;
        this.legLength = legLength;
        this.defaultHeight = defaultHeight;
        this.topRotOffset = topRotOffset;
        this.bottomRotOffset = bottomRotOffset;
        this.clawServo = clawServo;
    }
    public double radians2ServoBottom(double angleInRad){
        return angleInRad/Math.toRadians(maxServoDegreesBottom);
    }

    public double radians2ServoTop(double angleInRad){
        return angleInRad/Math.toRadians(maxServoDegreesTop);
    }
    public void moveToPos(double x, double y, double topRotOffset, double bottomRotOffset){
        //distance to point
        leg3 = Math.sqrt(x*x + y*y);
        //top angle of the servo triange
        theta2 = 2*Math.acos(0.5 * leg3 / legLength);
        //bottom angle calculated using identities of isosceles triangles
        theta1 = Math.PI/2 - (Math.PI-theta2)/2 + Math.atan(y/x);
        theta1/=2;
        //set position based on offset and convert to servo ticks
        topRot.setPosition(radians2ServoTop(theta2) + topRotOffset);
        bottomRot.setPosition(radians2ServoBottom(-(theta1 + piover4constantidk)) + bottomRotOffset);
    }

    public void outputThetas(){
        packet.put("theta1",theta1);
        packet.put("theta2", theta2);
        packet.put("leg3", leg3);
        dash.sendTelemetryPacket(packet);

    }

    public void setTopRot(double ticks){

        topRot.setPosition(ticks);
    }

    public void setBottomRot(double ticks){
        bottomRot.setPosition(ticks);
    }

    public void setBottomRad(double rad){
        setBottomRot(radians2ServoBottom(rad));
    }

    public void setTopRad(double rad){
        setTopRot(radians2ServoTop(rad));
    }

    public void setZero() {
        setBottomRot(0);
        setBottomRot(0);
    }
    public void close(){
        clawServo.setPosition(clawCloseDegrees);
    }
    public void open(){
        clawServo.setPosition(clawOpenDegrees);
    }
}
