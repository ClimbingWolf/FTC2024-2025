package org.firstinspires.ftc.teamcode.writtencode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@Config
public class ArmServoHolder {

    public Servo topRot;
    public double rotatorTheta;
    public double rotatorServoRangeDegrees = 128;
    public FtcDashboard dash = FtcDashboard.getInstance();

    public TelemetryPacket packet = new TelemetryPacket();

    public double legLength =8.5;

    public double topTheta;
    public double bottomTheta;
    public double leg3;

    public ArrayList<Double> topServoStarts;
    public ArrayList<Double> bottomServoStarts;
    public ArrayList<Double> topServoEnds;
    public ArrayList<Double> bottomServoEnds;


    public ArrayList<Servo> topServos;
    public ArrayList<Servo> bottomServos;
    public ArrayList<Double> rotatorServosEnds;
    public ArrayList<Double> rotatorServosStarts;

    public ArrayList<Servo> rotatorServos;
    public ArmServoHolder(ArrayList<Servo> topServos, ArrayList<Servo> bottomServos) {
        this.rotatorServos = rotatorServos;
        this.topServos = topServos;
        this.bottomServos = bottomServos;
        this.bottomServoStarts = new ArrayList<>();
        this.topServoStarts = new ArrayList<>();
        this.bottomServoEnds = new ArrayList<>();
        this.topServoEnds = new ArrayList<>();
        this.rotatorServos = new ArrayList<>();
        this.rotatorServosEnds = new ArrayList<>();
        this.rotatorServosStarts = new ArrayList<>();
    }
    public ArmServoHolder(ArrayList<Servo> topServos, ArrayList<Servo> bottomServos, ArrayList<Servo> rotatorServos) {
        this.rotatorServos = rotatorServos;
        this.topServos = topServos;
        this.bottomServos = bottomServos;
        this.bottomServoStarts = new ArrayList<>();
        this.topServoStarts = new ArrayList<>();
        this.bottomServoEnds = new ArrayList<>();
        this.topServoEnds = new ArrayList<>();
        this.rotatorServos = rotatorServos;
        this.rotatorServosEnds = new ArrayList<>();
        this.rotatorServosStarts = new ArrayList<>();
    }

    public void addBottomStart(double value){
        bottomServoStarts.add(value);
    }

    public void addRotatorStart(double value){
        rotatorServosStarts.add(value);
    }
    public void addRotatorEnd(double value){
        rotatorServosEnds.add(value);
    }
    public void setRotatorPos(double percentage){
        for (int i = 0; i < topServos.size(); i++) {
            rotatorServos.get(i).setPosition((rotatorServosEnds.get(i) - rotatorServosStarts.get(i)) * percentage + rotatorServosStarts.get(i));
        }
    }
    public void addTopStart(double value){
        topServoStarts.add(value);
    }
    public void addBottomEnd(double value){
        bottomServoEnds.add(value);
    }
    public void addTopEnd(double value){
        topServoEnds.add(value);
    }
    public void moveToPos(double x, double y) {
        leg3 = Math.sqrt(x * x + y * y);
        //leg3 is the distance away
        topTheta = 2 * Math.asin(0.5 * leg3 / legLength);
        //radians of top angle
        if (x != 0) {
            bottomTheta = (Math.PI - topTheta) / 2 + Math.atan(y / x);
        } else {
            bottomTheta = Math.PI / 2;
        }
        //radians of bottom angle
        setBottomRot(bottomTheta / (Math.PI / 2));
        setTopRot(topTheta / (Math.PI));
    }
    public void moveToPos(double x, double y, double z) {
        double xRotated = Math.sqrt(x*x + z*z);
        leg3 = Math.sqrt(xRotated * xRotated + y * y);
        //leg3 is the distance away
        topTheta = 2 * Math.asin(0.5 * leg3 / legLength);
        //radians of top angle
        if (x != 0) {
            bottomTheta = (Math.PI - topTheta) / 2 + Math.atan(y / xRotated);
        } else {
            bottomTheta = Math.PI / 2;
        }
        if(z !=0){
            rotatorTheta = Math.atan(x/z);
        }
        else{
            rotatorTheta = 0;
        }

        //radians of bottom angle
        setBottomRot(bottomTheta / (Math.PI / 2));
        setTopRot(topTheta / (Math.PI));
        setRotatorPos((rotatorTheta + Math.toRadians(rotatorServoRangeDegrees/2))/rotatorServoRangeDegrees);

    }





    public void outputConstants() {
        packet.put("topServosStart", topServoStarts);
        packet.put("bottomServosStart", bottomServoStarts);
        packet.put("topServoEnds", topServoEnds);
        packet.put("bottomServosEnds", bottomServoEnds);
        dash.sendTelemetryPacket(packet);
    }

    public void outputThetas() {
        packet.put("topTheta", topTheta);
        packet.put("bottomTheta", bottomTheta);
        dash.sendTelemetryPacket(packet);
    }

    public void outputServoPos() {
        for (int i = 0; i < topServos.size(); i++) {
            packet.put("topServo" + i, topServos.get(i).getPosition());
            packet.put("topServo" + i +"Percentage", topTheta / (Math.PI));
        }
        for (int i = 0; i < bottomServos.size(); i++) {
            packet.put("bottomServo" + i, bottomServos.get(i).getPosition());
            packet.put("bottomServo" + i +"Percentage", bottomTheta / (Math.PI / 2));
        }

        dash.sendTelemetryPacket(packet);

    }

    public void setTopRot(double percentage) {
        for (int i = 0; i < topServos.size(); i++) {
            topServos.get(i).setPosition((topServoEnds.get(i) - topServoStarts.get(i)) * percentage + topServoStarts.get(i));
        }
    }

    public void setBottomRot(double percentage) {
        for (int i = 0; i < bottomServos.size(); i++) {
            bottomServos.get(i).setPosition((bottomServoStarts.get(i) - bottomServoEnds.get(i)) * percentage + bottomServoEnds.get(i));
        }
    }
}

