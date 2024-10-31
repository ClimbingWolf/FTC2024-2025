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
    public FtcDashboard dash = FtcDashboard.getInstance();

    public TelemetryPacket packet = new TelemetryPacket();

    public double legLength;

    public double topTheta;
    public double bottomTheta;
    public double leg3;

    public ArrayList<Double> topServoStarts;
    public ArrayList<Double> bottomServoStarts;
    public ArrayList<Double> topServoEnds;
    public ArrayList<Double> bottomServoEnds;


    public ArrayList<Servo> topServos;
    public ArrayList<Servo> bottomServos;

    public ArmServoHolder(ArrayList<Servo> topServos, ArrayList<Servo> bottomServos) {
        this.topServos = topServos;
        this.bottomServos = bottomServos;

    }

    public void addBottomStart(double value){
        bottomServoStarts.add(value);
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
        }
        for (int i = 0; i < bottomServos.size(); i++) {
            packet.put("bottomServo" + i, bottomServos.get(i).getPosition());
        }
        dash.sendTelemetryPacket(packet);

    }

    public void setTopRot(double percentage) {
        for (int i = 0; i < topServos.size(); i++) {
            topServos.get(i).setPosition(topServoStarts.get(i) + topServoEnds.get(i) * percentage);
        }
    }

    public void setBottomRot(double percentage) {
        for (int i = 0; i < bottomServos.size(); i++) {
            bottomServos.get(i).setPosition(bottomServoEnds.get(i) + bottomServoStarts.get(i) * percentage);
        }
    }
}

