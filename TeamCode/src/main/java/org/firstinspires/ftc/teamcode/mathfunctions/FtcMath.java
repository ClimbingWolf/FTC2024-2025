package org.firstinspires.ftc.teamcode.mathfunctions;

public class FtcMath {
    public static double rotateX(double x, double y, double thetaRadians){
        return x*Math.cos(thetaRadians) - y*Math.sin(thetaRadians);
    }
    public static double rotateY(double x, double y, double thetaRadians){
        return x*Math.sin(thetaRadians) + y*Math.cos(thetaRadians);
    }
}
