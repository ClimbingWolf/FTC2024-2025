package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Config
public class pidConstantsAndFunctions {
    public final double SQRTOF3 = Math.sqrt(3);
    public  static double kPb = 15;
    public  static double kIb = 0;
    public  static double kDb = .8;
    public  static double kFb = .05;

    //PIDF constants for tlarm
    public static double kPt = 15;
    public static double kIt = 0;
    public  static double kDt = .8;
    public  static double kFt = 0.05;

    //constants for PIDtestOptimized

    public static double idleBlarm = 30;
    public static double idleTlarm = 35;

    public static double idleServo = 120;

    public static double servoOffsetTop = 320;
    public static double servoOffsetBottom = -27;

    public static double blarmOffset = 4;
    public static double tlarmOffset = -45;
    public static double posA = .85;
    public static double posB = .62;

    public static double tlarmOffsetFloor= 30;
    public static double blarmOffsetFloor = 29;

    public static double blarmMacroA = 19.8;
    public  static double blarmMacroB = 120;
    public static double tlarmMacroA = -109.8;
    public static double tlarmMacroB = 60;

    public static PIDFController pidfBlarm =  new PIDFController(kPb, kIb, kDb, kFb);

    public static PIDFController pidfTlarm = new PIDFController(kPt, kIt, kDt, kFt);

    public static double barLength = 13;

    public static double radiansToTicks(double radians){
        return radians/Math.PI * 2400;
    }

    public static ArrayList calculateArmThetas(double height, double distance, double barLength, double blarmOffsetDeg, double tlarmOffsetDeg, double servoOffsetDeg){
        double SQRTOF3 = Math.sqrt(3);
        double blarmOffsetFromFlat = degreesToRadians(blarmOffsetDeg);//edit these to edit the degree offset, but its pretty good for now
        double tlarmOffsetFromFlat = degreesToRadians(tlarmOffsetDeg);
        double servoOffset = degreesToRadians(servoOffsetDeg);

        double fullLength = Math.sqrt(height*height + distance*distance + height*distance*SQRTOF3);
        double theta3 = Math.asin((height*SQRTOF3)/(2.0*fullLength));
        double theta1 = Math.acos((fullLength*fullLength - 2.0*barLength*barLength)/(-2.0*barLength*barLength));
        double theta2 = (Math.PI - theta1)/2.0;
        double blarmTheta = Math.PI - (theta2 + blarmOffsetFromFlat)- theta3;
        double tlarmTheta = theta1 - tlarmOffsetFromFlat;
        double servoTheta = -(theta1 + theta2 + theta3) + servoOffset;
        ArrayList storeValues = new ArrayList();
        storeValues.add(blarmTheta);
        storeValues.add(tlarmTheta);
        storeValues.add(servoTheta);

        return storeValues;
    }

    public static ArrayList calculateArmThetasFloor(double distance, double barLength, double blarmOffsetDeg, double tlarmOffsetDeg, double servoOffsetDeg){
        ArrayList storeValues =calculateArmThetas(-1.2, distance+15.8, barLength, blarmOffsetDeg, tlarmOffsetDeg, servoOffsetDeg);
        double blarmAngle = (double)(storeValues.get(0));
        double tlarmAngle = (double)(storeValues.get(1));
        double servoAngle =(double)(storeValues.get(2));
        blarmAngle = Math.PI - blarmAngle;
        tlarmAngle = 2*Math.PI - tlarmAngle;
        storeValues = new ArrayList<>();
        storeValues.add(blarmAngle);
        storeValues.add(tlarmAngle);
        servoAngle = -servoAngle;
        storeValues.add(servoAngle);



        return storeValues;

    }

    public static ArrayList distanceAndAngleDegrees(double distance, double height){
        return null;
    }

    public static double degreesToRadians(double degrees){
        return (Math.PI *degrees /180);
    }
    public static double radiansToDegrees(double radians){
        return (radians*180/Math.PI);
    }

    public static double radiansTo5ServoPos(double radians){return .12*radians/Math.PI + .15;}

    public static double maxDistance(double height, double maxReach){
        double sin2piover3 = 0.866;
        double x = Math.asin((height*sin2piover3)/maxReach);
        double w = Math.PI - x - (2*Math.PI)/3;
        return maxReach*Math.sin(w)/sin2piover3;
    }

    public static double maxHeight(double distance, double maxReach){
        return maxDistance(distance, maxReach);
    }

    public static double avgArrayList(ArrayList arr){
        double total = 0;
        for (int i = 0; i < arr.size(); i++){
            total += (double)(arr.get(i));
        }
        return total/arr.size();
    }

}
