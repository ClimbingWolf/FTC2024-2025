package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp(name = "BlarmTest")
@Config
public class PidTeleopBlarm extends LinearOpMode {

    public static double distanceAdjuster = 15.5;
    public static double distance = distanceAdjuster;


    public ArrayList anglesOutputs= new ArrayList();

    public ArrayList banana = new ArrayList();
    //banana store the values of blarmAngle and hypotenuse distance
    public static double tlarmP = 13.0;
    public static double tlarmScale = 19.0;
    public static double tlarmOffset = 40.0;
    public static double tlarmI = 0.0;
    public static double tlarmD = 0.2;
    public static double tlarmF = 2.0;
    public static double height = 0;
    public static double randomConstant = 0;
    public static double kP = 15;
    public  static int degreeOffsetBottom = -5;
    //negative for bar face low, positive for high
    public int degreeOffsetTop = 4;
    public  static double kI = 0;
    public  static double kD = 1;
    public static double angleScaleBlarm = 13;

    public double gravityScaleAdjusted = 0;
    public static double gravityScaleAdjustedAdjuster = 200;

    public static double gravityScale = 1495;
    public double blarmDegreesFromFlat = 180 +degreeOffsetBottom;

    public  static double kF = 2;
    public int moveTo = 0;
    public double gravityModifier = 0;
    public double gravityModifierTlarm = 0;
    public static double blarmTheta = 0;
    public static double gravityScaleTlarm = -200;

    @Override
    public void runOpMode() {
        MotorEx blarm =  new MotorEx(hardwareMap, "blarm");
        MotorEx tlarm = new MotorEx(hardwareMap, "tlarm");

        DcMotor bright = hardwareMap.dcMotor.get("bright");
        DcMotor bleft = hardwareMap.dcMotor.get("bleft");
        DcMotor fright = hardwareMap.dcMotor.get("fright");
        DcMotor fleft = hardwareMap.dcMotor.get("fleft");

        fleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        blarm.setInverted(true);
        blarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        tlarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        blarm.resetEncoder();
        tlarm.resetEncoder();
        PIDFController pidfBlarm = new PIDFController(kP, kI, kD, kF);
        PIDFController pidfTlarm = new PIDFController(tlarmP, tlarmI,tlarmD, tlarmF);

        pidfBlarm.setSetPoint(moveTo);
        pidfTlarm.setSetPoint(moveTo);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            //angles for both arms
            distance = distanceAdjuster;
            banana = distanceAndAngleDegrees(distance, height);
            distance = (double)(banana.get(0));
            blarmTheta = -(double)(banana.get(0))*randomConstant;
            anglesOutputs = calculateArmThetas(0, distance, 15.5);
            //blarm stuff
            moveTo =  (int)(180 - degreeOffsetBottom -degreesToTicks((double)(anglesOutputs.get(0))));
            gravityModifier = epicGravityMath(blarm.getCurrentPosition(), degreesToTicks(blarmDegreesFromFlat));
            gravityModifierTlarm = epicGravityMath(tlarm.getCurrentPosition() - blarm.getCurrentPosition(), degreesToTicks(degreeOffsetBottom));
            gravityScaleAdjusted = gravityScale * gravityModifierTlarm * gravityScaleAdjustedAdjuster;
            double outputBlarm = pidfBlarm.calculate(blarm.getCurrentPosition(), (moveTo + blarmTheta*0.9) * angleScaleBlarm);
            outputBlarm = outputBlarm + gravityModifier*gravityScale;
            blarm.setVelocity(outputBlarm);
            //tlarm stuff

            moveTo =  (int)((degreesToTicks((double)(anglesOutputs.get(1)))-tlarmOffset) * tlarmScale) ;
            double output = pidfTlarm.calculate(tlarm.getCurrentPosition(), moveTo);
            tlarm.setVelocity(output + gravityScaleTlarm * gravityModifierTlarm);

            //telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("distance", distance);
            packet.put("gravityModifier", gravityModifier);
            packet.put("pos", blarm.getCurrentPosition());
            packet.put("bottomTheta", degreesToTicks((double)(anglesOutputs.get(0))));
            packet.put("topTheta", degreesToTicks((double)(anglesOutputs.get(1))));
            dashboard.sendTelemetryPacket(packet);


        }
    }
    public double epicGravityMath(double encoderPos, double ticksFromFlat){
        return -(Math.cos(Math.PI/268.2*(encoderPos+ticksFromFlat)));
    }

    public double degreesToTicks(double degrees){
        return degrees*1.455;
    }

    public double ticksToDegrees(double ticks){
        return ticks/1.455;
    }

    public ArrayList calculateArmThetas(double height, double distance, double barLength){
        double x = distance;
        double h2 = height;
        double l = barLength;
        ArrayList outputs = new ArrayList();
        double l3 = Math.sqrt(Math.pow(Math.pow(h2, 2) + ((Math.sqrt(3)*h2)/3.0 + x),2));
        double h = Math.sqrt(Math.pow(l, 2)- (Math.pow(l3,2)/4));
        outputs.add(Math.toDegrees(Math.asin(h2/l3) + Math.asin(h/l)));
        outputs.add(180-2*Math.toDegrees(Math.asin(h/l)));
        return outputs;
    }

    public ArrayList distanceAndAngleDegrees(double distance, double height){
        ArrayList values = new ArrayList();
        double d1 = Math.sqrt(distance*distance+height*height + distance*height);
        double theta = Math.asin((Math.sqrt(3)/(2*d1))*height);
        values.add(d1);
        values.add(theta);
        return values;
    }
}