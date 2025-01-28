package org.firstinspires.ftc.teamcode.writtencode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@Config
public class ExtenderController {


    public TelemetryPacket packet = new TelemetryPacket();

    public double legLength;

    public double ticksPerRev = 537.6 * 0.703 * 0.527472; //this changes with the motor so just test it
    //also it might not actually be ticksPerRev idk, but it works

    public double spoolDiameter = 3.3019685; //inches

    public double inPerRev = spoolDiameter * Math.PI;


    public ArrayList<Servo> topServos;
    public ArrayList<Servo> bottomServos;


    public ArrayList<Servo> rotatorServos;

    public Servo rotator;
    public Servo pitch;
    public MotorEx push;

    public PIDController pidController;
    public double pitchStart = 0.38;
    public double pitchEnd =0.1;
    public double power;
    public double rotEnd =0.77;
    public double rotStart =0.485;
    //pitch vertical => 0
    //pitch horizontal => 0.53
    //rotator fullLeft => 0.2
    //rotator fullRight => 0.65

    public double rotatorServoRangeDegrees = 180;

    public double kP =120;
    public double kI = 0;
    public double kD = 0.7;
    public static double armFromGround = 9.5;
    public double dist;
    public double flatDist;
    public double rotatorTheta;
    public double pitchAngle;

    public ExtenderController(Servo rotator, Servo pitch, MotorEx push){
        this.rotator = rotator;
        this.pitch = pitch;
        this.push = push;
        push.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        push.stopAndResetEncoder();
        push.setInverted(true);
        this.pidController = new PIDController(kP, kI, kD);
    }

    public void updatePID(double kP, double kI, double kD){
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);

    }

    public void pidfPush(double dist){
        dist = dist - 16;
        pidController.setSetPoint(dist/inPerRev * ticksPerRev);
        power = pidController.calculate(push.getCurrentPosition());
        push.setVelocity(power);
    }

    public void setPitchRot(double percentage) {
        pitch.setPosition(pitchEnd + (pitchStart - pitchEnd) * percentage);
    }
    public void setRotatorRot(double percentage){
        rotator.setPosition(rotStart + (rotEnd - rotStart) * percentage);
    }

    public void setPos(double x, double y, double z){
        if(x > 8){
            x =8;
        }
        else if (x<-8){
            x = -8;
        }
        y = y - armFromGround;
        dist = Math.sqrt(x*x + y*y + z*z);
        flatDist = Math.sqrt(x*x + z*z);
        pitchAngle = Math.atan(y/flatDist);
        rotatorTheta = 0;
        if(dist < 25 + 16 && dist >= 16){
            pidfPush(dist-16);
            setPitchRot(pitchAngle/(Math.PI/2));
            if(z !=0){
                rotatorTheta = Math.atan(z/x);
            }
            double rotatorPercentage = (rotatorTheta + Math.toRadians(rotatorServoRangeDegrees/2))/Math.toRadians(rotatorServoRangeDegrees);
            setRotatorRot(rotatorPercentage);
        }

    }
}

