package org.firstinspires.ftc.teamcode.dataFunctions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Config
public class ExtenderControllerBrokenArm {


    public TelemetryPacket packet = new TelemetryPacket();

    public double legLength;

    //public double ticksPerRev = 537.6 * 0.703; //this changes with the motor so just test it
    //also it might not actually be ticksPerRev idk, but it works

    //public double spoolDiameter = 3.3019685; //inches

    //public double inPerRev = spoolDiameter * Math.PI;


    public ArrayList<Servo> topServos;
    public ArrayList<Servo> bottomServos;


    public ArrayList<Servo> rotatorServos;

    public Servo rotator;
    public Servo pitch;
    //public MotorEx push;

    public PIDController pidController;
    public double pitchStart = 0.2;
    public double pitchEnd =0.42;
    public double power;
    public double rotEnd =0.485;
    public double rotStart =0.2;
    //pitch vertical => 0
    //pitch horizontal => 0.53
    //rotator fullLeft => 0.2
    //rotator fullRight => 0.65

    public double rotatorServoRangeDegrees = 180;

    public double kP =120;
    public double kI = 0;
    public double kD = 0.7;
    public double dist;
    public double flatDist;
    public double rotatorTheta;
    public double pitchAngle;

    public ExtenderControllerBrokenArm(Servo rotator, Servo pitch){
        this.rotator = rotator;
        this.pitch = pitch;
        //push.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //push.stopAndResetEncoder();
        //push.setInverted(true);
        this.pidController = new PIDController(kP, kI, kD);
    }

    public void updatePID(double kP, double kI, double kD){
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);

    }


    public void setPitchRot(double percentage) {
        pitch.setPosition(pitchEnd + (pitchStart - pitchEnd) * percentage);
    }
    public void setRotatorRot(double percentage){
        rotator.setPosition(rotStart + (rotEnd - rotStart) * percentage);
    }

    public void setPos(double x, double y, double z){
        dist = Math.sqrt(x*x + 81 + z*z); //beecause the arm has to move down
        flatDist = Math.sqrt(x*x + z*z);
        pitchAngle = Math.atan(y/flatDist);
        rotatorTheta = 0;
        if(dist <= 17){
            if(y==0){
                pitch.setPosition(0.35);
            }
            else {
                pitch.setPosition(0.4);
            }
            if(z !=0){
                rotatorTheta = Math.atan(z/Math.sqrt(17*17 - z*z));
            }
            double rotatorPercentage = (rotatorTheta + Math.toRadians(rotatorServoRangeDegrees/2))/Math.toRadians(rotatorServoRangeDegrees);
            if(rotatorPercentage > 0.7){
                rotatorPercentage = 0.7;
            }
            else if(rotatorPercentage < 0.3){
                rotatorPercentage = 0.3;
            }
            setRotatorRot(rotatorPercentage);
        }

    }
}

