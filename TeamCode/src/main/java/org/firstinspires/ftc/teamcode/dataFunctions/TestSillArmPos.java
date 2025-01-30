package org.firstinspires.ftc.teamcode.dataFunctions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Point;

import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "EVIL TELEOP")
@Config
public class TestSillArmPos extends LinearOpMode {

    public Servo pitch;
    public Servo rotator;

    public MotorEx push;

    public FtcDashboard dash;
    public TelemetryPacket packet;


    public  double kP = 160;
    public  double kI = 0;
    public  double kD = 0.4;

    public ExtenderController extenderController;
    public static double dist = 0;
    public static double xReach = 16;
    public static double yReach = 0;
    public static double zReach = 0;

    public static double clawClose = 0.2;
    public static double clawOpen = 0.4;

    public static double controlSpeed= 0.1;

    public Servo claw;

    public static double xOffsetCam = 0;
    public static double zOffsetCam = 7;
    public Point objPoint;
    public Init init;
    public static double rotatorPos = 0.5;
    public static double pitchPos = 0.5;




    @Override
    public void runOpMode() {
        init = new Init(gamepad1, false, "", hardwareMap);
        //rotator end => 0.59
        //rotator start => 0.04
        //packet.put("data", orientationArrList.toString());
        rotator = hardwareMap.servo.get("rotator");
        pitch = hardwareMap.servo.get("pitch");
        waitForStart();
        while (opModeIsActive()) {
            init.virtualGamepad = gamepad1;
            init.loop();
        }
    }
}
