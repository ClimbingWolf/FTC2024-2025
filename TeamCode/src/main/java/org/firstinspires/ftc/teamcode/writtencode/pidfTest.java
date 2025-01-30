package org.firstinspires.ftc.teamcode.writtencode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dataFunctions.ExtenderController;

@TeleOp(name = "ඞPidfTest")
@Config
public class pidfTest extends LinearOpMode {
    Servo rotator;
    Servo pitch;
    MotorEx push;
    ExtenderController extenderController;
    public static double kP=150;
    public static double kI=0;
    public static double kD=0.7;
    public static double dist = 0;

    public static double distMult = 1;

    public static double motorPos;

    public TelemetryPacket packet = new TelemetryPacket();
    public FtcDashboard dash;
    @Override
    public void runOpMode() {
        dash = FtcDashboard.getInstance();
        push = new MotorEx(hardwareMap, "push");
        pitch = hardwareMap.get(Servo.class, "pitch");
        rotator = hardwareMap.get(Servo.class, "rotator");
        push.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extenderController = new ExtenderController(rotator, pitch, push);
        waitForStart();
        while (opModeIsActive()) {

            //extenderController.setPitchRot(0.3);
            extenderController.updatePID(kP, kI, kD);
            extenderController.pidController.setP(kP);
            extenderController.pidController.setI(kI);
            extenderController.pidController.setD(kD);
            extenderController.pidfPush(dist * distMult);
            packet.put("encoder pos", push.encoder.getPosition());
            dash.sendTelemetryPacket(packet);
            //motorPos = push.getCurrentPosition()/extenderController.ticksPerRev * extenderController.inPerRev;
        }
    }
}
