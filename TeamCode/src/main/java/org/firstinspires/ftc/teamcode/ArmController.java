package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmController {

    private final DcMotorEx push;
    private final Servo rotator;
    private final Servo pitch;

    public static double rotatorPos;
    public static double pitchPos;

    private final FtcDashboard dashboard;

    public ArmController(HardwareMap hardwareMap) {
        push = hardwareMap.get(DcMotorEx.class, "push");
        rotator = hardwareMap.servo.get("rotator");
        pitch = hardwareMap.servo.get("pitch");

        dashboard = FtcDashboard.getInstance();

        rotatorPos = 0.375;
        pitchPos = 0.5;
    }

    public void run() {
        rotator.setPosition(rotatorPos);
        pitch.setPosition(pitchPos);

        TelemetryPacket t = new TelemetryPacket();
        t.put("pitch", pitch.getPosition());
        t.put("push", push.getCurrentPosition());
        dashboard.sendTelemetryPacket(t);
    }

    public void rotateArm(int degrees) {
        rotatorPos = degrees / 720.0;
    }

    public void setPitch(int pos) {

    }
}