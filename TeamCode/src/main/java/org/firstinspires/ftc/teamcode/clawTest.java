package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "clawTest")
@Config
public class clawTest extends LinearOpMode {



    private Servo claw;
    private Servo clawPitch;
    public static double pitchA = 0;

    public static double pitchB = .6;

    public static double posA = .8;
    public static double posB = .4;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        claw = hardwareMap.get(Servo.class, "claw");
        clawPitch = hardwareMap.get(Servo.class, "clawPitch");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run locks here.
            while (opModeIsActive()) {

                if(gamepad1.a) {
                    claw.setPosition(posA);
                }
                else if(gamepad1.b){
                    claw.setPosition(posB);
                }

                if (gamepad1.dpad_up){
                    clawPitch.setPosition(pitchA);
                }
                else if (gamepad1.dpad_down) {
                    clawPitch.setPosition(pitchB);
                }
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("clawPos", claw.getPosition());
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}