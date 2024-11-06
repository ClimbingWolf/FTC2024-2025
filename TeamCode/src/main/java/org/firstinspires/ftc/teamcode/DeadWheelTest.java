package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;

@TeleOp
public class DeadWheelTest extends LinearOpMode {

    FtcDashboard dash;
    GoBildaPinpointDriver pin;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();


        waitForStart();
        while(opModeIsActive()) {


            TelemetryPacket t = new TelemetryPacket();
            t.put("amogus", "123");
            dash.sendTelemetryPacket(t);
        }
    }
}
