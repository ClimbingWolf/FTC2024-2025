package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class DeadWheelTest extends LinearOpMode {
    FtcDashboard dash;
    DcMotor fleft;
    DcMotor bleft;
    DcMotor fright;
    DcMotor bright;

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.dcMotor.get("fleft");
        fright = hardwareMap.dcMotor.get("fright");
        bleft = hardwareMap.dcMotor.get("bleft");
        bright = hardwareMap.dcMotor.get("bright");

        fleft.setDirection(DcMotorSimple.Direction.REVERSE);
        bleft.setDirection(DcMotorSimple.Direction.REVERSE);

        dash = FtcDashboard.getInstance();

        waitForStart();
        resetRuntime();
        while(opModeIsActive()) {
            TelemetryPacket t = new TelemetryPacket();

            dash.sendTelemetryPacket(t);
        }
    }
}