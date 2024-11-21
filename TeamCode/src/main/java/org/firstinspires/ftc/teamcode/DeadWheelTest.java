package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver.GoBildaOdometryPods;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver.EncoderDirection;

@TeleOp
public class DeadWheelTest extends LinearOpMode {

    FtcDashboard dash;
    GoBildaPinpointDriver pin;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        pin = hardwareMap.get(GoBildaPinpointDriver.class, "pin");
        pin.setOffsets(7, 7);
        pin.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pin.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
        pin.resetPosAndIMU();

        waitForStart();
        resetRuntime();
        while(opModeIsActive()) {
            pin.update();
            TelemetryPacket p = new TelemetryPacket();

            Pose2D pos = pin.pos();
            p.put("X", pos.getX(DistanceUnit.MM));
            p.put("Y", pos.getY(DistanceUnit.MM));
            p.put("Heading", pos.getHeading(AngleUnit.DEGREES));
            p.put("Status", pin.getDeviceStatus());

            

            dash.sendTelemetryPacket(p);
        }
    }
}