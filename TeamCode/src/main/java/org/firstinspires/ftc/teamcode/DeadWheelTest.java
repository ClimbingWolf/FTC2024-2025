package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ClawController;

@TeleOp
@Config
public class DeadWheelTest extends LinearOpMode {
    FtcDashboard dash;
    MecanumDrive drive;
    ClawController claw;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        dash = FtcDashboard.getInstance();
        claw = new ClawController(hardwareMap);

        waitForStart();
        resetRuntime();
        while(opModeIsActive()) {
            Actions.runBlocking(claw.grabBlock(false));
        }
    }
}