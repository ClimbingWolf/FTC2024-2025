package org.firstinspires.ftc.teamcode;
import java.io.File;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.nio.charset.StandardCharsets;


@Autonomous(name = "Sample Auto")
@Config
public class SampleAuto extends LinearOpMode {

    FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectoryManager manager = new TrajectoryManager(drive, new Pose2d(-15.23, -63.46, Math.toRadians(90.0)))
                        .lineToConstantHeading(new Vector2d(-2.97, -33.37))
                        .waitSeconds(1.5)
                        .lineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(75)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-57, -51, Math.toRadians(93.00)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-56.23, -55.09, Math.toRadians(125.00)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-55.54, -11.43, Math.toRadians(90)))
                        .turn(90)
                        .lineToConstantHeading(new Vector2d(-25.00, -10.51));

        Pose2d startPose = new Pose2d(-15.23, -63.46, Math.toRadians(90.0));
        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;
        manager.nextTrajectory();
        timer.reset();
        //preload
        while(timer.seconds() < 1.5) {

        }
        manager.nextTrajectory();
        timer.reset();
        //pickup block 1
        while(timer.seconds() < 1.5) {

        }
        manager.nextTrajectory();
        timer.reset();
        // score block 1
        while(timer.seconds() < 1.5) {

        }
        manager.nextTrajectory();
        timer.reset();
        // pickup block 2
        while(timer.seconds() < 1.5) {

        }
        manager.nextTrajectory();
        timer.reset();
        // score block 2
        while(timer.seconds() < 1.5) {

        }
        manager.nextTrajectory();
        timer.reset();
        // pickup block 3
        while(timer.seconds() < 1.5) {

        }
        manager.nextTrajectory();
        timer.reset();
        // score block 3
        while(timer.seconds() < 1.5) {

        }
        while(manager.index <= manager.trajectories.size()) {
            manager.nextTrajectory();
        }
    }
}