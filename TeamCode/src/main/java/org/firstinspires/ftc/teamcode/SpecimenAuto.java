package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Specimen Auto")
public class SpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectoryManager trajectoryManager = new TrajectoryManager(drive, new Pose2d(15.46, -64, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(4.62, -34.38))
                .lineToConstantHeading(new Vector2d(61.38, -61.85));

        waitForStart();
        if(isStopRequested()) return;
        trajectoryManager.nextTrajectory();
        Thread.sleep(1000);
        trajectoryManager.nextTrajectory();
    }
}
