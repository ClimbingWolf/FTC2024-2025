package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class TrajectoryManager {
    ArrayList<Trajectory> trajectories = new ArrayList<>();
    ArrayList<Integer> sleepTimes = new ArrayList<>();
    SampleMecanumDrive drive;
    Pose2d start;
    int index = -1;
    int sleepIndex = 0;
    int offset = 0;

    public TrajectoryManager(SampleMecanumDrive drive, Pose2d start) {
        this.drive = drive;
        this.start = start;
    }


    public TrajectoryManager lineToConstantHeading(Vector2d vec) {
        Pose2d startPos = trajectories.isEmpty() ? start : trajectories.get(trajectories.size()-1).end();
        if (offset != 0) {
            startPos = startPos.plus(new Pose2d(0, 0, Math.toRadians(offset)));
            offset = 0;
        }
        trajectories.add(drive.trajectoryBuilder(startPos)
                        .lineToConstantHeading(vec)
                        .build());
        return this;
    }

    public TrajectoryManager lineToLinearHeading(Pose2d pos) {
        Pose2d startPos = trajectories.isEmpty() ? start : trajectories.get(trajectories.size()-1).end();
        if (offset != 0) {
            startPos = startPos.plus(new Pose2d(0, 0, Math.toRadians(offset)));
            offset = 0;
        }
        trajectories.add(drive.trajectoryBuilder(trajectories.isEmpty() ? start : trajectories.get(trajectories.size()-1).end())
                        .lineToLinearHeading(pos)
                        .build());
        return this;
    }

    public TrajectoryManager turn(int angle) {
        offset = angle;
        return this;
    }

    public void nextTrajectory() throws InterruptedException {
        index++;
        drive.followTrajectory(trajectories.get(index));
    }

    public TrajectoryManager waitSeconds(double seconds) {
        return this;
    }
}