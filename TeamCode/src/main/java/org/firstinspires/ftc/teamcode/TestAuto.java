package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.technotigers.technolib.actions.Actions;
import com.technotigers.technolib.actions.SequentialAction;

import org.firstinspires.ftc.teamcode.auto.BlueLeft;
import org.firstinspires.ftc.teamcode.devices.Arm;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class TestAuto extends LinearOpMode {
    private Follower follower;
    private BlueLeft blueLeft;
    private Arm arm;

    private PathChain scorePreload;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        Actions.runBlocking(new SequentialAction(
            blueLeft.runPath(scorePreload),
            arm.moveArm(135, 40)
        ));
    }

    private void initialize() {
        follower = new Follower(hardwareMap);
        blueLeft = new BlueLeft(follower);
        arm = new Arm(hardwareMap);

        Pose startPose = new Pose(108, 9, Math.toRadians(90));
        Pose scorePose = new Pose(125, 12, Math.toRadians(90));

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .build();
    }
}