package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.technotigers.technolib.actions.Action;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class BlueLeft {
    private final Follower follower;

    public BlueLeft(Follower follower) {
        this.follower = follower;
    }

    public Action runPath(PathChain path) {
        return new Action() {
            private final PathChain pathing = path;

            @Override
            public boolean run() {
                follower.followPath(pathing);
                return follower.isBusy();
            }
        };
    }
}