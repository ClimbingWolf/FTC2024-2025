package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class ClawController {
    private final CRServo claw;
    private boolean holding = true;

    public ClawController(HardwareMap hardwareMap) {
        // TODO: Claw Name
        claw = hardwareMap.crservo.get("claw");
    }

    public void setPower(int power) {
        claw.setPower(power);
    }

    public void releaseBlock() {
        holding = false;
    }

    public Action grabBlock(boolean eject) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (eject) claw.setPower(.6);
                else claw.setPower(-.6);
                return timer.now(TimeUnit.MILLISECONDS) <= 200;
            }
        };
    }

    public Action holdBlock() {
        holding = true;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPower(-.3);
                return holding;
            }
        };
    }

}
