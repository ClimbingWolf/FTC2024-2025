package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dataFunctions.Init;

@Config
@TeleOp(name="Teleop")
public class BasicTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Init init = new Init(gamepad1, false, "", hardwareMap);
        Servo rotator = hardwareMap.servo.get("rotator");
        Servo pitch = hardwareMap.servo.get("pitch");
        DcMotorEx push = hardwareMap.get(DcMotorEx.class, "push");

        double pitchPos = 0;
        double rotatorPos = 0;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                pitchPos += 0.01;
                pitch.setPosition(pitchPos);
            } else if (gamepad1.b) {
                pitchPos -= 0.01;
                pitch.setPosition(pitchPos);
            }

            if (gamepad1.b) {
                rotatorPos += 0.01;
                rotator.setPosition(rotatorPos);
            } else if (gamepad1.y) {
                rotatorPos -= 0.01;
                rotator.setPosition(rotatorPos);
            }

            if (gamepad1.dpad_up) {
                push.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                push.setPower(-0.5);
            }

            init.virtualGamepad = gamepad1;
            init.loop();
        }
    }
}
