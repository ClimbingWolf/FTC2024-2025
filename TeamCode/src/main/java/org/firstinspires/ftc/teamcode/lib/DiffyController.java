package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DiffyController {
    Servo leftDiffy;
    Servo rightDiffy;

    public DiffyController(HardwareMap hardwareMap) {
        leftDiffy = hardwareMap.servo.get("leftDiffy");
        rightDiffy = hardwareMap.servo.get("rightDiffy");
    }

    public void rotateWrist(int degrees) {
        leftDiffy.setPosition(degrees);
        rightDiffy.setPosition(-degrees);
    }

    public void rotateClaw(int degrees) {
        // TODO: Implement
    }
}
