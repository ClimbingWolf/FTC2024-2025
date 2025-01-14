package org.firstinspires.ftc.teamcode.devices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.technotigers.technolib.actions.Action;
import com.technotigers.technolib.actions.ParallelAction;

public class Arm {
    private final Servo turret;
    private final Servo extendoSuperSuperArmAmongusSusඞඞඞ;

    public Arm(HardwareMap hardwareMap) {
        turret = hardwareMap.servo.get("turret");
        extendoSuperSuperArmAmongusSusඞඞඞ = hardwareMap.servo.get("arm");
    }

    public Action spinTurret(int degrees) {
        return new Action() {
            private final int d = degrees;
            @Override
            public boolean run() {
                turret.setPosition(d);
                return turret.getPosition() != d;
            }
        };
    }

    public Action extendArm(int degrees) {
        return new Action() {
            private final int d = degrees;

            @Override
            public boolean run() {
                extendoSuperSuperArmAmongusSusඞඞඞ.setPosition(d);
                return extendoSuperSuperArmAmongusSusඞඞඞ.getPosition() != d;
            }
        };
    }

    public Action moveArm(int degrees, int length) {
        return new ParallelAction(
                spinTurret(degrees),
                extendArm(length)
        );
    }

}