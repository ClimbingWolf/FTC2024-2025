package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class RobotNoWorky extends LinearOpMode {
    private DcMotor fleft;
    private DcMotor bleft;
    private DcMotor fright;
    private DcMotor bright;

    public static double speed;

    @Override
    public void runOpMode() throws InterruptedException {
        fleft = hardwareMap.dcMotor.get("fleft");
        bleft = hardwareMap.dcMotor.get("bleft");
        fright = hardwareMap.dcMotor.get("fright");
        bright = hardwareMap.dcMotor.get("bright");

        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);

        double flp;
        double blp;
        double frp;
        double brp;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                flp = speed;
            } else if (gamepad1.dpad_up) {
                flp = -speed;
            } else {
                flp = 0;
            }

            if (gamepad1.b) {
                frp = speed;
            } else if (gamepad1.dpad_right) {
                frp = -speed;
            } else {
                frp = 0;
            }

            if (gamepad1.x) {
                blp = speed;
            } else if (gamepad1.dpad_left) {
                blp = -speed;
            } else {
                blp = 0;
            }

            if (gamepad1.a) {
                brp = speed;
            } else if (gamepad1.dpad_down) {
                brp = -speed;
            } else {
                brp = 0;
            }

            setPower(flp, blp, frp, brp);
        }
    }

    public void setPower(double fleft, double bleft, double fright, double bright) {
        this.fleft.setPower(fleft);
        this.bleft.setPower(bleft);
        this.fright.setPower(fright);
        this.bright.setPower(bright);
    }
}
