package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class DeadWheelTest extends LinearOpMode {

    FtcDashboard dash;

    public static int MAX_TIME;
    public static int MAX_SPEED;

    public static boolean right;
    public static boolean speedy;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor fleft = hardwareMap.dcMotor.get("fleft");
        DcMotor fright = hardwareMap.dcMotor.get("fright");
        DcMotor bright = hardwareMap.dcMotor.get("bright");
        DcMotor bleft = hardwareMap.dcMotor.get("bleft");

        if (right) {
            fright.setDirection(DcMotorSimple.Direction.REVERSE);
            bright.setDirection(DcMotorSimple.Direction.REVERSE);
            bleft.setDirection(DcMotorSimple.Direction.FORWARD);
            fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            fleft.setDirection(DcMotorSimple.Direction.REVERSE);
            bleft.setDirection(DcMotorSimple.Direction.REVERSE);
            bright.setDirection(DcMotorSimple.Direction.FORWARD);
            fright.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        waitForStart();
        resetRuntime();
        ElapsedTime spinTime = new ElapsedTime();
        while(opModeIsActive()) {
            double pow = Math.pow(2, spinTime.seconds() / MAX_TIME) - 1;
            double speed = .5;
            if (speedy) speed = 1;
            fleft.setPower(speed);
            bleft.setPower(-speed);
            fright.setPower(-speed);
            bright.setPower(speed);
        }
    }
}