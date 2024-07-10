package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Teleop")
public class movementCode extends LinearOpMode {
    private double powerConstant = 0.2;
    private DcMotor brarm;
    private DcMotor blarm;
    private DcMotor tlarm;
    private DcMotor trarm;
    private DcMotor bright;
    private DcMotor bleft;
    private DcMotor fleft;
    private DcMotor fright;
    private double y;
    private double x;
    private double rx;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        //int EncoderValue;

        //blarm = hardwareMap.get(DcMotor.class, "blarm");
        //tlarm = hardwareMap.get(DcMotor.class, "tlarm");

        bright = hardwareMap.get(DcMotor.class, "bright");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        fleft = hardwareMap.get(DcMotor.class, "fleft");

        fleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        tlarm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Put initialization blocks here.
        waitForStart();
        //EncoderValue = 0;
        while (opModeIsActive()) {
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            // basic teleop movement code
            bright.setPower(y+x-rx);
            bleft.setPower(y-x+rx);
            fright.setPower(y-x-rx);
            fleft.setPower(y+x+rx);

            //if (gamepad1.dpad_up) {
             //   brarm.setPower(powerConstant);
              //  blarm.setPower(powerConstant);
            //} else if (gamepad1.dpad_down) {
             //   brarm.setPower(-powerConstant);
             //   blarm.setPower(-powerConstant);
            //} else {
              //  brarm.setPower(0);
              //  blarm.setPower(0);
           // }

            //if (gamepad1.right_bumper) {
             //   trarm.setPower(powerConstant);
             //   tlarm.setPower(powerConstant);
            //} else if (gamepad1.left_bumper) {
             //   trarm.setPower(-powerConstant);
              //  tlarm.setPower(-powerConstant);
            //} else {
            //    trarm.setPower(0);
            //    tlarm.setPower(0);
            //}
        }
    }
}
