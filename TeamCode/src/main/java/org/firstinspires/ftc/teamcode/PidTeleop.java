package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "sussyimposter")
public class PidTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor blarm = hardwareMap.dcMotor.get("blarm");
        DcMotor brarm = hardwareMap.dcMotor.get( "brarm");
        DcMotor tlarm = hardwareMap.dcMotor.get( "tlarm");
        DcMotor trarm = hardwareMap.dcMotor.get( "trarm");

        DcMotor bright = hardwareMap.dcMotor.get( "bright");
        DcMotor bleft = hardwareMap.dcMotor.get( "bleft");
        DcMotor fright = hardwareMap.dcMotor.get( "fright");
        DcMotor fleft = hardwareMap.dcMotor.get( "fleft");

        fleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        blarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tlarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tlarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tlarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pid pidControl = new Pid();

        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            while(tlarm.getCurrentPosition()!= -140) {
                pidControl.update(tlarm, -140, tlarm.getCurrentPosition(), 5.0, 0.0, 0.0);
            }
            while (trarm.getCurrentPosition() != -140) {
                pidControl.update(trarm, -140, tlarm.getCurrentPosition(), 5.0, 0.0, 0.0);
            }
            telemetry.addData("tlarm", tlarm.getCurrentPosition());
            telemetry.addData("trarm", trarm.getCurrentPosition());
            telemetry.update();
        }
    }
}