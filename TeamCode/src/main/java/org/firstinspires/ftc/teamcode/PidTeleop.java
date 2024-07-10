package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "PIDFtest")
@Config
public class PidTeleop extends LinearOpMode {

    public static double kP = 5;
    public static double kI = 0;
    public static double kD = .1;
    
    public static double kF = 3;
    public static int moveTo = 100;

    @Override
    public void runOpMode() {
        DcMotor blarm = hardwareMap.dcMotor.get("blarm");
        DcMotor brarm = hardwareMap.dcMotor.get("brarm");
        MotorEx tlarm =  new MotorEx(hardwareMap, "tlarm");

        DcMotor bright = hardwareMap.dcMotor.get("bright");
        DcMotor bleft = hardwareMap.dcMotor.get("bleft");
        DcMotor fright = hardwareMap.dcMotor.get("fright");
        DcMotor fleft = hardwareMap.dcMotor.get("fleft");

        fleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        tlarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        blarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tlarm.resetEncoder();
        blarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        brarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFController pidf = new PIDFController(kP, kI, kD, kF);

        pidf.setSetPoint(moveTo);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double output = pidf.calculate(tlarm.getCurrentPosition(), moveTo);
            tlarm.setVelocity(output);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("tlarm", tlarm.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

        }
    }
}