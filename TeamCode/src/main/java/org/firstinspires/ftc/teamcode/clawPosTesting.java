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

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.ArrayList;

@TeleOp(name = "clawPosTesting")
@Config
public class clawPosTesting extends LinearOpMode {
    public static double blarmOffset = -5;
    public static double tlarmOffset = -38;
    public static double distance = 5;
    public double moveToTlarm;
    public double moveToBlarm;

    public ArrayList thetaStorage = new ArrayList();
    @Override
    public void runOpMode() {
        //initialize motors
        MotorEx blarm =  new MotorEx(hardwareMap, "blarm");
        MotorEx tlarm = new MotorEx(hardwareMap, "tlarm");

        DcMotor bright = hardwareMap.dcMotor.get("bright");
        DcMotor bleft = hardwareMap.dcMotor.get("bleft");
        DcMotor fright = hardwareMap.dcMotor.get("fright");
        DcMotor fleft = hardwareMap.dcMotor.get("fleft");

        fleft.setDirection(DcMotorSimple.Direction.REVERSE);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        blarm.setInverted(true);
        blarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        tlarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        blarm.resetEncoder();
        tlarm.resetEncoder();
        PIDFController pidfBlarm = pidConstantsAndFunctions.pidfBlarm;
        PIDFController pidfTlarm = pidConstantsAndFunctions.pidfTlarm;

        pidfBlarm.setSetPoint(0);
        pidfTlarm.setSetPoint(0);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            thetaStorage = pidConstantsAndFunctions.calculateArmThetasFloor(distance, pidConstantsAndFunctions.barLength, blarmOffset, tlarmOffset, 0);
            moveToBlarm = pidConstantsAndFunctions.radiansToTicks((double)(thetaStorage.get(0)));
            moveToTlarm = pidConstantsAndFunctions.radiansToTicks((double)(thetaStorage.get(1)));
            pidfBlarm.setSetPoint(moveToBlarm);
            pidfTlarm.setSetPoint(moveToTlarm);
            double outputBlarm = pidfBlarm.calculate(blarm.getCurrentPosition());
            blarm.setVelocity(outputBlarm);
            double outputTlarm = pidfTlarm.calculate(tlarm.getCurrentPosition());
            tlarm.setVelocity(outputTlarm);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("blarmTheta", pidConstantsAndFunctions.radiansToDegrees((double)(thetaStorage.get(0))));
            packet.put("tlarmTheta", pidConstantsAndFunctions.radiansToDegrees((double)(thetaStorage.get(1))));
            dashboard.sendTelemetryPacket(packet);
        }
    }

}