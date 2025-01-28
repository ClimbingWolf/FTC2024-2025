package org.firstinspires.ftc.teamcode.writtencode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp(name = "DiffyTest")
@Config
@Disabled
public class DiffyTest extends LinearOpMode {

    public Servo leftDiffy;

    public Servo rightDiffy;
    public DiffyControllerServo diffyController;

    public static double pitch = 0;
    public static double roll = 0;


    @Override
    public void runOpMode() {

        waitForStart();
        leftDiffy = hardwareMap.servo.get("leftDiffy");
        leftDiffy.setPosition(0.5);
        rightDiffy = hardwareMap.servo.get("rightDiffy");
        rightDiffy.setPosition(0.5);
        diffyController = new DiffyControllerServo(leftDiffy, rightDiffy, 2);
        while (opModeIsActive()) {
            diffyController.setPitchAndRollAngleDeg(pitch, roll);
        }
    }
}
