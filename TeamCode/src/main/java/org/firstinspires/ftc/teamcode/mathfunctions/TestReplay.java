package org.firstinspires.ftc.teamcode.mathfunctions;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.readFromFile;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.readParseFile;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.readParseFileNoBytes;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.string2ByteArr;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;



import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.dataFunctions.Init;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "STUPID CODE STUPID HATE")
@Config
public class TestReplay extends LinearOpMode {
    public int frame = -1;
    public ArrayList byteArrList = new ArrayList();
    public ArrayList orientationArrList = new ArrayList();
    public volatile Gamepad tempGamepad = new Gamepad();

    public String filename = "silly";


    public String fileStart = "byteData/";

    public String fullFileDest = fileStart + filename + ".txt";
    public String orientationFile = fullFileDest.substring(0, fullFileDest.length()-4) + "Orientation.txt";
    public Init init;
    public FtcDashboard dash;
    public TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {
        dash = FtcDashboard.getInstance();
        byteArrList = readParseFile(fullFileDest);
        orientationArrList = readParseFileNoBytes(orientationFile);
        init = new Init(tempGamepad, true, fullFileDest, hardwareMap);
        //packet.put("data", orientationArrList.toString());
        dash.sendTelemetryPacket(packet);
        waitForStart();
        while (opModeIsActive()) {
            frame +=1;
            if(frame < byteArrList.size()) {
                init.updateGamepad((byte[])byteArrList.get(frame));
                dash.sendTelemetryPacket(packet);
                init.loop();
            }

            else{
                stop();//writeToFile("" + firstAngle, "orientation.txt");
            }

        }

    }

    public void rotateRight(DcMotor fleft, DcMotor bleft, DcMotor bright, DcMotor fright, double power){
        fleft.setPower(-power);
        bleft.setPower(-power);
        bright.setPower(power);
        fright.setPower(power);
    }

    public void rotateLeft(DcMotor fleft, DcMotor bleft, DcMotor bright, DcMotor fright, double power){
        rotateRight(fleft, bleft, bright, fright, -power);
    }

    public double rotateX(double x, double y, double thetaRadians){
        return x*Math.cos(thetaRadians) - y*Math.sin(thetaRadians);
    }
    public double rotateY(double x, double y, double thetaRadians){
        return x*Math.sin(thetaRadians) + y*Math.cos(thetaRadians);
    }

}
