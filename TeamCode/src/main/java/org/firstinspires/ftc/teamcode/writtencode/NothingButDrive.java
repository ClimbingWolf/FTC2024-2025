package org.firstinspires.ftc.teamcode.writtencode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp(name = "Just Drive")
@Config
public class NothingButDrive extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    public ArmServoHolder armController;

    public CRServo clawServo;

    public static double xOffsetCam = -7;

    public static double camOffsetDeg = 35;
    public static double zOffsetCam = 7;

    public static double yOffsetCam = -12;
    public static double setPosVal;

    public static double bottomServoSet = 0;
    public static double topServoSet = 0;
    public static double movementSpeedmultiplier = 1;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();
    public double firstAngle = 0;
    public double y = 0;
    public double x = 0;
    public double rx = 0;
    public static double offsetOrientation = 0;

    public static double fleftMult = 1.0;
    public static double bleftMult = 1.0;

    public static double frightMult = 1.0;

    public static double brightMult = 1.0;

    public static double legLength = 8;

    public static double xReach = 5;
    public static double yReach = 0.1;

    public static int colorChoice = 0;


    public ArrayList<Servo> topServos = new ArrayList<>();
    public ArrayList<Servo> bottomServos = new ArrayList<>();

    public ArrayList<Servo> rotatorServos = new ArrayList<>();

    public Point objPoint = new Point();

    public static double armY = -4;

    public static double zReach;

    public DcMotor rightSlide;
    public DcMotor leftSlide;

    public CRServo gamingClaw;

    public static double epicPower = 1;

    //GetColorMaskPoints pipeline;


    @Override
    public void runOpMode() {
        gamingClaw = hardwareMap.crservo.get("claw");


        topServos.add(hardwareMap.servo.get("topServo"));
        bottomServos.add(hardwareMap.servo.get("leftBottom"));
        bottomServos.add(hardwareMap.servo.get("rightBottom"));
        rotatorServos.add(hardwareMap.servo.get("bottomRotator"));
        //clawServo = hardwareMap.crservo.get("claw");
        //Declare the armcontroller
        armController = new ArmServoHolder(topServos, bottomServos, rotatorServos, legLength);
        armController.addTopStart(0.05);
        armController.addBottomStart(0.09);//left
        armController.addBottomStart(1-0.09);//right
        armController.addTopEnd(0.6);
        armController.addBottomEnd(0.93);//left
        armController.addBottomEnd(1-0.93);//right
        armController.addRotatorStart(0);
        armController.addRotatorEnd(1);
        DcMotor bright = hardwareMap.dcMotor.get("bright");
        DcMotor bleft = hardwareMap.dcMotor.get("bleft");
        DcMotor fright = hardwareMap.dcMotor.get("fright");
        DcMotor fleft = hardwareMap.dcMotor.get("fleft");
        fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        boolean wheelie = false;
        boolean lock = false;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configurepd to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {
            //get the rotation of the robot and set it to angles
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            firstAngle = lastAngles.firstAngle + offsetOrientation;
            //get the x and y position of the joystick relative to the player
            //y = FtcMath.rotateY(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.toRadians(firstAngle));
            //x = FtcMath.rotateX(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.toRadians(firstAngle));
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            gamingClaw.setPower(epicPower);
            fleft.setPower(frontLeftPower);
            bleft.setPower(backLeftPower);
            fright.setPower(frontRightPower);
            bright.setPower(backRightPower);
            dashboard.sendTelemetryPacket(packet);
            packet.put("pos", "" + objPoint.x +", " + objPoint.y);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
