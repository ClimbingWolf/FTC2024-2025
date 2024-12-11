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
import org.firstinspires.ftc.teamcode.GetColorMaskPointsCopy;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp(name = "ControlServoWithController")
@Config
public class ControlServoWithController extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    public ArmServoHolder armController;

    public  double xOffsetCam = -7;
    public static double rotatorPos = 0.5;

    public  double slidesMultiplier = 1;
    public double slidesPower = 0;

    public  double camOffsetDeg = 35;
    public  double zOffsetCam = 7;

    public  double yOffsetCam = -12;
    public  double setPosVal;

    public  double bottomServoSet = 0;
    public  double topServoSet = 0;
    public  double movementSpeedmultiplier = 1;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();
    public double firstAngle = 0;
    public double y = 0;
    public double x = 0;
    public double rx = 0;
    public  double offsetOrientation = 0;

    public  double fleftMult = 1.0;
    public  double bleftMult = 1.0;

    public Servo leftDiffy;

    public Servo rightDiffy;
    public DiffyControllerServo diffyController;

    public  double pitch = 0;

    public static double controlMultiplier = 0.005;
    public  double roll = 0;

    public  double frightMult = 1.0;

    public  double brightMult = 1.0;

    public  double legLength = 8;

    public  double xReach = 5;
    public  double yReach = 0.1;

    public  int colorChoice = 0;

    public SlidesController slidesController;


    public ArrayList<Servo> topServos = new ArrayList<>();
    public ArrayList<Servo> bottomServos = new ArrayList<>();

    public ArrayList<Servo> rotatorServos = new ArrayList<>();



    public  double armY = -4;

    public  double zReach;

    GetColorMaskPointsCopy pipeline;

    public DcMotor rightSlide;
    public DcMotor leftSlide;

    public static double bottomRot = 0;
    public static double topRot = 0;


    @Override
    public void runOpMode() {
        /*
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        dashboard.startCameraStream(webcam, 0);
        pipeline = new GetColorMaskPoints();
        webcam.setPipeline(pipeline);
        pipeline.choice = colorChoice;//blue
        pipeline.zReal = yOffsetCam;
        pipeline.camAngleDeg = camOffsetDeg;

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming((int)pipeline.width,(int)pipeline.height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        */
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
        leftDiffy = hardwareMap.servo.get("leftDiffy");
        leftDiffy.setPosition(0.5);
        rightDiffy = hardwareMap.servo.get("rightDiffy");
        rightDiffy.setPosition(0.5);
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //slidesController = new SlidesController(rightSlide, leftSlide);


        diffyController = new DiffyControllerServo(leftDiffy, rightDiffy, 2);
        waitForStart();

        while (opModeIsActive()) {
            slidesPower = (gamepad1.right_stick_x) * slidesMultiplier;
            //objPoint = pipeline.realPoint;
            //get the rotation of the robot and set it to angles
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            firstAngle = lastAngles.firstAngle + offsetOrientation;
            //get the x and y position of the joystick relative to the player

            //if(gamepad1.a) {
            //    armController.moveToPos(objPoint.x + xOffsetCam, armY + 2, -objPoint.y - zOffsetCam);
            //}
            topRot += (gamepad1.left_stick_y) * controlMultiplier;
            bottomRot += (gamepad1.right_stick_y) * controlMultiplier;


            armController.setBottomRot(bottomRot);
            armController.setRotatorPos(rotatorPos);
            armController.setTopRot(topRot);
            armController.outputServoPos();
        }
    }
}
