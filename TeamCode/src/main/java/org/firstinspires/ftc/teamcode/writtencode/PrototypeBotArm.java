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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp(name = "PrototypeBotArm")
@Config
public class PrototypeBotArm extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    public ArmServoHolder armController;

    public CRServo clawServo;

    public static double legLength = 9.5;

    public static double setPosVal;

    public static double bottomServoSet = 0;

    public boolean fieldCentric = false;
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

    public static double xReach = 5;
    public static double yReach = 0.1;

    public static double zReach = 0;


    public ArrayList<Servo> topServos = new ArrayList<>();
    public ArrayList<Servo> bottomServos = new ArrayList<>();

    public ArrayList<Servo> rotatorServos = new ArrayList<>();


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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
        armController.addRotatorStart(1);
        armController.addRotatorEnd(0.06);
        DcMotor bright = hardwareMap.dcMotor.get("bright");
        DcMotor bleft = hardwareMap.dcMotor.get("bleft");
        DcMotor fright = hardwareMap.dcMotor.get("fright");
        DcMotor fleft = hardwareMap.dcMotor.get("fleft");
        fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bleft.setDirection(DcMotorSimple.Direction.REVERSE);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            if(fieldCentric) {
                y = FtcMath.rotateY(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.toRadians(firstAngle));
                x = FtcMath.rotateX(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.toRadians(firstAngle));
            }
            else {
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
            }
            rx = gamepad1.left_trigger-gamepad1.right_trigger;
            // basic teleop movement code
            bright.setPower((y-x-rx)*movementSpeedmultiplier * brightMult);
            fleft.setPower((y-x+rx)*movementSpeedmultiplier * fleftMult);
            fright.setPower((y+x-rx)*movementSpeedmultiplier * frightMult);
            bleft.setPower((y+x+rx)*movementSpeedmultiplier * bleftMult);
            dashboard.sendTelemetryPacket(packet);
            //if(gamepad1.a) {
            //    armController.moveToPos(xReach, yReach);
            //}
            //if(gamepad1.a){
            //    topServos.get(0).setPosition(setPosVal);
            //}
            //if(gamepad1.b){
            //    bottomServos.get(0).setPosition(setPosVal);
            //}
            //if(gamepad1.x){
            //    rotatorServos.get(0).setPosition(setPosVal);
            //}
            if(gamepad1.a) {
                armController.moveToPos(xReach, yReach, zReach);
            }
            if(gamepad1.b){
                topServos.get(0).setPosition(topServoSet);
                bottomServos.get(0).setPosition(bottomServoSet);
            }
            if(gamepad1.dpad_up && (Math.sqrt(Math.pow(xReach,2) + Math.pow(yReach+0.05,2)) < armController.legLength * 2)){
                yReach+=0.05;
            }
            if(gamepad1.dpad_right && (Math.sqrt(Math.pow(xReach+0.05,2) + Math.pow(yReach,2)) < armController.legLength * 2)){
                xReach+=0.05;
            }
            if(gamepad1.dpad_down && (Math.sqrt(Math.pow(xReach,2) + Math.pow(yReach-0.05,2)) < armController.legLength * 2)){
                yReach-=0.05;
            }
            if(gamepad1.dpad_left && (Math.sqrt(Math.pow(xReach-0.05,2) + Math.pow(yReach,2)) < armController.legLength * 2)){
                xReach-=0.05;
            }
            armController.outputRotatorPercentage();

        }
    }
}
