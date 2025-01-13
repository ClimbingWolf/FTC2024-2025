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
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@TeleOp(name = "ඞBasicDrive")
@Config
public class BasicDrive extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    public ArmServoHolder armController;
    public  double slidesMultiplier = 1;
    public double slidesPower = 0;
    public static boolean armMode = false;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();
    public double firstAngle = 0;
    public double y = 0;
    public double x = 0;
    public double rx = 0;
    public  double offsetOrientation = 0;
    public Servo leftDiffy;

    public Servo rightDiffy;
    public DiffyControllerServo diffyController;


    public static double legLength = 10.25;


    public ArrayList<Servo> topServos = new ArrayList<>();
    public ArrayList<Servo> bottomServos = new ArrayList<>();

    public ArrayList<Servo> rotatorServos = new ArrayList<>();


    public DcMotor rightSlide;
    public DcMotor leftSlide;

    public CRServo claw;
    public static double topEnd = 0.75;

    public double zArm = 0;

    public double yArm = 0;
    public double xArm = 0;
    public boolean aToggle = false;

    @Override
    public void runOpMode() {

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        topServos.add(hardwareMap.servo.get("topServo"));
        bottomServos.add(hardwareMap.servo.get("leftBottom"));
        bottomServos.add(hardwareMap.servo.get("rightBottom"));
        rotatorServos.add(hardwareMap.servo.get("bottomRotator"));

        //clawServo = hardwareMap.crservo.get("claw");
        //Declare the armcontroller
        armController = new ArmServoHolder(topServos, bottomServos, rotatorServos, legLength);
        armController.addTopStart(0.05);
        armController.addBottomStart(0);//left
        armController.addBottomStart(1);//right
        armController.addTopEnd(topEnd);
        armController.addBottomEnd(1);//left
        armController.addBottomEnd(0);//right
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
        claw = hardwareMap.crservo.get("claw");
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //slidesController = new SlidesController(rightSlide, leftSlide);
        diffyController = new DiffyControllerServo(leftDiffy, rightDiffy, 2);
        waitForStart();
        while (opModeIsActive()) {
            slidesPower = (gamepad1.left_bumper) ? 1:0 - ((gamepad1.right_bumper) ? 1:0) * slidesMultiplier;
            //get the rotation of the robot and set it to angles
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            firstAngle = lastAngles.firstAngle + offsetOrientation;
            //get the x and y position of the joystick relative to the player
            y = FtcMath.rotateY(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.toRadians(firstAngle));
            x = FtcMath.rotateX(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.toRadians(firstAngle));
            //double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            //double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
            double rx = gamepad1.right_trigger - gamepad1.left_trigger;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            fleft.setPower(frontLeftPower);
            bleft.setPower(backLeftPower);
            fright.setPower(frontRightPower);
            bright.setPower(backRightPower);
            dashboard.sendTelemetryPacket(packet);
            leftSlide.setPower(slidesPower);
            rightSlide.setPower(slidesPower);
            //slidesController.moveSlides(slidesPower);
            if(gamepad1.a) {
                if(aToggle) {
                    aToggle = false;
                    armMode = !armMode;
                    zArm = 0;
                    xArm = 0;
                }
            }
            else{
                aToggle = true;
            }

            if(armMode) {
                claw.setPower(-0.6);
                xArm += ((gamepad1.dpad_left) ? 1:0 - ((gamepad1.dpad_right) ? 1:0)) * 0.05;
                zArm += ((gamepad1.dpad_up) ? 1:0 - ((gamepad1.dpad_down) ? 1:0)) * 0.05;
                yArm += ((gamepad1.y) ? 1:0 - ((gamepad1.x) ? 1:0)) * 0.05;
                armController.moveToPos(xArm, yArm, zArm);
            }
            else{
                armController.setBottomRot(0.85);
                armController.setTopRot(1.05);
                claw.setPower(0.2);
            }
            packet.put("pos", " " + xArm + ", " + yArm + ", " + zArm);


        }
    }
}
