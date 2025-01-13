package org.firstinspires.ftc.teamcode.dataFunctions;
import static org.firstinspires.ftc.teamcode.WriteFile.byteArr2String;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.writeToFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GetColorMaskPointsCopy;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import org.ejml.dense.row.FMatrixComponent;
import org.firstinspires.ftc.teamcode.writtencode.ExtenderController;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import kotlin.UByteArray;

@Config
public class Init {
    public DcMotor bright;
    public DcMotor bleft;
    public BNO055IMU imu;
    public Servo claw;
    public DcMotor fright;
    public DcMotor fleft;
    public MotorEx push;
    public Servo pitch;
    public ExtenderController extenderController;
    
    public Servo rotator;

    public FtcDashboard dash;
    public TelemetryPacket packet =new TelemetryPacket();;
    
    public Gamepad virtualGamepad;

    public Orientation lastAngles;
    public double firstAngle;
    public double y;
    public double x;
    public boolean write;
    public ArrayList<String> byteDataArr;
    public ArrayList<String> orientationDataArr;

    public String fileDest;

    public double motorTicksPerInch = -12.57;

    public static double xReach = 1;
    public static double yReach = 0;
    public static double leftAddedPower = 0;
    public static double zReach = 0;
    public OpenCvCamera webcam;
    public int cameraMonitorViewId;

    public static boolean fieldCentric = false;
    public double yPickup;
    public static double dist = 0;
    public Point objPoint = new Point();
    public FtcDashboard dashboard;
    public GetColorMaskPointsCopy pipeline = new GetColorMaskPointsCopy();
    public static int colorChoice = 0;
    public static double yOffsetCam = -11;
    public static double camOffsetDeg = 32;
    public static double clawClose = 0.17;
    public static double clawOpen = 0.5;

    public static double controlSpeedX = 0.05;
    public static double controlSpeedZ = 0.1;

    public static double controllerOffsetX;
    public static double controllerOffsetZ;




    public static double xOffsetCam = 1;
    public static double zOffsetCam = 5;
    public static double pickupConst = 100;

    public boolean manualOverdrive = false;

    public Init(Gamepad gamepad, boolean write, String fileDest, HardwareMap hardwareMap){
        controllerOffsetX = 0;
        controllerOffsetZ = 0;

        dashboard = FtcDashboard.getInstance();
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        dashboard.startCameraStream(webcam, 0);

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
        dash = FtcDashboard.getInstance();
        byteDataArr = new ArrayList<String>();
        orientationDataArr = new ArrayList<String>();
        virtualGamepad = gamepad;
        this.write = write;
        this.fileDest = fileDest;
        //pitch vertical => 0
        //pitch horizontal => 0.53
        //rotator fullLeft => 0.2
        //rotator fullRight => 0.75

        push = new MotorEx(hardwareMap, "push");
        pitch = hardwareMap.servo.get("pitch");
        bright = hardwareMap.dcMotor.get("bright");
        bleft = hardwareMap.dcMotor.get("bleft");
        fright= hardwareMap.dcMotor.get("fright");
        fleft = hardwareMap.dcMotor.get("fleft");
        rotator = hardwareMap.servo.get("rotator");
        extenderController = new ExtenderController(rotator, pitch, push);
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
        claw = hardwareMap.servo.get("claw");
        push.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extenderController = new ExtenderController(rotator, pitch, push);
    }
    public void updateGamepad(byte[] byteArray){
        virtualGamepad.fromByteArray(byteArray);
    }
    public void loop(){

        //get the rotation of the robot and set it to angles
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        firstAngle = -lastAngles.firstAngle;
        //get the x and y position of the joystick relative to the player
        if(fieldCentric) {
            y = FtcMath.rotateY(-virtualGamepad.left_stick_x, virtualGamepad.left_stick_y, Math.toRadians(firstAngle));
            x = FtcMath.rotateX(-virtualGamepad.left_stick_x, virtualGamepad.left_stick_y, Math.toRadians(firstAngle));
        }
        else {
            y = virtualGamepad.left_stick_y; // Remember, Y stick value is reversed
            x = -virtualGamepad.left_stick_x; // Counteract imperfect strafing
        }
        double rx = virtualGamepad.left_trigger - virtualGamepad.right_trigger + leftAddedPower;
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
        //slidesController.moveSlides(slidesPower);
        controllerOffsetX += ((virtualGamepad.dpad_up) ? 1:0 - ((virtualGamepad.dpad_down) ? 1:0)) * controlSpeedX;
        controllerOffsetZ += ((virtualGamepad.dpad_right) ? 1:0 - ((virtualGamepad.dpad_left) ? 1:0)) * controlSpeedZ;
        if(virtualGamepad.left_bumper && virtualGamepad.right_bumper && virtualGamepad.left_trigger > 0.5 && virtualGamepad.right_trigger > 0.5){
            manualOverdrive = true;
        }
        if(virtualGamepad.a && objPoint.x >0) {
            if(!manualOverdrive) {
                extenderController.setPos(objPoint.x + xOffsetCam + controllerOffsetX, yPickup, objPoint.y + zOffsetCam + controllerOffsetZ);
            }
            else{
                extenderController.setPos(10 + xOffsetCam, yPickup,0 + zOffsetCam);
            }
            if(virtualGamepad.x){
                yPickup = -3.5;
            }
            else{
                yPickup = 1;
            }
        }
        else{
            controllerOffsetX = 0;
            controllerOffsetZ = 0;
            claw.setPosition(clawClose);
            yPickup = 0;
            objPoint = pipeline.realPoint;
        }

        if(virtualGamepad.b){

            extenderController.setRotatorRot(0.5);
            if(virtualGamepad.dpad_up){
                extenderController.pidfPush(25);
                extenderController.setPitchRot(0.9);
            }
            else if(virtualGamepad.dpad_left){
                extenderController.pidfPush(20);
            }
            else if (virtualGamepad.dpad_right){
                extenderController.pidfPush(10);
            }
            else if (virtualGamepad.dpad_down){
                extenderController.pidfPush(0);
            }
            else{
                extenderController.setPitchRot(0.75);
            }
        }
        else if(virtualGamepad.x){
            extenderController.setPitchRot(0.2);
        }
        if(virtualGamepad.right_bumper){
            claw.setPosition(clawOpen);
        }
        else{
            claw.setPosition(clawClose);
        }
        if(write){
            byteDataArr.add(byteArr2String(virtualGamepad.toByteArray()));
            orientationDataArr.add("" + firstAngle);
        }
        packet.put("pos", "" + (objPoint.x + xOffsetCam) + ", " + 0 + ", " + (objPoint.y + zOffsetCam));
        packet.put("xControl", controllerOffsetX);
        packet.put("zControl", controllerOffsetZ);
        dash.sendTelemetryPacket(packet);


    }
    public void pushDataToFile(){
        if(write) {
            StringBuilder str = new StringBuilder("");
            StringBuilder orientationStr = new StringBuilder("");

            // Traversing the ArrayList
            for(int i = 0 ; i < byteDataArr.size(); i++){
                str.append(byteDataArr.get(i)).append("ඞ");
                orientationStr.append(orientationDataArr.get(i)).append("ඞ");
            }

            // StringBuffer to String conversion
            String commaseparatedlist = str.toString();
            String orientationeparatedlist = orientationStr.toString();


            // Condition check to remove the tel
            commaseparatedlist = commaseparatedlist.substring(0, commaseparatedlist.length() - 1);
            orientationeparatedlist = orientationeparatedlist.substring(0, orientationeparatedlist.length() - 1);

            writeToFile(commaseparatedlist, fileDest);
            writeToFile(orientationeparatedlist, fileDest.substring(0, fileDest.length()-4) + "Orientation.txt");
        }
    }

    public void adjustRot(double orientation){
        double angleDiff = -Math.toRadians(orientation) + Math.toRadians(firstAngle);
        leftAddedPower = (Math.abs(angleDiff) > Math.PI) ? -angleDiff/5:0;
    }
}
