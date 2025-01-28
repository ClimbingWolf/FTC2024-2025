package org.firstinspires.ftc.teamcode.dataFunctions;

import static org.firstinspires.ftc.teamcode.WriteFile.byteArr2String;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.writeToFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GetColorMaskPointsCopy;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import org.firstinspires.ftc.teamcode.writtencode.ExtenderController;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
public class InitExceptTheArmIsCompletelyBroken {
    public DcMotor bright;
    public DcMotor bleft;
    public BNO055IMU imu;
    public Servo claw;
    public DcMotor fright;
    public DcMotor fleft;
    //public MotorEx push;
    public Servo pitch;
    public ExtenderControllerBrokenArm extenderController;

    public String team = "blue";

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

    public double rotatorRot = 0;

    public static boolean fieldCentric = true;
    public double yPickup;
    public static double dist = 0;
    public Point objPoint = new Point();
    public FtcDashboard dashboard;
    public GetColorMaskPointsCopy pipeline = new GetColorMaskPointsCopy();
    public static int colorChoice = 0;
    public static double yOffsetCam = -13;
    public static double camOffsetDeg = 40;
    public static double clawClose = 0.17;

    public double markeplier = -1;
    public static double clawOpen = 0.5;

    public static double controlSpeedX = 0.05;
    public static double controlSpeedZ = 0.1;

    public static double controllerOffsetX;
    public static double controllerOffsetZ;

    public boolean getSample = false;




    public static double xOffsetCam = -3;
    public static double zOffsetCam = -6;
    public static double pickupConst = 100;

    public boolean manualOverdrive = false;

    public boolean useWebcam = true;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public InitExceptTheArmIsCompletelyBroken(Gamepad gamepad, boolean write, String fileDest, HardwareMap hardwareMap){
        controllerOffsetX = 0;
        controllerOffsetZ = 0;
        if(!useWebcam){
            manualOverdrive = true;
        }

        dashboard = FtcDashboard.getInstance();
        if(useWebcam){
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
        }
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

        //push = new MotorEx(hardwareMap, "push");
        pitch = hardwareMap.servo.get("pitch");
        bright = hardwareMap.dcMotor.get("bright");
        bleft = hardwareMap.dcMotor.get("bleft");
        fright= hardwareMap.dcMotor.get("fright");
        fleft = hardwareMap.dcMotor.get("fleft");
        rotator = hardwareMap.servo.get("rotator");
        extenderController = new ExtenderControllerBrokenArm(rotator, pitch);
        fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);

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
        //push.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void updateGamepad(byte[] byteArray){
        virtualGamepad.fromByteArray(byteArray);
    }
    public void loop(){

        if(virtualGamepad.left_stick_button){
            markeplier = -0.3;
        }
        else{
            markeplier = -1;
        }

        //get the rotation of the robot and set it to angles
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        firstAngle = -lastAngles.firstAngle;
        //get the x and y position of the joystick relative to the player
        if(fieldCentric) {
            y = FtcMath.rotateY(virtualGamepad.left_stick_x, -virtualGamepad.left_stick_y, Math.toRadians(firstAngle));
            x = FtcMath.rotateX(virtualGamepad.left_stick_x, -virtualGamepad.left_stick_y, Math.toRadians(firstAngle));
        }
        else {
            y = virtualGamepad.left_stick_y; // Remember, Y stick value is reversed
            x = virtualGamepad.left_stick_x; // Counteract imperfect strafing
        }
        double rx = -virtualGamepad.left_trigger + virtualGamepad.right_trigger + leftAddedPower;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        if(!virtualGamepad.a) {
            fleft.setPower(frontLeftPower *markeplier);
            bleft.setPower(backLeftPower*markeplier);
            fright.setPower(frontRightPower*markeplier);
            bright.setPower(backRightPower*markeplier);
        }
        //slidesController.moveSlides(slidesPower);
        controllerOffsetX += ((virtualGamepad.dpad_up) ? 1:0 - ((virtualGamepad.dpad_down) ? 1:0)) * controlSpeedX;
        controllerOffsetZ += ((virtualGamepad.dpad_right) ? 1:0 - ((virtualGamepad.dpad_left) ? 1:0)) * controlSpeedZ;
        if(virtualGamepad.left_bumper && virtualGamepad.right_bumper && virtualGamepad.left_trigger > 0.5 && virtualGamepad.right_trigger > 0.5){
            manualOverdrive = true;
        }
        if((virtualGamepad.a || virtualGamepad.y) && objPoint.x >0) {
            if(virtualGamepad.y){
                pipeline.choice = 2;
            }
            else if (virtualGamepad.a){
                if(team.equals("blue")) {
                    pipeline.choice = 1;
                }
                else{
                    pipeline.choice = 0;
                }
            }

            if(!manualOverdrive) {
                if(extenderController.dist < 16.8 && extenderController.dist > 15 || getSample == true){
                    yPickup = -8.2;
                    extenderController.setPos(objPoint.x + xOffsetCam + controllerOffsetX, yPickup, objPoint.y + zOffsetCam + controllerOffsetZ);
                    getSample = true;
                    fleft.setPower(0);
                    bleft.setPower(0);
                    bright.setPower(0);
                    fright.setPower(0);
                }
                else {
                    getSample = false;
                    objPoint = pipeline.realPoint;
                    extenderController.setPos(objPoint.x + xOffsetCam + controllerOffsetX, yPickup, objPoint.y + zOffsetCam + controllerOffsetZ);
                    if (extenderController.dist <= 18) {
                        fleft.setPower(-0.3 * (dist - 18) / 18);
                        bleft.setPower(-0.3 * (dist - 18) / 18);
                        bright.setPower(-0.3 * (dist - 18) / 18);
                        fright.setPower(-0.3 *(dist - 18) / 18);
                    }
                }
            }
            else{
                rotatorRot = 0.5 * (1-(virtualGamepad.right_stick_x + 1)/2);
                if(rotatorRot > 0.6){
                    rotatorRot = 0.6;
                }
                else if(rotatorRot < 0.4){
                    rotatorRot = 0.4;
                }
                extenderController.setRotatorRot(rotatorRot);
            }



            if(extenderController.dist < 16.5 && extenderController.dist > 14.8){
                yPickup = -8.2;
                claw.setPosition(clawOpen);
            }

        }
        else{
            getSample = false;
            controllerOffsetX = 0;
            controllerOffsetZ = 0;
            yPickup = 0;
            objPoint = pipeline.realPoint;
        }
        if(virtualGamepad.x){
            extenderController.pitch.setPosition(0.485);
        }
        else if (!virtualGamepad.a && !virtualGamepad.b){
            extenderController.pitch.setPosition(0.35);
        }

        if(virtualGamepad.b){
            extenderController.setPitchRot(0.9);


            if(virtualGamepad.dpad_up){
                rotatorRot = 0.5;
                extenderController.setRotatorRot(rotatorRot);

            }
            else if(virtualGamepad.dpad_left){
                extenderController.setPitchRot(0.5);
            }
            else if (virtualGamepad.dpad_right){
                extenderController.setPitchRot(0.35);

            }
            else if (virtualGamepad.dpad_down){
                extenderController.setPitchRot(0.385);

            }
        }
        //else if(virtualGamepad.x){
            //extenderController.setPitchRot(0.2);
        //}
        if(virtualGamepad.right_bumper){
            claw.setPosition(clawOpen);
        }
        else{
            claw.setPosition(clawClose);
        }
        if(virtualGamepad.right_stick_button){
            imu.initialize(parameters);
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

            // StringBuffer to String
            String commaseparatedlist = str.toString();
            String orientationeparatedlist = orientationStr.toString();


            // Condition check to remove the tel
            commaseparatedlist = commaseparatedlist.substring(0, commaseparatedlist.length() - 1);
            orientationeparatedlist = orientationeparatedlist.substring(0, orientationeparatedlist.length() - 1);

            writeToFile(commaseparatedlist, fileDest);
            writeToFile(orientationeparatedlist, fileDest.substring(0, fileDest.length()-4) + "Orientation.txt");
        }
    }

}
