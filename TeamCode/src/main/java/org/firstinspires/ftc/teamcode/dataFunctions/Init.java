package org.firstinspires.ftc.teamcode.dataFunctions;
import static org.firstinspires.ftc.teamcode.WriteFile.byteArr2String;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.readFromFile;
import static org.firstinspires.ftc.teamcode.mathfunctions.ReadWriteFile.writeToFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GetColorMaskPointsCopy;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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
    public double servoPitch = 0;
    public ArrayList<String> byteDataArr;
    public ArrayList<String> orientationDataArr;

    public String fileDest;

    public double motorTicksPerInch = -12.57;

    public static double xReach = 20;
    public static double yReach = 5;
    public double leftAddedPower = 0;
    public static double zReach = 0.1;
    public OpenCvCamera webcam;
    public int cameraMonitorViewId;

    public static boolean fieldCentric = true;
    public double yPickup;
    public  double dist = 0;
    public Point objPoint = new Point();
    public FtcDashboard dashboard;
    public GetColorMaskPointsCopy pipeline = new GetColorMaskPointsCopy();
    public  int colorChoice = 0;

    public  double rotatorRot = 0.5;
    public  double yOffsetCam = -13;
    public  double yArmToFloor = 8;
    public  double camOffsetDeg = 30;
    public static double clawClose = 0.03;
    public static double clawOpen = 0.3;

    public  double controlSpeedX = 0.05;
    public  double controlSpeedZ = 0.1;

    public  double controllerOffsetX;
    public  double controllerOffsetZ;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();




    public  double xOffsetCam = 10;
    public  double zOffsetCam = 6;
    public  double pickupConst = 100;

    public boolean manualOverdrive = true;

    public boolean useWebcam = false;

    public double pidfGoal = 16;

    public double targetEncoderPos = 30;
    public SampleMecanumDrive drive;
    public double localizationX;
    public double localizationY;

    public String imuInfoFile = "";

    public String imuFileInfo = "";

    public double startX = 0;
    public double startY = 0;

    public double startIMU = 0;

    public double fleftPower =0;
    public double frightPower=0;
    public double brightPower=0;
    public double bleftPower=0;

    public Pose2d startPose;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 135 + 180;

    public static double rotationScaler = 1000;

    public static double translationScaler = 0;

    public static double logScaler = 5;

    public static double servoSpeed = 0.07;

    public double powerMult = 1;



    public Init(Gamepad gamepad, boolean write, String fileDest, HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
        //change this boolean back when write file
        if(write){
            imuFileInfo = readFromFile(imuInfoFile);
            String[] stringArr = imuFileInfo.split(",");
            startX = Double.parseDouble(stringArr[0]);
            startY = Double.parseDouble(stringArr[1]);
            startIMU = Double.parseDouble(stringArr[2]);
            startPose = new Pose2d(startX, startY, startIMU);
            drive.setPoseEstimate(startPose);
        }
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
    }
    public void updateGamepad(byte[] byteArray){
        virtualGamepad.fromByteArray(byteArray);
    }
    public void loop(){
        localizationX = drive.getPoseEstimate().getX();
        localizationY = drive.getPoseEstimate().getY();
        drive.updatePoseEstimate();
        if(!virtualGamepad.a) {
            extenderController.pidfPush(pidfGoal);
            extenderController.setPitchRot(servoPitch);
            extenderController.setRotatorRot(rotatorRot);
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
            y = -virtualGamepad.left_stick_y; // Remember, Y stick value is reversed
            x = -virtualGamepad.left_stick_x; // Counteract imperfect strafing
        }
        double rx = virtualGamepad.left_trigger - virtualGamepad.right_trigger + leftAddedPower;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        if(virtualGamepad.right_stick_button){
            powerMult = 0.3;
        }
        else{
            powerMult = 1;
        }
        fleftPower = (y + x + rx) * powerMult;
        bleftPower = (y - x + rx)* powerMult;
        frightPower = (y - x - rx)* powerMult;
        brightPower = (y + x - rx)* powerMult;
        if(virtualGamepad.y){
            goToPoint(targetX, targetY, targetHeading, drive.getPoseEstimate());
        }
        double frontLeftPower = fleftPower;
        double backLeftPower = bleftPower;
        double frontRightPower = frightPower;
        double backRightPower = brightPower;
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
        if(virtualGamepad.a) {
            extenderController.setPos(xReach, yReach,zReach);
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
            yPickup = 0;
            objPoint = pipeline.realPoint;
        }

        if(virtualGamepad.b){
            if(servoPitch < 0.9) {
                servoPitch += servoSpeed;
            }
            else{
                if(rotatorRot < 1.5) {
                    rotatorRot += servoSpeed;
                }
                else{
                    rotatorRot = 1.5;
                }
                servoPitch = 0.9;
            }
            if(virtualGamepad.dpad_up){
                pidfGoal = 40;
            }
            if(virtualGamepad.dpad_right){
                pidfGoal = 24;
            }
            if(virtualGamepad.dpad_down){
                pidfGoal = 17;
            }
        }
        else if(virtualGamepad.x && servoPitch == 0){
            servoPitch = -servoSpeed;
        }
        else if(servoPitch > 0 && rotatorRot == 0.5){
            servoPitch -= servoSpeed;
        }
        if((pidfGoal > 17 || Math.abs(extenderController.push.getCurrentPosition()) > Math.abs(targetEncoderPos)) && !virtualGamepad.b){
            pidfGoal = 17;
        }
        else if (rotatorRot > 0.5 && !virtualGamepad.b){
            rotatorRot -= servoSpeed;
        }
        else if (rotatorRot < 0.5&& !virtualGamepad.b){
            rotatorRot = 0.5;
        }
        if(virtualGamepad.right_bumper){
            claw.setPosition(clawOpen);
        }
        else{
            claw.setPosition(clawClose);
        }
        if(virtualGamepad.left_stick_button){
            localizationX= drive.getPoseEstimate().getX();
            localizationY = drive.getPoseEstimate().getY();
            imu.initialize(parameters);
            drive.setPoseEstimate(new Pose2d(localizationX, localizationY, 0));
        }
        if(write){
            byteDataArr.add(byteArr2String(virtualGamepad.toByteArray()));
            orientationDataArr.add("" + firstAngle);
        }
        packet.put("dist", extenderController.dist);
        packet.put("encoderPos", extenderController.push.getCurrentPosition());
        packet.put("xPos", drive.getPoseEstimate().getX());
        packet.put("yPos", drive.getPoseEstimate().getY());
        packet.put("Heading", drive.getPoseEstimate().getHeading());
        dash.sendTelemetryPacket(packet);
    }

    public void goToPoint(double targetX, double targetY, double targetHeading, Pose2d pose){
        double tempDegrees = Math.floorMod((long)Math.toDegrees(pose.getHeading()) - (long)targetHeading, 360);
        double rotatePower = 0;
        if (tempDegrees > 180){
            rotatePower = (360-tempDegrees)/180;
        }
        else{
            rotatePower = -tempDegrees/180;
        }
        //double rotate = (180-targetHeading + Math.toDegrees(pose.getHeading()))%360;
        //double rotatePower = (rotate)/rotationScaler;
        double dist = Math.sqrt(Math.pow(targetX - pose.getX(),2) + Math.pow(targetY - pose.getY(),2));
        double translateScale = (Math.log(dist *logScaler)+1) * translationScaler;
        if(translateScale < 0){
            translateScale = 0;
        }
        double noRotOutputX = -(targetX - pose.getX()) * translateScale;
        double noRotOutputY = (targetY - pose.getY()) * translateScale;
        double outputY = FtcMath.rotateX(-noRotOutputX, -noRotOutputY, pose.getHeading());
        double outputX = FtcMath.rotateY(-noRotOutputX, -noRotOutputY, pose.getHeading());
        packet.put("outputX", noRotOutputY);
        packet.put("outputY", noRotOutputX);
        packet.put("rotatePower", rotatePower);
        fleftPower += (outputY + outputX + rotatePower);
        bleftPower += (outputY - outputX + rotatePower);
        frightPower += (outputY - outputX - rotatePower);
        brightPower += (outputY + outputX - rotatePower);
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

}
