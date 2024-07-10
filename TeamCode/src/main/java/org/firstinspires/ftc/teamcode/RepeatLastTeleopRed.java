package org.firstinspires.ftc.teamcode;
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


import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "RedAutoBoard")
@Config
public class RepeatLastTeleopRed extends LinearOpMode {


    public ElapsedTime timer = new ElapsedTime();
    public static double tlarmticks = 200;
    public static double blarmticks = -200;

    public static String fileEnd = "data.txt";
    public String fileStart = "byteData/";

    public String fileName = fileStart + fileEnd;


    public Gamepad tempGamepad = new Gamepad();


    public byte[] tempBytes;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    public double movementSpeedmultiplier = .3;
    public TelemetryPacket packet = new TelemetryPacket();

    public double idleBlarm = pidConstantsAndFunctions.idleBlarm;
    public double idleTlarm = pidConstantsAndFunctions.idleTlarm;

    public double idleServo = pidConstantsAndFunctions.idleServo;

    public double blarmOffset = 4;//pidConstantsAndFunctions.blarmOffset;
    public double tlarmOffset = pidConstantsAndFunctions.tlarmOffset;

    public double tlarmOffsetFloor= pidConstantsAndFunctions.tlarmOffsetFloor;
    public double blarmOffsetFloor = pidConstantsAndFunctions.blarmOffsetFloor;

    public boolean modePlace = true;

    public boolean idleState = true;

    public boolean rotateToPos = false;

    //PIDF constants for blarm

    //parameters for math

    public double barLength = pidConstantsAndFunctions.barLength;
    public double height = 5;

    public double boardAngle = -90;
    public double pixelAngle = -135;

    public double distance = 5;

    public double bottomDistance = 5;

    public double sensorDistance = 0;


    public double moveToBlarm = 0;

    public double moveToTlarm = 0;

    public boolean moveServo = false;

    public double servoOffsetTop = 270;//pidConstantsAndFunctions.servoOffsetTop;
    public double servoOffsetBottom = pidConstantsAndFunctions.servoOffsetBottom;

    public Servo clawPitch;
    public double tempDegrees = 0;
    public Servo claw;

    public static int arrSize = 10;

    //temporary variables to hold data

    ArrayList armThetas = new ArrayList();

    public double blarmTheta = 0;

    public boolean faceForward = false;
    public boolean facePixels = false;

    public double tlarmTheta = 0;

    public ArrayList<String> byteDataArr= new ArrayList<String>();

    public double servoTheta = 0;
    public int arrCounter = 0;
    private double y = 0;
    private double x = 0;
    private double rx = 0;

    public ArrayList distances = new ArrayList<Double>();

    public ArrayList loopTimes = new ArrayList<Double>();


    public boolean close = false;
    public double turnLeftAdd = 0;
    public double correctLooptime = 0;
    public double avgLooptime = 0;
    public double looptimeDiff = 0;

    public double distanceAdjuster = 12;

    public double posA = .9;
    public double posB = .62;

    public DistanceSensor sensor;

    public double firstAngle = 0;

    public int frame = -18;

    public double turnMacroSpeed= 1;

    public double rotation = 0;
    public int leftOfTarget = 0;

    public double maxReach = 23;


    public double rotateOffset = 0;

    //constants for math
    OpenCvWebcam webcam;
    CameraDetectPipeline pipeline;
    String position = "";

    ArrayList byteArrList;
    ArrayList rrl;
    ArrayList rrm;
    ArrayList rrr;
    double presentVoltage;
    private VoltageSensor myControlHubVoltageSensor;

    public static double voltageExtraMultiplier = 1.1;

    public double teleOpVoltage = 0;
    public double voltageCoefficient = 0;

    public ArrayList voltageArr = new ArrayList<Double>();

    @Override
    public void runOpMode() {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        for(int i = 0; i < 10; i ++){
            presentVoltage = myControlHubVoltageSensor.getVoltage();
            voltageArr.add(presentVoltage);
            sleep(100);
        }
        presentVoltage = pidConstantsAndFunctions.avgArrayList(voltageArr);
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new CameraDetectPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        for (int i = 0; i< arrSize; i++){
            distances.add(0.0);
        }
        //Make funny arraylist the right size in a really dumb way

        //initialize motors
        sensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        clawPitch = hardwareMap.get(Servo.class, "clawPitch");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(posB);

        MotorEx blarm =  new MotorEx(hardwareMap, "blarm");
        MotorEx tlarm = new MotorEx(hardwareMap, "tlarm");
        MotorEx afghm = new MotorEx(hardwareMap, "afghm");

        DcMotor bright = hardwareMap.dcMotor.get("bright");
        DcMotor bleft = hardwareMap.dcMotor.get("bleft");
        DcMotor fright = hardwareMap.dcMotor.get("fright");
        DcMotor fleft = hardwareMap.dcMotor.get("fleft");

        fleft.setDirection(DcMotorSimple.Direction.FORWARD);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bleft.setDirection(DcMotorSimple.Direction.FORWARD);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
        blarm.setInverted(true);
        blarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        tlarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        blarm.resetEncoder();
        tlarm.resetEncoder();
        //imu code i found online
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        PIDFController pidfBlarm = pidConstantsAndFunctions.pidfBlarm;
        PIDFController pidfTlarm = pidConstantsAndFunctions.pidfTlarm;

        pidfBlarm.setSetPoint(moveToBlarm);
        pidfTlarm.setSetPoint(moveToTlarm);

        // Put initialization blocks here.
        claw.setPosition(posB);
        rrr = readFromFile(fileStart + "rrr.txt");
        rrm = readFromFile(fileStart +"rrm.txt");
        rrl = readFromFile(fileStart +"rrl.txt");
        waitForStart();
        dashboard.startCameraStream(webcam, 60);



        while (opModeIsActive()) {
            afghm.setVelocity(0.01);
            voltageCoefficient = teleOpVoltage * voltageExtraMultiplier/presentVoltage;
            timer.reset();

            if(frame < 0){
                if(pipeline.position.equals("right")){
                    teleOpVoltage = Double.parseDouble(strReadFromFile("voltage" + fileStart +"rrr.txt"));
                    correctLooptime = teleOpVoltage = Double.parseDouble(strReadFromFile("looptime" + fileStart +"rrr.txt"));
                    byteArrList = rrr;
                }
                else if(pipeline.position.equals("middle")){
                    teleOpVoltage = Double.parseDouble(strReadFromFile("voltage" + fileStart + "rrm.txt"));
                    correctLooptime = teleOpVoltage = Double.parseDouble(strReadFromFile("looptime" + fileStart +"rrm.txt"));

                    byteArrList = rrm;

                }
                else if (pipeline.position.equals("left")){
                    teleOpVoltage = Double.parseDouble(strReadFromFile("voltage" + fileStart +"rrl.txt" ));
                    correctLooptime = teleOpVoltage = Double.parseDouble(strReadFromFile("looptime" + fileStart +"rrl.txt"));
                    byteArrList =rrl;

                }
                else{

                }
            }
            frame+=1;
            if(frame < byteArrList.size() && frame > -1) {
                tempGamepad.fromByteArray((byte[]) byteArrList.get(frame));
                lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                firstAngle = lastAngles.firstAngle;
                if (tempGamepad.right_stick_button) {
                    movementSpeedmultiplier = .3;
                } else {
                    movementSpeedmultiplier = .3;
                }
                //teleop movement code
                y = rotateY(tempGamepad.left_stick_x, tempGamepad.left_stick_y, pidConstantsAndFunctions.degreesToRadians(firstAngle));
                x = rotateX(tempGamepad.left_stick_x, tempGamepad.left_stick_y, pidConstantsAndFunctions.degreesToRadians(firstAngle));
                rx = tempGamepad.left_trigger - tempGamepad.right_trigger + turnLeftAdd;

                // basic teleop movement code
                bright.setPower((y + x - rx) * movementSpeedmultiplier * voltageCoefficient);
                fleft.setPower((y - x + rx) * movementSpeedmultiplier* voltageCoefficient);
                fright.setPower((y - x - rx) * movementSpeedmultiplier* voltageCoefficient);
                bleft.setPower((y + x + rx) * movementSpeedmultiplier* voltageCoefficient);
                //moves the robot to the center

                //average the values from the sensor to stop pain;
                //distances.add(sensorDistance);
                //distances.remove(0);
                //sensorDistance = pidConstantsAndFunctions.avgArrayList(distances);
                //gamepade movements
                if (tempGamepad.dpad_up) {
                    distance = 10;
                    height = 6;
                    modePlace = true;
                    idleState = false;
                } else if (tempGamepad.dpad_down) {
                    bottomDistance = 7;
                    modePlace = false;
                    idleState = false;
                } else if (tempGamepad.dpad_left || tempGamepad.dpad_right) {
                    idleState = true;
                }

                if (tempGamepad.right_bumper) {
                    bottomDistance += .3;
                    distance += .3;
                } else if (tempGamepad.left_bumper) {
                    bottomDistance -= .3;
                    distance -= .3;
                }


                if (tempGamepad.x && modePlace && !idleState) {
                    height -= .3;
                } else if (tempGamepad.y && modePlace && !idleState) {
                    height += .3;
                } else if (tempGamepad.x && !faceForward) {
                    faceForward = true;
                    facePixels = false;
                } else if (tempGamepad.y && !facePixels) {
                    facePixels = true;
                    faceForward = false;
                }

                if (faceForward) {
                    tempDegrees = Math.floorMod((long) firstAngle - (long) boardAngle, 360);
                    if (tempDegrees > 180) {
                        turnLeftAdd = turnMacroSpeed;
                    } else {
                        turnLeftAdd = -turnMacroSpeed;
                    }
                    if (tempDegrees > 355 || tempDegrees < 5) {
                        faceForward = false;
                        turnLeftAdd = 0;
                    }
                }
                if (facePixels) {
                    tempDegrees = Math.floorMod((long) firstAngle - (long) pixelAngle, 360);
                    if (tempDegrees > 180) {
                        turnLeftAdd = turnMacroSpeed;
                    } else {
                        turnLeftAdd = -turnMacroSpeed;
                    }
                    if (tempDegrees > 355 || tempDegrees < 5) {
                        facePixels = false;
                        turnLeftAdd = 0;
                    }
                }


                //set angles based on distance and height and plug into set point
                if (modePlace) {
                    armThetas = pidConstantsAndFunctions.calculateArmThetas(height, distance, barLength, blarmOffset, tlarmOffset, servoOffsetTop);
                } else {
                    moveServo = true;
                    armThetas = pidConstantsAndFunctions.calculateArmThetasFloor(bottomDistance, barLength, blarmOffsetFloor, tlarmOffsetFloor, servoOffsetBottom);
                }

                if (idleState) {
                    moveServo = false;

                    //this is a dumb solution, but it wouldn't work when I tried other things
                    armThetas.set(0, pidConstantsAndFunctions.degreesToRadians(idleBlarm));
                    armThetas.set(1, pidConstantsAndFunctions.degreesToRadians(idleTlarm));
                    armThetas.set(2, pidConstantsAndFunctions.degreesToRadians(idleServo));
                    blarmTheta = (double) (armThetas.get(0));
                    tlarmTheta = (double) (armThetas.get(1));
                    servoTheta = (double) (armThetas.get(2));
                }
                //angle check to stop program from crashing
                if (!(Double.isNaN((double) (armThetas.get(0))) || Double.isNaN((double) (armThetas.get(1))))) {
                    blarmTheta = (double) (armThetas.get(0));
                    tlarmTheta = (double) (armThetas.get(1));
                    servoTheta = (double) (armThetas.get(2));
                } else if (distance < maxReach & height < maxReach && height != 0 && distance != 0 && modePlace == true && (tempGamepad.y || tempGamepad.x)) {
                    distance = pidConstantsAndFunctions.maxDistance(height, maxReach);
                    armThetas = pidConstantsAndFunctions.calculateArmThetas(height, distance, barLength, blarmOffset, tlarmOffset, servoOffsetTop);
                } else if (distance < maxReach & height < maxReach && height != 0 && distance != 0 && modePlace == true && (tempGamepad.left_bumper || tempGamepad.right_bumper)) {
                    height = pidConstantsAndFunctions.maxHeight(distance, maxReach);
                    armThetas = pidConstantsAndFunctions.calculateArmThetas(height, distance, barLength, blarmOffset, tlarmOffset, servoOffsetTop);
                }

                moveToTlarm = pidConstantsAndFunctions.radiansToTicks(tlarmTheta);
                moveToBlarm = pidConstantsAndFunctions.radiansToTicks(blarmTheta);
                pidfBlarm.setSetPoint(moveToBlarm);
                pidfTlarm.setSetPoint(moveToTlarm);

                //calculate output based on the pidf of the current and target positions
                double outputBlarm = pidfBlarm.calculate(blarm.getCurrentPosition());
                blarm.setVelocity(outputBlarm);
                double outputTlarm = pidfTlarm.calculate(tlarm.getCurrentPosition());
                tlarm.setVelocity(outputTlarm);
                if (tempGamepad.left_stick_button) {
                    moveServo = true;
                }

                //move to servo
                if (moveServo) {
                    clawPitch.setPosition(pidConstantsAndFunctions.radiansTo5ServoPos((double) (servoTheta)));
                }

                if (tempGamepad.a && !close) {
                    close = true;
                    tlarmOffsetFloor += 5;
                    claw.setPosition(posA);
                } else if (tempGamepad.b && close) {
                    close = false;
                    tlarmOffsetFloor -= 5;
                    claw.setPosition(posB);
                }
                loopTimes.add((double)timer.time(TimeUnit.MILLISECONDS));
                avgLooptime = pidConstantsAndFunctions.avgArrayList(loopTimes);
                if(correctLooptime- avgLooptime > 0){
                    looptimeDiff = correctLooptime- avgLooptime;
                }
                sleep((long)looptimeDiff);
                //pre-programmed positions


                //telemetry
                packet.put("position", pipeline.position);
                packet.put("sensorDistance", sensorDistance);
                packet.put("distance", distance);
                packet.put("height", height);
                packet.put("gamepadInfo", tempGamepad.toString());
                packet.put("voltageCoefficient", voltageCoefficient);
                packet.put("loopTime (ms)", avgLooptime);
                packet.put("looptimeDiff", looptimeDiff);
                packet.put("correctLooptime", correctLooptime);

                dashboard.sendTelemetryPacket(packet);
            }
            else{
                tlarm.setVelocity(-tlarmticks);
                blarm.setVelocity(blarmticks);
                writeToFile("" + firstAngle, "orientation.txt");
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

    // Method 2
    // Main driver method
    public static void writeToFile (String text, String toFileName) {

        // Using the properties of the specified "to" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);

        // Write the provided number to the newly declared filename.
        // See Note 3 above.
        ReadWriteFile.writeFile(myFileName, text);


    }   // end of method writeToFile()
    public static ArrayList readFromFile (String fromFileName) {
        ArrayList byteList = new ArrayList();

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newly declared filename.
        // s
        String[] stringArr = ReadWriteFile.readFile(myFileName).split("ඞ");

        for(int i =0; i < stringArr.length; i++){
            byteList.add(string2ByteArr(stringArr[i]));
        }

        return byteList;       // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()

    public static byte[] string2ByteArr(String string){
        return string.getBytes(StandardCharsets.ISO_8859_1);
    }

    public static String byteArr2String(byte[] byteArr){
        return new String(byteArr, StandardCharsets.ISO_8859_1);
    }

    public static String strReadFromFile (String fromFileName) {

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newlasy declared filename.
        // See Note 4 above.
        return ReadWriteFile.readFile(myFileName);
        // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()
}
