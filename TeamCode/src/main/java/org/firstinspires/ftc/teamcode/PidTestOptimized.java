package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;


import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Timer;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "BlueTeleOp")
@Config
public class PidTestOptimized extends LinearOpMode {

    public static double fireA = 0.0;

    public ElapsedTime timer = new ElapsedTime();
    public static double fireB = 0.8;

    public static double inputCounter = 0;

    public static double blarmMacroA = pidConstantsAndFunctions.blarmMacroA;
    public static double tlarmMacroA = pidConstantsAndFunctions.tlarmMacroA;

    public static double blarmMacroB = pidConstantsAndFunctions.blarmMacroB;
    public static double tlarmMacroB = pidConstantsAndFunctions.tlarmMacroB;
    public static String fileEnd = "data.txt";
    public String fileStart = "byteData/";

    public String fileName = fileStart + fileEnd;
    private VoltageSensor myControlHubVoltageSensor;




    public Gamepad tempGamepad = new Gamepad();

    public byte[] tempBytes;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    public double movementSpeedmultiplier = 1;
    public TelemetryPacket packet = new TelemetryPacket();

    public static double idleBlarm = pidConstantsAndFunctions.idleBlarm;
    public static double idleTlarm = pidConstantsAndFunctions.idleTlarm;

    public static double idleServo = pidConstantsAndFunctions.idleServo;

    public static double blarmOffset = 4;//pidConstantsAndFunctions.blarmOffset;
    public static double tlarmOffset = pidConstantsAndFunctions.tlarmOffset;

    public static double tlarmOffsetFloor= pidConstantsAndFunctions.tlarmOffsetFloor;
    public static double blarmOffsetFloor = 39;//p2idConstantsAndFunctions.blarmOffsetFloor;

    public boolean modePlace = true;

    public boolean idleState = true;

    public boolean rotateToPos = false;

    //PIDF constants for blarm

    //parameters for math

    public double barLength = pidConstantsAndFunctions.barLength;

    public static double offsetOrientation = 0;
    public static double height = 5;

    public static double boardAngle = 90;
    public static double pixelAngle = 135;

    public static double distance = 5;

    public static double bottomDistance = 5;

    public double sensorDistance = 0;


    public static double moveToBlarm = 0;

    public static double moveToTlarm = 0;

    public boolean moveServo = false;
    double presentVoltage;

    public static double servoOffsetTop = 240;//pidConstantsAndFunctions.servoOffsetTop;
    public static double servoOffsetBottom = 27;//pidConstantsAndFunctions.servoOffsetBottom;

    public Servo clawPitch;
    public double tempDegrees = 0;
    public Servo claw;

    public static int arrSize = 10;
    public ArrayList loopTimes = new ArrayList<Double>();

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

    public Servo fireInTheHole;


    public boolean close = false;
    public double turnLeftAdd = 0;

    public static double distanceAdjuster = 12;

    public static double posA = .85;
    public static double posB = .55;

    public DistanceSensor sensor;

    public double firstAngle = 0;

    public static double turnMacroSpeed= 1;

    public double rotation = 0;
    public int leftOfTarget = 0;

    public static double maxReach = 23;

    public static double rotateOffset = 0;


    //constants for math



    @Override
    public void runOpMode() {

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        offsetOrientation = Double.parseDouble(readFromFile("orientation.txt"));

        for (int i = 0; i< arrSize; i++){
            distances.add(0.0);
        }
        //Make funny arraylist the right size in a really dumb way

        //initialize motors
        sensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        clawPitch = hardwareMap.get(Servo.class, "clawPitch");
        claw = hardwareMap.get(Servo.class, "claw");
        fireInTheHole = hardwareMap.get(Servo.class, "fireInTheHole");
        claw.setPosition(posA);
        fireInTheHole.setPosition(fireA);

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
        afghm.setInverted(false);
        blarm.setInverted(true);
        blarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        tlarm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        blarm.resetEncoder();
        afghm.resetEncoder();
        tlarm.resetEncoder();
        //imu code i found online
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
        PIDFController pidfBlarm = pidConstantsAndFunctions.pidfBlarm;
        PIDFController pidfTlarm = pidConstantsAndFunctions.pidfTlarm;

        pidfBlarm.setSetPoint(moveToBlarm);
        pidfTlarm.setSetPoint(moveToTlarm);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Put initialization blocks here.
        ArrayList voltageArr = new ArrayList<Double>();
        claw.setPosition(posB);
        for(int i = 0; i < 10; i ++){
            presentVoltage = myControlHubVoltageSensor.getVoltage();
            voltageArr.add(presentVoltage);
            sleep(100);
        }

        writeToFile(""+presentVoltage, "voltage" + fileName);
        presentVoltage = pidConstantsAndFunctions.avgArrayList(voltageArr);
        waitForStart();
        while (opModeIsActive()) {
            timer.reset();
            tempGamepad.copy(gamepad1);
            byteDataArr.add(byteArr2String(tempGamepad.toByteArray()));
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            firstAngle = lastAngles.firstAngle + offsetOrientation;
            if(gamepad1.right_stick_button){
                movementSpeedmultiplier = .3;
            }
            else{
                movementSpeedmultiplier = 1;
            }
            //teleop movement code
            y = rotateY(gamepad1.left_stick_x, gamepad1.left_stick_y, pidConstantsAndFunctions.degreesToRadians(firstAngle));
            x = rotateX(gamepad1.left_stick_x, gamepad1.left_stick_y, pidConstantsAndFunctions.degreesToRadians(firstAngle));
            rx = gamepad1.left_trigger-gamepad1.right_trigger + turnLeftAdd;

            // basic teleop movement code
            bright.setPower((y+x-rx)*movementSpeedmultiplier);
            fleft.setPower((y-x+rx)*movementSpeedmultiplier);
            fright.setPower((y-x-rx)*movementSpeedmultiplier);
            bleft.setPower((y+x+rx)*movementSpeedmultiplier);
            //moves the robot to the center

            //average the values from the sensor to stop pain;
            //distances.add(sensorDistance);
            //distances.remove(0);
            //sensorDistance = pidConstantsAndFunctions.avgArrayList(distances);
            //gamepade movements
            if (gamepad1.dpad_up){
                distance = 10;
                height = 6;
                modePlace = true;
                idleState = false;
                if(inputCounter >= 4){
                    blarmOffset = blarmMacroA;
                    tlarmOffset = tlarmMacroA;
                    inputCounter = 5;
                }
                else{
                    inputCounter = 0;
                }
            }

            else if (gamepad1.dpad_down){
                bottomDistance = 7;
                modePlace = false;
                idleState = false;
            }

            else if(gamepad1.dpad_left || gamepad1.dpad_right){
                idleState = true;
            }

            if(gamepad1.a && gamepad1.b && gamepad1.dpad_right){
                fireInTheHole.setPosition(fireB);
            }

            if(gamepad1.dpad_left){
                if(inputCounter == 2||inputCounter==3) {
                    inputCounter = 3;
                }
                else if (inputCounter == 0){
                        inputCounter = 1;
                    }
            }
            if(gamepad1.dpad_right){
                if(inputCounter == 1) {
                    inputCounter = 2;
                }
                else if (inputCounter == 3){
                    inputCounter = 4;
                }
            }


            if (gamepad1.right_bumper){
                bottomDistance +=.3;
                distance +=.3;
            }
            else if (gamepad1.left_bumper){
                bottomDistance-=.3;
                distance-= .3;
            }

           if(inputCounter > 4){
               if(gamepad1.a){
                   tlarmOffset += .5;
               }
               if(gamepad1.b){
                   tlarmOffset -= .5;
               }
               if(gamepad1.x){
                   blarmOffset += .5;
               }
               if(gamepad1.y){
                   blarmOffset -= .5;
               }
           }
           blarmOffsetFloor += gamepad1.right_stick_y;

            if(gamepad1.x && modePlace && !idleState && inputCounter < 5){
                height -=.3;
            }
            else if(gamepad1.y && modePlace && !idleState && inputCounter < 5){
                height+=.3;
            }
            else if (gamepad1.x && !faceForward && inputCounter < 5){
                faceForward = true;
                facePixels = false;
            }
            else if (gamepad1.y && !facePixels && inputCounter < 5){
                facePixels = true;
                faceForward = false;
            }

            if (faceForward){
                tempDegrees = Math.floorMod((long)firstAngle - (long)boardAngle, 360);
                if(tempDegrees > 180){
                    turnLeftAdd = turnMacroSpeed;
                }
                else{
                    turnLeftAdd = -turnMacroSpeed;
                }
                if(tempDegrees > 355 || tempDegrees < 5){
                    faceForward = false;
                    turnLeftAdd = 0;
                }
            }
            if (facePixels){
                tempDegrees = Math.floorMod((long)firstAngle - (long)pixelAngle, 360);
                if(tempDegrees > 180){
                    turnLeftAdd = turnMacroSpeed;
                }
                else{
                    turnLeftAdd =  -turnMacroSpeed;
                }
                if(tempDegrees > 355 || tempDegrees < 5){
                    facePixels = false;
                    turnLeftAdd = 0;
                }
            }



            //set angles based on distance and height and plug into set point
            if (modePlace) {
                armThetas = pidConstantsAndFunctions.calculateArmThetas(height, distance, barLength, blarmOffset, tlarmOffset, servoOffsetTop);
            }
            else{
                moveServo = true;
                armThetas = pidConstantsAndFunctions.calculateArmThetasFloor(bottomDistance, barLength, blarmOffsetFloor, tlarmOffsetFloor, servoOffsetBottom);
            }

            if(idleState){
                pidfTlarm.setD(2);
                moveServo = false;

                //this is a dumb solution, but it wouldn't work when I tried other things
                armThetas.set(0,pidConstantsAndFunctions.degreesToRadians(idleBlarm));
                armThetas.set(1,pidConstantsAndFunctions.degreesToRadians(idleTlarm));
                armThetas.set(2,pidConstantsAndFunctions.degreesToRadians(idleServo));
                blarmTheta = (double) (armThetas.get(0));
                tlarmTheta = (double) (armThetas.get(1));
                servoTheta = (double)(armThetas.get(2));
            }
            else{
                pidfTlarm.setD(pidConstantsAndFunctions.kDt);
            }
            //angle check to stop program from crashing
            if (!(Double.isNaN((double)(armThetas.get(0)))||Double.isNaN((double)(armThetas.get(1))))) {
                blarmTheta = (double) (armThetas.get(0));
                tlarmTheta = (double) (armThetas.get(1));
                servoTheta = (double)(armThetas.get(2));
            }
            else if(distance < maxReach & height <maxReach&&height != 0 && distance != 0 && modePlace == true && (gamepad1.y || gamepad1.x)){
                distance = pidConstantsAndFunctions.maxDistance(height, maxReach);
                armThetas = pidConstantsAndFunctions.calculateArmThetas(height, distance, barLength, blarmOffset, tlarmOffset, servoOffsetTop);
            }
            else if(distance < maxReach & height <maxReach&&height != 0 && distance != 0 && modePlace == true && (gamepad1.left_bumper || gamepad1.right_bumper)){
                height  = pidConstantsAndFunctions.maxHeight(distance, maxReach);
                armThetas = pidConstantsAndFunctions.calculateArmThetas(height, distance, barLength, blarmOffset, tlarmOffset, servoOffsetTop);
            }

            moveToTlarm = pidConstantsAndFunctions.radiansToTicks(tlarmTheta);
            moveToBlarm = pidConstantsAndFunctions.radiansToTicks(blarmTheta);
            pidfBlarm.setSetPoint(moveToBlarm);
            pidfTlarm.setSetPoint(moveToTlarm);

            //calculate output based on the pidf of the current and target positions
            double outputBlarm = pidfBlarm.calculate(blarm.getCurrentPosition());
            blarm.setVelocity(outputBlarm);
            if(inputCounter > 4){
                afghm.setVelocity(outputBlarm);
            }
            else{
                afghm.setVelocity(0.01);
            }
            double outputTlarm = pidfTlarm.calculate(tlarm.getCurrentPosition());
            tlarm.setVelocity(outputTlarm);
            if(gamepad1.left_stick_button){
                moveServo = true;
            }

            //move to servo
            if(moveServo){
                clawPitch.setPosition(pidConstantsAndFunctions.radiansTo5ServoPos((double)(servoTheta)));
            }

            if(gamepad1.a && !close) {
                close = true;
                tlarmOffsetFloor+=5;
                claw.setPosition(posA);
            }
            else if(gamepad1.b && close){
                close = false;
                tlarmOffsetFloor-=5;
                claw.setPosition(posB);
            }

            if(gamepad1.a && inputCounter == 5){
                //tlarmOffset = tlarmMacroB;
                //blarmOffset = blarmMacroB;
            }
            loopTimes.add((double)timer.time(TimeUnit.MILLISECONDS));
            //pre-programmed positions


            //telemetry
            packet.put("firstAngle", firstAngle);
            packet.put("blarmOffset", blarmOffset);
            packet.put("tlarmOffset", tlarmOffset);
            packet.put("inputCount", inputCounter);
            packet.put("voltage", presentVoltage);
            packet.put("loopTime (ms)", pidConstantsAndFunctions.avgArrayList(loopTimes));
            dashboard.sendTelemetryPacket(packet);

        }
        //geeksforgeekscode to output the file
        StringBuilder str = new StringBuilder("");

        // Traversing the ArrayList
        for (String eachstring : byteDataArr) {

            // Each element in ArrayList is appended
            // followed by comma
            str.append(eachstring).append("ඞ");
        }

        // StringBuffer to String conversion
        String commaseparatedlist = str.toString();

        // Condition check to remove the tel
            commaseparatedlist= commaseparatedlist.substring(0, commaseparatedlist.length() - 1);

        writeToFile(commaseparatedlist, fileName);
        writeToFile("" + pidConstantsAndFunctions.avgArrayList(loopTimes), "looptime" + fileName);
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
    public static String readFromFile (String fromFileName) {

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newlasy declared filename.
        // See Note 4 above.
        return ReadWriteFile.readFile(myFileName);
        // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()

    public static byte[] string2ByteArr(String string){
        return string.getBytes(StandardCharsets.ISO_8859_1);
    }

    public static String byteArr2String(byte[] byteArr){
        return new String(byteArr, StandardCharsets.ISO_8859_1);
    }
}
