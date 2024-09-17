package org.firstinspires.ftc.teamcode.writtencode;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "BaseDriveCode")
@Config
public class BaseDrive extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
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


    @Override
    public void runOpMode() {

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
            y = FtcMath.rotateY(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.toRadians(firstAngle));
            x = FtcMath.rotateX(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.toRadians(firstAngle));
            rx = gamepad1.left_trigger-gamepad1.right_trigger;
            // basic teleop movement code
            bright.setPower((y+x-rx)*movementSpeedmultiplier * brightMult);
            fleft.setPower((y-x+rx)*movementSpeedmultiplier * fleftMult);
            fright.setPower((y-x-rx)*movementSpeedmultiplier * frightMult);
            bleft.setPower((y+x+rx)*movementSpeedmultiplier * bleftMult);
            // do an epic wheelie

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
