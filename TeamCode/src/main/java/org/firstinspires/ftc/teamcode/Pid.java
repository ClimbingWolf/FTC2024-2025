package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Pid {

    public static double Kp = 1.0;
    public static double Ki = 1.0;
    public static double Kd = 1.0;
    public static double reference = 1.0f;

    public boolean setPointIsNotReached = true;
    public double encoderPosition = 0;

    public double integralSum = 0;

    public double lastError = 0;

    public double error = 0;
    public double derivative = 0;
    public double out = 0;
    public ElapsedTime timer = new ElapsedTime();

    public Pid(){

    }
    void doPidf(){
        /*

         * Proportional Integral Derivative Controller

         */

        reference = 0;

        integralSum = 0;

        lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (setPointIsNotReached) {


            timer.reset();

        }

    }

    public void update(DcMotor armMotor, double reference, double position, double Kp, double Ki, double Kd){

        // obtain the encoder position
        position = armMotor.getCurrentPosition();
        // calculate the error
        error = reference - position;


        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        armMotor.setPower(out);

        lastError = error;

        timer.reset();

    }

}
