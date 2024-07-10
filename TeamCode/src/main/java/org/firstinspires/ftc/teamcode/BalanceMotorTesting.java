package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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


@TeleOp(name = "BalanceMotorTesting")
@Config

public class BalanceMotorTesting extends LinearOpMode {
    public double power = 0.0;

    double currentVelocity;

    double maxVelocity = 0.0;


    @Override

    public void runOpMode() {



        DcMotor motor = hardwareMap.dcMotor.get("blarm");

        waitForStart();



        while (opModeIsActive()) {
            power = epicGravityMath(motor.getCurrentPosition());
            motor.setPower(power);

            telemetry.addData("power", epicGravityMath(motor.getCurrentPosition()));

            telemetry.addData("encoderValue", motor.getCurrentPosition());

            telemetry.update();

        }


    }
    public double epicGravityMath(double encoderPos){
        return 0.5* (Math.cos(Math.PI/262*(encoderPos-22)));
    }

}