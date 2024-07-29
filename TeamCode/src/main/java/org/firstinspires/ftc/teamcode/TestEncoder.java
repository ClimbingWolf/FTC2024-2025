package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.File;
import java.nio.charset.StandardCharsets;


@TeleOp(name = "TestEncoder")
@Config
public class TestEncoder extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private BNO055IMU imu;
    public DcMotor bright;
    public DcMotor bleft;
    public DcMotor fright;
    public DcMotor fleft;
    public DcMotor brarm;

    public DcMotor perp;
    public int parallelWheel;

    public int perpWheel;
    public double deadWheel1Val;
    public Encoder parallelEncoder;
    public Encoder perpendicularEncoder;



    @Override
    public void init() {
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "brarm"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perp"));
        brarm = hardwareMap.dcMotor.get("brarm");
        perp = hardwareMap.dcMotor.get("perp");
        brarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    @Override
    public void init_loop(){


    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        parallelWheel = parallelEncoder.getCurrentPosition();
        perpWheel = perpendicularEncoder.getCurrentPosition();
        packet.put("parallelPos", parallelWheel);
        packet.put("perpPos", perpWheel);
        dashboard.sendTelemetryPacket(packet);
    }
    @Override
    public void stop(){

    }
    public void goToPoint(){

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
    public static String[] readFromFile (String fromFileName) {

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newly declared filename.
        // s
        return ReadWriteFile.readFile(myFileName).split("\n");


    }  // end of method readFromFile()

    public static byte[] string2ByteArr(String string){
        return string.getBytes(StandardCharsets.ISO_8859_1);
    }

    public static String byteArr2String(byte[] byteArr){
        return new String(byteArr, StandardCharsets.ISO_8859_1);
    }
}
