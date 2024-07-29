package org.firstinspires.ftc.teamcode;
import java.io.File;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.nio.charset.StandardCharsets;


@Autonomous(name = "UploadAutoData")
@Config
public class UploadAutoData extends LinearOpMode {

    private BNO055IMU imu;
    @Override
    public void runOpMode() {
	SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

   // We want to start the bot at x: 10, y: -8, heading: 90 degrees
	Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

	drive.setPoseEstimate(startPose);	
	Trajectory traj1 = drive.trajectoryBuilder(startPose)
		.splineTo(new Vector2d(0, 0), -1.30978993747255E-07)
		.splineTo(new Vector2d(-11.92045, 9.691962), -5.00865840751267E-08)
		.splineTo(new Vector2d(-24.68321, 25.39702), 2.2209596993862E-08)
		.splineTo(new Vector2d(-17.94977, 38.51656), 1.41321868765944E-07)
		.splineTo(new Vector2d(-5.629427, 47.71685), 3.7015773998422E-07)
		.splineTo(new Vector2d(13.66086, 53.27456), -3.59586875558143E-07)
		.splineTo(new Vector2d(28.08079, 48.15358), -5.81904283291965E-07)
		.splineTo(new Vector2d(41.26945, 40.26167), -2.46747264621135E-07)
		.splineTo(new Vector2d(47.23119, 30.66574), -6.73348918220917E-08)
		.splineTo(new Vector2d(26.76311, 31.00438), -1.22639963357453E-07)
		.splineTo(new Vector2d(6.305897, 31.31568), -4.52832785294207E-08)
		.splineTo(new Vector2d(14.4774, 21.84608), 1.33510918408509E-07)
		.splineTo(new Vector2d(32.0618, 11.36204), 2.03075289258066E-07)
		.splineTo(new Vector2d(50.62556, 2.706283), 2.11594170889394E-07)
		.splineTo(new Vector2d(70.47871, -1.337077), 1.0190309511937E-07)
		.splineTo(new Vector2d(77.9651, 15.85777), 2.46472766580047E-08)
		.splineTo(new Vector2d(77.91612, 31.24073), 4.15575653027959E-08)
		.build();
		action0();
		drive.followTrajectory(traj1);
	}

	public void action0(){
	
	}

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