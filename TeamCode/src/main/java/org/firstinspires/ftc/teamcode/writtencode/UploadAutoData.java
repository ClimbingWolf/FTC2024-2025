package org.firstinspires.ftc.teamcode.writtencode;
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
		waitForStart();
		if (isStopRequested()) return;	
		Trajectory traj1 = drive.trajectoryBuilder(startPose)
			.splineToSplineHeading(new Pose2d(18.70976, 1.187622,0),-0.0174098488013306)
			.splineToSplineHeading(new Pose2d(27.38271, -21.11165,0),-2.01354313835808)
			.splineToSplineHeading(new Pose2d(5.11669, -25.81701,0),3.04085076282218)
			.splineToSplineHeading(new Pose2d(-0.8616577, -5.37869,0),1.33255798503968)
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