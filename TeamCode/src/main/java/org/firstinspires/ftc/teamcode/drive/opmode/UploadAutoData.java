package org.firstinspires.ftc.teamcode.drive.opmode;
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
			.splineToSplineHeading(new Pose2d(9.985504, 22.56595,0),0.975646243263574)
			.splineToSplineHeading(new Pose2d(27.07364, 10.35364,0),-1.09544073199269)
			.splineToSplineHeading(new Pose2d(16.36446, -25.48673,0),-2.2848702173583)
			.splineToSplineHeading(new Pose2d(1.497254, -3.396731,0),1.9035921378281)
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