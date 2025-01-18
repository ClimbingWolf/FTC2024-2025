package org.firstinspires.ftc.teamcode.auto;
import java.io.File;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.nio.charset.StandardCharsets;


@Autonomous(name = "RedLeftAuto")
@Config
public class RedLeft extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        waitForStart();
        if (isStopRequested()) return;
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(18.40373, 0.2713332,0),0.00646656852922711)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToSplineHeading(new Pose2d(24.93333, 0,0),-0.041530428538243)
                .splineToSplineHeading(new Pose2d(14.59113, 44.84872,0),1.79283450558567)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToSplineHeading(new Pose2d(13.33333, 50.8,0),1.77686060062932)
                .splineToSplineHeading(new Pose2d(32.62225, 48.10669,-0.7853969),-0.230041667650082)
                .splineToSplineHeading(new Pose2d(47.09919, 31.86043,-1.570796),-1.23750032349251)
                .build();
        action0();
        drive.followTrajectory(traj1);
        action1();
        drive.followTrajectory(traj2);
        action2();
        drive.followTrajectory(traj3);
    }

    public void action0(){

    }

    public void action1(){

    }

    public void action2(){

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