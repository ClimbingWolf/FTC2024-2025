package org.firstinspires.ftc.teamcode;

import java.io.File;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.nio.charset.StandardCharsets;

@Autonomous(name = "WriteFile")
@Config
public class WriteFile extends LinearOpMode {

    private BNO055IMU imu;
    @Override
    public void runOpMode() {
        String fullString = "";
        writeToFile(fullString, "autoData.txt");
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
