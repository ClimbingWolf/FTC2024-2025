package org.firstinspires.ftc.teamcode.mathfunctions;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

public class ReadWriteFile {
    public static void writeToFile (String text, String toFileName) {

        // Using the properties of the specified "to" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);

        // Write the provided number to the newly declared filename.
        // See Note 3 above.
        com.qualcomm.robotcore.util.ReadWriteFile.writeFile(myFileName, text);


    }   // end of method writeToFile()
    public static String readFromFile (String fromFileName) {

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newlasy declared filename.
        // See Note 4 above.
        return com.qualcomm.robotcore.util.ReadWriteFile.readFile(myFileName);
        // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()

    public static byte[] string2ByteArr(String string){
        return string.getBytes(StandardCharsets.ISO_8859_1);
    }

    public static String byteArr2String(byte[] byteArr){
        return new String(byteArr, StandardCharsets.ISO_8859_1);
    }
    public static ArrayList readParseFile (String fromFileName) {
        ArrayList byteList = new ArrayList();

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newly declared filename.
        // s
        String[] stringArr = com.qualcomm.robotcore.util.ReadWriteFile.readFile(myFileName).split("ඞ");

        for(int i =0; i < stringArr.length; i++){
            byteList.add(string2ByteArr(stringArr[i]));
        }

        return byteList;       // provide the number to the Block calling this myBlock
    }
    public static ArrayList readParseFileNoBytes (String fromFileName) {
        ArrayList byteList = new ArrayList();

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newly declared filename.
        // s
        String[] stringArr = com.qualcomm.robotcore.util.ReadWriteFile.readFile(myFileName).split("ඞ");

        for(int i =0; i < stringArr.length; i++){
            byteList.add(stringArr[i]);
        }

        return byteList;       // provide the number to the Block calling this myBlock

    }
}
