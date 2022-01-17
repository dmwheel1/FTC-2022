package org.firstinspires.ftc.teamcode.FusionFramework;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CommandRecorder {

    private PrintWriter writer;
    private Telemetry   telemetry;
    private boolean     cmd_in_progress = false;
    private ElapsedTime cmd_timer = null;


    public CommandRecorder(String filename, Telemetry t) {
        telemetry = t; // pass on the connection to the Driver's Station telemetry
        try {
            // Using the Java Environment, find the external storage path and make sure there is a directory for our data
            final File dir = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/RobotPathInfo/" );
            // Create the directory if it doesn't exist
            if (!dir.exists())
            {
                if(!dir.mkdirs()){
                    telemetry.addLine("ALERT: could not create the SD Card directories");
                }
            }

            // Now, use the filename to create a file in that directory
            final File myFile = new File(dir, filename + ".txt");
            //Create the file if it doesn't exist
            if (!myFile.exists())
            {
                myFile.createNewFile();
            }

            // Use a PrintWriter to open the file for writing
            writer = new PrintWriter(myFile);

        } catch (FileNotFoundException  e) {
            telemetry.addLine( e.getMessage() );
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine( e.getMessage() );
            telemetry.update();
        }
    }

    public void finalize()
    {
        // Ensure the file is flushed and closed properly
        writer.flush();
        writer.close();
    }

    public boolean initializeCommandRecording() {
        // TMake sure the file is set up so we can write into it
        if (writer == null) {
            telemetry.addLine("File was not set up properly - it's null");
            telemetry.update();
            return false;
        }

        // Write something to the file to prove it can be opened
        long millis=System.currentTimeMillis();
        java.util.Date date=new java.util.Date(millis);

        writer.print("Opening file on "+ date  );

        // Set up a timer to time command actions
        cmd_timer = new ElapsedTime(MILLISECONDS);
        cmd_timer.reset();

        return true;
    }

    public void writeNewCommand( String command ) {
        cmd_timer.reset(); // start a timer to record how long to run the command
        writer.print( command ); // record the command
        cmd_in_progress = true;
    }

    public void writeEncoderValuesToFile( ) {
        // TODO: implement this
    }

    public void finishLastCommand( ) {
        if (cmd_in_progress == false) return;

        double time = cmd_timer.time(); // reports duration of timer in milliseconds
        writer.print( time + " );\n"); // terminate the command in the log
        cmd_in_progress = false;

        writer.flush(); // force data to be written to the file
    }

    public void finishLastCommand(int backright, int backleft) {
        if (cmd_in_progress == false) return;

        double time = cmd_timer.time(); // reports duration of timer in milliseconds
        writer.print( backright + ", " + backleft + " );\n"); // terminate the command in the log
        cmd_in_progress = false;

        writer.flush(); // force data to be written to the file
    }
}
