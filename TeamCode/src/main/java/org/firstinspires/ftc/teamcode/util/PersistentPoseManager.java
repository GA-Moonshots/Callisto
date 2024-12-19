package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.acmerobotics.roadrunner.Pose2d;
import java.io.*;

public class PersistentPoseManager {
    //TODO: Make file path variable
    private static final String POSE_FILE_PATH = Environment.getExternalStorageDirectory().getPath()+"/FIRST/pose.txt"; // Save file location

    /**
     * Saves the robot's pose to a file.
     *
     * @param pose The robot's current pose (x, y, heading).
     */
    public static void savePose(Pose2d pose) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(POSE_FILE_PATH))) {
            // Write pose data to file as a comma-separated string
            writer.write(pose.position.x + "," + pose.position.y + "," + pose.heading);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Loads the robot's pose from a file.
     *
     * @return The saved pose, or (0, 0, 0) if the file does not exist or an error occurs.
     */
    public static Pose2d loadPose() {
        try (BufferedReader reader = new BufferedReader(new FileReader(POSE_FILE_PATH))) {
            // Read pose data from the file
            String line = reader.readLine();
            if (line != null) {
                String[] data = line.split(",");
                double x = Double.parseDouble(data[0]);
                double y = Double.parseDouble(data[1]);
                double heading = Double.parseDouble(data[2]);
                return new Pose2d(x, y, heading);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        // Default pose if loading fails
        return new Pose2d(0, 0, 0);
    }
}
