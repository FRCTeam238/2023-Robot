package frc.core238.wrappers;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Filesystem;

public class ReverseChecker {

    public ReverseChecker() {
    }

    public static boolean checkReversed(String pathName){
        try (BufferedReader br =
            new BufferedReader(
                new FileReader(
                    new File(Filesystem.getDeployDirectory(), "pathplanner/" + pathName + ".path")))) {
        StringBuilder fileContentBuilder = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) {
            fileContentBuilder.append(line);
        }

        String fileContent = fileContentBuilder.toString();
        JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
        if (json.get("isReversed") == null) return false;

        return (Boolean) json.get("isReversed");

        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }
}
