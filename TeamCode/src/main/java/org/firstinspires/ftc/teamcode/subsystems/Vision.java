package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

import java.util.List;

public class Vision {
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;

    public Vision(HardwareMap hardwareMap, String[] elements, double zoom, double aspectRatio, String modelFile) {
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        vuforiaPOWERPLAY.initialize(
                "", // vuforia
                // LicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        tfod.useModelFromFile(modelFile, JavaUtil.createListWith(elements), true, true, 320);
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.55, true, true);
        tfod.setClippingMargins(0, 0, 0, 0);
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(zoom, aspectRatio);
    }
    public int readCone(){
        // [TBD] separate POWERPLAY details from vision system and put in robot
        int cameraReads = 2;
        List<Recognition> recognitions;
        Recognition recognition;
        double highestConfidence = 0;
        long time = System.currentTimeMillis();
        final double secondsToSearch = 2;
        while (System.currentTimeMillis() - time < secondsToSearch*1000) {
            recognitions = tfod.getRecognitions();
            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;
                if (recognition_item.getLabel().equals("green") && recognition_item.getConfidence() > .85){
                    if (recognition_item.getConfidence() > highestConfidence) {
                        cameraReads = 3;
                        highestConfidence = recognition.getConfidence();
                    }
                }
                else if(recognition_item.getLabel().equals("red")) {
                    if (recognition_item.getConfidence() > highestConfidence) {
                        cameraReads = 1;
                        highestConfidence = recognition.getConfidence();
                    }
                }
                else if(recognition_item.getLabel().equals("blue")) {
                    if (recognition_item.getConfidence() > highestConfidence) {
                        cameraReads = 2;
                        highestConfidence = recognition.getConfidence();
                    }
                }
            }
        }
        return cameraReads;

    }
}
