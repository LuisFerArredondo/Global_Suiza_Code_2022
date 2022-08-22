package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Team15600Lib.Enums.VisionState;
import org.firstinspires.ftc.teamcode.Team15600Lib.Threads.VisionThread;

import java.util.List;

/**
 * This class is just a template, copy and implement it later in project
 * */
public class TensorFlowThreadExample extends VisionThread {

    private static final String TFOD_MODEL_ASSET = "";
    private static final String[] LABELS = {};
    private static VisionState visionState = VisionState.OFF;

    private static final String VUFORIA_KEY = "AWzIAAP/////AAABmbsF9ADliEMWitjJquentfA+6qSSmzgslcMOzg3tHSPk9RZVRAR0MC+kU3JmWoLnB5oKWTq5aHIiiwagMvJV4NkBDjrIg/iHRg+ACB7cYJ2TZC/U2ekvBqj6AcYe8UmCmDlRjKtgeF2UWO6FwgKo9vFy5ix0X80q4/d5huU/1e+zGQ0EG/5VRIGiZA5oliiONBVcCxKvtPjdBSZXLet0IlQjtm6/nnKqlYieNSdSqLnkgZITyLEzEs6TgQ4oADyU89DORtqFScG1ROLgQweHLSv1OFVhsafaqfn/mXoXDAwwlHWDaHCzJHI8dzlqer93ZDsdwCB0XPcFWYBTJNMXJ0tnUB86/4ZnA4iEtNnOQcr4";
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public TensorFlowThreadExample(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void run() {
        if (tfod != null) {
            tfod.activate();

            //Note: 19.5:9 iphone 13 images aspect ratio
            //Normal values Magnification:2.5, aR:16.0/9.0
            tfod.setZoom(2.5, 19.5/9.0);
            visionState = VisionState.INITIALIZING;
            telemetry.addData("Vision state", visionState.toString());
        }

        while (!Thread.currentThread().isInterrupted()){
            FtcDashboard.getInstance().startCameraStream(tfod, 20);
            visionState = VisionState.PROCESSING;
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    visionState = VisionState.TARGET_FOUND;
                    /**
                     * Note: for cycle not needed
                     *
                     * can use "continue;" sentence to  jump a not important label
                     * */

                    for (Recognition recognition : updatedRecognitions) {
                        /** Get the name  and the confidence of the detected object*/
                        //recognition.getLabel();
                        //recognition.getConfidence();

                        /** Get the left Corner by the left and top pixel number in image*/
                        //recognition.getLeft();
                        //recognition.getTop();

                        /** Get the right Corner by the right and bottom pixel number in image*/
                        //recognition.getRight();
                        //recognition.getBottom();

                        /*TODO:
                        ** Left Corner
                        *       |------------------|
                        *       |                  |
                        *       |                  |
                        *       |                  |
                        *       |------------------|
                        *                    Right Corner
                        * */

                        /** Get the width and height of the detected object*/
                        //recognition.getWidth();
                        //recognition.getHeight();

                        /** Get the width and height of the camera image*/
                        //recognition.getImageWidth();
                        //recognition.getImageHeight();
                    }
                }
            }
            telemetry.addData("Vision state", visionState.toString());
        }
        FtcDashboard.getInstance().stopCameraStream();
        tfod.deactivate();
        visionState = VisionState.STOPPED;
        telemetry.addData("Vision state", visionState.toString());
    }

    @Override
    public VisionState getVisionThreadState() {
        return visionState;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;//min number of confidence 0-1
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
// double distance = (Constants.VisionConstants.HEIGHT_OF_OUTER_PORT
//    - Constants.VisionConstants.LIMELIGHT_FLOOR_CLEREANCE)
//        / Math.tan(Math.toRadians(ty.getDouble(0.0) + Constants.VisionConstants.LIMELIGHT_VERTICAL_ANGLE));
//
