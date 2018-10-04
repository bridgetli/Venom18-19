package org.firstinspires.ftc.teamcode;

import org.datavec.image.loader.NativeImageLoader;
import org.deeplearning4j.nn.modelimport.keras.KerasModelImport;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.nd4j.linalg.api.ndarray.INDArray;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import java.nio.ByteBuffer;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Vuforia2NeuralNetTest", group="Test")
public class Vuforia2NeuralNetTest extends LinearOpMode {

    private static final String VUFORIA_KEY = "AXb/g5n/////AAAAGSUed2rh5Us1jESA1cUn5r5KDUqTfwO2woh7MxjiLKSUyDslqBAgwCi0Qmc6lVczErnF5TIw7vG5R4TJ2igvrDVp+dP+3i2o7UUCRRj/PtyVgb4ZfNrDzHE80/6TUHifpKu4QCM04eRWYZocWNWhuRfytVeWy6NSTWefM9xadqG8FFrFk3XnvqDvk/6ZAgerNBdq5SsJ90eDdoAhgYEee40WxasoUUM9YVMvkWOqZgHSuraV2IyIUjkW/u0O+EkFtTNRUWP+aZwn1qO1H4Lk07AJYe21eqioBLMdzY7A8YqR1TeQ//0WJg8SFdXjuGbF6uHykBe2FF5UeyaehA0iTqfPS+59FLm8y1TuUt57eImq";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        telemetry.addData(">", "Press Play to Start.");
        telemetry.update();
        waitForStart();

        VuforiaLocalizer.CloseableFrame frame;
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        while (opModeIsActive()) {
            frame = vuforia.getFrameQueue().poll();
            if (frame != null) {
                Image image = frame.getImage(0);
                ByteBuffer byteBuffer = image.getPixels();

                String result = runNeuralNet(byteBuffer);
                telemetry.addData("Results", result);
                telemetry.update();
            }
        }
    }

    private String runNeuralNet(ByteBuffer image) {
        String modelFile = "TeamCode\\src\\main\\assets\\testModel1\\hamzaiswear.h5";
        MultiLayerNetwork model;
        INDArray input;
        INDArray output;

        try {
            //import keras model
            model = KerasModelImport.importKerasSequentialModelAndWeights(modelFile);
            model.init();

            //load input image & run inference
            input = new NativeImageLoader(28, 28, 1).asRowVector(image);

            //perform inference
            output = model.output(input);

            //output results
            return output.toString();
        }
        catch (Exception e) { System.out.print("Exception: " + e.toString() + " try again."); }
        return "ERROR";
    }
}