package org.firstinspires.ftc.teamcode;


import org.datavec.image.loader.NativeImageLoader;
import org.deeplearning4j.nn.graph.ComputationGraph;
import org.deeplearning4j.nn.modelimport.keras.KerasModelImport;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.nd4j.linalg.api.ndarray.INDArray;
import java.io.File;

public class deepLearningTest2 {

    public static void main(String[] args) {
        String modelFile = "TeamCode\\src\\main\\assets\\testModel2\\preservedInception.json";
        String inputFile = "TeamCode\\src\\main\\assets\\testModel1\\fashion-mnist-sprite.png";
        String weights = "TeamCode\\src\\main\\assets\\testModel2\\preservedInception.weights.best.hdf5";

        ComputationGraph model;
        INDArray input;
        INDArray output;

        try {

            //import keras model
            model = KerasModelImport.importKerasModelAndWeights(modelFile, weights);
            model.init();

            //load input image & run inference
            input = new NativeImageLoader().asMatrix(new File(inputFile));
            output = model.outputSingle(input);

            //output results
            System.out.println("Results: " + output.toString());
        }
        catch (Exception e) { System.out.print("Exception: " + e.toString() + " try again."); }
    }
}
