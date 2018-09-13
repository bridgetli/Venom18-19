package org.firstinspires.ftc.teamcode;

import org.deeplearning4j.nn.modelimport.keras.KerasModelImport;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.io.ClassPathResource;

public class deepLearningTest {

    public void main(String[] args) {
        MultiLayerNetwork model = null;
        while (model == null) {
            try {
                String simpleMlp = new ClassPathResource("model.h5").getFile().getPath();
                model = KerasModelImport.importKerasSequentialModelAndWeights(simpleMlp);
            } catch (Exception e) {
                System.out.print("Exception: " + e.toString() + " try again.");
            }
        }

        INDArray input = Nd4j.create();
        INDArray output = model.output(input);

    }
}
