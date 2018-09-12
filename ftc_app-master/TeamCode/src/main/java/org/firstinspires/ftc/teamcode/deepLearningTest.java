package org.firstinspires.ftc.teamcode;

import org.deeplearning4j.nn.modelimport.keras.KerasModelImport;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.nd4j.linalg.io.ClassPathResource;

public class deepLearningTest {

    public void main(String[] args) {
        try {
            String simpleMlp = new ClassPathResource("model.h5").getFile().getPath();
            MultiLayerNetwork model = KerasModelImport.importKerasSequentialModelAndWeights(simpleMlp);
        } catch(Exception e) {
            System.out.print("Exception: " + e.toString());
        }

    }
}
