package org.firstinspires.ftc.teamcode.kalmanFilter;

// Massive thanks to Dr Aleksandar Haber's youtube channel and his explanation of the Kalman Filter - those checking this code should subscribe if they want to understand more on machine learning.
// This is not part of any copyright it is simply a notice saying you would be fools not to check it out as they are undergraduate degree videos.
// Highly recommended to Steve, Saku and Daniel.
// Youtube channel link is here https://youtube.co.uk/@aleksandarhaber
// His youtube video has inspired this code

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.api.ops.impl.transforms.same.Round;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.inverse.InvertMatrix;

import java.lang.reflect.Array;

public class KalmanFilter {
    // define the system matrices
    // system matrices and covariances
    private INDArray A;
    private INDArray B;
    private INDArray C;


    private INDArray R;
    private INDArray Q;

    private INDArray x0;
    private INDArray P0;


    private int currentTimeStep;

    private INDArray estimates_aposteriori;


    private INDArray estimates_apriori;

    private INDArray estimationErrorCovarianceMatricesAposteriori;

    private INDArray estimationErrorCovarianceMatricesApriori;

    private INDArray gainMatrices;

    private INDArray errors;


    public KalmanFilter(INDArray x0, INDArray P0, INDArray A, INDArray B, INDArray C, INDArray Q, INDArray R) {
        this.A = A;
        this.x0 = x0;
        this.B = B;
        this.C = C;
        this.P0 = P0;
        this.Q = Q;
        this.R = R;
        this.currentTimeStep = 0;
        this.estimates_aposteriori.add(x0);
        this.estimationErrorCovarianceMatricesAposteriori.add(P0);
    }

    public void propagateDynamics(INDArray inputValue) {
        INDArray xk_minus = this.A.mul(this.estimates_aposteriori.getRow(this.currentTimeStep)).add(this.B.mul(inputValue));
        INDArray pk_minus = this.A.mul(this.estimationErrorCovarianceMatricesAposteriori.getColumn(this.currentTimeStep)).mul(this.A.transpose()).add(this.Q);
        this.estimates_apriori.add(xk_minus);
        this.estimationErrorCovarianceMatricesApriori.add(pk_minus);
        this.currentTimeStep = currentTimeStep + 1;
    }

    public INDArray getEstimates_aposteriori() {
        return estimates_aposteriori;
    }

    public void computeAposterioriEstimate(INDArray currentMeasurements) {
        INDArray G = this.R.add(this.C).mul(this.estimationErrorCovarianceMatricesApriori.getRow(this.currentTimeStep-1)).mul(this.C.transpose());
        INDArray kk = this.estimationErrorCovarianceMatricesApriori.getRow(this.currentTimeStep - 1).mul(this.C.transpose()).mul(InvertMatrix.invert(G, true));

        INDArray error_k = currentMeasurements.sub((this.C.mul(this.estimates_apriori.getRow(this.currentTimeStep - 1))));
        INDArray xk_plus = this.estimates_apriori.getRow(this.currentTimeStep - 1).add(kk.mul(error_k));

        INDArray IMinusKkC = Nd4j.eye(this.x0.shape()[0]).sub(kk.mul(error_k));
        INDArray Pk_plus = IMinusKkC.mul(this.estimationErrorCovarianceMatricesApriori.getRow(this.currentTimeStep - 1)).mul(IMinusKkC.transpose()).add(kk.mul(this.R).mul(kk.transpose()));

        this.gainMatrices.add(kk);
        this.errors.add(error_k);
        this.estimates_aposteriori.add(xk_plus);
        this.estimationErrorCovarianceMatricesAposteriori.add(Pk_plus);
    }
}
