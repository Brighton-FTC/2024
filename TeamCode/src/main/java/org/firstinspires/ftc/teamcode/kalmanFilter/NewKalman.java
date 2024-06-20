import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.inverse.InvertMatrix;

public class KalmanFilter {
    // System matrices and covariances
    private INDArray A;
    private INDArray B;
    private INDArray C;
    private INDArray R;
    private INDArray Q;
    private INDArray x;
    private INDArray P;

    public KalmanFilter(INDArray x0, INDArray P0, INDArray A, INDArray B, INDArray C, INDArray Q, INDArray R) {
        this.A = A;
        this.B = B;
        this.C = C;
        this.x = x0;
        this.P = P0;
        this.Q = Q;
        this.R = R;
    }

    // Propagation step
    public void propagateDynamics(INDArray u) {
        // xk- = A * xk-1 + B * u
        x = A.mmul(x).add(B.mmul(u));
        // Pk- = A * Pk-1 * A^T + Q
        P = A.mmul(P).mmul(A.transpose()).add(Q);
    }

    // Update step
    public void update(INDArray z) {
        // Compute Kalman gain
        INDArray S = C.mmul(P).mmul(C.transpose()).add(R);
        INDArray K = P.mmul(C.transpose()).mmul(InvertMatrix.invert(S, false));

        // Update estimate with measurement z
        INDArray y = z.sub(C.mmul(x)); // Innovation or measurement residual
        x = x.add(K.mmul(y));

        // Update error covariance
        INDArray I = Nd4j.eye(P.rows());
        P = (I.sub(K.mmul(C))).mmul(P);
    }

    // Get current state estimate
    public INDArray getState() {
        return x;
    }

    // Get current error covariance
    public INDArray getCovariance() {
        return P;
    }
}
