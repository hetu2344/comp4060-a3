package AS3;

import java.util.TreeMap;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import AS3.Frames.FrameKeys;

public class SotaInverseK {

    private static double NUMERICAL_DELTA_rad = 1e-10;
    private static double DISTANCE_THRESH = 1e-3; // 1mm

    public enum JType {  // We separate the jacobians into origin and rotation components to simplify the problem
        O, // origin
        R; // rotation / orientation
        
        public static final int OUT_DIM = 3; // each has 3 outputs
    }

    public TreeMap<FrameKeys, RealMatrix>[] J;
    public TreeMap<FrameKeys, RealMatrix>[] Jinv;

    @SuppressWarnings("unchecked")
    SotaInverseK(RealVector currentAngles, FrameKeys frameType) {
        J = new TreeMap[JType.values().length]; // length 2
        Jinv = new TreeMap[JType.values().length]; // length 2
        for (int i=0; i < JType.values().length; i++) { // initialize each entry as a TreeMap
            J[i] = new TreeMap<FrameKeys, RealMatrix>();
            Jinv[i] = new TreeMap<FrameKeys, RealMatrix>();
        }
       makeJacobian(currentAngles, frameType); // compute Jacobian matrix
    }

    // Makes both the jacobian and inverse from the current configuration for the
    // given frame type. Creates both JTypes.
    private void makeJacobian(RealVector currentAngles, FrameKeys frameType) {
        // TODO
        // Get frame's corresponding motor angles (L_HAND, R_HAND, or HEAD)
        int[] frameIndices = frameType.motorindices; // get index of motor joints in the frame
        RealVector frameAngles = MatrixUtils.createRealVector(new double[frameIndices.length]);
        for (int i = 0; i < frameAngles.getDimension(); i++) {
            frameAngles.setEntry(i, currentAngles.getEntry(frameIndices[i]));
        }
        int numMotors = frameAngles.getDimension(); // number of motors (3 for left & right hand, 4 for head)
        
        RealMatrix jacobian = MatrixUtils.createRealMatrix(JType.OUT_DIM, numMotors); // 3x3 or 3x4
    
        for (int i = 0; i < numMotors; i++) {
            RealVector pertubedAngles = currentAngles.copy(); // copy motor angles of that frame 
            pertubedAngles.setEntry(i, pertubedAngles.getEntry(i) + NUMERICAL_DELTA_rad); // theta + delta_theta
            // Solve FK for current angles -> f(theta)
            SotaForwardK currentFK = new SotaForwardK(currentAngles); // solve FK for the whole robot
            double[] currentPose = MatrixHelp.getTrans(currentFK.frames.get(frameType)).toArray(); // but get the current pose of the required frame
            // Solve FK for pertubed angles -> f(theta + delta_theta)
            SotaForwardK deltaFK = new SotaForwardK(pertubedAngles);
            double[] deltaPose = MatrixHelp.getTrans(deltaFK.frames.get(frameType)).toArray(); // get the pertubed pose of the required frame
            // derivate = (current - delta) / SMALL_DETA
            RealVector derivatives = MatrixUtils.createRealVector(currentPose).subtract(MatrixUtils.createRealVector(deltaPose).mapDivide(NUMERICAL_DELTA_rad)); 
            jacobian.setColumnVector(i, derivatives); // set the column of the jacobian
        }

        // origin jacobian
        J[JType.O.ordinal()].put(frameType, jacobian); // translate: JType[0] = <frameType, J>. Example: JType[0] = <L_HAND, J>

        // Compute inverse Jacobian using pseudo-inverse 
        RealMatrix jacobianInverse = MatrixHelp.pseudoInverse(jacobian);
        Jinv[JType.O.ordinal()].put(frameType, jacobianInverse); // origin jacobian inverse
    }
    
    // calculates the target absolute pose from the current pose, plus the given delta
    // using FK before calling solve.
    static public RealVector solveDelta(FrameKeys frameType, JType jtype, RealVector deltaEndPose, RealVector curMotorAngles) {
        //TODO if needed
        return solve(frameType, jtype, null, curMotorAngles);
    }

    // solves for the target pose on the given frame and type, starting at the current angle configuration.
    static public RealVector solve(FrameKeys frameType, JType jtype, RealVector targetPose, RealVector curMotorAngles) {
        SotaInverseK IK = new SotaInverseK(curMotorAngles, frameType);
        RealMatrix jacobianInverse = IK.Jinv[jtype.ordinal()].get(frameType);

        if (jacobianInverse == null) {
            throw new RuntimeException("Jacobian inverse is not computed!");
        }
        SotaForwardK FK = new SotaForwardK(curMotorAngles);
        RealVector error = targetPose.subtract(FK.frames.get(frameType).getColumnVector(3)); // target - current

        if (error.getNorm() < DISTANCE_THRESH) {
            return curMotorAngles;
        }
        RealVector deltaTheta = jacobianInverse.operate(error); // delta_theta = J+ * error
        RealVector solution = curMotorAngles.add(deltaTheta); // theta = theta + delta_theta
        return solution;
    }   
}