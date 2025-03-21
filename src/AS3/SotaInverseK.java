package AS3;

import java.util.TreeMap;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import AS3.Frames.FrameKeys;

public class SotaInverseK {

    private static double NUMERICAL_DELTA_rad = 1e-10;
    private static double DISTANCE_THRESH = 1e-3; // 1mm
    private static int MAX_TRIES = 10; // loop 15 times

    public enum JType { // We separate the jacobians into origin and rotation components to simplify the
                        // problem
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
        for (int i = 0; i < JType.values().length; i++) { // initialize each entry as a TreeMap
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
        System.out.println("frameAngles: "+frameAngles);
        // System.out.println();

        int numMotors = frameAngles.getDimension(); // number of motors (3 for left & right hand, 4 for head)

        // Create origin and rotation matrices - 3x3 or 3x4
        RealMatrix jacobianO = MatrixUtils.createRealMatrix(JType.OUT_DIM, numMotors); 
        RealMatrix jacobianR = MatrixUtils.createRealMatrix(JType.OUT_DIM, numMotors); 

        for (int i = 0; i < numMotors; i++) {
            // Solve FK for current angles -> f(theta)
            SotaForwardK currentFK = new SotaForwardK(currentAngles); // solve FK for the whole robot
            RealVector currentPose = MatrixHelp.getTrans(currentFK.frames.get(frameType)).getSubVector(0, 3); // but get the current pose of the required frame

            // Solve FK for pertubed angles -> f(theta + delta_theta)
            RealVector pertubedAngles = currentAngles.copy(); // copy motor angles of that frame, reset every iteration
            pertubedAngles.setEntry(frameIndices[i], pertubedAngles.getEntry(frameIndices[i]) + NUMERICAL_DELTA_rad); // theta + delta_theta of the correct joint
            
            SotaForwardK perturbedFK = new SotaForwardK(pertubedAngles);
            RealVector perturbedPose = MatrixHelp.getTrans(perturbedFK.frames.get(frameType)).getSubVector(0, 3); // get the pertubed pose of the required frame

            // derivate = (current - delta) / SMALL_DETA
            RealVector derivativesO = (perturbedPose.subtract(currentPose)).mapDivide(NUMERICAL_DELTA_rad);
            //System.out.println("derivativesO:" + derivativesO);
            jacobianO.setColumnVector(i, derivativesO); // set the column of the jacobian

            // Build rotation Jacobian
            RealVector currentRotation = MatrixUtils.createRealVector(MatrixHelp.getYPR(currentFK.frames.get(frameType)));
            RealVector perturbedRotation = MatrixUtils.createRealVector(MatrixHelp.getYPR(perturbedFK.frames.get(frameType)));

            RealVector derivativesR = (perturbedRotation.subtract(currentRotation)).mapDivide(NUMERICAL_DELTA_rad);
            //System.out.println("derivativesR:" + derivativesR);
            //System.out.println();
            jacobianR.setColumnVector(i, derivativesR);
        }
        J[JType.O.ordinal()].put(frameType, jacobianO); // origin jacobian
        J[JType.R.ordinal()].put(frameType, jacobianR); // rotation jacobian

        // Compute inverse Jacobian using pseudo-inverse
        Jinv[JType.O.ordinal()].put(frameType, MatrixHelp.pseudoInverse(jacobianO)); // origin jacobian inverse
        Jinv[JType.R.ordinal()].put(frameType, MatrixHelp.pseudoInverse(jacobianR)); // rotation jacobian inverse
    }

    // calculates the target absolute pose from the current pose, plus the given delta using FK before calling solve.
    static public RealVector solveDelta(FrameKeys frameType, JType jtype, RealVector deltaEndPose, RealVector curMotorAngles) {
        // TODO if needed
        return solve(frameType, jtype, null, curMotorAngles);
    }

    // solves for the target pose on the given frame and type, starting at the current angle configuration.
    static public RealVector solve(FrameKeys frameType, JType jtype, RealVector targetPose, RealVector curMotorAngles) {
        System.out.println("Solving for " + frameType);
        RealVector solution = curMotorAngles.copy();
        MatrixHelp.printVector("solution ", solution);
        // Get frame's corresponding motor angles (L_HAND, R_HAND, or HEAD)
        int[] frameIndices = frameType.motorindices; // get index of motor joints in the frame

        // Solve FK for current position
        SotaForwardK FK = new SotaForwardK(curMotorAngles);
        RealVector currentPose = MatrixHelp.getTrans(FK.frames.get(frameType)).getSubVector(0, 3);
        RealVector error = targetPose.subtract(currentPose); // x_d - FK(theta_i)

        double bestError = error.copy().getNorm(); // storing smallest error, starting from the current error
        MatrixHelp.printVector("Prev error ", error);
        System.out.println("----- Prev error norm: "+error.getNorm());

        int tries = 0;
        while (bestError > DISTANCE_THRESH && tries < MAX_TRIES) {   
            MatrixHelp.printVector("Prev curMotorAngles", curMotorAngles);                 
            SotaInverseK IK = new SotaInverseK(curMotorAngles, frameType); // Make jacobian matrix
            RealMatrix jacobianInverse = IK.Jinv[jtype.ordinal()].get(frameType); // Compute pseudo-inverse of Jacobian matrix
            
            if (jacobianInverse == null) {
                throw new RuntimeException("Jacobian inverse is not computed!");
            }
        
            RealVector deltaTheta = jacobianInverse.operate(error); // delta_theta = J+ * error
            System.out.println("----- deltaTheta: "+deltaTheta);
            System.out.println();
            // theta_{i+1} = theta + delta_theta - update new motor angle for new FK solver to get the new error
            for (int i = 0; i < deltaTheta.getDimension(); i++) {
                curMotorAngles.setEntry(frameIndices[i], curMotorAngles.getEntry(frameIndices[i]) + deltaTheta.getEntry(i));
            }
            MatrixHelp.printVector("New curMotorAngles", curMotorAngles);

            FK = new SotaForwardK(curMotorAngles); // re-calculate FK
            // Update error
            RealVector newPose = MatrixHelp.getTrans(FK.frames.get(frameType)).getSubVector(0, 3);
            error = targetPose.subtract(newPose); // update error
            MatrixHelp.printVector("New error ", error);
            System.out.println("----- New error norm: "+error.getNorm());

            // Save smallest error and theta
            if (error.getNorm() < bestError) {
                bestError = error.getNorm(); // replace smaller error
                System.out.println("----- Best error norm: "+bestError);
                solution = curMotorAngles.copy(); // update solution to the new set of joint angles with smaller error
                MatrixHelp.printVector("Solution updated", solution);
            }

            tries++; // next iteration
            System.out.println("=================================");
        }
        MatrixHelp.printVector("Final solution", solution);
        System.out.println("Best error: " + bestError);
        return solution;
    }
}