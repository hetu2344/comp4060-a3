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
        System.out.println();

        int numMotors = frameAngles.getDimension(); // number of motors (3 for left & right hand, 4 for head)

        // Create origin and rotation matrices - 3x3 or 3x4
        RealMatrix jacobianO = MatrixUtils.createRealMatrix(JType.OUT_DIM, numMotors); 
        RealMatrix jacobianR = MatrixUtils.createRealMatrix(JType.OUT_DIM, numMotors); 

        for (int i = 0; i < numMotors; i++) {
            System.out.println("=========ITERATION "+ (i+1));
            RealVector pertubedAngles = currentAngles.copy(); // copy motor angles of that frame, reset every iteration
            pertubedAngles.setEntry(frameIndices[i], pertubedAngles.getEntry(frameIndices[i]) + NUMERICAL_DELTA_rad); // theta + delta_theta of the correct joint

            // Solve FK for current angles -> f(theta)
            SotaForwardK currentFK = new SotaForwardK(currentAngles); // solve FK for the whole robot
            double[] currentPoseHomogenous = MatrixHelp.getTrans(currentFK.frames.get(frameType)).toArray(); // but get the current pose of the required frame
            double[] currentPose = new double[currentPoseHomogenous.length-1]; // remove last element 1
            // for (int j = 0; j < currentPose.length; j++) {
            //     currentPose[j] = currentPoseHomogenous[j];
            //     System.out.println("currentPose["+j+"]: " + currentPose[j]);
            // }
            // System.out.println();
            // Solve FK for pertubed angles -> f(theta + delta_theta)
            SotaForwardK perturbedFK = new SotaForwardK(pertubedAngles);
            double[] perturbedPoseHomogenous = MatrixHelp.getTrans(perturbedFK.frames.get(frameType)).toArray(); // get the pertubed pose of the required frame
            double[] perturbedPose = new double[perturbedPoseHomogenous.length-1]; // remove last element 1
            // for (int j = 0; j < perturbedPose.length; j++) {
            //     perturbedPose[j] = perturbedPoseHomogenous[j];
            //     System.out.println("perturbedPose["+j+"]: " + perturbedPose[j]);
            // }
            // System.out.println();

            // derivate = (current - delta) / SMALL_DETA
            RealVector derivativesO = (MatrixUtils.createRealVector(currentPose).subtract(MatrixUtils.createRealVector(perturbedPose)).mapDivide(NUMERICAL_DELTA_rad));
            // System.out.println("derivativesO:" + derivativesO);
            /// System.out.println();
            jacobianO.setColumnVector(i, derivativesO); // set the column of the jacobian

            // Build rotation Jacobian
            double[] currentRotation = MatrixHelp.getYPR(currentFK.frames.get(frameType));
            double[] perturbedRotation = MatrixHelp.getYPR(perturbedFK.frames.get(frameType));

            RealVector derivativesR = (MatrixUtils.createRealVector(currentRotation).subtract(MatrixUtils.createRealVector(perturbedRotation)).mapDivide(NUMERICAL_DELTA_rad));
            // System.out.println("derivativesR:" + derivativesR);
            // System.out.println();
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
        RealVector solution = curMotorAngles.copy();
        double bestError = Double.MAX_VALUE;
        // Solve FK for current position
        SotaForwardK FK = new SotaForwardK(curMotorAngles);
        RealVector translation = FK.frames.get(frameType).getColumnVector(3).getSubVector(0, 3);
        // System.out.println("homogenousTrans: "+ homogenousTrans);
        // System.out.println("Printing targetPose");
        // for (int i = 0; i < targetPose.getDimension(); i++) {
        //     System.out.println("targetPose["+i+"]" + targetPose.getEntry(i)+" ");        
        // }
        // System.out.println();
        // System.out.println("translation: "+translation);
        // System.out.println();
        RealVector error = targetPose.subtract(translation); // x_d - FK(theta_i)
        // System.out.println("Printing errors");
        // for (int i = 0; i < error.getDimension(); i++) {
        //     System.out.println("error["+i+"]" + error.getEntry(i)+" ");        
        // }
        // System.out.println();

        // Get frame's corresponding motor angles (L_HAND, R_HAND, or HEAD)
        int[] frameIndices = frameType.motorindices; // get index of motor joints in the frame

        int tries = 0;
        while (error.getNorm() > DISTANCE_THRESH && tries < MAX_TRIES) {    
            System.out.println("Making Jacobian");        
            SotaInverseK IK = new SotaInverseK(curMotorAngles, frameType); // Make jacobian matrix
            System.out.println("Printing Jacobian");        
            RealMatrix jacobian = IK.J[jtype.ordinal()].get(frameType); // Compute pseudo-inverse of Jacobian matrix
            for (int i = 0; i < jacobian.getRowDimension(); i ++) {
                for (int j = 0; j < jacobian.getColumnDimension(); j++) {
                    System.out.printf("%10.6f ", jacobian.getEntry(i, j));
                }
                System.out.println();
            }
            System.out.println();
            System.out.println("Printing Jacobian inverse");        
            RealMatrix jacobianInverse = IK.Jinv[jtype.ordinal()].get(frameType); // Compute pseudo-inverse of Jacobian matrix
            for (int i = 0; i < jacobianInverse.getRowDimension(); i ++) {
                for (int j = 0; j < jacobianInverse.getColumnDimension(); j++) {
                    System.out.printf("%10.6f ", jacobianInverse.getEntry(i, j));
                }
                System.out.println();
            }
            System.out.println();
            
            // if (jacobianInverse == null) {
            //     throw new RuntimeException("Jacobian inverse is not computed!");
            // }
        
            RealVector deltaTheta = jacobianInverse.operate(error); // delta_theta = J+ * error
            System.out.println("deltaTheta: "+deltaTheta);
            System.out.println();
            // curMotorAngles = curMotorAngles.add(deltaTheta); 
            // theta_{i+1} = theta + delta_theta
            for (int i = 0; i < deltaTheta.getDimension(); i++) {
                curMotorAngles.setEntry(frameIndices[i], curMotorAngles.getEntry(frameIndices[i])+deltaTheta.getEntry(i));
            }
            FK = new SotaForwardK(curMotorAngles); // re-calculate FK
            // Update error
            error = targetPose.subtract(FK.frames.get(frameType).getColumnVector(3).getSubVector(0, 3)); // update error
            MatrixHelp.printVector("Printing error ", error);
            // Save smallest error and theta
            if (error.getNorm() < bestError) {
                bestError = error.getNorm(); // replace smaller error
                solution = curMotorAngles.copy(); // update theta
                MatrixHelp.printVector("Best solution ", solution);
            }
            tries++; // next iteration
            System.out.println("=================================");
        }
        return solution;
    }
}