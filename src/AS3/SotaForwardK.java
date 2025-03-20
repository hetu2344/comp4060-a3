package AS3;

import java.util.HashMap;
import java.util.Map;
import org.apache.commons.math3.linear.*;
import AS3.Frames.FrameKeys;

public class SotaForwardK {

    public final Map<FrameKeys, RealMatrix> frames = new HashMap<>();

    public RealVector endEffectorState = null; // a single vector representing the combined state of the end effector. needs to be in the same order as in the IK

    public SotaForwardK(double[] angles) { this(MatrixUtils.createRealVector(angles)); }
    public SotaForwardK(RealVector angles) {
        // TODO
        // constructs all the frame matrices and stores them in a Map that maps
        // a frame type (FrameKey) to the frame matrix.
        // ---------------- TO DO ----------------
        //======== setup Transformation matrices
        // Base to origin
        RealMatrix _base_to_origin = MatrixUtils.createRealIdentityMatrix(4); // 1
        RealMatrix _body_to_base = MatrixHelp.T(MatrixHelp.rotZ(angles.getEntry(0)), 0.0, 0.0, 0.005); // 2
        
        // Left hand
        RealMatrix _l_shoulder_to_body = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(1)), 0.039, 0.0, 0.1415); // 3
        // use rodriques formula to rotate the elbow to the shoulder
        RealMatrix _l_elbow_to_l_shoulder = MatrixHelp.T(MatrixHelp.rotRodrigues(0.6258053, 0.329192519, 0.707106769, angles.getEntry(2)), 0.0225, -0.03897, 0.0); // 4
        RealMatrix l_hand_l_elbow = MatrixHelp.T(MatrixHelp.rotRodrigues(0.6258053, 0.329192519, 0.707106769, 0), 0.0225, -0.03897, 0.0); // 18

        // Right hand
        RealMatrix _r_shoulder_to_body = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(3)), -0.039, 0.0, 0.1415); // 5
        // use rodriques formula to rotate the elbow to the shoulder
        RealMatrix _r_elbow_to_r_shoulder = MatrixHelp.T(MatrixHelp.rotRodrigues(-0.6258053, 0.329192519, 0.707106769, angles.getEntry(4)), -0.0225, -0.03897, 0.0); // 6
        RealMatrix r_hand_r_elbow = MatrixHelp.T(MatrixHelp.rotRodrigues(-0.6258053, 0.329192519, 0.707106769, 0), -0.0225, -0.03897, 0.0); // 19

        // Head Y -> R -> P, but swap order because of CSotaMotion's indexing
        RealMatrix _head_Y_to_body = MatrixHelp.T(MatrixHelp.rotZ(angles.getEntry(5)), 0.0, 0.0, 0.190); // 7
        RealMatrix _head_P_to_head_R = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(6)), 0.0, 0.0, 0.0); // 8
        RealMatrix _head_R_to_head_Y = MatrixHelp.T(MatrixHelp.rotY(angles.getEntry(7)), 0.0, 0.0, 0.0); // 9

        //========== Precalculate combined chains
        RealMatrix _body_to_origin = _base_to_origin.multiply(_body_to_base); // 10
        // Left hand
        RealMatrix _l_shoulder_to_origin = _body_to_origin.multiply(_l_shoulder_to_body); // 11
        RealMatrix _l_elbow_to_origin = _l_shoulder_to_origin.multiply(_l_elbow_to_l_shoulder); //  12 
        RealMatrix _l_hand_to_origin = _l_elbow_to_origin.multiply(l_hand_l_elbow); // 20
        // Right hand
        RealMatrix _r_shoulder_to_origin = _body_to_origin.multiply(_r_shoulder_to_body); // 13
        RealMatrix _r_elbow_to_origin = _r_shoulder_to_origin.multiply(_r_elbow_to_r_shoulder); // 14
        RealMatrix _r_hand_to_origin = _r_elbow_to_origin.multiply(r_hand_r_elbow); // 21

        // HeadY -> HeadP -> HeadR
        RealMatrix _head_Y_to_origin = _body_to_origin.multiply(_head_Y_to_body); // 15
        RealMatrix _head_R_to_origin = _head_Y_to_origin.multiply(_head_R_to_head_Y); // 16
        RealMatrix _head_P_to_origin = _head_R_to_origin.multiply(_head_P_to_head_R); // 17

        // Stores them in a Map that maps a frame type (FrameKey) to the frame matrix.
        frames.put(FrameKeys.L_HAND, _l_hand_to_origin); // left elbow -> shoulder -> body -> base -> origin 
        frames.put(FrameKeys.R_HAND, _r_hand_to_origin); // right elbow -> shoulder -> body -> base -> origin
        frames.put(FrameKeys.HEAD, _head_P_to_origin); // head pitch -> head roll -> head yaw -> body -> base -> origin * HEAD is correct
    }
}