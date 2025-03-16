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
        RealMatrix _base_to_origin = MatrixUtils.createRealIdentityMatrix(4); 
        RealMatrix _body_to_base = MatrixHelp.T(MatrixHelp.rotZ(angles.getEntry(0)), 0.0, 0.0, 0.005);
        
        // Left hand
        RealMatrix _l_shoulder_to_body = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(1)), 0.039, 0.0, 0.1415);
        RealMatrix _l_elbow_to_l_shoulder = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(2)), 0.0225, -0.3897, 0.0);

        // Right hand
        RealMatrix _r_shoulder_to_body = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(3)), -0.039, 0.0, 0.1415);
        RealMatrix _r_elbow_to_r_shoulder = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(4)), -0.0225, -0.03897, 0.0);

        // Head
        RealMatrix _head_Y_to_body = MatrixHelp.T(MatrixHelp.rotZ(angles.getEntry(5)), 0.0, 0.0, 0.190);
        RealMatrix _head_P_to_head_R = MatrixHelp.T(MatrixHelp.rotX(angles.getEntry(6)), 0.0, 0.0, 0.0);
        RealMatrix _head_R_to_head_Y = MatrixHelp.T(MatrixHelp.rotY(angles.getEntry(7)), 0.0, 0.0, 0.0);

        //========== Precalculate combined chains
        RealMatrix _body_to_origin = _base_to_origin.multiply(_body_to_base);
        // Left hand
        RealMatrix _l_shoulder_to_origin = _body_to_origin.multiply(_l_shoulder_to_body);
        RealMatrix _l_elbow_to_origin = _l_shoulder_to_origin.multiply(_l_elbow_to_l_shoulder); 
        // Right hand
        RealMatrix _r_shoulder_to_origin = _body_to_origin.multiply(_r_shoulder_to_body);
        RealMatrix _r_elbow_to_origin = _r_shoulder_to_origin.multiply(_r_elbow_to_r_shoulder);
        // HeadY -> HeadP -> HeadR
        RealMatrix _head_Y_to_origin = _body_to_origin.multiply(_head_Y_to_body);
        RealMatrix _head_R_to_origin = _head_Y_to_origin.multiply(_head_R_to_head_Y);
        RealMatrix _head_P_to_origin = _head_R_to_origin.multiply(_head_P_to_head_R);

        // Stores them in a Map that maps a frame type (FrameKey) to the frame matrix.
        frames.put(FrameKeys.L_HAND, _l_elbow_to_origin); // left elbow -> shoulder -> body -> base -> origin 
        frames.put(FrameKeys.R_HAND, _r_elbow_to_origin); // right elbow -> shoulder -> body -> base -> origin
        frames.put(FrameKeys.HEAD, _head_P_to_origin); // head pitch -> head roll -> head yaw -> body -> base -> origin
    }
}