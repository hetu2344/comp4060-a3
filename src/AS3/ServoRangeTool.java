package AS3;

import java.io.*;
import java.util.Map;
import java.nio.file.Paths;
import java.nio.file.Files;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;

import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CSotaMotion;

public class ServoRangeTool implements Serializable {
    private static final long serialVersionUID = 1L;
    private final int NUM_MOTORS = 8; // 8 motors in the Sota
    private Byte[] servoIDs = null; // array of servo IDs

    private Short[] _minpos = null; // internal arrays for precalcualted values
    private Short[] _maxpos = null;
    private Short[] _midpos = null;
    private Map<Byte, Integer> IDtoIndex = null; 
    private Map<Byte, Double[]> _motorRanges_rad = null; 

    final static String FILENAME = "../resources/robot_pose.txt"; // file destination

    ServoRangeTool(Byte[] servoIDs) {
        // todo
        this.servoIDs = servoIDs;
        this._minpos = new Short[NUM_MOTORS]; // array of minpos for 8 motors
        this._maxpos = new Short[NUM_MOTORS]; // array of maxpos for 8 motors
        this._midpos = new Short[NUM_MOTORS]; // array of midpos for 8 motors
        this.IDtoIndex = new HashMap<Byte, Integer>(); // map of servoID to index in the arrays
        IDtoIndex.put(this.servoIDs[0], 0);
        IDtoIndex.put(this.servoIDs[1], 1);
        IDtoIndex.put(this.servoIDs[2], 2);
        IDtoIndex.put(this.servoIDs[3], 3);
        IDtoIndex.put(this.servoIDs[4], 4);
        IDtoIndex.put(this.servoIDs[5], 5);
        IDtoIndex.put(this.servoIDs[6], 6);
        IDtoIndex.put(this.servoIDs[7], 7);
        this._motorRanges_rad = new HashMap<Byte, Double[]>(); // map of servoID to min, max pos in radians
        _motorRanges_rad.put(CSotaMotion.SV_BODY_Y, new Double[] { -1.077363736, 1.077363736 });
        _motorRanges_rad.put(CSotaMotion.SV_L_SHOULDER, new Double[] { -2.617993878, 1.745329252 });
        _motorRanges_rad.put(CSotaMotion.SV_L_ELBOW, new Double[] { -1.745329252, 1.221730476 });
        _motorRanges_rad.put(CSotaMotion.SV_R_SHOULDER, new Double[] { -1.745329252, 2.617993878 });
        _motorRanges_rad.put(CSotaMotion.SV_R_ELBOW, new Double[] { -1.221730476, 1.745329252 });
        _motorRanges_rad.put(CSotaMotion.SV_HEAD_Y, new Double[] { -1.495996502, 1.495996502 });
        _motorRanges_rad.put(CSotaMotion.SV_HEAD_P, new Double[] { -2.617993878, 2.617993878 });
        _motorRanges_rad.put(CSotaMotion.SV_HEAD_R, new Double[] { -1.495996502, 1.495996502 });
    }

    public void register(CRobotPose pose) {
        // register(pose.getServoAngles(_servoIDs));
    }

    public void register(Short[] pos) {
        // todo

        // add in min array
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (this._minpos[i] == null || this._minpos[i] > pos[i]) {
                this._minpos[i] = pos[i];
            }
        }

        // add in max array
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (this._maxpos[i] == null || this._maxpos[i] < pos[i]) {
                this._maxpos[i] = pos[i];
            }
        }

        // update the mid array
        for (int i = 0; i < NUM_MOTORS; i++) {
            // System.out.println("Index " + i + ": min = " + _minpos[i] + ", max = " +
            // _maxpos[i]);
            _midpos[i] = (short) ((_minpos[i] + _maxpos[i]) / 2);
            // System.out.println("Index " + i + ": mid = " + _midpos[i]);
        }
    }

    /// ==================== Export as CRobotPose objects
    /// ====================
    private CRobotPose makePose(Short[] pos) { // convert short[] to CRobotPose object
        CRobotPose returnPose = new CRobotPose();
        returnPose.SetPose(servoIDs, pos);
        return returnPose;
    }

    public CRobotPose getMinPose() {
        return makePose(_minpos);
    }

    public CRobotPose getMaxPose() {
        return makePose(_maxpos);
    }

    public CRobotPose getMidPose() {
        return makePose(_midpos);
    }

    /// ==================== Angle <-> motor pos conversions
    /// ====================
    public RealVector calcAngles(CRobotPose pose) { // convert pose in motor positions to radians
        RealVector angles = MatrixUtils.createRealVector(new double[NUM_MOTORS]);
        for (int i = 0; i < angles.getDimension(); i++) {
            // Add entries of motors to the angles vector
            // Convert the motor position to radians
            // Note: use servoIDs[i] to get the servoID
            angles.setEntry(i, posToRad(servoIDs[i], pose.getServoAngle(servoIDs[i])));
        }
        return angles;
    }

    public CRobotPose calcMotorValues(RealVector angles) { // convert pose in angles to motor positions
        CRobotPose pose = new CRobotPose();
        Map<Byte, Short> pos = new HashMap<Byte, Short>();
        for (int i = 0; i < angles.getDimension(); i++) {
            // Add entries of motors to the pose object
            // Convert the angle to motor position
            // Note: use servoIDs[i] to get the servoID
            pos.put(servoIDs[i], radToPos(servoIDs[i], angles.getEntry(i)));
        }
        pose.SetPose(pos);
        return pose; // TODO
    }

    private double posToRad(Byte servoID, Short pos) { // convert motor position to angle, in radians
        if (! _motorRanges_rad.containsKey(servoID)) {
            throw new IllegalArgumentException("Servo ID not found in motor ranges");
        }
        Double[] radianLimits = _motorRanges_rad.get(servoID); // get min & max radian limits of selected servoID
        Short[] posLimits = new Short[] { _minpos[IDtoIndex.get(servoID)], _maxpos[IDtoIndex.get(servoID)] }; // get min & max motor limits of selected servoID
        double rad_min = radianLimits[0];
        double rad_max = radianLimits[1];
        double pos_min = posLimits[0];
        double pos_max = posLimits[1];
        double rad = rad_min + (pos - pos_min) / (pos_max - pos_min) * (rad_max - rad_min);
        return rad; // TODO
    }

    private short radToPos(Byte servoID, double angle) { // convert angles, in radians, to motor position
        if (!_motorRanges_rad.containsKey(servoID)) {
            throw new IllegalArgumentException("Servo ID not found in motor ranges");
        }
        Double[] radianLimits = _motorRanges_rad.get(servoID); // get min & max radian limits of selected servoID
        Short[] posLimits = new Short[] { _minpos[IDtoIndex.get(servoID)], _maxpos[IDtoIndex.get(servoID)] }; // get min & max motor limits of selected servoID
        double rad_min = radianLimits[0];
        double rad_max = radianLimits[1];
        double pos_min = posLimits[0];
        double pos_max = posLimits[1];
        short pos = (short) (pos_min + (angle - rad_min) / (rad_max - rad_min) * (pos_max - pos_min));
        return pos; // TODO
    }

    /// ==================== Pretty Print
    /// ///====================
    private String formattedLine(String title, Byte servoID, Short[] minpos, Short[] maxpos, Short[] middle, Short[] pos) {
        // int i = 0;
        int i = IDtoIndex.get(servoID);
        double rad = 0;
        if (pos != null)
            System.out.println("Read pos["+i+"]: " + pos[i]);
            rad = posToRad(servoID, pos[i]);
        String format = "%14s %8d %8d %8d    %.2f rad";
        return String.format(format, title, minpos[i], middle[i], maxpos[i], rad);
    }

    public void printMotorRanges() {
        printMotorRanges(null);
    }

    public void printMotorRanges(Short[] pos) { // will print the current position as given by the pos array
        System.out.println("-------------");
        // Note: CSotaMotion.SV_BODY_Y returns Byte index = 0, CSotaMotion.SV_L_SHOULDER
        // returns Byte index = 1, etc.
        System.out.println(formattedLine("Body Y: ", CSotaMotion.SV_BODY_Y, _minpos, _maxpos, _midpos, pos));
        System.out.println(formattedLine("L Shoulder: ", CSotaMotion.SV_L_SHOULDER, _minpos, _maxpos, _midpos, pos));
        System.out.println(formattedLine("L Elbow: ", CSotaMotion.SV_L_ELBOW, _minpos, _maxpos, _midpos, pos));
        System.out.println(formattedLine("R Shoulder: ", CSotaMotion.SV_R_SHOULDER, _minpos, _maxpos, _midpos, pos));
        System.out.println(formattedLine("R Elbow: ", CSotaMotion.SV_R_ELBOW, _minpos, _maxpos, _midpos, pos));
        System.out.println(formattedLine("Head Y: ", CSotaMotion.SV_HEAD_Y, _minpos, _maxpos, _midpos, pos));
        System.out.println(formattedLine("Head P: ", CSotaMotion.SV_HEAD_P, _minpos, _maxpos, _midpos, pos));
        System.out.println(formattedLine("Head R: ", CSotaMotion.SV_HEAD_R, _minpos, _maxpos, _midpos, pos));
    }

    /// ==================== LOAD AND SAVE
    /// ====================
    public static ServoRangeTool Load() {
        return ServoRangeTool.Load(FILENAME);
    }

    public static ServoRangeTool Load(String filename) {
        try (ObjectInputStream ois = new ObjectInputStream(Files.newInputStream(Paths.get(filename)))) {
            return (ServoRangeTool) ois.readObject();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void save() {
        save(FILENAME);
    }

    public void save(String filename) {
        // todo
        try (ObjectOutputStream oos = new ObjectOutputStream(Files.newOutputStream(Paths.get(filename)))) {
            oos.writeObject(this);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}