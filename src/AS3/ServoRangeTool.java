package AS3;

import java.io.*;
import java.util.Map;
import java.nio.file.Paths;
import java.nio.file.Files;
import java.util.Map;
import java.util.HashMap;

import org.apache.commons.math3.linear.RealVector;

import jp.vstone.RobotLib.CRobotPose;
import jp.vstone.RobotLib.CSotaMotion;

public class ServoRangeTool implements Serializable {
    private static final long serialVersionUID = 1L;
    private final int NUM_MOTORS = 8;
    private Byte[] servoIDs = null;
    
    private Short[] _minpos = null;  // internal arrays for precalcualted values
    private Short[] _maxpos = null;
    private Short[] _midpos = null;
    private Map<Byte, Integer> IDtoIndex = null;

    final static String FILENAME = "../resources/robot_pose.txt";

    ServoRangeTool(Byte[] servoIDs) {
        //todo
        this.servoIDs = servoIDs;
        this._minpos = new Short[NUM_MOTORS];
        this._maxpos = new Short[NUM_MOTORS];
        this._midpos = new Short[NUM_MOTORS];
        this.IDtoIndex = new HashMap<Byte, Integer>();
        IDtoIndex.put(this.servoIDs[0], 0);
        IDtoIndex.put(this.servoIDs[1], 1);
        IDtoIndex.put(this.servoIDs[2], 2);
        IDtoIndex.put(this.servoIDs[3], 3);
        IDtoIndex.put(this.servoIDs[4], 4);
        IDtoIndex.put(this.servoIDs[5], 5);
        IDtoIndex.put(this.servoIDs[6], 6);
        IDtoIndex.put(this.servoIDs[7], 7);
    }


    public void register(CRobotPose pose) {
        // register(pose.getServoAngles(_servoIDs));
     }
     public void register(Short[] pos) {
         //todo

         // add in min array
         for(int i = 0; i < NUM_MOTORS; i++) {
             if(this._minpos[i] == null || this._minpos[i] > pos[i]){
                 this._minpos[i] = pos[i];
             }
         }

         // add in max array
         for(int i = 0; i < NUM_MOTORS; i++) {
             if(this._maxpos[i] == null || this._maxpos[i] < pos[i]){
                 this._maxpos[i] = pos[i];
             }
         }

         // update the mid array
         for(int i = 0; i < NUM_MOTORS; i++) {
//             System.out.println("Index " + i + ": min = " + _minpos[i] + ", max = " + _maxpos[i]);
             _midpos[i] = (short) ((_minpos[i] + _maxpos[i]) / 2);
//             System.out.println("Index " + i + ": mid = " + _midpos[i]);
         }
     }

    
    ///==================== Export as CRobotPose objects
    ///====================
    private CRobotPose makePose(Short[] pos) {  // convert short[] to CRobotPose object
        CRobotPose returnPose = new CRobotPose();
        returnPose.SetPose(servoIDs, pos);
        return returnPose;
    }

    public CRobotPose getMinPose() { return makePose(_minpos);}
    public CRobotPose getMaxPose() { return makePose(_maxpos);}
    public CRobotPose getMidPose() { return makePose(_midpos);}

    ///==================== Angle <-> motor pos conversions
    ///====================
    public RealVector calcAngles(CRobotPose pose) { // convert pose in motor positions to radians
        return null; // TODO
    }

    public CRobotPose calcMotorValues(RealVector angles) { // convert pose in angles to motor positions
        return null; // TODO
    }

    private double posToRad(Byte servoID, Short pos) { // convert motor position to angle, in radians 
        return 0; // TODO
    }

    private short radToPos(Byte servoID, double angle) { // convert angles, in radians, to motor position
        return 0; // TODO
    }
    
	///==================== Pretty Print
    /// ///====================
	private String formattedLine(String title, Byte servoID, Short[] minpos, Short[] maxpos, Short[] middle, Short[] pos) {
//		int i = 0;
        int i = IDtoIndex.get(servoID);
        double rad = 0;
        if (pos != null) rad = posToRad(servoID, pos[i]);
		String format = "%14s %8d %8d %8d    %.2f rad";
		return String.format(format, title, minpos[i], middle[i], maxpos[i], rad);
	}

    public void printMotorRanges() {printMotorRanges(null);}
	public void printMotorRanges(Short[] pos) {  // will print the current position as given by the pos array
		System.out.println("-------------");
		System.out.println( formattedLine("Body Y: ", CSotaMotion.SV_BODY_Y, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("L Shoulder: ", CSotaMotion.SV_L_SHOULDER, _minpos, _maxpos, _midpos, pos));
        System.out.println( formattedLine("L Elbow: ", CSotaMotion.SV_L_ELBOW, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("R Shoulder: ", CSotaMotion.SV_R_SHOULDER, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("R Elbow: ", CSotaMotion.SV_R_ELBOW, _minpos, _maxpos, _midpos, pos));
        System.out.println( formattedLine("Head Y: ", CSotaMotion.SV_HEAD_Y, _minpos, _maxpos, _midpos, pos));
		System.out.println( formattedLine("Head P: ", CSotaMotion.SV_HEAD_P, _minpos, _maxpos, _midpos, pos));
        System.out.println( formattedLine("Head R: ", CSotaMotion.SV_HEAD_R, _minpos, _maxpos, _midpos, pos));
	}

    ///==================== LOAD AND SAVE
    ///====================
    public static ServoRangeTool Load(){ return ServoRangeTool.Load(FILENAME);}
    public static ServoRangeTool Load(String filename){
        try(ObjectInputStream ois = new ObjectInputStream(Files.newInputStream(Paths.get(filename)))) {
            return (ServoRangeTool) ois.readObject();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void save() { save(FILENAME);}
    public void save(String filename) {
        //todo
        try(ObjectOutputStream oos = new ObjectOutputStream(Files.newOutputStream(Paths.get(filename)))) {
            oos.writeObject(this);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}