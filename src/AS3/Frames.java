package AS3;

import jp.vstone.RobotLib.CSotaMotion;

import java.util.TreeMap;

public class Frames {
    private static TreeMap<Byte, Integer> IDtoIndex = new TreeMap<Byte, Integer>() {
        {
            put(CSotaMotion.SV_BODY_Y, 0);
            put(CSotaMotion.SV_L_SHOULDER, 1);
            put(CSotaMotion.SV_L_ELBOW, 2);
            put(CSotaMotion.SV_R_SHOULDER, 3);
            put(CSotaMotion.SV_R_ELBOW, 4);
            put(CSotaMotion.SV_HEAD_Y, 5);
            put(CSotaMotion.SV_HEAD_R, 6);
            put(CSotaMotion.SV_HEAD_P, 7);
        }
    }; // Convert Byte i to Index i-1 for i >= 1

    public enum FrameKeys {
        L_HAND(IDtoIndex.get(CSotaMotion.SV_BODY_Y),
                IDtoIndex.get(CSotaMotion.SV_L_SHOULDER),
                IDtoIndex.get(CSotaMotion.SV_L_ELBOW)),

        R_HAND(IDtoIndex.get(CSotaMotion.SV_BODY_Y),
                IDtoIndex.get(CSotaMotion.SV_R_SHOULDER),
                IDtoIndex.get(CSotaMotion.SV_R_ELBOW)),

        HEAD(IDtoIndex.get(CSotaMotion.SV_BODY_Y),
                IDtoIndex.get(CSotaMotion.SV_HEAD_Y),
                IDtoIndex.get(CSotaMotion.SV_HEAD_R),
                IDtoIndex.get(CSotaMotion.SV_HEAD_P));

        // store the motor indices that contribute to each frame here for use later.
        // hint: use IDtoIndex and the CSotaMotion. constants to make this easy to do.
        public int[] motorindices;

        FrameKeys(int... motorindices) {
            this.motorindices = motorindices;
        }

        // public static void main(String args[]) {
        //     System.out.println("Left hand");
        //     int[] id = L_HAND.motorindices;
        //     for (int i = 0; i < id.length; i++) {
        //         System.out.print(id[i] + " ");
        //     }
        
        //     System.out.println("\nRight hand");
        //     int[] id1 = R_HAND.motorindices;
        //     for (int i = 0; i < id1.length; i++) {
        //         System.out.print(id1[i] + " ");
        //     }
        // }
    }
}