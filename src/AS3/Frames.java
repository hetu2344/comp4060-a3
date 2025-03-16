package AS3;

import jp.vstone.RobotLib.CSotaMotion;

public class Frames {

    public enum FrameKeys{  
        L_HAND(),
        R_HAND(),
        HEAD(); 
        
        // store the motor indices that contribute to each frame here for use later.
        // hint: use IDtoIndex and the CSotaMotion. constants to make this easy to do.
        public int[] motorindices;
        FrameKeys(int... motorindices){
            this.motorindices = motorindices;            
        }
    }
}