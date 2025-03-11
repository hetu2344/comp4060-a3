package AS3;

import jp.vstone.RobotLib.*;

public class Test {
    static final String TAG = "TEST";   // set this to support the Sota logging system
    static final String RESOURCES = "../resources/";
    static final String SOUNDS = RESOURCES+"sound/";

    public static void main(String[] args) {
        CRobotUtil.Log(TAG, "Start " + TAG);

        CRobotPose pose = new CRobotPose();  // classes to manage robot pose information
        CRobotMem mem = new CRobotMem(); // connector for the Sota's information system (VSMD), connects via internal socket.
        CSotaMotion motion = new CSotaMotion(mem);   // motion control class. Pass it an instantiated CRobotMem

        if(mem.Connect()){
            while(true){
                if(motion.isButton_Power()){
                    CRobotUtil.Log(TAG, "Power Button Pressed " + TAG);
                } else {
                    CRobotUtil.Log(TAG, "Power Button NOT Pressed " + TAG);
                }
                CRobotUtil.wait(2000);
            }
        }
    }
}
