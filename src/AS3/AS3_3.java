package AS3;

import jp.vstone.RobotLib.*;

public class AS3_3 {
    static final String TAG = "AS3_3";   // set this to support the Sota logging system
    static final String RESOURCES = "../resources/";
    static final String SOUNDS = RESOURCES+"sound/";

    public static void main(String[] args) {
        CRobotUtil.Log(TAG, "Start " + TAG);

        CRobotPose pose = new CRobotPose();  // classes to manage robot pose information
        CRobotMem mem = new CRobotMem(); // connector for the Sota's information system (VSMD), connects via internal socket.
        CSotaMotion motion = new CSotaMotion(mem);   // motion control class. Pass it an instantiated CRobotMem


        if(mem.Connect()){
            CRobotUtil.Log(TAG, "connect " + TAG);
            motion.InitRobot_Sota();  // initialize the Sota VSMD
            CRobotUtil.Log(TAG, "Rev. " + mem.FirmwareRev.get());
            
            ServoRangeTool ranges = ServoRangeTool.Load();

            // turning off the motor
            CRobotUtil.Log(TAG, "Servo Motors Off");
            motion.ServoOff();

            CRobotUtil.Log(TAG, "ServoRange Tool LOAD complete ");
            
            // clear screen and move cursor to top left
            System.out.print("\033[H\033[2J"); System.out.flush();
            // Enter the while loop
            while(!motion.isButton_Power()){
                System.out.print("\033[H"); // move cursor to top left before redrawing

                // printing angle values
                MatrixHelp.printVector(ranges.calcAngles(motion.getReadPose()));

                // printing motor ranges
                ranges.printMotorRanges(motion.getReadpos());

                // wait for 0.1 secs
                CRobotUtil.wait(100);
            }

            CRobotUtil.Log(TAG, "ServoRange Tool SAVE complete ");
            ranges.save();
        }
    }
}