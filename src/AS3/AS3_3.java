package AS3;

import org.apache.commons.math3.linear.RealVector;
import jp.vstone.RobotLib.*;

public class AS3_3 {
    static final String TAG = "AS3_3";   // set this to support the Sota logging system
    static final String RESOURCES = "../resources/";
    static final String SOUNDS = RESOURCES+"sound/";

    public static void main(String[] args) {
        CRobotUtil.Log(TAG, "Start " + TAG);

        // CRobotPose pose = new CRobotPose();  // classes to manage robot pose information
        CRobotMem mem = new CRobotMem(); // connector for the Sota's information system (VSMD), connects via internal socket.
        CSotaMotion motion = new CSotaMotion(mem);   // motion control class. Pass it an instantiated CRobotMem

        if (mem.Connect()) {
            CRobotUtil.Log(TAG, "connect " + TAG);
            motion.InitRobot_Sota();  // initialize the Sota VSMD
            CRobotUtil.Log(TAG, "Rev. " + mem.FirmwareRev.get());

            // Load servo ranges from file created in Task 1
            ServoRangeTool srt = new ServoRangeTool(motion.getDefaultIDs()); 
            ServoRangeTool.Load();

            CRobotUtil.Log(TAG, "ServoRange Tool LOAD complete ");

            motion.ServoOff(); // Turn off servo execution
            CRobotUtil.Log(TAG, "Servo Motors Off");

            System.out.print("\033[H\033[2J"); System.out.flush();
            // Enter the while loop
            while (!motion.isButton_Power()) {
                System.out.print("\033[H");  

                Short[] pos = motion.getReadpos();
                
                srt.register(pos);

                // Convert to radians
                // RealVector angles = srt.calcAngles(pos);
                // MatrixHelp.printVector(angles);

                // Print table
                srt.printMotorRanges();

                CRobotUtil.wait(100);
            }

            CRobotUtil.Log(TAG, "ServoRange Tool SAVE complete ");
            srt.save();
        }
    }
}
