package AS3;

import jp.vstone.RobotLib.*;

public class AS3_2 {
    static final String TAG = "AS3_2";   // set this to support the Sota logging system
    static final String RESOURCES = "../resources/";
    static final String SOUNDS = RESOURCES+"sound/";

    public static void main(String[] args) {
        CRobotUtil.Log(TAG, "Start " + TAG);

        // CRobotPose pose = new CRobotPose();  // classes to manage robot pose information
        CRobotMem mem = new CRobotMem(); // connector for the Sota's information system (VSMD), connects via internal socket.
        CSotaMotion motion = new CSotaMotion(mem);   // motion control class. Pass it an instantiated CRobotMem

        if(mem.Connect()){
            CRobotUtil.Log(TAG, "connect " + TAG);
            motion.InitRobot_Sota();  // initialize the Sota VSMD
            CRobotUtil.Log(TAG, "Rev. " + mem.FirmwareRev.get());

            Byte[] ids = motion.getDefaultIDs();
            // Load servo ranges from file created in Task 1
            ServoRangeTool srt = ServoRangeTool.Load();
            // Retrieve servo ranges
            CRobotPose targetPose = srt.getMidPose();
            CRobotPose maxPose = srt.getMaxPose();
            CRobotPose minPose = srt.getMinPose();

            CRobotUtil.Log(TAG, "ServoRange Tool LOAD complete ");

            srt.printMotorRanges();
            
            CRobotUtil.Log(TAG, "Servo Motors On");
            motion.ServoOn(); // Turn on to execute robot movement

            CRobotUtil.Log(TAG, "Moving Robot to neutral position");
//            Short[] pos = targetPose.getServoAngles(ids);     // read motor positions into an array

//            for(int i = 0; i < pos.length;i++){
//                CRobotUtil.Log(TAG, "Read Pos. ID:" + ids[i] + " , Pos:" + pos[i]);
//            }
            motion.play(targetPose, 2000); // move to the neutral position (mid position)

            // wait until motion has finished
            motion.waitEndinterpAll();   // also async public boolean isEndInterpAll()
            CRobotUtil.Log(TAG, "Robot is at neutral position");
            CRobotUtil.wait(2000);   //pause the program / current thread
            
            CRobotUtil.Log(TAG, "Moving each joint to min, max and then back to neutral");
            Short[] servoAngles = null;
            for(int i = 0; i < ids.length; i++){
                // move joint to min
                servoAngles = targetPose.getServoAngles(ids);
                servoAngles[i] = minPose.getServoAngles(ids)[i];
                targetPose.SetPose(ids, servoAngles);
                motion.play(targetPose, 1750);
                // wait until motion has finished
                motion.waitEndinterpAll();   // also async public boolean isEndInterpAll()
                CRobotUtil.wait(1500);   //pause the program / current thread
                
                // move to max
                servoAngles = targetPose.getServoAngles(ids);
                servoAngles[i] = maxPose.getServoAngles(ids)[i];
                targetPose.SetPose(ids, servoAngles);
                motion.play(targetPose, 1750);
                // wait until motion has finished
                motion.waitEndinterpAll();   // also async public boolean isEndInterpAll()
                CRobotUtil.wait(1500);   //pause the program / current thread
                
                // move back to neutral
                targetPose = srt.getMidPose();
                motion.play(targetPose, 1750);
                // wait until motion has finished
                motion.waitEndinterpAll();   // also async public boolean isEndInterpAll()
                CRobotUtil.wait(1500);   //pause the program / current thread
            }

            CRobotUtil.Log(TAG, "Movement of all joints complete");

            CRobotUtil.Log(TAG, "Servo Motors Off");
            motion.ServoOff();
        }
    }
}