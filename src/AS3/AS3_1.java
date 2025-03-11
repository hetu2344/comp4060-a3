package AS3;

import java.awt.Color;

import jp.vstone.RobotLib.*;

public class AS3_1 {
    static final String TAG = "AS3_1";   // set this to support the Sota logging system
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
            
            Byte[] ids = motion.getDefaultIDs();
            ServoRangeTool srt = ServoRangeTool.Load();

            // turning off the motor
            CRobotUtil.Log(TAG, "Servo Motors Off");
            motion.ServoOff();

            CRobotUtil.Log(TAG, "ServoRange Tool LOAD complete ");
//            srt.Load();

//            srt.printMotorRanges();
            long lastTime = System.currentTimeMillis();
            // clear screen and move cursor to top left
            System.out.print("\033[H\033[2J"); System.out.flush();
            // Enter the while loop
            while(!motion.isButton_Power()){
                System.out.print("\033[H"); // move cursor to top left before redrawing

                Short[] pos = motion.getReadpos();     // read motor positions into an array
//                Byte[] ids = motion.getDefaultIDs();  // get an array of the motor IDs

//                for(int i = 0; i < pos.length;i++){
//                    CRobotUtil.Log(TAG, "Read Pos. ID:" + ids[i] + " , Pos:" + pos[i]);
//                }
                srt.register(pos);
                
                if(System.currentTimeMillis() - lastTime > 3000){
                    srt.printMotorRanges();
                    lastTime = System.currentTimeMillis();                    
                }

//                System.out.println();
//                srt.printMotorRanges();
//                System.out.println();
                // wait for 0.1 secs
                CRobotUtil.wait(100);
            }

            CRobotUtil.Log(TAG, "ServoRange Tool SAVE complete ");
            srt.save();
            
        }
    }
}