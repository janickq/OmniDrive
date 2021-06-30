package frc.robot.commands.auto;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Globals;

// import the commands


/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class MoveTest2 extends AutoCommand
{
    private int state=0;
    private boolean flag = false;
    private boolean m_endFlag = false;
    private AutoCommand cmd;
    //private final ShuffleboardTab tab = Shuffleboard.getTab("Debug");

    //private final NetworkTableEntry D_state = tab.add("State", 0).getEntry();

	public MoveTest2()
    {

    }
    @Override
    public void initialize()
    {

    }
    @Override
    public void execute()
    {
        if (flag==false) {
            //launch command group
            cmd = new RotateTest();
            cmd.schedule();
            flag = true;
            Globals.runFlag = true;
            state++;
        }
        else {
            if (Globals.runFlag == false) {
                //command group finished, reset flag
                state++;
                flag = false;
                if (state==10)
                    m_endFlag = true;

            }
        }
        //D_state.setNumber(state);
    }

    @Override
    public boolean isFinished()
    {
        return m_endFlag;
    }
    @Override
    public void end(boolean interrupted)
    {
        //D_state.setNumber(-1);
    }
}
