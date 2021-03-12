package frc.robot.commands.auto;

//WPI imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
//RobotContainer import
import frc.robot.RobotContainer;

//Subsystem imports
import frc.robot.subsystems.OmniDrive;

/**
 * SimpleDrive class
 * <p>
 * This class drives a motor 
 */
public class MoveRobot extends CommandBase
{
    //Grab the subsystem instance from RobotContainer
    private final static OmniDrive m_drive = RobotContainer.m_omnidrive;
    private double tgtDist, curDist=0;
    private double curSpeed, tgtSpeed;
    private double dT = 0.02;
    private boolean endFlag = false;
    private int profType;

    /**
     * Constructor
     */
    public MoveRobot(int type, double dist, double speed)
    {
        addRequirements(m_drive); // Adds the subsystem to the command
        tgtDist = dist;
        curDist = 0;
        curSpeed = speed;
        profType = type;
    }

    /**
     * Runs before execute
     */
    @Override
    public void initialize()
    {
        
    }

    /**
     * Called continously until command is ended
     */
    @Override
    public void execute()
    {
        //Do speed profile
        if (curDist<tgtDist) {
            if (profType==0)
                m_drive.setRobotSpeed(curSpeed,0,0);
            else if (profType==1)
                m_drive.setRobotSpeed(0,curSpeed,0);
            else if (profType==2)
                m_drive.setRobotSpeed(0,0,curSpeed);
            curDist += curSpeed*dT;
        }
        else {
            endFlag = true;
        }
//
    }

    /**
     * Called when the command is told to end or is interrupted
     */
    @Override
    public void end(boolean interrupted)
    {
        m_drive.setRobotSpeed(0,0,0);
        m_drive.setMotorSpeedAll(0);
    }

    /**
     * Creates an isFinished condition if needed
     */
    @Override
    public boolean isFinished()
    {
        return endFlag;
    }

}