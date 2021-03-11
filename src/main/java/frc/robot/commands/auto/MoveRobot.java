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

    /**
     * Constructor
     */
    public MoveRobot(double dist)
    {
        addRequirements(m_drive); // Adds the subsystem to the command
        tgtDist = dist;
        curDist = 0;
        curSpeed = 0.5;
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
        m_drive.setRobotSpeed(0,0,curSpeed);
        curDist += curSpeed*dT;
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
        //Check if distance reached
        if (curDist>=tgtDist)
            return true;
        else
            return false;
    }

}