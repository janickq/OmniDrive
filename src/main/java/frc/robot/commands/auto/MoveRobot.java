package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//WPI imports
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    private final TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private final int m_dir;

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
        if (type==2){
            m_constraints = new TrapezoidProfile.Constraints(1.0*Math.PI, 2.0*Math.PI);
        }
        else{
            m_constraints = new TrapezoidProfile.Constraints(1.0, 2.0);
        }
        m_setpoint = new TrapezoidProfile.State(0, 0);
        if (tgtDist>0) {
            m_dir = 1;
        }
        else {
            m_dir = -1;
            tgtDist = -tgtDist;
        }
        m_goal = new TrapezoidProfile.State(tgtDist, 0);

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
        /*
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
        }*/
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(dT);
        
        if (m_setpoint.position<m_goal.position) {
            if (profType==0)
                m_drive.setRobotSpeed(m_setpoint.velocity*m_dir,0,0);
            else if (profType==1)
                m_drive.setRobotSpeed(0,m_setpoint.velocity*m_dir,0);
            else if (profType==2)
                m_drive.setRobotSpeed(0,0,m_setpoint.velocity*m_dir);
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