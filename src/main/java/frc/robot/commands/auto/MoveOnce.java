package frc.robot.commands.auto;

// import the commands
import frc.robot.commands.auto.MoveRobot;;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class MoveOnce extends AutoCommand
{
    /**
     * Constructor
     */
    public MoveOnce()
    {
        /**
         * Calls SimpleDrive at a speed of 50% waits 5 seconds and stops the motors
         */
        super(
            new MoveRobot(0, 0.5, 0.5),  
            new MoveRobot(2, Math.PI/2, Math.PI/2),
            new MoveRobot(0, 0.5, 0.5),  
            new MoveRobot(2, Math.PI/2, Math.PI/2),
            new MoveRobot(0, 0.5, 0.5),  
            new MoveRobot(2, Math.PI/2, Math.PI/2),
            new MoveRobot(0, 0.5, 0.5),  
            new MoveRobot(2, Math.PI/2, Math.PI/2)
            );
        //new StopMotors()); 
    }
}