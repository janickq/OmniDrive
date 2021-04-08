package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.MoveRobot;
import frc.robot.commands.auto.MoveRobotSense;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class RotateTest extends AutoCommand
{
   
	public RotateTest()
    {

        super(
            new MoveRobotSense(1, 0.5, 0, 0.0, 0.5,()->false)
            
        );
    }
}
