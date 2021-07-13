package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import the commands
import frc.robot.commands.auto.MoveRobot;
import frc.robot.commands.auto.RotateTest;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class AutoMainCmd extends SequentialCommandGroup
{

	public AutoMainCmd()
    {
        super(
            new MoveRobot(2, -Math.PI/4, 0, 0, Math.PI),  
            new MoveRobot(2, Math.PI/4, 0, 0, Math.PI),
            new LoopCmd(new RotateTest()),
            new MoveRobot(2, Math.PI/4, 0, 0, Math.PI)
            );
    }
}
