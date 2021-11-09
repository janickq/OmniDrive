package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Translation2d;
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

      @Override
    public void initialize() {

                
        super.initialize();
    }

	public AutoMainCmd()
    {

        super (
            new MoveFromAtoB(new Translation2d(0,0), new Translation2d(2,2))

            );

        // super(
        //     new MoveRobot(2, -Math.PI/4, 0, 0, Math.PI),  
        //     new MoveRobot(2, Math.PI/4, 0, 0, Math.PI),
        //     new LoopCmd(new RotateTest()),
        //     new MoveRobot(2, Math.PI/4, 0, 0, Math.PI)
        //     );
    }
}
