package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
// import the commands
import frc.robot.commands.auto.MoveRobot;
import frc.robot.commands.auto.RotateTest;
import frc.robot.MyGenerateTrajectory;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class AutoMainCmd extends SequentialCommandGroup
{   

    private static CentripetalAccelerationConstraint curveConstraint = new CentripetalAccelerationConstraint(1.0);
    private static TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5).addConstraint(curveConstraint);

    private static List<Translation2d> waypoints = List.of(
        new Translation2d(0.0, 0.0), //start
        new Translation2d(0.0, 1.0), 
        new Translation2d(1.0, 1.0)
        // new Translation2d(1.0, 0.5), 
        // new Translation2d(1.0, 1.0), 
        // new Translation2d(0.0, 1.0)
      );        

    private static Trajectory exampleTrajectory =
    MyGenerateTrajectory.generateTrajectory(waypoints,config, 0.05);

	public AutoMainCmd()
    {
        
        super (
             new OmniControllerCommand(
                exampleTrajectory,
                RobotContainer.m_omnidrive::getPose,
                // Position contollers
                new PIDController(0.5, 0, 0),
                new PIDController(0.5, 0, 0),
                new ProfiledPIDController(1, 0, 0, new Constraints(Math.PI, Math.PI) ),
                RobotContainer.m_omnidrive)

            );

        // super(
        //     new MoveRobot(2, -Math.PI/4, 0, 0, Math.PI),  
        //     new MoveRobot(2, Math.PI/4, 0, 0, Math.PI),
        //     new LoopCmd(new RotateTest()),
        //     new MoveRobot(2, Math.PI/4, 0, 0, Math.PI)
        //     );
    }
}
