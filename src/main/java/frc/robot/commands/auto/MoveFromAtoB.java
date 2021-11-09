package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Astar.Tile;
import frc.robot.Astar.element.Node;
// import the commands
import frc.robot.MyGenerateTrajectory;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class MoveFromAtoB extends SequentialCommandGroup
{   

    private Trajectory mTrajectory ;
    //Set max velocity, acceleration and centripedal acceleration (turn speed)
    private CentripetalAccelerationConstraint mCurveConstraint = new CentripetalAccelerationConstraint(0.8);
    private TrajectoryConfig mConfig = new TrajectoryConfig(0.5, 0.5).addConstraint(mCurveConstraint).setReversed(false);

    private Translation2d mStart;
    private Translation2d mFinal;
    // for Testing purpose
    static private List<Translation2d> waypoints = List.of(
        new Translation2d(0.0, 0.0), //start
        new Translation2d(0.5, 0.5), 
        new Translation2d(1.0, 0.5), 
        new Translation2d(1.0, 1.0), 
        new Translation2d(0.0, 1.0)
      );        

    static MyGenerateTrajectory myGenerateTrajectory = new MyGenerateTrajectory();

    @Override
    public void initialize() {

        //Call A* to solve path from point A to point B
        // Tile nodeStart, nodeEnd;
        // nodeStart = new Tile(0,0);
        // nodeEnd = new Tile(0,0);

        // RobotContainer.m_Astar.setStart(nodeStart);
        // RobotContainer.m_Astar.setEnd(nodeEnd);
        // RobotContainer.m_Astar.solve();
        // ArrayList<Node> pathInNodes = RobotContainer.m_Astar.getPath();

        // ArrayList<Tile> path = new ArrayList<Tile>();

        // //Conver Node to Tile
        // if (path != null) {
        //     for (Node n : pathInNodes) {
        //         if (n instanceof Tile) {
        //             path.add((Tile) n);
        //         }
        //     }
        // }

        //Convert Tile to x,y unit in metre


        //generate trajectory based on A* output
        //Two different methods. Currently only Clampbed cubic works.
        //Quintic is better but there's problem with vmx wpilib
        mTrajectory =
            //myGenerateTrajectory.generateTrajectoryClampedCubic(waypoints, mConfig, 0.05);
            myGenerateTrajectory.generateTrajectoryQuinticHermite(waypoints, mConfig, 0.05);
             
        super.initialize();
    }

    public Trajectory getTrajectory() {
        return mTrajectory;
    }

	public MoveFromAtoB(Translation2d A, Translation2d B)
    {
        super (
            new OmniTrackTrajectoryCommand(
                myGenerateTrajectory::getTrajectory,
                RobotContainer.m_omnidrive::getPose,
                // Position contollers
                new PIDController(0.5, 0, 0),
                new PIDController(0.5, 0, 0),
                new ProfiledPIDController(1, 0, 0, new Constraints(Math.PI, Math.PI) ),
                RobotContainer.m_omnidrive )
        );
        mStart = A;
        mFinal = B;

    }
}
