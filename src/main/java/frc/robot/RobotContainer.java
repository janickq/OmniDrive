/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Astar.AStarAlgorithm;
import frc.robot.Astar.Grid;
import frc.robot.Astar.Tile;
import frc.robot.commands.TeleCmd;
import frc.robot.commands.auto.AutoMainCmd;
import frc.robot.commands.gamepad.OI;
import frc.robot.subsystems.Menu;
import frc.robot.subsystems.OmniDrive;
import frc.robot.subsystems.Sensor;

public class RobotContainer {

  //subsystems
  public final static OI m_oi = new OI();
  public final static OmniDrive m_omnidrive = new OmniDrive();
  public final static Sensor m_sensor = new Sensor(); 
  //user menu
  public final static Menu m_menu = new Menu(m_oi);
  //commands
  public final static TeleCmd m_teleCmd = new TeleCmd(m_omnidrive, m_oi);
  public final static AutoMainCmd m_autoCmd = new AutoMainCmd();

  public static final int X_SIZE = 45;
  public static final int Y_SIZE = 90;
  //Create grid
  public static Grid m_Grid = new Grid(X_SIZE, Y_SIZE);
  public static AStarAlgorithm m_Astar;
        

  public RobotContainer()
  {
    //Create new instances

    //Set the default command for the hardware subsytem
    //m_omnidrive.setDefaultCommand(m_teleCmd);

    //Init Astar grid
    //Pre assign neighbours
    for (Tile t : m_Grid.getTiles()) {
      t.calculateNeighbours(m_Grid, true);
    }

    //Add known obstacles here!!!
    m_Grid.AddObstacle(10,10, 10,4, 0);
    m_Grid.AddObstacle(10,60, 10,4, Math.PI/4);
    m_Grid.AddObstacle(40,50, 2,8, 0);
    m_Grid.ExpandObstacles();

    //Create solver
    m_Astar = new AStarAlgorithm(m_Grid);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCmd;
  }
  public Command getTeleopCommand() {
    // An ExampleCommand will run in autonomous
    return m_teleCmd;
  }
}
