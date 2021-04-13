package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.MoveBack;
import frc.robot.commands.auto.MoveCurve;
import frc.robot.commands.auto.MoveLeft;
import frc.robot.commands.auto.MoveRight;
import frc.robot.commands.gamepad.OI;

public class Menu extends SubsystemBase
{
    private boolean up_state = false;
    private boolean dn_state = false;
    private AutoCommand[] menuCmds ;


    private final OI m_oi = RobotContainer.m_oi;

    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Menu");
    //private final NetworkTableEntry D_servoPos = tab.add("Servo Position", 0).withWidget(BuiltInWidgets.kNumberSlider)
    //       .withProperties(Map.of("min", 0, "max", 300)).getEntry();
    private final NetworkTableEntry D_button = tab.add("button", "?").getEntry();
    private final NetworkTableEntry D_menu = tab.add("menu", 0).getEntry();

    public Menu() {
        menuCmds = new AutoCommand[4];
        menuCmds[0] = new MoveRight();
        menuCmds[1] = new MoveLeft();
        menuCmds[2] = new MoveCurve();
        menuCmds[3] = new MoveBack();
    
    }
    public Command GetCmd() {
        Command ptr=null;
        switch(Globals.menuItem) {
            case 0: ptr = new MoveLeft();  
            break;
            case 1: ptr = new MoveRight();
            break;
            case 2: ptr = new MoveBack();
            break;
            case 3: ptr = new MoveCurve();
            break;
        }
        return ptr;
    }

    @Override
    public void periodic()
    {
        if (m_oi.getDriveYButton()) {
            if (!up_state) {
                Globals.menuItem++;
                up_state = true;
            }
        }
        else {
            up_state = false;
        }

        if (m_oi.getDriveAButton()) {
            if (!dn_state) {
                Globals.menuItem--;
                dn_state = true;
            }
        }
        else {
            dn_state = false;
        }
        
        D_menu.setNumber( Globals.menuItem);

        m_oi.getDriveStartButton();
        m_oi.buttonStart.whenPressed( GetCmd() );

    }
}