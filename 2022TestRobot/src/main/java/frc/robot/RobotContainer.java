package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CmdDriveJoystick;
import frc.robot.commands.GrpAutoExample;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.ShifterState;


public class RobotContainer {
    private Drive m_driveSubsystem = new Drive();


    private XboxController m_driveController = new XboxController(0);

    private JoystickButton m_driveAButton = new JoystickButton(m_driveController, Constants.ButtonMappings.A_BUTTON);
    private JoystickButton m_driveRBButton = new JoystickButton(m_driveController, Constants.ButtonMappings.RIGHT_BUMPER);
    
    private XboxController m_operatorController = new XboxController(1);
    /* 
    Declares the RB Joystick Bumber For The Operator's Controller
     */
    private JoystickButton m_operatorRBButton = new JoystickButton(m_operatorController, Constants.ButtonMappings.RIGHT_BUMPER); 
    /*
    Declares the LB Joystick Bumber For The Operator's Controller
    */
    private JoystickButton m_operatorLBButton = new JoystickButton(m_operatorController, Constants.ButtonMappings.LEFT_BUMPER);
    private double hoodAngle = 0;
    private double shooterSpeed = 0;
    private JoystickButton m_operatorAButton = new JoystickButton(m_operatorController, Constants.ButtonMappings.A_BUTTON);
    private JoystickButton m_operatorYButton = new JoystickButton(m_operatorController, Constants.ButtonMappings.Y_BUTTON);
    private JoystickButton m_operatorXButton = new JoystickButton(m_operatorController, Constants.ButtonMappings.X_BUTTON);


    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        //command is broken; defaults to turning on both commands in the indexer
        //m_ballPathSubsystem.setDefaultCommand(new CmdBallPathDefault(m_ballPathSubsystem));
        m_driveSubsystem.setDefaultCommand(new CmdDriveJoystick(m_driveSubsystem, 
                                                                () -> applyDeadband(0.6 * -m_driveController.getLeftY(), Constants.ARCADE_DRIVE_DEADBAND), 
                                                                () -> applyDeadband( 0.65 * -m_driveController.getRightX(), Constants.ARCADE_DRIVE_DEADBAND)));
    }  

    public Command getAutonomousCommand() {
        return new GrpAutoExample(m_driveSubsystem);
    }

    private double applyDeadband(double raw, double deadband) {
        /* Please don't modify, but please do ask if you wanna know how it works! */
        
        double modified = 0.0;

        deadband = Math.abs(deadband);

        if (raw < -deadband)
            modified = ((raw + 1) / (1 - deadband)) - 1;
        else if (raw > deadband)
            modified = ((raw - 1) / (1 - deadband)) + 1;

        return modified;
    }
}