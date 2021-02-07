/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2021.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2021.autonomous.MoveForward;
import team3647.frc2021.commands.ArcadeDrive;
import team3647.frc2021.inputs.Joysticks;
import team3647.frc2021.subsystems.Drivetrain;
import team3647.lib.util.DriveSignal;
import team3647.lib.util.Units;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final Joysticks mainController = new Joysticks(0);
    

    private final Drivetrain m_drivetrain = new Drivetrain(Constants.cDrivetrain.leftMasterConfig,
            Constants.cDrivetrain.rightMasterConfig, Constants.cDrivetrain.leftSlave1Config,
            Constants.cDrivetrain.rightSlave1Config, Constants.cDrivetrain.leftSlave2Config, 
            Constants.cDrivetrain.rightSlave2Config, Constants.cDrivetrain.leftMasterPIDConfig, 
            Constants.cDrivetrain.rightMasterPIDConfig, Constants.cDrivetrain.kWheelDiameter, 
            Constants.cDrivetrain.kS, Constants.cDrivetrain.kV, Constants.cDrivetrain.kA);

    private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_commandScheduler.registerSubsystem(m_drivetrain);
    m_commandScheduler.setDefaultCommand(m_drivetrain, 
      new ArcadeDrive(m_drivetrain, mainController::getLeftStickY,
      mainController::getRightStickX, mainController.rightJoyStickPress::get));
    
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  public void init(){

    m_drivetrain.init();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new MoveForward(Units.feet_to_meters(10), m_drivetrain);
  }

  public void stopDrivetrain() {
    m_drivetrain.setOpenLoop(DriveSignal.BRAKE);
  }

}
