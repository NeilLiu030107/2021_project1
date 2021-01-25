/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team3647.frc2021.autonomous;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import team3647.frc2021.subsystems.Drivetrain;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MoveForward extends PIDCommand {

  private static double kP;
  private static double kI;
  private static double kD;


  /**
   * Creates a new MoveForward.
   */
  public MoveForward(double targetDistance, Drivetrain drivetrain) {

    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        drivetrain::getLeftPosition,
        // This should return the setpoint (can also be a constant)
        targetDistance,
        // This uses the output
        output -> drivetrain.arcadeDrive(0.5, 0, false)
          // Use the output here
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    
    addRequirements(drivetrain);
    getController().setTolerance(0.1);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
