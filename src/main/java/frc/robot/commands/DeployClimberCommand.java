package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.subsystems.Climber;
import libs.Elastic;

public class DeployClimberCommand extends SequentialCommandGroup {
  /**
   * Deploys the climber outward, and switches the dashboard to the rear camera view.
   * @param climber
   */
  public DeployClimberCommand(Climber climber) {
    super(
      new InstantCommand(() -> Elastic.selectTab(DashboardConstants.ELASTIC_CLIMB_TAB)),
      new InstantCommand(() -> climber.runClimberPivot(0.75), climber),
      new InstantCommand(climber::setClimberHasDeployedFlag),
      new WaitUntilCommand(() -> climber.getPivotPosition() >= 0.5), // rotations to out
      new InstantCommand(climber::stopClimber, climber)
    );
  }
}
