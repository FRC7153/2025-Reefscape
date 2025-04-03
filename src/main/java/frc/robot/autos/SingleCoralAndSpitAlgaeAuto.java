package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SingleCoralAndSpitAlgaeAuto extends SequentialCommandGroup {
  private final String name;

  /**
   * Drives, places an L4 Coral, picks up an algae, and spits it out.
   * @param drive
   * @param elevator
   * @param manipulator
   * @param led
   * @param pathName Name of initial path to follow from PathPlanner (starting point to reef side).
   * @param reefSide Side of reef to use (index in LockOnAlignments).
   * @param highAlgae Whether to use high algae or low algae intake.
   */
  public SingleCoralAndSpitAlgaeAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator, LED led, String pathName, int reefSide, boolean highAlgae) {
    super(
      // Place coral, grab algae
      new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, pathName, reefSide, highAlgae, false),
      new WaitCommand(0.35),
      // Spin slowly towards cage
      new InstantCommand(() -> drive.drive(0.0, 0.0, 2.0, true, false), drive),
      new WaitUntilCommand(() -> isLookingAtOpponentCages(drive.getPosition(false).getRotation())),
      new InstantCommand(drive::stop, drive),
      // Spit
      new InstantCommand(() -> manipulator.setManipulatorVelocity(-0.5), manipulator),
      new WaitCommand(0.6),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      // Stow arm
      new PrintCommand("SingleCoralAndSpitAlgaeAuto finished!"),
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW).repeatedly()
    );

    // Make sure everything is required
    addRequirements(drive, elevator, manipulator);

    name = String.format(
      "SingleCoralAndSpitAlgaeAuto(%s, %d, %s)", 
      pathName, 
      reefSide,
      highAlgae ? "HIGH" : "LOW"
    );
  }

  @Override
  public String getName() {
    return name;
  }

  /**
   * @param angle Non-field oriented robot angle.
   * @return Whether that angle is looking at the opponent's cages.
   */
  private static boolean isLookingAtOpponentCages(Rotation2d angle) {
    double deg = Units.radiansToDegrees(MathUtil.angleModulus(angle.getRadians()));
    return (deg >= -40 && deg <= 0);
  }
}
