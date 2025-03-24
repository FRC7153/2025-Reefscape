package frc.robot.util.dashboard;

import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.BuildConstants;
import frc.robot.autos.BackAndForthTestAuto;
import frc.robot.autos.CleanSwervesAuto;
import frc.robot.autos.SimpleDriveTestAuto;
import frc.robot.autos.SingleCoralAndAlgaeAuto;
import frc.robot.autos.SingleCoralAndSpitAlgaeAuto;
import frc.robot.commands.sysid.ElevatorSysIdCommand;
import frc.robot.commands.sysid.ManipulatorPivotSysIdCommand;
import frc.robot.commands.sysid.SysIdCharacterizationCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;
import frc.robot.subsystems.swerve.SwerveSysId;
import frc.robot.util.Util;
import frc.robot.util.logging.ConsoleLogger;

public final class AutoChooser {
  private static final Command noOpCommand = new PrintCommand("No-op auto selected.");

  // Each option is a pair of the starting Pose2d and a supplier for the auto command
  private final SendableChooser<Pair<Pose2d, Supplier<Command>>> chooser = new SendableChooser<>();
  private Command currentLoadedCommand = null;

  private final SwerveDrive drive;

  private final Alert noAutoLoadedAlert = new Alert("No auto loaded yet (run pregame)", AlertType.kInfo);

  public AutoChooser(SwerveDrive drive, Elevator elevator, Climber climber, Manipulator manipulator, LED led) {
    this.drive = drive;

    // On change
    chooser.onChange((Pair<Pose2d, Supplier<Command>> newAuto) -> {
      currentLoadedCommand = null;
      noAutoLoadedAlert.set(true);
    });

    // Starting positions
    Pose2d startingCenter = new Pose2d(7.071, 4.031, Rotation2d.k180deg);
    Pose2d startingLeft = new Pose2d(7.071, 6.16, Rotation2d.k180deg);
    Pose2d startingRight = new Pose2d(7.071, 1.92, Rotation2d.k180deg);

    // Add no op autos option
    chooser.setDefaultOption("CENTER No-op", Pair.of(startingCenter, () -> noOpCommand));
    chooser.addOption("LEFT No-op", Pair.of(startingLeft, () -> noOpCommand));
    chooser.addOption("RIGHT No-op", Pair.of(startingRight, () -> noOpCommand));

    // Rear center reef single coral single algae autos
    chooser.addOption("CENTER CENTER Single Piece", 
      Pair.of(startingCenter, () -> new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, "CenterStartToReefH", 3, false))
    );

    chooser.addOption("RIGHT CENTER Single Piece", 
      Pair.of(startingRight, () -> new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, "RightStartToReefH", 3, false))
    );

    chooser.addOption("LEFT CENTER Single Piece", 
      Pair.of(startingLeft, () -> new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, "LeftStartToReefH", 3, false))
    );

    // Left left reef single coral single algae auto
    chooser.addOption("LEFT LEFT Single Piece", 
      Pair.of(startingLeft, () -> new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, "LeftStartToReefI", 4, true))
    );

    // Right right reef single coral single algae auto
    chooser.addOption("RIGHT RIGHT Single Piece", 
      Pair.of(startingRight, () -> new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, "RightStartToReefF", 2, true))
    );

    // Rear center reef single coral spit algae autos
    chooser.addOption("CENTER CENTER Single Piece & Spit", 
      Pair.of(startingCenter, () -> new SingleCoralAndSpitAlgaeAuto(drive, elevator, manipulator, led, "CenterStartToReefH", 3, false))
    );

    // Clean
    chooser.addOption("DANGER CLEAN SWERVES", 
      Pair.of(null, () -> new CleanSwervesAuto(drive))
    );

    // Autos that are used for testing
    if (BuildConstants.INCLUDE_TEST_AUTOS) {
      // Add Swerve SysId drive autos
      chooser.addOption("SYSID Swerve Drive Q+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), true, true)));
      chooser.addOption("SYSID Swerve Drive Q-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), true, false)));
      chooser.addOption("SYSID Swerve Drive D+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), false, true)));
      chooser.addOption("SYSID Swerve Drive D-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), false, false)));

      // Add Swerve SysId steer autos
      chooser.addOption("SYSID Swerve Steer Q+", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), true, true)));
      chooser.addOption("SYSID Swerve Steer Q-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), true, false)));
      chooser.addOption("SYSID Swerve Steer D+", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), false, true)));
      chooser.addOption("SYSID Swerve Steer D-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), false, false)));

      // Drive characterization test auto
      chooser.addOption("Auto Drive Test", 
        Pair.of(null, () -> new SimpleDriveTestAuto(drive)));
      chooser.addOption("Back and Forth Drive Test", 
        Pair.of(null, () -> new BackAndForthTestAuto(drive)));
      chooser.addOption("Path 1 Test", 
        Pair.of(null, () -> SwervePaths.getFollowPathCommand(drive, "Path 1 Test", true).withName("Path 1 Test")));
      chooser.addOption("Rotate 1 Test", 
        Pair.of(null, () -> SwervePaths.getFollowPathCommand(drive, "Rotate 1 Test", true).withName("Rotate 1 Test")));

      // Add Elevator SysID auto
      chooser.addOption("SYSID Elevator Q+",
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), true, true)));
      chooser.addOption("SYSID Elevator Q-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), true, false)));
      chooser.addOption("SYSID Elevator D+", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), false, true)));
      chooser.addOption("SYSID Elevator D-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), false, false)));

      chooser.addOption("SYSID Elevator Full", 
        Pair.of(null, () -> new ElevatorSysIdCommand(elevator)));

      chooser.addOption("SYSID Manipulator Pivot Full", 
        Pair.of(null, () -> new ManipulatorPivotSysIdCommand(elevator)));

      // Add Manipulator Pivot SysID auto
      chooser.addOption("SYSID Manipulator Pivot Q+",
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), true, true)));
      chooser.addOption("SYSID Manipulator Pivot Q-",
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), true, false)));
      chooser.addOption("SYSID Manipulator Pivot D+",
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), false, true)));
      chooser.addOption("SYSID Manipulator Pivot D-",
        Pair.of(null, () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), false, false)));

      // Add Climber SysId auto
      chooser.addOption("SYSID Climber Pivot Q+", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), true, true)));
      chooser.addOption("SYSID Climber Pivot Q-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), true, false)));
      chooser.addOption("SYSID Climber Pivot D+", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), false, true)));
      chooser.addOption("SYSID Climber Pivot D-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), false, false)));
    }

    // Add to dashboard
    SmartDashboard.putData("Auto", chooser);
    noAutoLoadedAlert.set(true);
  }

  /**
   * Loads the currently selected command.
   */
  public void loadAutoCommand() {
    Pair<Pose2d, Supplier<Command>> selected = chooser.getSelected();
    currentLoadedCommand = selected.getSecond().get();
    System.out.printf("New auto loaded: %s\n", currentLoadedCommand.getName());
    noAutoLoadedAlert.set(false);

    // Reset position if disabled
    if (DriverStation.isDisabled() && selected.getFirst() != null) {
      Pose2d startPose = (Util.isRedAlliance()) ? FlippingUtil.flipFieldPose(selected.getFirst()) : selected.getFirst();
      drive.resetOdometry(startPose);
    } else if (selected.getFirst() == null) {
      ConsoleLogger.reportWarning("Loaded auto does not specify a starting configuration.");
    }
  }

  /**
   * @return The currently selected command, loading it if it's not already loaded.
   */
  public Command getCurrentSelectedCommand() {
    if (currentLoadedCommand == null) {
      loadAutoCommand();
    }

    return currentLoadedCommand;
  }
}
