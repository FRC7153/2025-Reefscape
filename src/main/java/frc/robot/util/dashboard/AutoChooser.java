package frc.robot.util.dashboard;

import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.BuildConstants;
import frc.robot.autos.AutoCommandHandler;
import frc.robot.autos.SinglePieceCenterAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;

public final class AutoChooser {
  private static final Command noOpCommand = new PrintCommand("No-op auto selected.");

  private final SendableChooser<Pair<Pose2d, Supplier<Command>>> chooser = new SendableChooser<>();
  private Command currentLoadedCommand = null;

  private final SwerveDrive drive;

  private final Alert noAutoLoadedAlert = new Alert("No auto loaded yet (run pregame)", AlertType.kInfo);

  public AutoChooser(SwerveDrive drive, Elevator elevator, Climber climber, Manipulator manipulator) {
    this.drive = drive;

    // On change
    chooser.onChange((Pair<Pose2d, Supplier<Command>> newAuto) -> {
      currentLoadedCommand = null;
      noAutoLoadedAlert.set(true);
    });

    // Init Named Commands
    AutoCommandHandler.initNamedCommands(elevator);

    // Add default option
    //chooser.setDefaultOption("No-op", () -> noOpCommand); // TODO set starting pose here

    // Center autos
    chooser.addOption("CENTER Single Piece", 
      Pair.of(new Pose2d(7.071, 4.031, Rotation2d.k180deg), () -> new SinglePieceCenterAuto(drive, elevator, manipulator))
    );

    // Autos that are used for testing
    if (BuildConstants.INCLUDE_TEST_AUTOS) {
      /*
      // Add Swerve SysId drive autos
      chooser.addOption("SYSID Swerve Drive Q+", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), true, true));
      chooser.addOption("SYSID Swerve Drive Q-", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), true, false));
      chooser.addOption("SYSID Swerve Drive D+", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), false, true));
      chooser.addOption("SYSID Swerve Drive D-", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleDriveRoutine(drive), false, false));

      // Add Swerve SysId steer autos
      chooser.addOption("SYSID Swerve Steer Q+", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), true, true));
      chooser.addOption("SYSID Swerve Steer Q-", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), true, false));
      chooser.addOption("SYSID Swerve Steer D+", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), false, true));
      chooser.addOption("SYSID Swerve Steer D-", 
        () -> new SysIdCharacterizationCommand(SwerveSysId.getModuleSteerRoutine(drive), false, false));

      // Drive characterization test auto
      chooser.addOption("Auto Drive Test", () -> new SimpleDriveTestAuto(drive));
      chooser.addOption("Back and Forth Drive Test", () -> new BackAndForthTestAuto(drive));
      chooser.addOption("Path 1 Test", 
        () -> SwervePaths.getFollowPathCommand(drive, "Path 1 Test", true).withName("Path 1 Test"));
      chooser.addOption("Rotate 1 Test", 
        () -> SwervePaths.getFollowPathCommand(drive, "Rotate 1 Test", true).withName("Rotate 1 Test"));

      // PathPlanner translational SysId autos
      //chooser.addOption("SYSID Translation Q+", () -> new SysIdCharacterizationCommand(drive.getPathRoutine(), true, true));
      //chooser.addOption("SYSID Translation Q-", () -> new SysIdCharacterizationCommand(drive.getPathRoutine(), true, false));
      //chooser.addOption("SYSID Translation D+", () -> new SysIdCharacterizationCommand(drive.getPathRoutine(), false, true));
      //chooser.addOption("SYSID Translation D-", () -> new SysIdCharacterizationCommand(drive.getPathRoutine(), false, false));

      // Add Elevator SysID auto
      chooser.addOption("SYSID Elevator Q+",
        () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), true, true));
      chooser.addOption("SYSID Elevator Q-", 
        () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), true, false));
      chooser.addOption("SYSID Elevator D+", 
        () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), false, true));
      chooser.addOption("SYSID Elevator D-", 
        () -> new SysIdCharacterizationCommand(elevator.getElevatorRoutine(), false, false));

      chooser.addOption("SYSID Elevator Full", () -> new ElevatorSysIdCommand(elevator));

      chooser.addOption("SYSID Manipulator Pivot Full", () -> new ManipulatorPivotSysIdCommand(elevator));
      
     // Add Manipulator Pivot SysID auto
      chooser.addOption("SYSID Manipulator Pivot Q+",
        () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), true, true));
      chooser.addOption("SYSID Manipulator Pivot Q-",
        () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), true, false));
      chooser.addOption("SYSID Manipulator Pivot D+",
        () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), false, true));
      chooser.addOption("SYSID Manipulator Pivot D-",
        () -> new SysIdCharacterizationCommand(elevator.getManipulatorPivotRoutine(), false, false));
      
     // Add Climber SysId auto
      chooser.addOption("SYSID Climber Pivot Q+", 
        () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), true, true));
      chooser.addOption("SYSID Climber Pivot Q-", 
        () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), true, false));
      chooser.addOption("SYSID Climber Pivot D+", 
        () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), false, true));
      chooser.addOption("SYSID Climber Pivot D-", 
        () -> new SysIdCharacterizationCommand(climber.getClimberPivotRoutine(), false, false));
         */
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

    // Reset position
    Pose2d startPose = (Util.isRedAlliance()) ? FlippingUtil.flipFieldPose(selected.getFirst()) : selected.getFirst();
    drive.resetOdometry(startPose);
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
