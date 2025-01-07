package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.HardwareConstants;

public final class SwerveModule {
  private final String name;

  // Alert
  private final Alert driveMotorAlert, steerMotorAlert, steerCANCoderAlert, steerNotHomedAlert;

  // Drive hardware
  private final TalonFX driveMotor;
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;

  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0)
    .withOverrideBrakeDurNeutral(true);
  private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0.0);
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final StaticBrake staticBrakeRequest = new StaticBrake();

  // Steer hardware
  private final SparkMax steerMotor;
  private final SparkClosedLoopController steerPID;
  private final RelativeEncoder steerBuiltInEncoder;

  private final CANcoder steerCANCoder;
  private final StatusSignal<Angle> steerAngle;

  /** State (for logging) */
  public final SwerveModuleState state = new SwerveModuleState();

  /**
   * Initializes a new Swerve Module. 
   * Kraken X60 for drive, NEO for steer, CANCoder for absolute angle.
   * @param name Unique readable name to identify this module.
   * @param driveMotorCAN Drive Kraken X60 CAN id.
   * @param steerMotorCAN Steer CAN Spark Max CAN id.
   * @param steerEncoderCAN Steer CANCoder CAN id.
   * @param steerCANCoderOffsetRots Steer CANCoder absolute position offset, in rotations.
   */
  public SwerveModule(
    String name, 
    int driveMotorCAN, 
    int steerMotorCAN, 
    int steerEncoderCAN, 
    double steerCANCoderOffsetRots
  ) {
    this.name = name;

    // Initialize Alerts
    driveMotorAlert = new Alert("Swerve Module " + name + " drive motor error", AlertType.kError);
    steerMotorAlert = new Alert("Swerve Module " + name + " steer motor error", AlertType.kError);
    steerCANCoderAlert = new Alert("Swerve Module " + name + " steer CANCoder error", AlertType.kError);
    steerNotHomedAlert = new Alert("Swerve Module " + name + " steer encoder failed to home", AlertType.kError);
    steerNotHomedAlert.set(true);

    // Initialize DRIVE MOTOR
    driveMotor = new TalonFX(driveMotorCAN, HardwareConstants.CANIVORE);

    driveMotor.getConfigurator().apply(SwerveConstants.DRIVE_CONFIG);
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();

    // Initialize STEER ENCODER
    steerCANCoder = new CANcoder(steerEncoderCAN, HardwareConstants.CANIVORE);

    MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs()
      .withAbsoluteSensorDiscontinuityPoint(SwerveConstants.CANCODER_RANGE)
      .withSensorDirection(SwerveConstants.CANCODER_DIRECTION)
      .withMagnetOffset(steerCANCoderOffsetRots);

    steerCANCoder.getConfigurator().apply(cancoderConfig);
    steerAngle = steerCANCoder.getAbsolutePosition();

    // Initialize STEER MOTOR
    steerMotor = new SparkMax(steerMotorCAN, MotorType.kBrushless);
    steerMotor.configure(SwerveConstants.STEER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    steerBuiltInEncoder = steerMotor.getEncoder();
    steerPID = steerMotor.getClosedLoopController();

    // Default state
    setBrakeMode(true);
    setRequest(new SwerveModuleState(), true); // zero velocity, zero angle
  }

  public StatusSignal<Angle> getDrivePosition() {
    return drivePosition.clone();
  }

  public StatusSignal<Angle> getSteerAngle() {
    return steerAngle.clone();
  }

  /**
   * Homes the steer Neo's built-in relative encoder.
   */
  public void homeEncoder() {
    steerAngle.refresh();

    System.out.printf(
      "SwerveModule %s homed from %f deg to %f deg.\n",
      name,
      steerBuiltInEncoder.getPosition() / SwerveConstants.STEER_RATIO * 360.0,
      steerAngle.getValueAsDouble() * 360.0
    );

    REVLibError resp = steerBuiltInEncoder.setPosition(steerAngle.getValueAsDouble() * SwerveConstants.STEER_RATIO);
    steerNotHomedAlert.set(!resp.equals(REVLibError.kOk));
  }

  /**
   * @param brake Whether the steer and drive motors should have brake mode enabled.
   */
  public void setBrakeMode(boolean brake) {
    steerMotor.configure(
      new SparkMaxConfig().idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters);
    driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /**
   * @param request Velocity and angle to apply.
   * @param closedLoop Open loop control (teleop) or closed loop control (auto).
   */
  public void setRequest(SwerveModuleState request, boolean closedLoop) {
    // Optimize
    steerAngle.refresh();
    Rotation2d currentAngle = Rotation2d.fromRotations(steerAngle.getValueAsDouble());
    request.optimize(currentAngle);
    request.cosineScale(currentAngle);

    // Set angle
    steerPID.setReference(
      request.angle.getRotations() * SwerveConstants.STEER_RATIO, ControlType.kPosition);

    // Set velocity
    if (closedLoop) {
      // Closed loop velocity control (auto)
      driveMotor.setControl(driveVelocityRequest.withVelocity(
        request.speedMetersPerSecond / SwerveConstants.WHEEL_CIRCUMFERENCE
      ));
    } else {
      // Open loop control (teleop)
      driveMotor.setControl(driveDutyCycleRequest.withOutput(
        request.speedMetersPerSecond / SwerveConstants.MAX_WHEEL_VELOCITY
      ));
    }
  }

  /**
   * Steer heading will default to 0.0 degrees (forward).
   * @param voltage Voltage request (for characterization).
   */
  public void setVoltageRequest(double voltage) {
    steerPID.setReference(0.0, ControlType.kPosition);
    driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
  }

  public void updateSwerveState() {
    // Refresh signals
    driveVelocity.refresh();
    steerAngle.refresh();

    // Update state
    state.angle = Rotation2d.fromRotations(steerAngle.getValueAsDouble());
    //state.angle = Rotation2d.fromRotations(steerBuiltInEncoder.getPosition() / SwerveConstants.STEER_RATIO);
    state.speedMetersPerSecond = driveVelocity.getValueAsDouble() * SwerveConstants.WHEEL_CIRCUMFERENCE;
  }

  /**
   * Checks all the hardware and triggers an alert.
   */
  public void checkHardware() {
    // Check drive motor faults
    driveMotorAlert.set(!driveMotor.isAlive() || !driveMotor.isConnected());

    // Check steer motor faults
    steerMotorAlert.set(steerMotor.hasActiveFault() || steerMotor.hasActiveWarning());

    // Check steer encoder faults
    steerCANCoderAlert.set(steerCANCoder.isConnected());
  }
}
