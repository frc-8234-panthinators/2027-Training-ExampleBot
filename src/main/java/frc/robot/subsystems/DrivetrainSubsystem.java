package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.telemetry.SwerveDriveTelemetryConfig;
import yams.telemetry.SwerveModuleTelemetryConfig;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class DrivetrainSubsystem extends SubsystemBase
{
    private final SwerveDrive drive = new SwerveDrive(null);
    private final Field2d field = new Field2d();

    private AngularVelocity maxAngularVelocity = DegreesPerSecond.of(360);
  // 1 m/s is a conservative starting cap; raise it once PID and feedforward are tuned.
    private LinearVelocity  maxLinearVelocity  = MetersPerSecond.of(1);
    
    public SwerveInputStream getSpeedsSupplier(DoubleSupplier translationXScalar,
                                               DoubleSupplier translationYScalar,
                                               DoubleSupplier rotationScalar)
    {
      return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
          .withMaximumAngularVelocity(maxAngularVelocity)
          .withMaximumLinearVelocity(maxLinearVelocity)
          .withDeadband(0.01)
          .withCubeRotationControllerAxis()
          .withCubeTranslationControllerAxis()
          .withAllianceRelativeControl();
  }

  public Supplier<ChassisSpeeds> getScaledSpeeds(DoubleSupplier translationXScalar,
                                                 DoubleSupplier translationYScalar,
                                                 DoubleSupplier rotationScalar)
  {
    return () -> new ChassisSpeeds(maxLinearVelocity.times(translationXScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maxLinearVelocity.times(translationYScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maxAngularVelocity.times(rotationScalar.getAsDouble())
                                                                      .in(RadiansPerSecond));
  }

  public SwerveModule createModule(TalonFX drive, TalonFX azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    MechanismGearing driveGearing = new MechanismGearing(12.75); //Placeholder value
    MechanismGearing azimuthGearing = new MechanismGearing(6.75) //Placeholder value
    Distance wheelDiameter = Inches.of(4) //Placeholder value
    SmartMotorControllerConfig driveConfig = new SmartMotorControllerConfig(this)
      .withWheelDiameter(wheelDiameter)
      .withClosedLoopController(-.3, 0, 0)
      .withGearing(driveGearing)
      .withFeedforward(new SimpleMotorforward(0,
                                              12.0 / (maxLinearVelocity.in(MetersPerSecond) /
                                                      wheelDiameter.in(Meters)),
                                              0.01))
  }


}
