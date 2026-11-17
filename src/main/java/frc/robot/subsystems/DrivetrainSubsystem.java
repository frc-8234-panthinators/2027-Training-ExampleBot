package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class DrivetrainSubsystem extends SubsystemBase
{
    private final SwerveDrive drive;
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

    MechanismGearing driveGearing = new MechanismGearing(6.03); // Drive gearRatio

    MechanismGearing azimuthGearing = new MechanismGearing(26.09); // Angle gearRatio

    Distance wheelDiameter = Inches.of(4); // Diameter of Wheel
    SmartMotorControllerConfig driveConfig = new SmartMotorControllerConfig(this)
        .withWheelDiameter(wheelDiameter)

        .withClosedLoopController(0.3, 0, 0)
        .withGearing(driveGearing)

        .withFeedforward(new SimpleMotorFeedforward(0,
                                                       12.0 / (maxLinearVelocity.in(MetersPerSecond) /
                                                              wheelDiameter.in(Meters)),
                                                    0.01))

        .withStatorCurrentLimit(Amps.of(60)) // drive statorCurrentLimit
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
  
    SmartMotorControllerConfig azimuthConfig = new SmartMotorControllerConfig(this)
        .withClosedLoopController(1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 1))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20)) // angle statorCurrentLimit
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    //I believe this has something to do with the solenoids, not sure
    SmartMotorController driveSMC = new TalonFXWrapper(drive, DCMotor.getKrakenX60(1), driveConfig);
    SmartMotorController azimuthSMC = new TalonFXWrapper(azimuth, DCMotor.getKrakenX60(1), azimuthConfig);
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, new SwerveModuleTelemetryConfig().withDataLogName("Swerve/" + moduleName)
                                                                    .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH))
        .withLocation(location)
        .withOptimization(true);

    return new SwerveModule(moduleConfig);
  }

  public DrivetrainSubsystem()
  {
    Pigeon2 gyro = new Pigeon2(13);
    // 13 is our CAN Id

    //10.875 and 11.375 are the actual distances from the module centers to the robot center in each axis
    var fl = createModule(new TalonFX(7), 
                          new TalonFX(8),
                          new CANcoder(9),
                          "frontleft",
                          new Translation2d(Inches.of(10.875), Inches.of(11.375)));

    var fr = createModule(new TalonFX(10),
                          new TalonFX(11),
                          new CANcoder(12),
                          "frontright",
                          new Translation2d(Inches.of(10.875), Inches.of(-11.375)));

    var bl = createModule(new TalonFX(4),
                          new TalonFX(5),
                          new CANcoder(6),
                          "backleft",
                          new Translation2d(Inches.of(-10.875), Inches.of(11.375)));

    var br = createModule(new TalonFX(1),
                          new TalonFX(2),
                          new CANcoder(3),
                          "backright",
                          new Translation2d(Inches.of(-10.875), Inches.of(-11.375)));

    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro::getRotation3d)
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(2.7, 0, 0.2))
        .withRotationController(new PIDController(34, 0, 0.5))
        .withTelemetry("swerve", new SwerveDriveTelemetryConfig().withDataLogName("Swerve")
                                                              .withTelemetryVerbosity(SmartMotorControllerConfig.TelemetryVerbosity.HIGH));

    drive = new SwerveDrive(config);

    SmartDashboard.putData("Field", field);            
  }
  
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get())).withName("Field Oriented Drive");
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveToPose(Pose2d pose)
  {
    return drive.driveToPose(pose);
  }

    public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command lock()
  {
    return run(drive::lockPose);
  }

  public Command azimuthSysId()
  {
    return runOnce(() -> {});
  }

  public Command driveSysId()
  {
    return runOnce(() -> {});
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }
}


