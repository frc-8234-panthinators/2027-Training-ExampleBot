package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class DrivetrainSubsystem extends SubsystemBase
{
    private final SwerveDrive drive = new SwerveDrive(null);
    private final Field2d field = new Field2d();

    private AngularVelocity maxSpeedsAngularVelocity = DegreesPerSecond.of(360);
  // 1 m/s is a conservative starting cap; raise it once PID and feedforward are tuned.
    private LinearVelocity  maxSpeedsLinearVelocity  = MetersPerSecond.of(1);
    
    public SwerveInputStream getSpeedsSupplier(DoubleSupplier translationXScalar,
                                               DoubleSupplier translationYScalar,
                                               DoubleSupplier rotationScalar)
    {
      return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
          .withMaximumAngularVelocity(maxSpeedsAngularVelocity)
          .withMaximumLinearVelocity(maxSpeedsLinearVelocity)
          .withDeadband(0.01)
          .withCubeRotationControllerAxis()
          .withCubeTranslationControllerAxis()
          .withAllianceRelativeControl();
  }

  public Supplier<ChassisSpeeds> getScaledSpeeds(DoubleSupplier translationXScalar,
                                                 DoubleSupplier translationYScalar,
                                                 DoubleSupplier rotationScalar)
  {
    return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                                                                      .in(RadiansPerSecond));
  }

  public SwerveModule createModule(Talon drive, Talon azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2D location)
}
