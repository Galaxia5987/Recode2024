package frc.robot.subsystems.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Distance
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.littletonrobotics.junction.AutoLogOutput
import swervelib.SwerveDrive
import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration
import java.util.function.DoubleSupplier

class Swerve private constructor() : SubsystemBase() {

    private val swerveDrive = SwerveDrive(
        SwerveConstants.SWERVE_CONFIG,
        SwerveConstants.SWERVE_CONTROLLER_CONFIG,
        SwerveConstants.MAX_SPEED)

    @AutoLogOutput
    private var robotVelocity = ChassisSpeeds()
        get() = swerveDrive.robotVelocity

    @AutoLogOutput
    private var robotPose = Pose2d()
        get() = swerveDrive.pose

    companion object {
        private var instance: Swerve? = null

        fun getInstance(): Swerve{
            if (instance==null) instance = Swerve()
            return instance ?: throw IllegalStateException("swerve instance is null")
        }
    }

    init {
        swerveDrive.setOdometryPeriod(1.0/250.0)
        swerveDrive.setHeadingCorrection(true)
        swerveDrive.setMotorIdleMode(true)
    }

    private fun configAutoBuilder(){
        AutoBuilder.configureHolonomic(
            {robotPose},
            this::resetOdometry,
            {robotVelocity},
            this::setChassisSpeeds,
            SwerveConstants.holonomicPathFollowerConfig,
            Constants::isRed,
            this
        )
    }

    fun resetGyro(rotation: Rotation3d = Rotation3d()){
        swerveDrive.setGyro(Rotation3d())
    }

    fun resetOdometry(pose: Pose2d){
        swerveDrive.resetOdometry(pose)
    }

    fun setChassisSpeeds(speeds: ChassisSpeeds){
        swerveDrive.setChassisSpeeds(speeds)
    }

    fun driveCommand(forward: DoubleSupplier, strafe: DoubleSupplier, rotation: DoubleSupplier): Command {
        return Commands.run({swerveDrive.driveFieldOriented(
            forward.asDouble, strafe.asDouble, rotation.asDouble)})
    }

    private fun SwerveDrive.driveFieldOriented(xOutput: Double, yOutput: Double, omegaOutput: Double){
        driveFieldOriented(ChassisSpeeds(
            SwerveConstants.MAX_SPEED*xOutput,
            SwerveConstants.MAX_SPEED*yOutput,
            SwerveConstants.MAX_SPEED*omegaOutput))
    }
}