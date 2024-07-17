package frc.robot.subsystems.swerve

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import swervelib.SwerveDrive
import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration
import java.util.function.DoubleSupplier

class Swerve(
    val swerveConfig: SwerveDriveConfiguration,
    val controllerConfig: SwerveControllerConfiguration,
    val maxSpeed: Double
) : SwerveDrive(swerveConfig, controllerConfig, maxSpeed) {

    init {
        this.setOdometryPeriod(1.0/250.0)
        this.setHeadingCorrection(true)
        this.setMotorIdleMode(true)
    }

    fun SwerveDrive.driveFieldOriented(xOutput: Double, yOutput: Double, omegaOutput: Double){
        driveFieldOriented(ChassisSpeeds(
            maxSpeed*xOutput,
            maxSpeed*yOutput,
            maxSpeed*omegaOutput))
    }

    fun driveCommand(forward: DoubleSupplier, strafe: DoubleSupplier, rotation: DoubleSupplier): Command {
        return Commands.run({driveFieldOriented(
            forward.asDouble, strafe.asDouble, rotation.asDouble)})
    }
}