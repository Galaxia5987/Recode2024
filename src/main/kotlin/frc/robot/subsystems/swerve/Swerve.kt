package frc.robot.subsystems.swerve

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import swervelib.SwerveDrive
import java.util.function.DoubleSupplier

class Swerve private constructor() : SubsystemBase() {

    val swerveInputs = LoggedSwerveInputs()
    val moduleInputs =  arrayOf(LoggedModuleInputs(), LoggedModuleInputs(), LoggedModuleInputs(), LoggedModuleInputs())

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
        configAutoBuilder()
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
        val desiredSpeeds = ChassisSpeeds(
            SwerveConstants.MAX_SPEED*xOutput,
            SwerveConstants.MAX_SPEED*yOutput,
            SwerveConstants.MAX_SPEED*omegaOutput
        )
        println(SwerveConstants.MAX_SPEED*xOutput)
        swerveInputs.desiredSpeeds = desiredSpeeds
        driveFieldOriented(desiredSpeeds)
    }

    private fun updateInputs(){
        for (i in 0..3){
            moduleInputs[i].moduleState = swerveDrive.modules[i].state
            moduleInputs[i].moduleDistance = swerveDrive.modules[i].position.distanceMeters
            moduleInputs[i].angle = swerveDrive.modules[i].state.angle
            moduleInputs[i].angleSetpoint =
                swerveDrive.kinematics.toSwerveModuleStates(swerveInputs.desiredSpeeds)[i].angle
            moduleInputs[i].driveMotorVelocity = swerveDrive.modules[i].state.speedMetersPerSecond
            moduleInputs[i].driveMotorVelocitySetpoint =
                swerveDrive.kinematics.toSwerveModuleStates(swerveInputs.desiredSpeeds)[i].speedMetersPerSecond
//            moduleInputs.angleMotorVelocity =swerveInputs.desiredSpeeds
        }
    }

    override fun periodic() {
        updateInputs()
        Logger.processInputs("Swerve", swerveInputs)
        for (i in 0..3){
            Logger.processInputs("Swerve/Module${i + 1}", moduleInputs[i])
        }
    }
}