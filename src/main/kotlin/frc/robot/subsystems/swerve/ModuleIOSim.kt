package frc.robot.subsystems.swerve

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.units.Units

class ModuleIOSim() : ModuleIO {
    private val driveMotor = TalonFXSim(
        1,
        1 / SwerveConstants.DRIVE_REDUCTION,
        SwerveConstants.DRIVE_MOTOR_MOMENT_OF_INERTIA,
        1 / SwerveConstants.DRIVE_REDUCTION
    )
    private val angleMotor = TalonFXSim(
        1,
        1 / SwerveConstants.ANGLE_REDUCTION,
        SwerveConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA,
        1 / SwerveConstants.ANGLE_REDUCTION
    )
    private val velocityController = PIDController(
        SwerveConstants.DRIVE_KP.get(),
        SwerveConstants.DRIVE_KI.get(),
        SwerveConstants.DRIVE_KD.get(),
        0.02
    )

    private val angleController = PIDController(
        SwerveConstants.ANGLE_KP.get(),
        SwerveConstants.ANGLE_KI.get(),
        SwerveConstants.ANGLE_KD.get(),
        0.02
    )
    private val driveControlRequest = VelocityVoltage(0.0).withEnableFOC(true)
    private val angleControlRequest = PositionVoltage(0.0).withEnableFOC(true)
    override val inputs = LoggedModuleInputs()

    init {
        driveMotor.setController(velocityController)
        angleMotor.setController(angleController)
    }

    override fun updateInputs() {
        driveMotor.update(Timer.getFPGATimestamp())
        angleMotor.update(Timer.getFPGATimestamp())

        inputs.driveMotorVelocity =
            Units.rpsToMetersPerSecond(
                driveMotor.velocity, SwerveConstants.WHEEL_DIAMETER / 2
            )

        inputs.angleMotorAppliedVoltage = angleMotor.appliedVoltage
        inputs.angle = Rotation2d.fromRotations(angleMotor.position)

        inputs.moduleDistance =
            Units.rpsToMetersPerSecond(
                driveMotor.position, SwerveConstants.WHEEL_DIAMETER / 2
            )
        inputs.moduleState = moduleState

    }

    override var angle
        get() = inputs.angle
        set(angle) {
            inputs.angleSetpoint = angle
            angleMotor.setControl(angleControlRequest.withPosition(angle.rotations))
        }

    override var velocity
        get() = inputs.driveMotorVelocity
        set(velocity) {
            inputs.driveMotorVelocitySetpoint = velocity
            driveControlRequest.withVelocity(
                Units.metersToRotations(velocity, SwerveConstants.WHEEL_DIAMETER / 2)
            )
            driveMotor.setControl(driveControlRequest)
        }

    override val moduleState
        get() = SwerveModuleState(velocity, angle)

    override val modulePosition
        get() = SwerveModulePosition(
            Units.rpsToMetersPerSecond(
                driveMotor.position, SwerveConstants.WHEEL_DIAMETER / 2
            ),
            inputs.angle
        )

    override fun stop() {
        driveControlRequest.withVelocity(0.0)
        driveMotor.setControl(driveControlRequest)

        angleControlRequest.withVelocity(0.0)
        angleMotor.setControl(angleControlRequest)
    }

    override fun checkModule(): Command? {
        return Commands.run(
            {
                driveControlRequest.withVelocity(0.8 * SwerveConstants.MAX_X_Y_VELOCITY)
                driveMotor.setControl(driveControlRequest)
                angleControlRequest.withVelocity(0.2 * SwerveConstants.MAX_X_Y_VELOCITY)
                angleMotor.setControl(angleControlRequest)
            })
    }
}
