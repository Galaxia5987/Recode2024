package frc.robot.subsystems.swerve

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.Utils
import frc.robot.lib.units.Units

class ModuleIOTalonFX(
    driveMotorID: Int,
    angleMotorID: Int,
    encoderID: Int,
    private val driveConfig: TalonFXConfiguration,
    private val angleConfig: TalonFXConfiguration,
    private val encoderConfig: CANcoderConfiguration,
    private val encoderOffset: Double
) : ModuleIO {
    private val driveMotor = TalonFX(driveMotorID, "swerveDrive")
    private val angleMotor = TalonFX(angleMotorID, "swerveDrive")

    private val encoder = CANcoder(encoderID, "swerveDrive")

    private val angleControlRequest = PositionVoltage(0.0).withEnableFOC(true).withSlot(0)
    private val velocityControlRequest = VelocityVoltage(0.0).withEnableFOC(true)
    override val inputs = LoggedModuleInputs()

    init {
        updatePID()

        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        driveMotor.configurator.apply(driveConfig)
        driveMotor.setPosition(0.0)

        angleConfig.ClosedLoopGeneral.ContinuousWrap = true
        angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        angleMotor.configurator.apply(angleConfig)
        angleMotor.setPosition(0.0)

        encoderConfig.MagnetSensor.withMagnetOffset(encoderOffset)
        encoder.configurator.apply(encoderConfig)

        BaseStatusSignal.setUpdateFrequencyForAll(
            200.0,
            driveMotor.velocity,
            driveMotor.position,
            angleMotor.position,
            angleMotor.motorVoltage
        )

        driveMotor.setNeutralMode(NeutralModeValue.Brake)
        angleMotor.setNeutralMode(NeutralModeValue.Brake)
    }

    override fun updateInputs() {
        inputs.driveMotorPosition = driveMotor.position.value
        inputs.driveMotorVelocity =
            Units.rpsToMetersPerSecond(
                driveMotor.velocity.value, SwerveConstants.WHEEL_DIAMETER / 2
            )
        inputs.driveMotorVoltage = driveMotor.motorVoltage.value
        inputs.driveMotorAcceleration =
            Units.rpsToMetersPerSecond(
                driveMotor.acceleration.value,
                SwerveConstants.WHEEL_DIAMETER / 2
            )

        inputs.angle =
            Utils.normalize(Rotation2d.fromRotations(angleMotor.position.value))
        inputs.angleMotorAppliedVoltage = angleMotor.motorVoltage.value
        inputs.angleMotorVelocity = angleMotor.velocity.value

        inputs.moduleDistance =
            Units.rpsToMetersPerSecond(
                inputs.driveMotorPosition, SwerveConstants.WHEEL_DIAMETER / 2
            )
        inputs.moduleState = moduleState

        inputs.encoderHasFaults =
            encoder.fault_Hardware.value ||
            encoder.fault_Undervoltage.value ||
            encoder.fault_BadMagnet.value ||
            encoder.fault_BootDuringEnable.value ||
            encoder.fault_UnlicensedFeatureInUse.value

        inputs.absolutePosition = encoder.absolutePosition.value
        inputs.moduleState = moduleState
    }

    override fun updatePID() {
        driveConfig
            .Slot0
            .withKP(SwerveConstants.DRIVE_KP.get())
            .withKI(SwerveConstants.DRIVE_KI.get())
            .withKD(SwerveConstants.DRIVE_KD.get())
            .withKV(SwerveConstants.DRIVE_KV.get())
            .withKS(SwerveConstants.DRIVE_KS.get())
            .withKA(SwerveConstants.DRIVE_KA.get())
        angleConfig
            .Slot0
            .withKP(SwerveConstants.ANGLE_KP.get())
            .withKI(SwerveConstants.ANGLE_KI.get())
            .withKD(SwerveConstants.ANGLE_KD.get())
            .withKV(SwerveConstants.ANGLE_KV.get())
            .withKS(SwerveConstants.ANGLE_KS.get())
            .withKA(SwerveConstants.ANGLE_KA.get())

        driveMotor.configurator.apply(driveConfig.Slot0)
        angleMotor.configurator.apply(angleConfig.Slot0)
    }

    override var angle
        get() = inputs.angle
        set(angle) {
            var angle = angle
            angle = Utils.normalize(angle)
            inputs.angleSetpoint = angle
            val error = angle.minus(inputs.angle)
            angleControlRequest
                .withPosition(inputs.angle.rotations + error.rotations)
                .withFeedForward(SwerveConstants.ANGLE_KS.get())
                .withEnableFOC(true)
            angleMotor.setControl(angleControlRequest)
        }

    override var velocity
        get() = inputs.driveMotorVelocity
        set(velocity) {
            inputs.driveMotorVelocitySetpoint = velocity

            velocityControlRequest
                .withVelocity(Units.metersToRotations(velocity, SwerveConstants.WHEEL_DIAMETER / 2))
                .withEnableFOC(true)
            driveMotor.setControl(velocityControlRequest)
        }

    override val moduleState
        get() = SwerveModuleState(velocity, angle)

    override val modulePosition
        get() = SwerveModulePosition(inputs.moduleDistance, angle)

    override fun stop() {
        driveMotor.stopMotor()
        angleMotor.stopMotor()
    }

    override fun checkModule(): Command? {
        return Commands.run(
            {
                driveMotor.set(0.8)
                angleMotor.set(0.2)
            })
    }

    override fun setVoltage(volts: Double) {
        driveMotor.setControl(VoltageOut(volts))
    }
}
