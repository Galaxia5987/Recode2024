package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.Ports

class ShooterIOReal : ShooterIO {
    override val topRollerInputs = LoggedRollerInputs()
    override val bottomRollerInputs = LoggedRollerInputs()
    private val topMotor = TalonFX(Ports.Shooter.TOP_MOTOR_ID)
    private val bottomMotor = TalonFX(Ports.Shooter.BOTTOM_MOTOR_ID)
    private val topControl = VelocityVoltage(0.0)
    private val bottomControl = VelocityVoltage(0.0)

    init {
        topMotor.setNeutralMode(NeutralModeValue.Coast)
        bottomMotor.setNeutralMode(NeutralModeValue.Coast)

        topMotor.configurator.apply(ShooterConstants.topMotorConfiguration)
        bottomMotor.configurator.apply(ShooterConstants.bottomMotorConfiguration)
    }

    override fun setTopVelocity(velocity: MutableMeasure<Velocity<Angle>>) {
        topMotor.setControl(topControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun setBottomVelocity(velocity: MutableMeasure<Velocity<Angle>>) {
        bottomMotor.setControl(bottomControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        topMotor.stopMotor()
        bottomMotor.stopMotor()
    }

    override fun updateInputs() {
        topRollerInputs.velocity.mut_replace(
            topMotor.velocity.value, Units.RotationsPerSecond
        )
        topRollerInputs.voltage.mut_replace(topMotor.motorVoltage.value, Units.Volts)

        bottomRollerInputs.velocity.mut_replace(
            bottomMotor.velocity.value, Units.RotationsPerSecond
        )
        bottomRollerInputs.voltage.mut_replace(bottomMotor.velocity.value, Units.Volts)
    }
}