package frc.robot.subsystems.galiShooter

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.Ports

class ShooterIOReal : ShooterIO {
    override val bottomInputs = LoggedRollerInputs()
    override val topInputs = LoggedRollerInputs()

    private val topMotor = TalonFX(Ports.GaliShooter.TOP_MOTOR_ID)
    private val bottomMotor = TalonFX(Ports.GaliShooter.BOTTOM_MOTOR_ID)

    private val topControl = VelocityVoltage(0.0)
    private val bottomControl = VelocityVoltage(0.0)

    init {
        topMotor.setNeutralMode(NeutralModeValue.Coast)
        bottomMotor.setNeutralMode(NeutralModeValue.Coast)

        topMotor.configurator.apply(ShooterConstants.topMotorConfiguration)
        bottomMotor.configurator.apply(ShooterConstants.bottomMotorConfiguration)
    }

    override fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {
        topMotor.setControl(topControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {
        bottomMotor.setControl((bottomControl.withVelocity(velocity.`in`(Units.RotationsPerSecond))))
    }

    override fun stop() {
        topMotor.stopMotor()
        bottomMotor.stopMotor()
    }

    override fun updateInputs() {
        topInputs.voltage = Units.Volt.of(topMotor.motorVoltage.value)
        topInputs.currentVelocity = Units.RotationsPerSecond.of(topMotor.velocity.value)

        bottomInputs.voltage = Units.Volt.of(bottomMotor.motorVoltage.value)
        bottomInputs.currentVelocity = Units.RotationsPerSecond.of(bottomMotor.velocity.value)
    }
}