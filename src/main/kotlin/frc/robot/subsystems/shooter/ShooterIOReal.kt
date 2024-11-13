package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.Ports

class ShooterIOReal : ShooterIO {
    override var inputs: LoggedShooterInputs = LoggedShooterInputs()
    var topMotor: TalonFX = TalonFX(Ports.Shooter.TOP_MOTOR_ID)
    var bottomMotor: TalonFX = TalonFX(Ports.Shooter.BOTTOM_MOTOR_ID)
    var bottomControlRequest: VelocityVoltage = VelocityVoltage(0.0)
    var topControlRequest: VelocityVoltage = VelocityVoltage(0.0)

    init {
        topMotor.configurator.apply(ShooterConstants.configTop)
        bottomMotor.configurator.apply(ShooterConstants.configBottom)
    }

    override fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {
        topMotor.setControl(topControlRequest.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {
        bottomMotor.setControl(bottomControlRequest.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun updateInput() {
        inputs.topVelocity = Units.RotationsPerSecond.of(topControlRequest.Velocity)
        inputs.buttomVelocity = Units.RotationsPerSecond.of(bottomControlRequest.Velocity)
    }
}