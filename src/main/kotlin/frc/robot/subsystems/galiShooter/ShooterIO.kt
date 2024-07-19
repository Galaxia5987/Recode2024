package frc.robot.subsystems.galiShooter

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import org.team9432.annotation.Logged

interface ShooterIO {
    val topInputs: RollerInputs
    val bottomInputs: RollerInputs

    fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {

    }

    fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {
    }

    fun stop() {

    }

    fun updateInputs() {

    }
}

@Logged
open class RollerInputs {
    var currentVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
    var velocitySetpoint: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
    var voltage: Measure<Voltage> = Units.Volt.zero()
}
