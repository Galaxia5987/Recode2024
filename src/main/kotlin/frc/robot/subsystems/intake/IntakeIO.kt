package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import org.team9432.annotation.Logged

interface IntakeIO {
    val inputs: LoggedIntakeInputs
        get() = LoggedIntakeInputs()

    fun setSpinPower(power: Double) {

    }
    fun setCenterPower(power: Double) {

    }
    fun setAngle(angle: Rotation2d) {

    }

    fun updateInputs() {

    }

    @Logged
    open class IntakeInputs {
        var spinMotorVoltage = 0.0
        var centerMotorVoltage = 0.0
        var angleMotorAngle: Measure<Angle> = Units.Degree.zero()
        var angleSetPoint: Measure<Angle> = Units.Degree.zero()
    }
}
