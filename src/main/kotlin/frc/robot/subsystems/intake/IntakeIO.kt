package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d

interface IntakeIO {

    fun setSpinPower(power: Double) {

    }
    fun setCenterPower(power: Double) {

    }
    fun setAngle(angle: Rotation2d) {

    }
}
