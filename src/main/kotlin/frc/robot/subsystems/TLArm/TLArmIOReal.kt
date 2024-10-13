package frc.robot.subsystems.TLArm

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.Ports

class TLArmIOReal : TLArmIO {
    override var inputs = LoggedTLArmInputs()
    val motor: TalonFX = TalonFX(Ports.TLArm.TL_MOTOR_ID)
    override fun updateInput() {
        inputs.currentPose =
            Units.Centimeter.of(motor.position.value * TLArmConstants.dramRadius.`in`(Units.Centimeter))

    }

    override fun setPosition(setPoint: Measure<Distance>) {
        motor.setPosition(setPoint.`in`(Units.Meters) / TLArmConstants.dramRadius.`in`(Units.Meters))
    }
}