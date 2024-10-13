package frc.robot.subsystems.telescopicArm

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.Ports

class TelescopicArmIOReal : TelescopicArmIO {
    override var inputs = LoggedTLArmInputs()
    val motor: TalonFX = TalonFX(Ports.TLArm.TL_MOTOR_ID)
    var controlRequest: PositionVoltage = PositionVoltage(0.0)
    override fun updateInput() {
        inputs.currentPose =
            Units.Centimeter.of(motor.position.value * TelescopicArmConstants.dramRadius.`in`(Units.Centimeter))

    }

    override fun setDistance(distance: Measure<Distance>) {
        motor.setControl(
            controlRequest.withPosition(
                distance.`in`(Units.Meters) / TelescopicArmConstants.dramRadius.`in`(
                    Units.Meters
                )
            )
        )
    }
}