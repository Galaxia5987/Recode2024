package frc.robot.subsystems.telescopicArm

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import frc.robot.lib.motors.TalonFXSim

class TelescopicArmIOSim : TelescopicArmIO {
    override var inputs: LoggedTLArmInputs = LoggedTLArmInputs()
    var controlRequest: PositionVoltage = PositionVoltage(0.0)
    private var motor = TalonFXSim(
        1,
        TelescopicArmConstants.GEAR_RATIO,
        TelescopicArmConstants.MOMENT_OF_INERTIA,
        TelescopicArmConstants.CONVERSION_FACTOR
    )
    private var positionControler: PIDController =
        PIDController(TelescopicArmConstants.KP, TelescopicArmConstants.KI, TelescopicArmConstants.KD)

    init {
        motor.setController(positionControler)
    }

    override fun updateInput() {
        TODO("Not yet implemented")
    }

    override fun setDistance(distance: Measure<Distance>) {
        val rotationToDistance = distance.`in`(Units.Meters) / TelescopicArmConstants.dramRadius.`in`(
            Units.Meters
        )
        motor.setControl(controlRequest.withPosition(rotationToDistance))
    }
}