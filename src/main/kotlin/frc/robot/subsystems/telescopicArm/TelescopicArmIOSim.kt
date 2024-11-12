package frc.robot.subsystems.telescopicArm

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class TelescopicArmIOSim : TelescopicArmIO {
    override var inputs: LoggedTLArmInputs = LoggedTLArmInputs()
    private var controlRequest: PositionVoltage = PositionVoltage(0.0)


    private var motor = TalonFXSim(
        1,
        TelescopicArmConstants.GEAR_RATIO,
        TelescopicArmConstants.MOMENT_OF_INERTIA,
        TelescopicArmConstants.CONVERSION_FACTOR * TelescopicArmConstants.DRUM_RADIUS.`in`(Units.Meters)
    )
    private var positionControler: PIDController =
        PIDController(TelescopicArmConstants.KP, TelescopicArmConstants.KI, TelescopicArmConstants.KD)

    init {
        motor.setController(positionControler)
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.currentPose = Units.Centimeter.of(motor.position)
    }

    override fun setHeight(distance: Measure<Distance>) {
        motor.setControl(controlRequest.withPosition(distance.`in`(Units.Meters)))
    }
}