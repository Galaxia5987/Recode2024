package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.motors.TalonFXSim
import frc.robot.subsystems.hood.HoodConstants.ANGLE_KD
import frc.robot.subsystems.hood.HoodConstants.ANGLE_KI
import frc.robot.subsystems.hood.HoodConstants.ANGLE_KP
import frc.robot.subsystems.hood.HoodConstants.GEAR_RATIO
import frc.robot.subsystems.hood.HoodConstants.MOMENT_OF_INERTIA

class HoodIOSim : HoodIO {
    override var inputs: LoggedInputHood = LoggedInputHood()
    private val motor = TalonFXSim(
        1, GEAR_RATIO, MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
        GEAR_RATIO
    )
    private val angleControl = PositionVoltage(0.0)
    private val pidController = PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD)

    init {
        motor.setController(pidController)
    }

    override fun updateInputs() {
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.angleMotorVoltage = Units.Volt.of(motor.appliedVoltage)
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(angleControl.withPosition(angle))
    }
}