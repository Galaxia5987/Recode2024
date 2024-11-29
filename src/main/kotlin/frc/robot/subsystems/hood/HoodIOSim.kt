package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim


class HoodIOSim : HoodIO {
    override var inputs: LoggedInputHood = LoggedInputHood()
    private val motor = TalonFXSim(
        1, HoodConstants.GEAR_RATIO, HoodConstants.MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
        HoodConstants.GEAR_RATIO
    )
    private val angleControl = PositionVoltage(0.0)
    private val pidController = PIDController(HoodConstants.ANGLE_KP, HoodConstants.ANGLE_KI, HoodConstants.ANGLE_KD)

    init {
        motor.setController(pidController)
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.angleMotorVoltage = Units.Volt.of(motor.appliedVoltage)
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(angleControl.withPosition(angle))
    }
}