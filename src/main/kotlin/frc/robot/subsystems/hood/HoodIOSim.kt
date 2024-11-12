package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class HoodIOSim : HoodIO {
    override val inputs = LoggedHoodInputs()
    private val motor: TalonFXSim =
        TalonFXSim(
            1,
            HoodConstants.GEAR_RATIO,
            HoodConstants.MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
            HoodConstants.GEAR_RATIO
        )

    private val control = MotionMagicDutyCycle(0.0)
    private val dutyCycleOut = DutyCycleOut(0.0)

    init {
        motor.setProfiledController(
            ProfiledPIDController(
                HoodConstants.GAINS.kP,
                HoodConstants.GAINS.kI,
                HoodConstants.GAINS.kD,
                TrapezoidProfile.Constraints(
                    HoodConstants.MAX_VELOCITY, HoodConstants.MAX_ACCELERATION
                )
            )
        )
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(control.withPosition(angle.`in`(Units.Rotations)))
    }

    override fun setGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double, kG: Double) {
        motor.setController(PIDController(kP, kI, kD))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())

        inputs.internalAngle = Units.Rotations.of(motor.position)
        inputs.voltage = Units.Volts.of(motor.appliedVoltage)
    }
}