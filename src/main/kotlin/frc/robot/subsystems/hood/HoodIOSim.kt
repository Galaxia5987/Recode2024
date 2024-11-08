package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class HoodIOSim : HoodIO {
    override val inputs = LoggedHoodInputs()
    private val motor: TalonFXSim =
        TalonFXSim(
            1,
            GEAR_RATIO,
            MOMENT_OF_INERTIA.`in`(Units.Kilogram.mult(Units.Meters).mult(Units.Meters)),
            GEAR_RATIO
        )

    private val control = MotionMagicDutyCycle(0.0)
    private val dutyCycleOut = DutyCycleOut(0.0)

    init {
        motor.setProfiledController(
            ProfiledPIDController(
                GAINS.kP,
                GAINS.kI,
                GAINS.kD,
                TrapezoidProfile.Constraints(
                    MAX_VELOCITY, MAX_ACCELERATION
                )
            )
        )
    }

    override fun setAngle(angle: Measure<Angle>) {
        motor.setControl(control.withPosition(angle.`in`(Units.Rotations)))
    }

    override fun setGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double, kG: Double) {
        motor.setController(PIDController(kP, kI, kD))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())

        inputs.internalAngle.mut_replace(motor.position, Units.Rotations)
        inputs.voltage.mut_replace(motor.appliedVoltage, Units.Volts)
    }
}