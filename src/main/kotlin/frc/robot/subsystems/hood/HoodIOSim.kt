package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class HoodIOSim : HoodIO {
    private val motor: TalonFXSim = TalonFXSim(
        1,
        HoodConstants.GEAR_RATIO,
        HoodConstants.MOMENT_OF_INERTIA.`in`(Units.Kilogram.mult(Units.Meters).mult(Units.Meters)),
        HoodConstants.GEAR_RATIO
    )

    private val control = MotionMagicDutyCycle(0.0)
    private val dutyCycleOut = DutyCycleOut(0.0)

    init {
        motor.setProfiledController(
            ProfiledPIDController(
                HoodConstants.kP.get(),
                HoodConstants.kI.get(),
                HoodConstants.kD.get(),
                TrapezoidProfile.Constraints(
                    HoodConstants.MAX_VELOCITY, HoodConstants.MAX_ACCELERATION
                )
            )
        )
    }

    override fun setAngle(angle: MutableMeasure<Angle>) {
        motor.setControl(control.withPosition(angle.`in`(Units.Rotations)))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())

        inputs.internalAngle.mut_replace(motor.position, Units.Rotations)
        inputs.voltage.mut_replace(motor.appliedVoltage, Units.Volts)
    }
}