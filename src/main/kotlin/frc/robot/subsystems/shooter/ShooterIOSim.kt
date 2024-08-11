package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class ShooterIOSim : ShooterIO {
    override val topRollerInputs = LoggedRollerInputs()
    override val bottomRollerInputs = LoggedRollerInputs()
    private val topMotor = TalonFXSim(
        1,
        ShooterConstants.GEAR_RATIO_TOP,
        ShooterConstants.MOMENT_OF_INERTIA_TOP.`in`(
            Units.Kilograms.mult<Distance>(Units.Meters).mult(Units.Meters)
        ),
        1.0
    )

    private val bottomMotor = TalonFXSim(
        1,
        ShooterConstants.GEAR_RATIO_BOTTOM,
        ShooterConstants.MOMENT_OF_INERTIA_BOTTOM.`in`(
            Units.Kilograms.mult(Units.Meters).mult(Units.Meters)
        ),
        1.0
    )

    private val topControl = VelocityVoltage(0.0)
    private val bottomControl = VelocityVoltage(0.0)
    private val stop = DutyCycleOut(0.0)

    init {
        topMotor.setController(
            PIDController(
                ShooterConstants.TOP_kP.get(),
                ShooterConstants.TOP_kI.get(),
                ShooterConstants.TOP_kD.get()
            )
        )
        bottomMotor.setController(
            PIDController(
                ShooterConstants.BOTTOM_kP.get(),
                ShooterConstants.BOTTOM_kI.get(),
                ShooterConstants.BOTTOM_kD.get()
            )
        )
    }

    override fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {
        topMotor.setControl(topControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {
        bottomMotor.setControl(bottomControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        bottomMotor.setControl(stop)
        topMotor.setControl(stop)
    }

    override fun updateInputs() {
        topMotor.update(Timer.getFPGATimestamp())
        bottomMotor.update(Timer.getFPGATimestamp())

        topRollerInputs.velocity.mut_replace(
            topMotor.velocity, Units.RotationsPerSecond
        )

        bottomRollerInputs.velocity.mut_replace(
            bottomMotor.velocity, Units.RotationsPerSecond
        )
    }
}