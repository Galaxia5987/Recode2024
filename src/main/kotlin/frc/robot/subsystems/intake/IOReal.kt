package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import frc.robot.Ports
import org.team9432.annotation.Logged

class IOReal : IntakeIO {

    override val inputs = LoggedIntakeInputs()
    private val motor = TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    private val spinMotor = TalonFX(Ports.Intake.SPIN_MOTOR_ID)
    private val centerMotor = CANSparkMax(Ports.Intake.CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val angleControl = PositionVoltage(0.0)

    init {

        centerMotor.restoreFactoryDefaults()
        centerMotor.setSmartCurrentLimit(40)
        centerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        centerMotor.inverted=true
        centerMotor.burnFlash()

        val spinConfig = TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
        }

        spinMotor.configurator.apply(spinConfig)

        val angleConfig = TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Feedback = FeedbackConfigs().apply {
                RotorToSensorRatio = 1.0
                SensorToMechanismRatio = IntakeConstants.GEAR_RATIO
            }
            Slot0 = Slot0Configs().apply {
                kP = IntakeConstants.kP
                kI = IntakeConstants.kI
                kD = IntakeConstants.kD

            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
        }


        motor.configurator.apply(angleConfig)

        motor.setPosition(126.2 / 360.0)
    }

    override fun setCenterPower(power: Double) {
        centerMotor.set(power)

    }

    override fun setAnglePower(power: Double) {
        spinMotor.set(power)
    }

    override fun setAngle(angle: Measure<Angle>) {
        motor.setControl(angleControl)
    }

    override fun reset() {
        motor.setPosition(0.0)
    }
}
