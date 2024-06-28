package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units

object ClimbConstants {
    const val GEAR_RATIO = 12.0
    const val STOPPER_MOTOR_CURRENT_LIMIT = 20
    const val STOPPER_MOTOR_CURRENT_THRESHOLD = 15 //TODO: calibrate
    val MOTOR_CONFIG = TalonFXConfiguration()

    val OPEN_POSITION: MutableMeasure<Angle> = Units.Degree.of(40.0).mutableCopy()
    val CLOSED_POSITION: MutableMeasure<Angle> = Units.Degree.of(140.0).mutableCopy()

    fun initConstants(){
        MOTOR_CONFIG.withMotorOutput(
            MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        ).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(40.0)
            .withSupplyCurrentLimit(40.0)
    }
}