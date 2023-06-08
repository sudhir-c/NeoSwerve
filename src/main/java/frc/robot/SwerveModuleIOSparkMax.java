package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {


    //Wheel rotations per one motor rotation
    private static final double DRIVE_GEAR_RATIO = (48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double STEER_GEAR_RATIO = (150.0) / (7.0);

    private static final double WHEEL_RADIUS = Units.inchesToMeters(1.95);

    private static final double DRIVE_COEFFICIENT = (2 * Math.PI * WHEEL_RADIUS) / (DRIVE_GEAR_RATIO);
    private static final double STEER_COEFFICIENT = (2 * Math.PI) / (STEER_GEAR_RATIO);

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final CANCoder canCoder;


    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private SparkMaxPIDController turnPID;


    private final double MAX_VELOCITY = 15.0;


    public SwerveModuleIOSparkMax(int driveId, int steerId, int steerEncoderId, double steerOffsetRad) {
        driveMotor = new CANSparkMax(driveId, CANSparkMaxLowLevel.MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerId, CANSparkMaxLowLevel.MotorType.kBrushless);
        canCoder = new CANCoder(steerEncoderId);

        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.magnetOffsetDegrees = Units.radiansToDegrees(steerOffsetRad);
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoder.configAllSettings(canCoderConfig);

        turnPID = steerMotor.getPIDController();
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        turnPID.setPositionPIDWrappingMinInput(0);
        turnPID.setP(1.5);
        turnPID.setI(0.0);
        turnPID.setD(0.0); //0.1
        turnPID.setFF(0.0);

        driveEncoder.setPositionConversionFactor(DRIVE_COEFFICIENT);
        steerEncoder.setPositionConversionFactor(STEER_COEFFICIENT);

        driveEncoder.setPosition(0);
        steerEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition()) - steerOffsetRad);

        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveMotor.setSmartCurrentLimit(20);
        driveMotor.enableVoltageCompensation(10.0);

        steerMotor.setSmartCurrentLimit(20);
        steerMotor.enableVoltageCompensation(10.0);

        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        steerEncoder.setMeasurementPeriod(10);
        steerEncoder.setAverageDepth(2);

        driveMotor.setCANTimeout(0);
        steerMotor.setCANTimeout(0);

        driveMotor.burnFlash();
        steerMotor.burnFlash();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.steerPositionRad = canCoder.getPosition();
        inputs.canCoderAbsolutePosition = canCoder.getAbsolutePosition();
    }

    @Override
    public void setTargetDriveVelocity(double targetDriveVelocity) {
        driveMotor.set(targetDriveVelocity / MAX_VELOCITY);
    }

    @Override
    public void setTargetSteerPosition(double targetSteerPosition) {
        turnPID.setReference(targetSteerPosition, CANSparkMax.ControlType.kPosition);
    }
}
