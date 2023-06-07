package frc.robot;

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
    private static final double STEER_COEFFICIENT = (2 * Math.PI) / (DRIVE_GEAR_RATIO);

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;


    private SparkMaxPIDController builtinTurningPID;
    private PIDController turnPID;

    private final double absoluteOffsetRad;

    private final double MAX_VELOCITY = 10.0;


    public SwerveModuleIOSparkMax(int driveId, int steerId, double steerOffsetRad) {
        absoluteOffsetRad = steerOffsetRad;
        driveMotor = new CANSparkMax(driveId, CANSparkMaxLowLevel.MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerId, CANSparkMaxLowLevel.MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        turnPID = new PIDController(0.5,0,0);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        driveEncoder.setPositionConversionFactor(DRIVE_COEFFICIENT);
        steerEncoder.setPositionConversionFactor(STEER_COEFFICIENT);

        steerEncoder.setPosition(steerOffsetRad);

        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveMotor.setSmartCurrentLimit(20);
        driveMotor.enableVoltageCompensation(10.0);

        steerMotor.setSmartCurrentLimit(20);
        steerMotor.enableVoltageCompensation(10.0);

        driveEncoder.setPosition(0);
        steerEncoder.setPosition(0);

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
        inputs.steerPositionRad = steerEncoder.getPosition();
    }

    @Override
    public void setTargetDriveVelocity(double targetDriveVelocity) {
        driveMotor.set(targetDriveVelocity / MAX_VELOCITY);
    }

    @Override
    public void setTargetSteerPosition(double targetSteerPosition) {
        steerMotor.set(turnPID.calculate(steerEncoder.getPosition(), targetSteerPosition));
    }
}
