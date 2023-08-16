package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final String name;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void setTargetState(SwerveModuleState state) {
        double currentAngle = inputs.steerPositionRad;
        double targetAngle = MathUtil.inputModulus(state.angle.getRadians(), 0, 2 * Math.PI);
        double modulusAngle = MathUtil.inputModulus(currentAngle, 0, 2 * Math.PI);
        double errorAngle =     MathUtil.inputModulus(targetAngle - modulusAngle, -Math.PI, Math.PI);
        double resultAngle = currentAngle + errorAngle;

        io.setTargetSteerPosition(resultAngle);
        io.setTargetDriveVelocity(state.speedMetersPerSecond);

    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, Rotation2d.fromRadians(inputs.steerPositionRad));
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/" + name + "Module", inputs);
    }

}
