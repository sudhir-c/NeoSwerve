package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final String name;
    private final SwerveModuleIO.SwerveModuleIOInputs inputs = new SwerveModuleIO.SwerveModuleIOInputs();

    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void setTargetState(SwerveModuleState state) {

    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

}
