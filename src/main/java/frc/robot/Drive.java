package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private static final double WHEELBASE = 13.5;
    private static final double TRACKWIDTH = 13.5;


    //m/s
    private final double maxVelocity;
    private final double maxAngularVelocity;

    private SwerveModuleState[] states = new SwerveModuleState[4];
    private final SwerveModule[] modules;

    private final SwerveDriveKinematics kinematics;

    private final SwerveModulePosition[] positions;

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    public Drive(
            SwerveModuleIO frontLeft,
            SwerveModuleIO frontRight,
            SwerveModuleIO backLeft,
            SwerveModuleIO backRight) {

        modules = new SwerveModule[] {
                new SwerveModule(frontLeft, "FrontLeft"),
                new SwerveModule(frontRight, "FrontRight"),
                new SwerveModule(backLeft, "BackLeft"),
                new SwerveModule(backRight, "BackRight")
        };



        maxVelocity = frontLeft.getMaxVelocity();
        maxAngularVelocity = maxVelocity / Math.hypot(WHEELBASE / 2, TRACKWIDTH / 2);

        positions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        var frontLeftLocation = new Translation2d(WHEELBASE / 2, TRACKWIDTH / 2);
        var frontRightLocation = new Translation2d(WHEELBASE / 2, -TRACKWIDTH/2);
        var backLeftLocation = new Translation2d(-WHEELBASE / 2 , TRACKWIDTH / 2);
        var backRightLocation = new Translation2d(-WHEELBASE / 2 , -TRACKWIDTH / 2);

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    }

    @Override
    public void periodic() {
       gyroIO.updateInputs(gyroInputs);
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs();
        }

        SwerveModuleState[] optimized = new SwerveModuleState[4];
        states = kinematics.toSwerveModuleStates(targetVelocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
        for (int i = 0; i < optimized.length; i++) {
            optimized[i] = SwerveModuleState.optimize(states[i], modules[i].getPosition().angle);
            modules[i].setTargetState(optimized[i]);
        }
    }

    public void setTargetVelocity(ChassisSpeeds target) {
        targetVelocity = target;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(gyroInputs.yaw);
    }

}
