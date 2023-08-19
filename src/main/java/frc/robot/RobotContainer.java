package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Drive drive;
    private final XboxController controller = new XboxController(0);

    public RobotContainer() {
        drive = new Drive(new GyroNavX(), new SwerveModuleIOSparkMax(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET), new SwerveModuleIOSparkMax(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET), new SwerveModuleIOSparkMax(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET), new SwerveModuleIOSparkMax(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET));
        drive.setDefaultCommand(new DefaultDrive(drive,
                () -> -adjustJoystickValue(controller.getLeftY() * drive.getMaxVelocity()),
                () -> -adjustJoystickValue(controller.getLeftX() * drive.getMaxVelocity()),
                () -> -adjustJoystickValue(controller.getRightX() * drive.getMaxAngularVelocity()),
                controller::getRightStickButton
                ));
        configureBindings();
    }

    private static double adjustJoystickValue(double v) {
        v = MathUtil.applyDeadband(v, 0.3);
        v = Math.copySign(v * v, v);
        return v;
    }

    private void configureBindings() {
        new Trigger(controller::getAButton).onTrue(new InstantCommand(() -> drive.zeroYaw()));
    }
}
