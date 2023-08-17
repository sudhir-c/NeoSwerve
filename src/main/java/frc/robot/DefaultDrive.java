package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final Drive drive;

    private final double ROTATION_COEFFICIENT = 0.6;

    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier theta;
    private final BooleanSupplier joystick;


    public DefaultDrive(Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier joystick) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.joystick = joystick;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double xV;
        double yV;
        double thetaV;

        xV = x.getAsDouble();
        yV = y.getAsDouble();
        thetaV = theta.getAsDouble();
        ChassisSpeeds target = ChassisSpeeds.fromFieldRelativeSpeeds(xV, yV, thetaV, drive.getRotation());
        Logger.getInstance().recordOutput("Drive/TargetXVelocity", xV);
        Logger.getInstance().recordOutput("Drive/TargetYVelocity", yV);
        Logger.getInstance().recordOutput("Drive/TargetThetaVelocity", thetaV);
        //ChassisSpeeds target = new ChassisSpeeds(xV, yV, thetaV);
        drive.setTargetVelocity(target);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setTargetVelocity(new ChassisSpeeds());
    }


}
