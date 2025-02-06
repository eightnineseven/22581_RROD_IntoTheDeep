package org.firstinspires.ftc.teamcode.aRROD.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.PathChain;

import com.pedropathing.follower.Follower;
public class FollowPathCommand extends CommandBase {

    public Follower follower;
    private  PathChain pathChain;
    private boolean holdEnd = true;

    public FollowPathCommand(Follower follower, PathChain pathChain) {

        this.follower = follower;
        this.pathChain = pathChain;
    }



    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position
     * @return This command for compatibility in command groups
     */
    public FollowPathCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain,1,true);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
