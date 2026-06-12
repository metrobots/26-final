## Possible Drivetrain Fixes
- [ ] If possible, zero the absolute analog encoders using Rev Hardware Client and a module alignment tool
- [ ] If you can not zero the encoders using an alignment tool, then use Rev Hardware Client to find what value the encoders read for any given position. They should be zeroed at some multiple of 45 degrees. You can also do from within Elastic by adding widgets for `[Front/Back] [Left/Right] Raw Angle` (the `SmartDashboard` calls are in the `periodic` method of `Drivetrain.java`).
- [ ] Use the `Field` widget in Elastic to see where the robot thinks it should be (with `isFieldRelative = true`). If possible, start the physical robot from that same position and see where the paths deviate.
- [ ] Check encoder offsets (I'm not sure how the ones in `Constants.java` were found)
- [ ] Check if turning off `isFieldRelative` changes how well the robot controls. Do this by changing the last argument in of `drivetrain.drive()` in `RobotContainer()` from `true` to `false`.

## Other
- [ ] Change all instances of `spark.set()` to `spark.setVoltage()` since `setVoltage()` has voltage compensation.
- [ ] Use absolute encoder value directly instead of syncing with relative encoder in `Module.java`.


## Notes

- The absolute analog encoders (based on what I found) should have outputs in the range [0.0, 1.0].
- Use `Constants.java` to find what the CAN IDs are for the turning motor controllers
- Make sure that in Rev Hardware Client, in the absolute encoder tab, you have analog encoder selected. I don't have access to the UI right now, so I can't give exact instructions for how to change that.
- I'm 98% sure the encoders on the swerve modules are [Thrifty Absolute Magnetic Encoders](https://www.thethriftybot.com/products/thrifty-absolute-magnetic-encoder), but make sure to check that.

> If in doubt, Google it. And don't call me while I'm off the clock. You know who you are. - The Big M
