# BLine autonomous path files

This is where BLine looks for path JSON by default (`new Path("name")` loads
`<name>.json` from this folder - see `frc.robot.lib.BLine.JsonUtils.PROJECT_ROOT`).

There is nothing here yet. Real paths get authored using the BLine Web editor and
exported into this folder, then referenced by name from `BLineAutoRoutines`
(`src/main/java/frc/robot/auto/BLineAutoRoutines.java`).

Loading a path file that doesn't exist throws at robot startup, so don't reference a
file here until it actually exists in this folder.
