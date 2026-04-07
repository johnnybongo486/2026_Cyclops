# Team 6329 Robot Code

## Stack
- Java, WPILib (command-based)
- Swerve: CTRE Swerve (Phoenix 6, generated via Tuner X)
- Motors/encoders: TalonFX + CANcoder (Phoenix 6)
- Logging: CTRE SignalLogger (Hoot files)
- Deploy target: roboRIO 2.0

## Conventions
- Subsystems in `src/main/java/frc/robot/subsystems/`
- Swerve generated code in `src/main/java/frc/robot/generated/` — DO NOT 
  hand-edit; regenerate from Tuner X instead
- Constants in `Constants.java`, grouped by subsystem
- Units: meters, radians, seconds. Phoenix 6 uses rotations natively — 
  always be explicit about conversions
- Prefer Phoenix 6 StatusSignal + BaseStatusSignal.refreshAll() over 
  individual getter calls in periodic()
- Use Phoenix 6 control requests (VelocityVoltage, MotionMagicVoltage, 
  etc.) — not the legacy set() API

## What I care about
- Correctness first, then performance, then style
- Flag unit mismatches (rotations vs radians vs meters) aggressively
- When tuning, explain the *why* of gain changes, not just new values
- Prefer Tuner X SysId routines and the resulting gains over hand-tuning
- Watch for CAN bus utilization — we're on a single CANivore

## What NOT to do
- Don't hand-edit generated swerve code
- Don't mix Phoenix 5 and Phoenix 6 APIs
- Don't introduce new dependencies without asking
- Don't change Constants without showing me the diff and reasoning

## Logging
- SignalLogger writes Hoot files to USB at /media/sda1/
- Event markers go through frc.robot.util.MatchLog — don't call 
  SignalLogger.writeString directly for events
- Every log has metadata/gitSha — use it when reporting bugs