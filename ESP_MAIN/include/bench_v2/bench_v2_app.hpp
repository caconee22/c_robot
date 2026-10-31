#pragma once
// Contract
// Role: V2 bench console, snapshots and low-output manual test orchestration.
// Called from: dedicated setup()/loop(); update keeps the V1 safety call order.
// Failure: existing SystemManager and MotorController own all safety decisions.
// UART telemetry is best-effort and never waits for transmit space.
namespace BenchV2App {
void begin();
void update();
}
