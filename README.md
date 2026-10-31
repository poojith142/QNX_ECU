# ECU Watchdog + Recovery Supervisor (QNX 8.0, Raspberry Pi 4)

A fault-tolerant supervisor for three simulated ECUs (ENGINE, BRAKE, INFOTAINMENT) that talk over CAN.
It detects four fault classes, restarts the failed service, and records a timestamped failure timeline.
Built for the QNX India Hackathon.

## Architecture

| Thread / process | Priority | Job |
|---|---|---|
| ECU child x3 (`posix_spawn`) | 10 | Heartbeat over CAN (`/dev/can0/txN`) + worker thread |
| CAN Reader | 45 | Reads `/dev/can1/rx0`, updates heartbeat / progress / cycle time |
| Health Monitor | 40 | Single owner of service state, writes the event timeline |
| Recovery Manager | 50 | Kills and restarts a service, enforces the restart limit |
| Watchdog | 50 | 50 ms pulse timer; classifies each service |

Communication uses QNX message passing (`MsgSend` / `MsgReceive` / `MsgReply`) and pulses from `SIGEV_PULSE` timers.

## Fault classes

| Fault | Detected by | Result |
|---|---|---|
| Crash | `waitpid(WNOHANG)` sees the process exit | FAILED, restart |
| Hang | 3 missed heartbeats (300 ms) | FAILED, restart |
| Deadlock | Heartbeat alive but progress counter frozen for 500 ms | FAILED, restart |
| Overrun | Reported worker cycle above 50 ms | DEGRADED, clears when back in budget |

Recovery is limited to 3 restarts per 30 s window, then the service is latched FAILED until `reset <id>`.
A restart that produces no heartbeat within 2 s is reported as a recovery timeout and retried.

## Real-time design choices

- Default QNX priority inheritance on the Health and Recovery channels; fixed priority on the Watchdog channel.
- `mlockall()` so the detection and recovery path never page-faults.
- Frames from a previous process (stale pid) are ignored after a restart.
- Kernel trace user events for every state change and injected fault (visible in the System Profiler).
- Built-in measurements via the `stats` command: watchdog jitter, detection latency, worst recovery, supervisor CPU.

## Build and run

```bash
make                  # uses qcc, target gcc_ntoaarch64le (override: make TARGET=...)
~/start_can.sh        # bring up CAN drivers first
./ecu_sup             # interactive
./ecu_sup auto        # scripted demo
```

Commands: `status | stats | log | hang|crash|deadlock|overrun|clear <0-2> | reset <0-2> | quit`

## Results

| Metric | Measured |
|---|---|
| Watchdog max jitter | 0.016 ms (avg: 0.002 ms) |
| Crash detection latency | 36 ms |
| Hang detection latency | 255 ms |
| Overrun detection latency | 272 ms |
| Worst recovery time | 104 ms (outage time: 359 ms) |
| Supervisor CPU | 0.05% (141 ms CPU in 258,359 ms run) |

## Source layout

| File | Role |
|---|---|
| `ecu_sup.h` | Timing and priority policy, CAN frame format, interfaces between modules |
| `main.c` | Entry point, command line interface, scripted demo |
| `supervisor.c` | Health Monitor, Recovery Manager, Watchdog, CAN reader, status and stats |
| `ecu_service.c` | Simulated ECU (child process): heartbeat, worker, fault injection |
| `rt_util.c` | Error checks, clocks, priorities, pulse timers, `mlockall`, trace markers |
| `Makefile` | Build rules (`qcc`) |

One binary: the supervisor starts each ECU by re-running itself as `ecu_sup ecu <id>`.