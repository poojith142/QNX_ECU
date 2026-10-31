/*
 * ecu_sup.h - shared configuration, CAN wire format and module interfaces
 *
 * Modules
 *   rt_util.c      real-time helpers: error checks, clocks, priorities, pulse timers, trace marks
 *   ecu_service.c  the simulated ECU (child process): heartbeat + worker + fault injection
 *   supervisor.c   Health Monitor, Recovery Manager, Watchdog, CAN reader, status / stats
 *   main.c         command line interface and program entry
 */
#ifndef ECU_SUP_H
#define ECU_SUP_H

#include <stdint.h>
#include <sys/types.h>
#include <time.h>

/* ---- Timing policy (all in ms unless noted) ---- */
enum {
    NUM_SVC         = 3,       /* ENGINE, BRAKE, INFOTAINMENT                   */
    HEARTBEAT_MS    = 100,     /* ECU heartbeat period                          */
    MISSED_BEATS    = 3,       /* missed beats => FAILED                        */
    STALL_MS        = 500,     /* no worker progress => deadlock                */
    BUDGET_MS       = 50,      /* worker cycle above this => DEGRADED (overrun) */
    RECOVERY_TMO_MS = 2000,    /* a restart must produce a heartbeat by then    */
    WINDOW_MS       = 30000,   /* restart-limit window                          */
    MAX_RESTARTS    = 3,       /* per window, then latched FAILED               */
    WD_TICK_MS      = 50,      /* watchdog period                               */
    GRACE_MS        = 1000,    /* startup grace before heartbeats are required  */
    WORK_US         = 20000,   /* nominal worker cycle (us)                     */
    OVERRUN_US      = 120000   /* extra cycle time injected by FAULT_OVERRUN    */
};

/* ---- Scheduling policy (SCHED_RR priorities) ---- */
enum { PRIO_ECU = 10, PRIO_HEALTH = 40, PRIO_CAN = 45, PRIO_RECOVERY = 50, PRIO_WATCHDOG = 50 };

/* ---- Pulse codes: recovery-timeout pulse code = PULSE_RECOVERY_BASE + service id ---- */
enum { PULSE_TICK = 1, PULSE_RECOVERY_BASE = 10 };

/* ---- Faults an ECU can be told to simulate (carried in the SIGUSR1 value) ---- */
enum { FAULT_CLEAR, FAULT_HANG, FAULT_CRASH, FAULT_DEADLOCK, FAULT_OVERRUN, FAULT_COUNT };

#define EVENT_LOG_PATH "/tmp/ecu_events.log"

/* ---- CAN payload sent by every ECU heartbeat (8 bytes) ---- */
typedef struct __attribute__((packed)) {
    uint8_t id;          /* service id                                  */
    uint8_t seq;         /* heartbeat sequence number                   */
    uint8_t progress;    /* worker progress counter (stalls on deadlock) */
    uint8_t cycle_ms;    /* last worker cycle time, saturated at 255    */
    pid_t   pid;         /* sender process id (rejects stale frames)    */
} can_frame_t;

/* ---- rt_util.c ---- */
int     rt_check(int value, const char *what);            /* exit on -1, else return value  */
void    rt_check_errno(int rc, const char *what);         /* exit on non-zero errno result  */
int64_t rt_now_ns(void);
int64_t rt_now_ms(void);
void    rt_set_priority(int prio);                        /* SCHED_RR for the calling thread */
void    rt_lock_memory(void);                             /* mlockall: no page faults        */
void    rt_trace_mark(int slot, const char *text);        /* user event in the kernel trace  */
void    rt_start_thread(void *(*fn)(void *));
timer_t rt_make_pulse_timer(int chid, int code);          /* timer that sends a pulse        */
void    rt_timer_start(timer_t t, int ms, int periodic);

/* ---- ecu_service.c ---- */
int     ecu_run(int id);                                  /* child-process entry, never returns on success */

/* ---- supervisor.c ---- */
int     sup_start(const char *argv0);                     /* 0 on success, -1 if the log cannot open */
void    sup_stop(void);                                   /* print summary, kill all ECUs            */
void    sup_print_status(void);
void    sup_print_stats(void);
void    sup_reset(int id);                                /* clear restart limit and restart         */
int     sup_inject_named(const char *fault, int id);      /* 1 if "fault" is a known fault name      */

#endif /* ECU_SUP_H */
