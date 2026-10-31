/*
 * supervisor.c - ECU watchdog + recovery supervisor
 *
 *   ECU child x3 --CAN tx--> /dev/can1/rx0 --> CAN Reader (45)
 *                                                 | heartbeat / progress / cycle time
 *   50 ms pulse timer --> Watchdog (50) <---------+  (shared service table, mutex-guarded)
 *                           |  MsgSend(state change)      |  MsgSend(restart request)
 *                           v                             v
 *                   Health Monitor (40)           Recovery Manager (50)
 *                   (owns state + timeline)       (kill + posix_spawn, restart limit)
 *
 * Fault classes detected: crash, lost heartbeat, deadlock (no progress), overrun (cycle > budget).
 *
 * Real-time design notes
 *  - Health/Recovery channels use default QNX priority inheritance: a receiver is boosted to the
 *    priority of its sender, so a high-priority sender never waits behind a lower-priority thread.
 *  - The watchdog channel is FIXED_PRIORITY: timer pulses must not change the watchdog's priority.
 *  - Memory is locked (mlockall) so detection and recovery never take a page fault.
 *  - Every state change is also written to the QNX kernel trace (System Profiler user events).
 */
#include "ecu_sup.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/wait.h>
#include <unistd.h>

enum { TRACE_SLOT_INJECT = 8 };   /* state changes use trace slots 0..3 (= state_t) */

typedef enum { ST_OK, ST_DEGRADED, ST_FAILED, ST_RECOVERING } state_t;

static const char *const STATE_NAME[] = {"OK", "DEGRADED", "FAILED", "RECOVERING"};
static const char *const SVC_NAME[NUM_SVC] = {"ENGINE", "BRAKE", "INFOTAINMENT"};
static const char *const FAULT_NAME[FAULT_COUNT] = {"clear", "hang", "crash", "deadlock", "overrun"};

/* State-change request sent to the Health Monitor */
typedef struct { int id; state_t state; char msg[56]; } event_t;

/* Per-service record, shared between threads (guarded by svc_lock) */
typedef struct {
    pid_t   pid;
    state_t state;
    int     cycle_ms;                    /* last reported worker cycle time        */
    int     restarts, window_restarts;   /* lifetime / current-window restart count */
    uint8_t progress;                    /* last worker progress counter           */
    int64_t last_beat, last_progress;    /* time of last heartbeat / progress move */
    int64_t t_inject, t_fail, recovery_ms, window_start;
} svc_t;

/* Real-time evidence collected at run time (guarded by svc_lock) */
typedef struct {
    int64_t ticks, jitter_sum_ns, jitter_max_ns;        /* watchdog wake-up jitter  */
    int64_t detections, detect_sum_ms, detect_max_ms;   /* fault -> detection       */
    int64_t recovery_max_ms;                            /* worst restart-to-OK time */
} stats_t;

extern char **environ;
static svc_t           svc[NUM_SVC];
static stats_t         stats;
static pthread_mutex_t svc_lock = PTHREAD_MUTEX_INITIALIZER;
static int             health_ch, recovery_ch, watchdog_ch;   /* channels (owners)  */
static int             health_conn, recovery_conn;            /* connections (send) */
static timer_t         recovery_timer[NUM_SVC];
static _Atomic int     shutting_down;
static char            self_path[PATH_MAX];
static int64_t         t_start;
static FILE           *log_fp;

/* ======================= Helpers ======================= */

static long long stamp(void) { return (long long)(rt_now_ms() - t_start); }

/* Print to console and to the event log */
static void emit(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    fputs(buf, stdout);
    fputs(buf, log_fp);
    fflush(log_fp);
}

/* Synchronous state-change report to the Health Monitor */
static void report(int id, state_t st, const char *msg)
{
    event_t e = {.id = id, .state = st};
    snprintf(e.msg, sizeof e.msg, "%s", msg);
    MsgSend(health_conn, &e, sizeof e, NULL, 0);
}

static void request_restart(int id) { MsgSend(recovery_conn, &id, sizeof id, NULL, 0); }

/* Start one ECU child process; returns its pid, or -1 on failure (recovery timeout retries) */
static pid_t spawn_ecu(int id)
{
    char arg[8];
    char *argv[] = {self_path, "ecu", arg, NULL};
    pid_t pid = -1;
    int rc;
    snprintf(arg, sizeof arg, "%d", id);
    rc = posix_spawn(&pid, self_path, NULL, NULL, argv, environ);
    if (rc) {
        fprintf(stderr, "spawn ecu %d: %s\n", id, strerror(rc));
        return -1;
    }
    return pid;
}

/* ======================= Health Monitor (prio 40) =======================
 * Single owner of service state; writes the failure timeline. */

static void *health_monitor(void *arg)
{
    event_t e;
    (void)arg;
    rt_set_priority(PRIO_HEALTH);
    for (;;) {
        int rcvid = MsgReceive(health_ch, &e, sizeof e, NULL);
        char note[80] = "", tag[64];
        int64_t t = rt_now_ms();
        svc_t *s;

        if (rcvid <= 0)
            continue;
        if (e.id < 0 || e.id >= NUM_SVC) {
            MsgError(rcvid, EINVAL);
            continue;
        }
        s = &svc[e.id];

        pthread_mutex_lock(&svc_lock);
        if (s->t_inject && (e.state == ST_FAILED || e.state == ST_DEGRADED)) {
            int64_t latency = t - s->t_inject;
            snprintf(note, sizeof note, " [detected %lldms after fault]", (long long)latency);
            if (s->state == ST_OK || s->state == ST_DEGRADED) {      /* first detection only */
                stats.detections++;
                stats.detect_sum_ms += latency;
                if (latency > stats.detect_max_ms)
                    stats.detect_max_ms = latency;
            }
        }
        if ((e.state == ST_FAILED || e.state == ST_RECOVERING) &&
            (s->state == ST_OK || s->state == ST_DEGRADED)) {
            s->t_fail = t;
            if (!s->t_inject)
                s->t_inject = t;
        }
        if (e.state == ST_OK && s->state == ST_RECOVERING) {
            s->recovery_ms = t - s->t_fail;
            if (s->recovery_ms > stats.recovery_max_ms)
                stats.recovery_max_ms = s->recovery_ms;
            snprintf(note, sizeof note, " [recovery %lldms, outage %lldms]",
                     (long long)s->recovery_ms, (long long)(t - s->t_inject));
        }
        if (e.state == ST_OK)
            s->t_inject = 0;
        s->state = e.state;
        pthread_mutex_unlock(&svc_lock);

        /* LED hook: drive the GPIO LEDs for service e.id from state e.state here */
        snprintf(tag, sizeof tag, "%s %s", SVC_NAME[e.id], STATE_NAME[e.state]);
        rt_trace_mark(e.state, tag);
        emit("[%6lld ms] %-12s %-10s %s%s\n", stamp(), SVC_NAME[e.id], STATE_NAME[e.state], e.msg, note);
        MsgReply(rcvid, 0, NULL, 0);
    }
    return NULL;
}

/* ======================= Recovery Manager (prio 50) =======================
 * Kill + restart, bounded by MAX_RESTARTS per WINDOW_MS. */

static void restart_service(int id)
{
    svc_t *s = &svc[id];
    int64_t t = rt_now_ms();
    pid_t old, fresh;
    int give_up;

    if (atomic_load(&shutting_down))
        return;

    pthread_mutex_lock(&svc_lock);
    old = s->pid;
    if (t - s->window_start > WINDOW_MS) {
        s->window_start = t;
        s->window_restarts = 0;
    }
    give_up = ++s->window_restarts > MAX_RESTARTS;
    pthread_mutex_unlock(&svc_lock);

    if (give_up) {
        report(id, ST_FAILED, "restart limit reached - latched (use: reset <id>)");
        return;
    }
    report(id, ST_RECOVERING, "restarting service");

    if (old > 0) {
        kill(old, SIGKILL);
        waitpid(old, NULL, 0);
    }
    fresh = spawn_ecu(id);
    t = rt_now_ms();
    pthread_mutex_lock(&svc_lock);
    s->pid = fresh;
    s->last_beat = s->last_progress = t;
    s->restarts++;
    pthread_mutex_unlock(&svc_lock);
    rt_timer_start(recovery_timer[id], RECOVERY_TMO_MS, 0);   /* one-shot recovery timeout */
}

static void *recovery_manager(void *arg)
{
    int id;
    (void)arg;
    rt_set_priority(PRIO_RECOVERY);
    for (;;) {
        int rcvid = MsgReceive(recovery_ch, &id, sizeof id, NULL);
        if (rcvid <= 0)
            continue;
        MsgReply(rcvid, 0, NULL, 0);      /* release the requester, then do the slow work */
        if (id >= 0 && id < NUM_SVC)
            restart_service(id);
    }
    return NULL;
}

/* ======================= Watchdog (prio 50) ======================= */

/* Decide a monitored service's next state. Caller holds svc_lock. */
static state_t assess(svc_t *s, int64_t t, char *msg, size_t n)
{
    if (s->state != ST_OK && s->state != ST_DEGRADED)
        return s->state;

    if (s->pid > 0 && waitpid(s->pid, NULL, WNOHANG) > 0) {
        s->pid = -1;
        snprintf(msg, n, "process exited (crash)");
        return ST_FAILED;
    }
    if (t - s->last_beat >= MISSED_BEATS * HEARTBEAT_MS) {
        snprintf(msg, n, "heartbeat lost for %lldms", (long long)(t - s->last_beat));
        return ST_FAILED;
    }
    if (t - s->last_progress > STALL_MS) {
        snprintf(msg, n, "deadlock: no progress for %lldms", (long long)(t - s->last_progress));
        return ST_FAILED;
    }
    if (s->state == ST_OK && s->cycle_ms > BUDGET_MS) {
        snprintf(msg, n, "overrun: cycle %dms > %dms", s->cycle_ms, BUDGET_MS);
        return ST_DEGRADED;
    }
    if (s->state == ST_DEGRADED && s->cycle_ms <= BUDGET_MS) {
        snprintf(msg, n, "overrun cleared");
        return ST_OK;
    }
    return s->state;
}

/* Measure how far each watchdog wake-up deviates from the nominal tick period */
static void record_tick(int64_t t_ns)
{
    static int64_t last_ns;
    if (last_ns) {
        int64_t jitter = llabs((t_ns - last_ns) - WD_TICK_MS * 1000000LL);
        pthread_mutex_lock(&svc_lock);
        stats.ticks++;
        stats.jitter_sum_ns += jitter;
        if (jitter > stats.jitter_max_ns)
            stats.jitter_max_ns = jitter;
        pthread_mutex_unlock(&svc_lock);
    }
    last_ns = t_ns;
}

static void *watchdog(void *arg)
{
    struct _pulse pulse;
    (void)arg;
    rt_set_priority(PRIO_WATCHDOG);
    for (;;) {
        int64_t t_ns, t;
        MsgReceivePulse(watchdog_ch, &pulse, sizeof pulse, NULL);
        t_ns = rt_now_ns();
        t    = t_ns / 1000000;

        if (pulse.code >= PULSE_RECOVERY_BASE) {            /* recovery-timeout pulse */
            int id = pulse.code - PULSE_RECOVERY_BASE, stuck;
            pthread_mutex_lock(&svc_lock);
            stuck = svc[id].state == ST_RECOVERING;
            pthread_mutex_unlock(&svc_lock);
            if (stuck) {
                report(id, ST_FAILED, "recovery timeout");
                request_restart(id);
            }
            continue;
        }
        record_tick(t_ns);
        for (int id = 0; id < NUM_SVC; id++) {              /* periodic tick */
            char msg[56] = "";
            state_t was, next;
            pthread_mutex_lock(&svc_lock);
            was  = svc[id].state;
            next = assess(&svc[id], t, msg, sizeof msg);
            pthread_mutex_unlock(&svc_lock);
            if (next != was) {
                report(id, next, msg);
                if (next == ST_FAILED)
                    request_restart(id);
            }
        }
    }
    return NULL;
}

/* ======================= CAN reader (prio 45) ======================= */

static void *can_reader(void *arg)
{
    int fd = open("/dev/can1/rx0", O_RDONLY);
    (void)arg;
    rt_set_priority(PRIO_CAN);
    if (fd < 0) {
        perror("open rx0");
        exit(1);
    }
    for (;;) {
        uint8_t raw[16];
        can_frame_t f;
        ssize_t n = read(fd, raw, sizeof raw);
        int64_t t = rt_now_ms();
        int recovered = 0;

        if (n < (ssize_t)sizeof f)
            continue;
        memcpy(&f, raw, sizeof f);
        if (f.id >= NUM_SVC)
            continue;

        pthread_mutex_lock(&svc_lock);
        svc_t *s = &svc[f.id];
        if (f.pid == s->pid) {                  /* frames from an old process are ignored */
            s->last_beat = t;
            s->cycle_ms  = f.cycle_ms;
            if (f.progress != s->progress) {
                s->progress = f.progress;
                s->last_progress = t;
            }
            recovered = s->state == ST_RECOVERING;
        }
        pthread_mutex_unlock(&svc_lock);
        if (recovered)
            report(f.id, ST_OK, "heartbeat from new process");
    }
    return NULL;
}

/* ======================= Public interface ======================= */

static void inject(int id, int fault)
{
    pid_t pid;
    char tag[64];
    pthread_mutex_lock(&svc_lock);
    pid = svc[id].pid;
    if (fault != FAULT_CLEAR)
        svc[id].t_inject = rt_now_ms();
    pthread_mutex_unlock(&svc_lock);
    snprintf(tag, sizeof tag, "inject %s %s", FAULT_NAME[fault], SVC_NAME[id]);
    rt_trace_mark(TRACE_SLOT_INJECT, tag);
    emit("[%6lld ms] >> inject %s into %s\n", stamp(), FAULT_NAME[fault], SVC_NAME[id]);
    if (pid > 0)
        sigqueue(pid, SIGUSR1, (union sigval){.sival_int = fault});
}

int sup_inject_named(const char *fault, int id)
{
    if (id < 0 || id >= NUM_SVC)
        return 0;
    for (int f = 0; f < FAULT_COUNT; f++) {
        if (!strcmp(fault, FAULT_NAME[f])) {
            inject(id, f);
            return 1;
        }
    }
    return 0;
}

void sup_reset(int id)
{
    pthread_mutex_lock(&svc_lock);
    svc[id].window_restarts = 0;
    svc[id].t_inject = rt_now_ms();         /* outage is counted from the reset */
    pthread_mutex_unlock(&svc_lock);
    request_restart(id);
}

void sup_print_status(void)
{
    pthread_mutex_lock(&svc_lock);
    for (int i = 0; i < NUM_SVC; i++)
        printf("  %-12s %-10s pid=%-8d restarts=%d last_recovery=%lldms cycle=%dms\n",
               SVC_NAME[i], STATE_NAME[svc[i].state], (int)svc[i].pid, svc[i].restarts,
               (long long)svc[i].recovery_ms, svc[i].cycle_ms);
    pthread_mutex_unlock(&svc_lock);
}

/* Real-time evidence: wake-up jitter, detection latency, recovery time, supervisor CPU load */
void sup_print_stats(void)
{
    struct timespec cpu;
    double cpu_ms, wall_ms = (double)(rt_now_ms() - t_start);
    stats_t s;

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu);
    cpu_ms = cpu.tv_sec * 1000.0 + cpu.tv_nsec / 1e6;
    pthread_mutex_lock(&svc_lock);
    s = stats;
    pthread_mutex_unlock(&svc_lock);

    printf("  watchdog tick %dms : n=%lld avg jitter=%.3fms max jitter=%.3fms\n", WD_TICK_MS,
           (long long)s.ticks, s.ticks ? s.jitter_sum_ns / 1e6 / s.ticks : 0.0, s.jitter_max_ns / 1e6);
    printf("  detection latency  : n=%lld avg=%lldms worst=%lldms\n", (long long)s.detections,
           (long long)(s.detections ? s.detect_sum_ms / s.detections : 0), (long long)s.detect_max_ms);
    printf("  worst recovery     : %lldms\n", (long long)s.recovery_max_ms);
    printf("  supervisor CPU     : %.2f%% (%.0fms CPU in %.0fms)\n",
           wall_ms > 0 ? 100.0 * cpu_ms / wall_ms : 0.0, cpu_ms, wall_ms);
}

/* Create IPC objects, start the ECUs and the four supervisor threads */
int sup_start(const char *argv0)
{
    if (!_cmdname(self_path))
        snprintf(self_path, sizeof self_path, "%s", argv0);
    t_start = rt_now_ms();
    log_fp  = fopen(EVENT_LOG_PATH, "w");
    if (!log_fp) {
        perror("log");
        return -1;
    }
    rt_lock_memory();

    /* Health/Recovery: default priority inheritance. Watchdog: fixed, pulses must not re-prioritise it. */
    health_ch     = rt_check(ChannelCreate(0), "ChannelCreate health");
    recovery_ch   = rt_check(ChannelCreate(0), "ChannelCreate recovery");
    watchdog_ch   = rt_check(ChannelCreate(_NTO_CHF_FIXED_PRIORITY), "ChannelCreate watchdog");
    health_conn   = rt_check(ConnectAttach(0, 0, health_ch, _NTO_SIDE_CHANNEL, 0), "ConnectAttach health");
    recovery_conn = rt_check(ConnectAttach(0, 0, recovery_ch, _NTO_SIDE_CHANNEL, 0), "ConnectAttach recovery");

    for (int i = 0; i < NUM_SVC; i++) {
        recovery_timer[i] = rt_make_pulse_timer(watchdog_ch, PULSE_RECOVERY_BASE + i);
        svc[i].pid = spawn_ecu(i);
        svc[i].last_beat = svc[i].last_progress = rt_now_ms() + GRACE_MS;
    }
    rt_start_thread(health_monitor);
    rt_start_thread(recovery_manager);
    rt_start_thread(can_reader);
    rt_start_thread(watchdog);
    rt_timer_start(rt_make_pulse_timer(watchdog_ch, PULSE_TICK), WD_TICK_MS, 1);
    return 0;
}

/* Stop supervising: print the summary and kill every ECU child */
void sup_stop(void)
{
    atomic_store(&shutting_down, 1);
    sup_print_status();
    sup_print_stats();
    for (int i = 0; i < NUM_SVC; i++) {
        pid_t pid;
        pthread_mutex_lock(&svc_lock);
        pid = svc[i].pid;
        pthread_mutex_unlock(&svc_lock);
        if (pid > 0)
            kill(pid, SIGKILL);
    }
}
