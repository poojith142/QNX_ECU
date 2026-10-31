/*
 * ecu_service.c - one simulated ECU, run as a child process of the supervisor
 *
 *   main thread : 100 ms pulse timer -> sends a CAN heartbeat frame (unless "hung")
 *   worker      : fixed-period work loop that advances a progress counter
 *   faults      : delivered as SIGUSR1 + value (sigqueue) from the supervisor CLI
 */
#include "ecu_sup.h"

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <sys/neutrino.h>
#include <unistd.h>

static const char *const CAN_TX[NUM_SVC] = {"/dev/can0/tx2", "/dev/can0/tx3", "/dev/can0/tx4"};

static _Atomic int     g_fault;                    /* current simulated fault (FAULT_*) */
static _Atomic uint8_t g_progress, g_cycle_ms;     /* reported in every heartbeat       */
static pthread_mutex_t mutex_a = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_b = PTHREAD_MUTEX_INITIALIZER;

static void on_fault_signal(int sig, siginfo_t *info, void *ctx)
{
    (void)sig;
    (void)ctx;
    if (info->si_value.sival_int == FAULT_CRASH)
        _exit(1);
    atomic_store(&g_fault, info->si_value.sival_int);
}

/* Second half of the ABBA deadlock: takes B then A */
static void *deadlock_peer(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&mutex_b);
    usleep(50000);
    pthread_mutex_lock(&mutex_a);
    return NULL;
}

/* First half: takes A then B while the peer holds B and waits for A */
static void deadlock_self(void)
{
    pthread_t peer;
    rt_check_errno(pthread_create(&peer, NULL, deadlock_peer, NULL), "pthread_create");
    pthread_mutex_lock(&mutex_a);
    usleep(50000);
    pthread_mutex_lock(&mutex_b);
}

static void *worker(void *arg)
{
    (void)arg;
    for (;;) {
        int64_t t0 = rt_now_ms();
        int fault = atomic_load(&g_fault), cycle;
        if (fault == FAULT_DEADLOCK)
            deadlock_self();
        usleep(WORK_US + (fault == FAULT_OVERRUN ? OVERRUN_US : 0));
        cycle = (int)(rt_now_ms() - t0);
        atomic_store(&g_cycle_ms, cycle > 255 ? 255 : cycle);
        atomic_fetch_add(&g_progress, 1);
    }
    return NULL;
}

int ecu_run(int id)
{
    struct sigaction sa = {.sa_sigaction = on_fault_signal, .sa_flags = SA_SIGINFO};
    can_frame_t frame = {.id = id, .pid = getpid()};
    struct _pulse pulse;
    int chid = rt_check(ChannelCreate(0), "ChannelCreate");
    int fd;

    if (id < 0 || id >= NUM_SVC)
        return 1;
    fd = open(CAN_TX[id], O_WRONLY);
    if (fd < 0) {
        perror(CAN_TX[id]);
        return 1;
    }
    sigaction(SIGUSR1, &sa, NULL);
    rt_lock_memory();
    rt_set_priority(PRIO_ECU);
    rt_start_thread(worker);
    rt_timer_start(rt_make_pulse_timer(chid, PULSE_TICK), HEARTBEAT_MS, 1);

    for (;;) {
        MsgReceivePulse(chid, &pulse, sizeof pulse, NULL);
        if (atomic_load(&g_fault) == FAULT_HANG)
            continue;                       /* simulated hang: stay silent */
        frame.seq++;
        frame.progress = atomic_load(&g_progress);
        frame.cycle_ms = atomic_load(&g_cycle_ms);
        if (write(fd, &frame, sizeof frame) != (ssize_t)sizeof frame)
            perror("can write");            /* the supervisor will see the missing beat */
    }
}
