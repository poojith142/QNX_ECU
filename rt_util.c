/*
 * rt_util.c - small real-time helpers shared by the supervisor and the ECU service
 */
#include "ecu_sup.h"

#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/trace.h>

/* Abort with a message when a call that returns -1 on error fails; returns the value otherwise */
int rt_check(int value, const char *what)
{
    if (value < 0) {
        perror(what);
        exit(EXIT_FAILURE);
    }
    return value;
}

/* Same, for calls that return an errno value directly (pthread_*, posix_spawn) */
void rt_check_errno(int rc, const char *what)
{
    if (rc) {
        fprintf(stderr, "%s: %s\n", what, strerror(rc));
        exit(EXIT_FAILURE);
    }
}

int64_t rt_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000000000LL + t.tv_nsec;
}

int64_t rt_now_ms(void)
{
    return rt_now_ns() / 1000000;
}

void rt_set_priority(int prio)
{
    struct sched_param sp = {.sched_priority = prio};
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sp))
        printf("warn: priority %d not set\n", prio);
}

/* Keep code and data resident: no page faults on the detection / recovery path */
void rt_lock_memory(void)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
        perror("warn: mlockall");
}

/* Insert a user event into the kernel trace (visible in the System Profiler) */
void rt_trace_mark(int slot, const char *text)
{
    TraceEvent(_NTO_TRACE_INSERTUSRSTREVENT, _NTO_TRACE_USERFIRST + slot, text);
}

void rt_start_thread(void *(*fn)(void *))
{
    pthread_t th;
    rt_check_errno(pthread_create(&th, NULL, fn, NULL), "pthread_create");
}

/* QNX timer that delivers a pulse (with the given code) to a channel */
timer_t rt_make_pulse_timer(int chid, int code)
{
    struct sigevent ev;
    timer_t t;
    int coid = rt_check(ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0), "ConnectAttach");
    SIGEV_PULSE_INIT(&ev, coid, SIGEV_PULSE_PRIO_INHERIT, code, 0);
    rt_check(timer_create(CLOCK_MONOTONIC, &ev, &t), "timer_create");
    return t;
}

void rt_timer_start(timer_t t, int ms, int periodic)
{
    struct timespec first = {ms / 1000, (ms % 1000) * 1000000L};
    struct itimerspec it  = {.it_value = first};
    if (periodic)
        it.it_interval = first;
    rt_check(timer_settime(t, 0, &it, NULL), "timer_settime");
}
