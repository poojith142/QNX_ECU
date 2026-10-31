/*
 * main.c - program entry and command line interface
 *
 *   ./ecu_sup          interactive supervisor
 *   ./ecu_sup auto     scripted fault-injection demo
 *   ./ecu_sup ecu <n>  internal: run ECU service n (started by the supervisor)
 *
 * Prereq: ~/start_can.sh (CAN drivers up)
 */
#include "ecu_sup.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define COMMAND_LIST "status | stats | log | hang|crash|deadlock|overrun|clear <0-2> | reset <0-2> | quit"

/* Execute one command line; returns 1 on "quit" */
static int run_command(const char *line)
{
    char cmd[16];
    int id = -1, n = sscanf(line, "%15s %d", cmd, &id);

    if (n < 1)
        return 0;
    if (!strcmp(cmd, "quit"))
        return 1;
    if (!strcmp(cmd, "status")) {
        sup_print_status();
    } else if (!strcmp(cmd, "stats")) {
        sup_print_stats();
    } else if (!strcmp(cmd, "log")) {
        (void)system("cat " EVENT_LOG_PATH);
    } else if (n == 2 && id >= 0 && id < NUM_SVC) {
        if (!strcmp(cmd, "reset"))
            sup_reset(id);
        else
            (void)sup_inject_named(cmd, id);
    } else {
        puts("commands: " COMMAND_LIST);
    }
    return 0;
}

/* Scripted demo: one fault class every 4 s, covering all detection paths */
static void run_demo(void)
{
    static const char *const script[] = {"hang 1", "crash 0", "deadlock 2", "overrun 1",
                                         "clear 1", "reset 0", "status"};
    for (size_t k = 0; k < sizeof script / sizeof script[0]; k++) {
        sleep(4);
        printf("> %s\n", script[k]);
        run_command(script[k]);
    }
    sleep(2);
}

int main(int argc, char **argv)
{
    char line[64];

    if (argc > 2 && !strcmp(argv[1], "ecu"))
        return ecu_run(atoi(argv[2]));                      /* child-process entry */

    setvbuf(stdout, NULL, _IOLBF, 0);
    if (sup_start(argv[0]))
        return 1;

    puts("ECU supervisor running. Commands: " COMMAND_LIST);
    if (argc > 1 && !strcmp(argv[1], "auto"))
        run_demo();
    else
        while (fgets(line, sizeof line, stdin) && !run_command(line))
            ;

    sup_stop();
    return 0;
}
