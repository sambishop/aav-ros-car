#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CaptureModeBuilder.h"
#include "PlaybackModeBuilder.h"

static const char *HELP =
"Usage: cam_tracker MODE\n\
\n\
MODE must either equal \"capture\" or \"playback\".\n";

ModeBuilder *createModeBuilder(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "capture") == 0) {
        return new CaptureModeBuilder();
    } else if (argc == 2 && strcmp(argv[1], "playback") == 0) {
        return new PlaybackModeBuilder();
    }
    return NULL;
}

int main(int argc, char **argv)
{
    ModeBuilder *builder = createModeBuilder(argc, argv);
    if (!builder) {
        fprintf(stderr, "%s", HELP);
        exit(2);
    }
    printf("Hello, world!\n");
}

