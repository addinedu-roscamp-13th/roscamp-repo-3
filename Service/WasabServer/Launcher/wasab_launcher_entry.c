#include <stdio.h>
#include <unistd.h>

int main(void) {
    const char *launcher =
        "/home/ane/dev_ws/src/roscamp-repo-3/Service/WasabServer/Launcher/launch_wasab.sh";

    execl("/bin/bash", "bash", launcher, (char *)NULL);
    perror("WaSaB launcher failed");
    return 1;
}
