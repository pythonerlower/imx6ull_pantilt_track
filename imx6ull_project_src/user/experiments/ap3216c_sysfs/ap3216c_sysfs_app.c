#include <stdio.h>
#include <unistd.h>

int main(void)
{
    FILE *fp;
    int ir, als, ps;

    while (1) {
        fp = fopen("/sys/class/misc/ap3216c/ir", "r");
        if (fp) { fscanf(fp, "%d", &ir); fclose(fp); } else ir = -1;

        fp = fopen("/sys/class/misc/ap3216c/als", "r");
        if (fp) { fscanf(fp, "%d", &als); fclose(fp); } else als = -1;

        fp = fopen("/sys/class/misc/ap3216c/ps", "r");
        if (fp) { fscanf(fp, "%d", &ps); fclose(fp); } else ps = -1;

        printf("IR=%d  ALS=%d  PS=%d\n", ir, als, ps);
        fflush(stdout);
        sleep(1);
    }
    return 0;
}
