/* ************************************************************************** */
/** Protothread E.g. 1
 * Simple thread example
 */
/* ************************************************************************** */
// Done in config.h
//#define _SUPPRESS_PLIB_WARNING 1

#include "config.h"
#include "pt_cornell_1_2_1.h"

// Done in pt_cornell_1_2_1.h
//#include <stdio.h>

static int protothread1_flag, protothread2_flag;

static int protothread1(struct pt *pt)
{
    PT_BEGIN(pt);
    while (1) {
        PT_WAIT_UNTIL(pt, protothread2_flag != 0); // Wait for thread 2 to set it's flag
        printf("Protothread 1 running/n");
        protothread1_flag = 1; // Let the other protothread run
        protothread2_flag = 0;
    }
    PT_END(pt);
}

static int protothread2(struct pt *pt)
{
    PT_BEGIN(pt);
    while (1) {
        protothread2_flag = 1; // Let the other protothread run
        PT_WAIT_UNTIL(pt, protothread1_flag != 0);
        printf("Protothread 2 running\n");
        protothread1_flag = 0; // reset thread1 flag 
    }
    PT_END(pt);
}

static struct pt pt1, pt2; // Protothread data structs

int main(void)
{
    PT_INIT(&pt1);
    PT_INIT(&pt2);
    while (1) {
        protothread1(&pt1);
        protothread2(&pt2);
    }
}
 