/*******************************************************************************
 *                                                                             *
 *  Filename    : test.c                                                       *
 *  Project     : Project Name                                                 *
 *  Version     : 1.0                                                          *
 *  Author      : Spiros Dontas                                                *
 *  Email       : spirosdontas@gmail.com                                       *
 *                                                                             *
 *  Description : ...                                                          *
 *                                                                             *
 *******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main () {

    int fd = open("/dev/lunix0-temp", O_RDONLY);

    char buff[100];

    read(fd, buff, 2);

    printf("%s\n", buff);

    read(fd, buff, 3);

    printf("%s\n", buff);

    return 0;

}
