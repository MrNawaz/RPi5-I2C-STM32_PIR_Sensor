// read_stm32_on_RPi5.c
// Minimal I2C reader for STM32 at 0x27
// Prints only "Person at desk" or "No one at desk" with timestamp
// Low read frequency (~ every 20 seconds)

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>

#define STM32_ADDR     0x27
#define I2C_BUS        "/dev/i2c-1"
#define RETRY_DELAY_US 50000     // 50 ms between retries
#define POLL_DELAY_US  20000000  // 20 seconds between reads
#define MAX_RETRIES    3

static void print_timestamp(void) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    printf("[%02d:%02d:%02d] ", t->tm_hour, t->tm_min, t->tm_sec);
}

int main(void) {
    int fd = open(I2C_BUS, O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, STM32_ADDR) < 0) {
        perror("Failed to set I2C slave address");
        close(fd);
        return 1;
    }

    printf("Reading STM32 I2C data every 20 seconds...\n");

    while (1) {
        uint8_t buf[3] = {0};
        ssize_t r;
        int tries = 0;

        while (tries < MAX_RETRIES) {
            r = read(fd, buf, sizeof(buf));
            if (r == sizeof(buf)) break;   // success
            if (r < 0 && (errno == EREMOTEIO || errno == 121)) {
                tries++;
                usleep(RETRY_DELAY_US);
                continue;
            } else {
                perror("Read failed");
                break;
            }
        }

        if (r == sizeof(buf)) {
            uint8_t motion = buf[0];
            print_timestamp();
            if (motion)
                printf("👤 Person at desk\n");
            else
                printf("❌ No one at desk\n");
        } else {
            print_timestamp();
            printf("⚠️  STM32 not ready\n");
        }

        usleep(POLL_DELAY_US); // wait before next read
    }

    close(fd);
    return 0;
}
