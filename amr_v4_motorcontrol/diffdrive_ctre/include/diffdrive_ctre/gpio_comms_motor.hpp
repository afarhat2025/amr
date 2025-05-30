#ifndef DIFFDRIVE_GPIO_COMMS_motor_DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_GPIO_COMMS_motor_DIFFBOT_SYSTEM_HPP_

#include <gpiod.h>
#include <error.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <unistd.h>

// #include <jetgpio.h>
// #include <iostream>


class gpio_comms
{
public:
    struct gpiod_chip *chip;
    struct gpiod_line *line_85;
    struct gpiod_line_request_config config_85;

    int estop;

    void init()
    {

        chip = gpiod_chip_open("/dev/gpiochip0");
        if (!chip)
        {
            perror("Error opening GPIO chip");
            exit(EXIT_FAILURE);
        }
        
        line_85 = gpiod_chip_get_line(chip, 38);
        if (!line_85)
        {
            perror("Error getting GPIO line (offset 98)");
            gpiod_chip_close(chip);
            exit(EXIT_FAILURE);
        }

        memset(&config_85, 0, sizeof(config_85));
        config_85.consumer = "gpio_comms_85";
        config_85.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
        config_85.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN ;

        
        if (gpiod_line_request(line_85, &config_85, GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW) != 0)
        {
            perror("Error requesting GPIO line (offset 85)");
            gpiod_chip_close(chip);
            exit(EXIT_FAILURE);
        }

    }

    void readgpio()
    {
        int value_98 = gpiod_line_get_value(line_85);
        if (value_98 < 0)
        {
            perror("Error reading GPIO value (offset 98)");
            gpiod_chip_close(chip);
            exit(EXIT_FAILURE);
        }
        estop = value_98;

    }

    void cleanup()
    {
        gpiod_line_release(line_85);
        gpiod_chip_close(chip);
    }
};
#endif  // DIFFDRIVE_GPIO_COMMS_motor_DIFFBOT_SYSTEM_HPP_