/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "sdkconfig.h"
#include "status_led.h"
#include <stdio.h>


void app_main(void) {
  printf("This app is built for running ons: " CONFIG_EXAMPLE_PRODUCT_NAME
         "\n");

  hello_led();

  /* This will call func() either from func_dev.c or func_prod.c, depending on
   * the build configuration.
   */
  printf("See README.md for building and running other app configurations.\n");
}
