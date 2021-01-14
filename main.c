#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include "can.h"
#include "log.h"
#include "pinmap.h"
#include <stdint.h>
#include <inttypes.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#include "pinmap.h"
#include "cmd.h"
#include "log.h"
#include "uarts.h"
#include "adcs.h"
#include "inputs.h"
#include "outputs.h"
#include "uart_rings.h"
#include "vl6180x.h"
#include "hdc2080.h"
#include "data.h"
#include "cal.h"
#include "can.h"

port_n_pins_t can_tx = CAN_TX;
port_n_pins_t can_rx = CAN_RX;

static bool initialised_can = false;

#define PACKET_LENGTH 2

#define CAN_CONTROLLER CAN1

void can_bus_init()
{
    if(initialised_can) return;
    // Enable clock to the CAN peripheral
    rcc_periph_clock_enable(RCC_CAN1);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CANEN);

    // It looks like the GPIOs need to be set up before the CAN controller can work
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(can_tx.port, GPIO_AF9, GPIO_PUPD_PULLUP, can_tx.pins);
    gpio_mode_setup(can_rx.port, GPIO_AF9, GPIO_PUPD_PULLUP, can_rx.pins);

    // Reset the can peripheral
    can_reset(CAN1);

    // Initialize the can peripheral
    int initfail = can_init(
            CAN1, // The can ID

            // Time Triggered Communication Mode?
            // http://www.datamicro.ru/download/iCC_07_CANNetwork_with_Time_Trig_Communication.  pdf
            false, // No TTCM

            // Automatic bus-off management?
            // When the bus error counter hits 255, the CAN will automatically
            // remove itself from the bus. if ABOM is disabled, it won't
            // reconnect unless told to. If ABOM is enabled, it will recover the
            // bus after the recovery sequence.
            true, // Yes ABOM

            // Automatic wakeup mode?
            // 0: The Sleep mode is left on software request by clearing the SLEEP
            // bit of the CAN_MCR register.
            // 1: The Sleep mode is left automatically by hardware on CAN
            // message detection.
            false, // Do not wake up on message rx

            // No automatic retransmit?
            // If true, will not automatically attempt to re-transmit messages on
            // error
            false, // Do not auto-retry

            // Receive FIFO locked mode?
            // If the FIFO is in locked mode,
            //  once the FIFO is full NEW messages are discarded
            // If the FIFO is NOT in locked mode,
            //  once the FIFO is full OLD messages are discarded
            false, // Discard older messages over newer

            // Transmit FIFO priority?
            // This bit controls the transmission order when several mailboxes are
            // pending at the same time.
            // 0: Priority driven by the identifier of the message
            // 1: Priority driven by the request order (chronologically)
            false, // TX priority based on identifier

            //// Bit timing settings
            //// Assuming 48MHz base clock, 87.5% sample point, 500 kBit/s data rate
            //// http://www.bittiming.can-wiki.info/
            // Resync time quanta jump width
            CAN_BTR_SJW_1TQ, // 16,
            // Time segment 1 time quanta width
            CAN_BTR_TS1_3TQ, // 13,
            // Time segment 2 time quanta width
            CAN_BTR_TS2_4TQ, // 2,
            // Baudrate prescaler
            12,

            // Loopback mode
            // If set, CAN can transmit but not receive
            false,

            // Silent mode
            // If set, CAN can receive but not transmit
            false);

    if(initfail) {
        platform_raw_msg("failure in initialising CAN bus");
    }

    can_enable_irq(CAN_CONTROLLER, CAN_IER_FMPIE0 | CAN_IER_FMPIE1);
    nvic_enable_irq(NVIC_CAN1_SCE_IRQ);

    initialised_can = true;

}

void can_bus_send() {
    static int id = 1;
    uint8_t packet[PACKET_LENGTH];

    packet[0] = 5;
    can_transmit(CAN_CONTROLLER, id++, false, false, PACKET_LENGTH, packet);
}

int main(void) {

    rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_48MHZ]);
    uarts_setup();

    rcc_periph_clock_enable(PORT_TO_RCC(LED_PORT));
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_clear(LED_PORT, LED_PIN);

    systick_set_frequency(1, rcc_ahb_frequency);
    systick_counter_enable();

    log_init();
    platform_raw_msg("----start----");

    can_bus_init();

    return 0;
}
