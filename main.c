/*
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

#define PORT_TO_RCC(_port_)   (RCC_GPIOA + ((_port_ - GPIO_PORT_A_BASE) / 0x400))
#define LED_PORT    GPIOB
#define LED_PIN     GPIO13
#define CAN_PORT    GPIOB
#define CAN_TX_PIN  GPIO9
#define CAN_RX_PIN  GPIO8


int main(void)
{
    rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
    rcc_periph_clock_enable(PORT_TO_RCC(LED_PORT));

    // LED GPIO setup
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);

    // Enable clock to the CAN peripheral
    rcc_periph_clock_enable(RCC_CAN1);

    // It looks like the GPIOs need to be set up before the CAN controller can work
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(CAN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CAN_TX_PIN | CAN_RX_PIN);
    gpio_set_af(CAN_PORT, GPIO_AF9, CAN_TX_PIN | CAN_RX_PIN);

    // Reset and initialise the can peripheral
    can_reset(CAN1);
    int initfail = can_init(
            CAN1,            // The can ID
            false,           // No TTCM
            true,            // Yes ABOM
            false,           // Do not wake up on message rx
            false,           // Do not auto-retry
            false,           // Discard older messages over newer
            false,           // TX priority based on identifier
            CAN_BTR_SJW_1TQ, // Resync time quanta jump width
            CAN_BTR_TS1_3TQ, // Time segment 1 time quanta width
            CAN_BTR_TS2_4TQ, // Time segment 2 time quanta width
            12,              // baudrate prescaler
            false,           // loopback mode
            false            // silent mode
            );

    // hang here if CAN initialisation fails
    while(initfail)
        ;

    static uint8_t data[8] = {0, 1, 2, 0, 0, 0, 0, 0};
    while (1) {
        data[0]++;
        if(can_transmit(CAN1,
                0,     /* (EX/ST)ID: CAN ID */
                false, /* IDE: CAN ID extended? */
                false, /* RTR: Request transmit? */
                8,     /* DLC: Data length */
                data) != -1) gpio_toggle(LED_PORT, LED_PIN);

        for (int i = 0; i < 1000000; i++) /* Wait a bit. */
            __asm__("nop");
    }

    return 0;
}
