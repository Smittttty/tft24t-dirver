#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>

#include "io_util.h"
#include "intr_util.h"
#include "defines.h"
#include "nucleo_clock.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


#define MODE 	0

#define LCDLED 		GPIO5
#define LCDDC 		GPIO0
#define LCDCS 		GPIO1
#define LCDRST 		GPIO3

#define COLOR(r, g, b) (((r * 31 / 255) << 11) | ((g * 63 / 255) << 5) | (b * 31 / 255))
#define RED 	COLOR(255, 0, 0)
#define GREEN 	COLOR(0, 255, 0)
#define BLUE 	COLOR(0, 0, 255)
#define WHITE 	COLOR(255, 255, 255)
#define BLACK 	COLOR(0, 0, 0)

struct vec {
	int x;
	int y;
};

struct rect {
	int x;
	int y;
	int width;
	int height;
	struct vec velocity;
};

struct rect draw_rect;
struct rect draw_rect2;

bool in_rect(int x, int y, struct rect* rect)
{
	return x >= rect->x && x <= rect->x + rect->width && y >= rect->y && y <= rect->y + rect->height;
}

void delay_cycles( uint32_t cycles )
{
    while ( cycles ) {
        __asm__("nop");
        cycles--;
    }
}

static void rcc_setup(void)
{
    // clock for USARTs, SPI ports
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    // clock for SPI1
    rcc_periph_clock_enable(RCC_SPI1);
}

static void gpio_master_spi_setup(void)
{
	// master spi
	// SPI1 as master
	// SCK is PB3 on CN10-31
	// MOSI is PB5 on CN10-29
	// MISO is PB4 on CN10-27
	// NSS is PA10 on CN10-33, gpio only
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);
	gpio_set(GPIOA, GPIO10);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3|GPIO4|GPIO5);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO3|GPIO4|GPIO5 );
}

static void spi_master_setup( uint32_t spi )
{
	spi_reset( spi );
	spi_set_master_mode( spi );
	//spi_set_baudrate_prescaler( spi, SPI_CR1_BR_FPCLK_DIV_2);
	spi_send_msb_first( spi );
	spi_set_unidirectional_mode( spi );
	spi_set_dff_8bit( spi );
	spi_set_standard_mode( spi, MODE );

	// required if hardware nss not used
	spi_enable_software_slave_management( spi );
	spi_set_nss_high( spi );

	spi_enable( spi );
}

//set dc value
void set_dc(bool high)
{ 
	//dc on PC0
	GPIOC_ODR &= ~(1);
	GPIOC_ODR |= high;
}

//set cs value
void set_cs(bool cs)
{
	//cs on PC1
	GPIOC_ODR &= 1;
	GPIOC_ODR |= (cs << 1);
}

void lcd_send(uint16_t data, bool is_data)
{
	gpio_clear(GPIOA, GPIO10);
	set_dc(is_data);
	//set_cs(1);
	spi_xfer(SPI1, data);
	//set_cs(0);
	gpio_set(GPIOA, GPIO10);
}

void lcd_send16(uint16_t data, bool is_data)
{
	lcd_send((data >> 8) & 0xFF, is_data);
	lcd_send(data & 0xFF, is_data);
}

void lcd_send_array(uint16_t *data, bool is_data, int size)
{
	gpio_clear(GPIOA, GPIO10);
	set_dc(is_data);
	//set_cs(1);
	for(int i = 0; i < size; i++)
		spi_xfer(SPI1, data[i]);
	//set_cs(0);
	gpio_set(GPIOA, GPIO10);
}

void init_ili9341(void)
{
	lcd_send(ILI9341_PWCTR1, ILI9341_COMMAND);
	lcd_send(0x23, ILI9341_DATA);

	lcd_send(ILI9341_PWCTR2, ILI9341_COMMAND);
	lcd_send(0x10, ILI9341_DATA);

	lcd_send(ILI9341_VMCTR1, ILI9341_COMMAND);
	lcd_send(0x3e, ILI9341_DATA);
	lcd_send(0x28, ILI9341_DATA);

	lcd_send(ILI9341_VMCTR2, ILI9341_COMMAND);
	lcd_send(0x86, ILI9341_DATA);

	lcd_send(ILI9341_MADCTL, ILI9341_COMMAND);
	lcd_send(0x48, ILI9341_DATA);

	lcd_send(ILI9341_PIXFMT, ILI9341_COMMAND);
	lcd_send(0x55, ILI9341_DATA);

	lcd_send(ILI9341_FRMCTR1, ILI9341_COMMAND);
	lcd_send(0x00, ILI9341_DATA);
	lcd_send(0x18, ILI9341_DATA);

	lcd_send(ILI9341_DFUNCTR, ILI9341_COMMAND);
	lcd_send(0x08, ILI9341_DATA);
	lcd_send(0x82, ILI9341_DATA);
	lcd_send(0x27, ILI9341_DATA);

	//look up command.
	lcd_send(0xF2, ILI9341_COMMAND);
	lcd_send(0x00, ILI9341_DATA);

	lcd_send(ILI9341_GAMMASET, ILI9341_COMMAND);
	lcd_send(0x01, ILI9341_DATA);

	uint32_t gmctrp1Data[] = {0x0F, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00};
	lcd_send(ILI9341_GMCTRP1, ILI9341_COMMAND);
	lcd_send_array(&gmctrp1Data, ILI9341_DATA, 15);

	uint32_t gmctrn1Data[] = {0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f};
	lcd_send(ILI9341_GMCTRN1, ILI9341_COMMAND);
	lcd_send_array(&gmctrn1Data, ILI9341_DATA, 15);
	
	lcd_send(ILI9341_SLPOUT, ILI9341_COMMAND);

	//delay 1/8 of a second before turning on.
	delay_cycles(333333);

	lcd_send(ILI9341_DISPON, ILI9341_COMMAND);

}

void reset_lcd(void)
{
	gpio_set(GPIOC, LCDRST);
	delay_cycles(13333);
	gpio_clear(GPIOC, LCDRST);
	delay_cycles(66666);
	gpio_set(GPIOC, LCDRST);
	delay_cycles(333333);
}

void init_lcd(void)
{
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCDLED|LCDCS|LCDDC|LCDRST);


	//turn backlight on
	gpio_set(GPIOC, LCDLED);
	reset_lcd();

	init_ili9341();
	
}

void update_rect(struct rect *r)
{
	if(r->x <= 0 || (r->x + r->width) >= 320)
		r->velocity.x *= -1;

	if(r->y <= 0 || (r->y + r->height) >= 240)
		r->velocity.y *= -1;

	r->x += r->velocity.x;
	r->y += r->velocity.y;
}

void display()
{
	lcd_send(ILI9341_CASET, ILI9341_COMMAND);
	lcd_send(0, ILI9341_DATA);
	lcd_send(0, ILI9341_DATA);
	lcd_send((ILI9341_TFTWIDTH-1) >> 8, ILI9341_DATA);
	lcd_send((ILI9341_TFTWIDTH-1), ILI9341_DATA);

	lcd_send(ILI9341_PASET, ILI9341_COMMAND);
	lcd_send(0, ILI9341_DATA);
	lcd_send(0, ILI9341_DATA);
	lcd_send((ILI9341_TFTHEIGHT-1) >> 8, ILI9341_DATA);
	lcd_send((ILI9341_TFTHEIGHT-1), ILI9341_DATA);

	lcd_send(ILI9341_RAMWR, ILI9341_COMMAND);

	uint16_t color = BLACK;

	for(int i = 0; i < ILI9341_TFTPIX; i++){
		int x = i / 240;
		int y = i % 240;
		color = BLACK;
		if(in_rect(x, y, &draw_rect))
			color = BLUE;
		if(in_rect(x, y, &draw_rect2))
			color = GREEN;
		if(in_rect(x, y, &draw_rect2) && in_rect(x, y, &draw_rect))
			color = GREEN | BLUE;

		lcd_send16(color, ILI9341_DATA);
	}


	update_rect(&draw_rect);
	update_rect(&draw_rect2);
	//check_collision(&draw_rect, &draw_rect2);
}

int main(void)
{
	//ram_intr_setup();

	draw_rect.x = 50; draw_rect.y = 50; draw_rect.width = 50; draw_rect.height = 50;
	draw_rect2.x = 150; draw_rect2.y = 50; draw_rect2.width = 50; draw_rect2.height = 50;
	draw_rect.velocity.x = draw_rect.velocity.y = 20;
	draw_rect2.velocity.x = draw_rect2.velocity.y = -20;

	nucleo_clock_sysclk(100);
	rcc_setup();
	gpio_master_spi_setup();

	//nucleo_stm32f446_frequency( 32 );

	pccom_setup( USART2 );
	spi_master_setup( SPI1 );
	init_lcd();
	display();
	//lcd_send(ILI9341_CASET, ILI9341_COMMAND);
	while(true){
		display();
		//delay_cycles(100000);
		//__asm__("nop");
	}
	return 0;
}


