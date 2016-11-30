# tft24t-dirver

This is a c library that works with libopencm3 to control the TJCTM24024-SPI lcd display. The implementation was tested on the *Nucleo-446re*.

The TJCTM24024-SPI is a 2.4 inch SPI TFT 320x240 pixel display. These are fairly inexpensive displays that you can find on eBay.

To run this you will need:
* Nucleo-446re Prototyping board
* TJCTM24024-SPI lcd display
* Bread board and bread board connectors.

You will need to connect the display to the Nucleo. For my implementation I used the following setup.
* LED connected to PC5
* D/C connected to PC0
* CS connected to PA10
* RST connected to PC3
* SCK connected to PB3
* MOSI connected to PB5

The display is fairly slow. I upped the clock speed of the nucleo to 100mhz to make it as fast as possible but it still it not where I would like it.

Adam
