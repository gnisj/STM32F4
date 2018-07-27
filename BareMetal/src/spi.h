#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED

//---------------------------------------------------------------------
// Global variables etc.
//---------------------------------------------------------------------
enum spiSpeed { SPI_SLOW, SPI_MEDIUM, SPI_FAST };


//---------------------------------------------------------------------
// Function prototypes
//---------------------------------------------------------------------
void spiInit(SPI_TypeDef *SPIx);

int spiReadWrite(SPI_TypeDef* SPIx,
                 uint8_t *rbuf,
                 const uint8_t *tbuf,
                 int cnt,
                 enum spiSpeed speed);

uint8_t eepromReadStatus();

void csInit(void);

void spiLoopbackTest();

/*int spiReadWrite16(SPI_TypeDef* SPIx,
                   uint16_t *rbuf,
                   const uint16_t *tbuf,
                   int cnt,
                   enum spiSpeed speed);
*/
#endif //SPI_H
