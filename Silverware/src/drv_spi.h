
// soft spi  header file 
//
#include <inttypes.h>

void spi_init(void);
void spi_cson(void);	
void spi_csoff(void);
void spi_cs1on(void);	
void spi_cs1off(void);
void spi_cs2on(void);	
void spi_cs2off(void);
void spi_sendbyte( int );
int spi_sendrecvbyte( int);
int spi_sendzerorecvbyte( void );



