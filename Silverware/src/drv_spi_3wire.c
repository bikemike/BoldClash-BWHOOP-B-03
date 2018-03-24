

#include "project.h"
#include "drv_spi.h"
#include "binary.h"
#include "config.h"

#ifdef SOFTSPI_3WIRE

GPIO_InitTypeDef mosi_init_struct;
int mosi_out = 0;

void spi_init(void)
{    
	// spi port inits

		GPIO_InitTypeDef  GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = SPI_CLK_PIN;
	GPIO_Init(SPI_CLK_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN;
	GPIO_Init(SPI_SS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SS2_PIN;
	GPIO_Init(SPI_SS2_PORT, &GPIO_InitStructure);
	
		
  mosi_init_struct.GPIO_Pin = SPI_MOSI_PIN;
  mosi_init_struct.GPIO_Mode = GPIO_Mode_IN;
  mosi_init_struct.GPIO_OType = GPIO_OType_PP;
  mosi_init_struct.GPIO_PuPd = GPIO_PuPd_UP;
  mosi_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);
	
	//mosi_out = 0; // already
	
	spi_csoff();

}



void mosi_input( void)
{
	if ( mosi_out)
	{
	mosi_out = 0;
	mosi_init_struct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct); 
	}
	
}

void mosi_output( void)
{
	if ( !mosi_out)
	{
	mosi_out = 1;
	mosi_init_struct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);	
	}
	
}

#define gpioset( port , pin) port->BSRR = pin
#define gpioreset( port , pin) port->BRR = pin

#define MOSIHIGH gpioset( SPI_MOSI_PORT, SPI_MOSI_PIN)
#define MOSILOW gpioreset( SPI_MOSI_PORT, SPI_MOSI_PIN);
#define SCKHIGH gpioset( SPI_CLK_PORT, SPI_CLK_PIN);
#define SCKLOW gpioreset( SPI_CLK_PORT, SPI_CLK_PIN);


#define READMOSI (SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)

#pragma push

#pragma Otime
#pragma O2

void spi_cson( )
{
	//spi_cs1on();
	spi_cs2on();
}
void spi_csoff( )
{
	//spi_cs1off();
	spi_cs2off();
}

void spi_cs1on( )
{
	SPI_SS_PORT->BRR = SPI_SS_PIN;
}

void spi_cs1off( )
{
	SPI_SS_PORT->BSRR = SPI_SS_PIN;
}
void spi_cs2on( )
{
	SPI_SS2_PORT->BRR = SPI_SS2_PIN;
}

void spi_cs2off( )
{
	SPI_SS2_PORT->BSRR = SPI_SS2_PIN;
}


void spi_sendbyte ( int data)
{
mosi_output();
for ( int i =7 ; i >=0 ; i--)
	{
		if (  (data>>i)&1  ) 
		{
			MOSIHIGH;
		}
		else 
		{
			MOSILOW;
		}
	
		SCKHIGH;
		SCKLOW;
	}
}


 int spi_recvbyte( void)
{ int recv = 0;

	for ( int i = 7 ; i >=0 ; i--)
	{
						
		SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & (int)SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
		
	}	

    return recv;
}


 int spi_recvbyte_unrolled( void)
{ uint8_t recv = 0;
						
		SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
    
    	SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
    
    	SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
        
        SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
        
        		SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
        
        		SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
        
        		SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
        
        		SCKHIGH;

		recv = recv<<1;

        recv = recv|((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN)?1:0);
        
		SCKLOW;
			

    return recv;
}

/*

 int spi_sendzerorecvbyte( )
{ int recv = 0;
	MOSILOW;

	for ( int i = 7 ; i >=0 ; i--)
	{
		recv = recv<<1;
		
		SCKHIGH;
		
		if ( READMISO ) recv= recv|1;

		SCKLOW;
		
	}	
    return recv;
}
*/

#pragma pop

#endif







