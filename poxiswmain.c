/*
 * poxivpmn.c: Poxi SW Solution file
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xscutimer.h"
#include "sleep.h"

#define POXIREG(k) (*(unsigned int *)(XPAR_POXI4IF_0_S00_AXI_BASEADDR+4*k))
#define SPI_DONE ((POXIREG(1)) & 0x00008000) // 16th bit of this resister is used to check SPI done bit
#define SPI_TX_BIT POXIREG(2) // used to transmit SPI data bits
#define ADC_VAL POXIREG(2) // used to read ADC conversion result
#define SPI_CONFIG POXIREG(1) //
#define RedIred POXIREG(3) // 0th and 1st bits are used for Infrared and Red LED on/off

#define ch_A 1 // DAC channels A,B,C,D (1,2,3,4)
#define ch_B 2
#define ch_C 3
#define ch_D 4
#define DAC 1 // To select device DAC,PGA,ADC (1,2,3)
#define PGA 2
#define ADC 3
#define Buffer_L 3000// Size of Internal Buffer

// following are the highpass and lowpass filter coefficients
#define BH_0 0.997337820133346
#define BH_1 -1.99467564026669
#define BH_2 0.997337820133346
#define AH_1 -1.99466855305249
#define AH_2 0.994682727480893
#define BL_0 0.00134871194835634
#define BL_1 0.00269742389671268
#define BL_2 0.00134871194835634
#define AL_1 -1.89346414636183
#define AL_2 0.898858994155252

typedef struct {
double b0, b1, b2, a1, a2;
double u_k1, u_k2, y_k1, y_k2;
    double y_out;
}Filterobj;

static Filterobj HP_f,LP_f;

static int R_size = 0, IR_size = 0;
static double IRbuffer[Buffer_L],Rbuffer[Buffer_L],Rbuffer_filt[Buffer_L],IRbuffer_filt[Buffer_L];
static double pofilt(double);
static void pofilt_init(void);
static double ADC_Read(void);
static double adc_reading = 0;
static double filt_out;
static int Logger_flag,sample = 0;

// ---- interrupt controller -----
static XScuGic  Intc;					// interrupt controller instance
static XScuGic_Config  *IntcConfig;		// configuration instance

// ---- scu timer -----
static XScuTimer  pTimer;				// private Timer instance
static XScuTimer_Config  *pTimerConfig;	// configuration instance
// 100Hz => 3333333
#define TIMER_LOAD_VALUE  166666		// 2ms for Red/IRd

static volatile unsigned int ISR_Count;

/*
 * ------------------------------------------------------------
 * Interrupt handler (ZYNQ private timer)
 * ------------------------------------------------------------
 *
 * ------------------------------------------------------------
 * Step 1: Turn ON Red LED
 * Step 2: Read ADC value and update it to internal buffer and Turn OFF LEDs
 * Step 3: Turn ON Infrared LED
 * Step 4: Read ADC value and update it to internal buffer and Turn OFF LEDs
 * ------------------------------------------------------------
 */
static void TimerIntrHandler(void *CallBackRef)
{
	XScuTimer *TimerInstance = (XScuTimer *)CallBackRef;

	XScuTimer_ClearInterruptStatus(TimerInstance);

    switch(ISR_Count)
    {
        case 0:
            RedIred = 0b01; // 24th bit is used for red LED in HW
            ISR_Count++;
            break;
        case 1:
            adc_reading = ADC_Read();
            filt_out = pofilt(adc_reading);
            RedIred = 0;
            if (R_size < Buffer_L)
            {
                Rbuffer_filt[R_size] = filt_out;
                Rbuffer[R_size] = adc_reading;
                R_size++;
            }
            ISR_Count++;
            break;
        case 2:
            RedIred = 0b10; // 25th bit is used for Infrared LED in HW
            ISR_Count++;
            break;
        case 3:
            adc_reading = ADC_Read();
            filt_out = pofilt(adc_reading);

            RedIred = 0;
            if (IR_size < Buffer_L)
            {
                IRbuffer_filt[IR_size] = filt_out;
                IRbuffer[IR_size] = adc_reading;
                IR_size++;
            }
            ISR_Count = 0;
            break;
        default:;
    }

}

void pofilt_init() // To initialize filter objects
{
HP_f.b0 = BH_0;
HP_f.b1 = BH_1;
HP_f.b2 = BH_2;
HP_f.a1 = AH_1;
HP_f.a2 = AH_2;
HP_f.u_k1 = 0.0;
HP_f.u_k2 = 0.0;
HP_f.y_k1 = 0.0;
HP_f.y_k2 = 0.0;
HP_f.y_out = 0.0;
LP_f.b0 = BL_0;
LP_f.b1 = BL_1;
LP_f.b2 = BL_2;
LP_f.a1 = AL_1;
LP_f.a2 = AL_2;
LP_f.u_k1 = 0.0;
LP_f.u_k2 = 0.0;
LP_f.y_k1 = 0.0;
LP_f.y_k2 = 0.0;
LP_f.y_out = 0.0;
}

double pofilt(double u) //Filter for poxi data (High pass 0.3 Hz and Low pass 6 Hz)
{
HP_f.y_out = HP_f.b0*u + HP_f.b1*HP_f.u_k1 + HP_f.b2*HP_f.u_k2
            - HP_f.a1*HP_f.y_k1 - HP_f.a2* HP_f.y_k2;
HP_f.u_k2 = HP_f.u_k1;
HP_f.u_k1 = u;
HP_f.y_k2 = HP_f.y_k1;
HP_f.y_k1 = HP_f.y_out;
LP_f.y_out = LP_f.b0*HP_f.y_out + LP_f.b1*LP_f.u_k1 + LP_f.b2*LP_f.u_k2
            - LP_f.a1*LP_f.y_k1 - LP_f.a2* LP_f.y_k2;
LP_f.u_k2 = LP_f.u_k1;
LP_f.u_k1 = HP_f.y_out;
LP_f.y_k2 = LP_f.y_k1;
LP_f.y_k1 = LP_f.y_out;

   return LP_f.y_out;
}

static void DAC_config() // DAC configuration Function
{
  while(!SPI_DONE);
  SPI_CONFIG = 0x00010a10;  // (Data Length = 24("01"),CPOL = 1, CPHA = 0)
  SPI_TX_BIT = 0x00280000;  // Reset DAC internal registers and channels
  while(!SPI_DONE);
  SPI_CONFIG = 0x00010a10;
  SPI_TX_BIT = 0x0020000F;  // Power-up DAC channels
  while(!SPI_DONE);
  SPI_CONFIG = 0x00010a10;
  SPI_TX_BIT = 0x00300000;  // setting-up LDAC register
  while(!SPI_DONE);
  SPI_CONFIG = 0x00010a10;
  SPI_TX_BIT = 0x00380001;  // setting-up DAC internal reference
}

static void DAC_Write(int chan, int val) // DAC write input Function
{
    unsigned int xval;

	xval = (val & 0x0fff) << 4;
	if (chan == ch_A) {
		xval |= 0x0180000;
	} else if (chan == ch_B) {
		xval |= 0x0190000;
	} else if (chan == ch_C) {
		xval |= 0x01a0000;
	} else if (chan == ch_D) {
		xval |= 0x01b0000;
	}

  while(!SPI_DONE);
  SPI_CONFIG = 0x00010a10;
  SPI_TX_BIT = xval;
}

static void PGA_Write() // PGA write input Function
{
   while(!SPI_DONE);
   SPI_CONFIG = 0x00020710;
   SPI_TX_BIT = (0b0100000000000000) | 4;
}

static double ADC_Read(void) // ADC Read Function, Conversion value will be updated to ADC_VAL resister after transmission is done
{
  while(!SPI_DONE);
  SPI_CONFIG = 0x00030680;
  SPI_TX_BIT = 0b0000000100000000;
  while(!SPI_DONE);
  return 0x0fff & ADC_VAL;
}

// To transfer sensor recorded data to GUI for Plotting the output
static void TransferTo_GUI(){
	if (Logger_flag){
	        // For logging data start string with "~"
			if(sample<Buffer_L)
			{
	          printf("~ %f %f %f %f\n",Rbuffer[sample],IRbuffer[sample],Rbuffer_filt[sample],Rbuffer_filt[sample]);
	          sample++;
			}
			else
			{
				printf("@ %f\n",13.000);
			}
	    }
}

static void ClearAll() // clear outputs on exit
{
    RedIred = 0; // Turn OFF LEDs
    while(!SPI_DONE);
    SPI_CONFIG = 0x00010a10;  // (Data Length = 24("01"),CPOL = 1, CPHA = 0)
    SPI_TX_BIT = 0x0020003F;  // Power-Down DAC channels
    while(!SPI_DONE);
    SPI_CONFIG = 0x00030680;  // (Data Length = 16("01"),CPOL = 1, CPHA = 0)
    SPI_TX_BIT = 0b0010000100000000;  // Power-Down ADC channels
    while(!SPI_DONE);
    SPI_CONFIG = 0x00020710;  // (Data Length = 16("01"),CPOL = 1, CPHA = 0)
    SPI_TX_BIT = 0x2000;
    SPI_CONFIG = 0;
    SPI_TX_BIT = 0;
    RedIred = 0;
    R_size = 0;
    IR_size = 0;
}

int main()
{
    init_platform();
    print("--- Poxi IVP V0.0a ---\n\r");
    ISR_Count = 0;
    double redarray[Buffer_L]; // just for Testing and will be removed in final version
	int  i,j,m;
    int case_no = 0,rc = 0,intensity = 0, n;
    Logger_flag = 0;
    sample = 0;

    printf(" * initialize exceptions...\n\r");
    Xil_ExceptionInit();

    printf(" * lookup config GIC...\n\r");
    IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
    printf(" * initialize GIC...\n\r");
    XScuGic_CfgInitialize(&Intc, IntcConfig, IntcConfig->CpuBaseAddress);

	// Connect the interrupt controller interrupt handler to the hardware
    printf(" * connect interrupt controller handler...\n\r");
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
				(Xil_ExceptionHandler)XScuGic_InterruptHandler, &Intc);

    printf(" * lookup config scu timer...\n\r");
    pTimerConfig = XScuTimer_LookupConfig(XPAR_XSCUTIMER_0_DEVICE_ID);
    printf(" * initialize scu timer...\n\r");
    XScuTimer_CfgInitialize(&pTimer, pTimerConfig, pTimerConfig->BaseAddr);
    printf(" * Enable Auto reload mode...\n\r");
	XScuTimer_EnableAutoReload(&pTimer);
    printf(" * load scu timer...\n\r");
    XScuTimer_LoadTimer(&pTimer, TIMER_LOAD_VALUE);

    printf(" * set up timer interrupt...\n\r");
    XScuGic_Connect(&Intc, XPAR_SCUTIMER_INTR, (Xil_ExceptionHandler)TimerIntrHandler,
    				(void *)&pTimer);
    printf(" * enable interrupt for timer at GIC...\n\r");
    XScuGic_Enable(&Intc, XPAR_SCUTIMER_INTR);
    printf(" * enable interrupt on timer...\n\r");
    XScuTimer_EnableInterrupt(&pTimer);

   // Enable interrupts in the Processor.
    printf(" * enable processor interrupts...\n\r");
	Xil_ExceptionEnable();

    printf(" 1. Turn on RED LED\n 2. Turn on IR LED\n 3. Initialize DAC\n 4. Enter the intensity to be increase\n 5. Increase CH_A intensity\n");
    printf(" 6. Increase CH_B intensity\n 7. Set PGA gain\n 8. Read ADC value\n 9. Enable interrupt\n 10. Disable interrupt\n");
    printf(" 11. Turn OFF LEDs\n 12. Enter the intensity to be decrease\n 13. Exit\n");

    while(!(rc == 10))
    {
        if(rc!=11)
        {
         //  printf("btst>> ");
           scanf("%d",&case_no);
        }
        switch(case_no)
        {
        case 1: printf("1. RED LED is ON\n");
                RedIred = 0b01;
                break;
        case 2: printf("2. IR LED is ON\n");
                RedIred = 0b10;
                break;
        case 3: printf("3. Initialized DAC\n");
                DAC_config();
                break;
        case 4: printf("4. Enter the intensity to be increased\n");
                scanf("%d",&n);
                intensity = intensity + n;
                break;
        case 5: printf("5. Set CH_A intensity by %d\n",intensity);
                DAC_Write(ch_A,intensity);
                break;
        case 6: printf("6. Set CH_B intensity by %d\n",intensity);
                DAC_Write(ch_B,intensity);
                break;
        case 7: printf("7. Set PGA gain by Entered value\n");
               // scanf("%d", &gain);
                PGA_Write();
                break;
        case 8: for(i=0;i<10;i++) // just for Testing
                {
                	redarray[i] = ADC_Read();
                	printf("ADC value is: %lf\n", redarray[i]);
                }
                rc = 11;
                case_no = 13;
                break;
        case 9: printf("9. Interrupt is Enabled\n");
                pofilt_init();
                printf(" * start timer...\n\r");
                XScuTimer_Start(&pTimer);
                do
                {
                    if(IR_size==Buffer_L)
                    {
                        rc = 11;
                        case_no = 10;
                    }
                }while(case_no!=10);
                break;
        case 10: printf("10. Interrupt is Disabled\n");
        		 XScuTimer_Stop(&pTimer);
        		 rc = 9;
                 break;
        case 11: printf("11. LEDs are Turned off\n");
                 RedIred = 0b00;
                 break;
        case 12: printf("12. Enter the intensity to be decrease\n");
                 scanf("%d",&n);
                 intensity = intensity - n;
                 break;
        case 13: printf("13. Exit\n");
                 ClearAll();
                 rc = 10;
                 break;
        case 14: // To start sending data to GUI for logging
        		Logger_flag = 1;
        		printf("Flag is Enable for Data logging.\n");
        		break;
        case 15: TransferTo_GUI();
                break;
        default: case_no = 13;
        }
    };

	Xil_ExceptionDisable();
    XScuTimer_DisableInterrupt(&pTimer);
    XScuGic_Disable(&Intc, XPAR_SCUTIMER_INTR);
    usleep(20000);
    print("Thank you for using Poxi IVP V0.0a.\n\r");
    cleanup_platform();
    return 0;
}
