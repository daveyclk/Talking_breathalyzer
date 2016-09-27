/****************************************************************************
* Title                 :   iBreathe Breathalyzer
* Filename              :   breathalyzer
* Author                :   Dave Clarke
* Origin Date           :   27/09/2016
* Notes                 :   Breathalyzer utilizing Hexiware, Alcohol click and Wolksense
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials       Description
*  27/09/16       1.0.0            DC        Interface Created.
*
*****************************************************************************/

/**
 * @page TEST_CFG Test Configurations
 * <h3> Test configuration : </h3>
 * @par
 * <ul>
 * <li><b> MCU           </b> :      MK64FN1M0XXX12               </li>
 * <li><b> Dev. Board    </b> :      HEXIWEAR                     </li>
 * <li><b> Oscillator    </b> :      12 MHz external              </li>
 * <li><b> Ext. Modules  </b> :      Alcohol Click on mikroBUS 1  </li>
 * <li><b> SW            </b> :      mBed OS5    </li>
 * </ul>
 */

/**
 * @mainpage
 * <h3> Breathalyser created with HEXIWEAR and mBed OS 5 </h3>
 * @par This will show you how much you've drunk and tell you with an Emoticon if
 * you're too hammered to even consider driving. Using the Hexiware app the readings
 * are transmitted to the cloud via bluetooth. Is it time to give up drinking yet?!
 
 * <h3> Alcohol Features </h3>
 * @par Alcohol click, Hexiwear docking station, Hexiware
 */

/******************************************************************************
* Includes
*******************************************************************************/

#include "mbed.h"
#include "Hexi_KW40Z.h"
#include "Hexi_OLED_SSD1351.h"
#include "OLED_types.h"
#include "OpenSans_Font.h"
#include "string.h"
#include "iBreatheImages.h"

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

#define LED_ON      0
#define LED_OFF     1

                          // Pointers to:  
const uint8_t   *welcome, // Welcome screen image
                *blank,   // blank image
                *blow,    // Start Blowing Image
                *drink,   // You've been drinking image
                *drive,   // Don't drive image
                *hang,    // You'll have a hangover image
                *ini,     // Initialising image
                *sober;   // Sober as a judge image

const float     Vadc_3V3  = 0.00005035;    // 16-Bit ADC step 3V3/65535   = 0.05035 mV
       
float           Vrl = 0,                   // Output voltage
                ambientAlc = 0,            // Abmient Output voltage from sensor
                SensorRes = 0,             // SensorRes (Ohm) - Sensor resistance
                SensorRes1 = 0,            // SensorRes (Ohm) - Sensor resistance
                ppm = 0,                   // ppm
                ppm_1 = 0,                 // Ambient ppm variable  
                ratio = 0;                 // SensorRes/LoadRes ratio
              
unsigned short  adc_rd = 0; //Initialise anologue read variable

const uint8_t   ppmText[] = "ppm:"; // text for PPM label

char            text[20],           // Text array variables
                text2[20],
                text3[20];

float           value[20],  // initial sensor set up values
                value1[20]; // initial sensor set up values

bool            isFirstBoot = true;                   
                

/******************************************************************************
* Function Prototypes
*******************************************************************************/

void sysinit(void);
void ReadSensor();
void CalculatePPM(int times, bool amb);
void ambient(int times);
void StartHaptic(void);
void StopHaptic(void const *n);
void ButtonUp(void);
void txTask(void);

/******************************************************************************
* Instance setups
*******************************************************************************/
/* Define timer for haptic feedback */
RtosTimer hapticTimer(StopHaptic, osTimerOnce);

//set up Analog read pin for alcohol sensor
AnalogIn Alcohol(PTB2);

/* Instantiate the SSD1351 OLED Driver */ 
SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15); /* (MOSI,SCLK,POWER,CS,RST,DC) */
      
/* Get OLED Class Default Text Properties */
oled_text_properties_t textProperties = {0};

/* Instantiate the Hexi KW40Z Driver (UART TX, UART RX) */ 
KW40Z kw40z_device(PTE24, PTE25);

/* LED and Haptic Set ups */
DigitalOut redLed(LED1);
DigitalOut greenLed(LED2);
DigitalOut blueLed(LED3);
DigitalOut haptic(PTB9);

/******************************************************************************
* Bluetooth button functions and passkey function
*******************************************************************************/

void ButtonRight(void)
{
    StartHaptic();
    kw40z_device.ToggleAdvertisementMode();
    blueLed = kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/
    redLed = !kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/
    greenLed = !kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/
}

void ButtonLeft(void)
{
    StartHaptic();
    kw40z_device.ToggleAdvertisementMode();
    blueLed = kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/
    redLed = !kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/
    greenLed = !kw40z_device.GetAdvertisementMode(); /*Indicate BLE Advertisment Mode*/
}

void PassKey(void)
{
    StartHaptic();
    strcpy((char *) text,"PAIR CODE");
    oled.TextBox((uint8_t *)text,0,25,95,18);
  
    /* Display Bond Pass Key in a 95px by 18px textbox at x=0,y=40 */
    sprintf(text3,"%d", kw40z_device.GetPassKey());
    oled.TextBox((uint8_t *)text3,0,40,95,18);
}

/******************************************************************************
* Main
*******************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
     /* Set pointers to the BMPs stored in  iBreatheImages.c */
     welcome = iBreatheWS_bmp;    // Welcome screen image 
     blank = iBreatheBlank_bmp;   // blank image
     blow = iBreatheBlow_bmp;    // Start Blowing Image
     drink = iBreatheDrink_bmp;   // You've been drinking image
     drive = iBreatheDrive_bmp;   // Don't drive image
     hang = iBreatheHang_bmp;    // You'll have a hangover image
     ini = iBreatheini_bmp;     // Initialising image
     sober = iBreatheSober_bmp;   // Sober as a judge image
       
     /* Set initial Values */
     sysinit();
}

/******************************************************************************
* Public Function Definitions
*******************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function readSensor(void)
* -------------------------------------------------------------------------------------------------
* Overview: Read sensor
* Input: None
* Output: None
**************************************************************************************************/

void ReadSensor()
{
    /* Read 16 Bit Analog value */
    adc_rd = Alcohol.read_u16();

    // pause 200ms 
    Thread::wait(200);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function CalculatePPM(Int times, bool amb)
* -------------------------------------------------------------------------------------------------
* Overview: Calculation of PPM
* Input: times = Number of samples to take, amb sets either ambient reading or test reading (true for test)
* Output: None
**************************************************************************************************/

void CalculatePPM(int times, bool amb)
{
        float   lgPPM;
        
        /* Read values x times */
        for(int x = 0; x < times; x++)
        {
            ReadSensor();    
            value[x] = ((float)adc_rd  * Vadc_3V3);
            StartHaptic();
            Thread::wait(50);
        }    
        
        /* Calculate the average value for accuratcy */
        for(int y = 0; y < times; y++)
        {
            Vrl += value[y];
        }
        Vrl = Vrl / times;       

        /* Set SensorRes reference value */
        SensorRes =    (Vrl / 3.3);
        
        /* Set ratio */
        ratio     = SensorRes1 / SensorRes;

        /* Convert to PPM */
        lgPPM = ( log10( ratio ) * -1.5512 ) + 2.5911;
        
        /* If true create test result, flase creates reference result */
        if (amb == true)
        {
            ppm = pow( 10, lgPPM );
        }
        else
        {
            ppm_1 = pow( 10, lgPPM );
        }    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function void ambient(int times)
* -------------------------------------------------------------------------------------------------
* Overview: Reading of the Ambient Voltgage of the sensor for reference
* Input: times = Number of samples to take
* Output: None
**************************************************************************************************/

void ambient(int times)
{
    /* Read ambient values x times flashing green led*/
    for(int x = 0; x < times; x++)
    {
        redLed      = LED_OFF;
        greenLed    = LED_OFF;
        blueLed     = LED_OFF;
        
        ReadSensor();
        value1[x] = (float)adc_rd  * Vadc_3V3; 
        
        redLed      = LED_OFF;
        greenLed    = LED_ON;
        blueLed     = LED_OFF;
        Thread::wait(48);   
    }
    
    /* Calculate the average value for accuratcy */
    for(int y = 0; y < times; y++)
    {
        ambientAlc+=value1[y];            
    }
    ambientAlc = ambientAlc / times;
  
    /* Set SensorRes1 reference value */
    SensorRes1 =   (ambientAlc / 3.3);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function void StartHaptic(void)
* -------------------------------------------------------------------------------------------------
* Overview: Start Buzzing haptic motor
* Input:  None
* Output: None
**************************************************************************************************/

void StartHaptic(void)
{
    hapticTimer.start(50);
    haptic = 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function void StopHaptic(void)
* -------------------------------------------------------------------------------------------------
* Overview: Stop Buzzing haptic motor
* Input:  None
* Output: None
**************************************************************************************************/

void StopHaptic(void const *n) {
    haptic = 0;
    hapticTimer.stop();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function void ButtonUp(void)
* -------------------------------------------------------------------------------------------------
* Overview: Function called whe up button pressed, Begins Breathilyzer testing procedure
* Input:  None
* Output: None
**************************************************************************************************/

void ButtonUp(void)
{
    StartHaptic();
    
    bool Ref = false;
    bool Test = true;
        
    /* LED set to green for test beginning*/
    redLed      = LED_OFF;
    greenLed    = LED_ON;
    blueLed     = LED_OFF;
    
    /* Fill 96px by 96px Screen with 96px by 96px Initialising Image starting at x=0,y=0 */   
    oled.DrawImage(ini,0,0);
    
    /* first boot bug work around to stop junk values */
    if (isFirstBoot == true)
    {
        /*read ambient atmosphere levels with 10 samples and set flag to show it's ambient basline figure*/
        ambient(1);
        CalculatePPM(1,Ref);
        isFirstBoot = false;
    }
    
    /*read ambient atmosphere levels with 10 samples and set flag to show it's ambient basline figure*/
    ambient(10);
    CalculatePPM(10,Ref);
    
    
    /* Fill 96px by 96px Screen with 96px by 96px Blowing Image starting at x=0,y=0 */  
    oled.DrawImage(blow,0,0);
    
    /*read breathe alcohol levels with 10 samples and set flag to show it's breathilyzer test figure*/
    CalculatePPM(10,Test);
    
    /*Calculate the difference in Alcohol level based on Ambient and test sample*/
    ppm = ppm - ppm_1;
    
    /*Throw away any values less than 0*/
    if (ppm < 0)
    {
         ppm = 0;
    }     
    
    /* Fill 96px by 96px Screen with 96px by 96px Blank Background Image starting at x=0,y=0 */
    oled.DrawImage(blank,0,0);     
    
    /* Show Calculated alcohol level in PPM and send data via bluetooth to the cloud */
    oled.SetTextProperties(&textProperties);
    oled.Label(ppmText,20,36);
    sprintf(text,"%.2f",ppm);
    oled.Label((uint8_t *)text,50,36);
    Thread::wait(1000);
    
    /* Currently sending to the Pressure variable as a temp place holder */
    kw40z_device.SendPressure(ppm * 10);
    
    /* You've got a Hangover coming!*/
    if ( ppm > 200)
    {
        redLed      = LED_ON;
        greenLed    = LED_OFF;
        blueLed     = LED_OFF;
        
        StartHaptic();
        
        /* Fill 96px by 96px Screen with 96px by 96px Hangover Image starting at x=0,y=0 */
        oled.DrawImage(hang,0,0);
    }   
    
    /* You Shouldn't drive */
    else if (ppm < 200 && ppm > 150)
    {
        redLed      = LED_ON;
        greenLed    = LED_OFF;
        blueLed     = LED_ON;
        
        StartHaptic();
      
        /* Fill 96px by 96px Screen with 96px by 96px Don't Drive Image starting at x=0,y=0 */      
        oled.DrawImage(drive,0,0);
    } 
    
    /* You've had a drink */
    else if (ppm < 150 && ppm > 50)
    {
        redLed      = LED_OFF;
        greenLed    = LED_ON;
        blueLed     = LED_ON;
        
        StartHaptic();
    
        /* Fill 96px by 96px Screen with 96px by 96px Had a drink Image starting at x=0,y=0 */     
        oled.DrawImage(drink,0,0);
    }
    
    /* Sober as a judge*/   
    else
    {
        redLed      = LED_OFF;
        greenLed    = LED_ON;
        blueLed     = LED_OFF;
        
        StartHaptic();
        oled.DrawImage(sober,0,0);    
    }
    Thread::wait(5000);
    
    /* Go back to start screen */
    sysinit();   
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function void sysint(void)
* -------------------------------------------------------------------------------------------------
* Overview: System Initial Values Set up
* Input: None
* Output: None
**************************************************************************************************/
void sysinit(void)
{
    /* Set LED to Blue by default*/      
    redLed      = LED_OFF;
    greenLed    = LED_OFF;
    blueLed     = LED_ON;
    
    /* Turn on the backlight of the OLED Display */
    oled.DimScreenON();
    
    /* Fill 96px by 96px Screen with 96px by 96px Welcome Image starting at x=0,y=0 */
    oled.DrawImage(welcome,0,0);
    
    /* Register callbacks to application functions */
    kw40z_device.attach_buttonUp(&ButtonUp);
    kw40z_device.attach_buttonLeft(&ButtonLeft);
    kw40z_device.attach_buttonRight(&ButtonRight);
    kw40z_device.attach_passkey(&PassKey);
    
    /* Send sensor data to bluetooth */
    kw40z_device.SendSetApplicationMode(GUI_CURRENT_APP_SENSOR_TAG);
    
    /* Change font color to White */
    oled.GetTextProperties(&textProperties); 
    textProperties.fontColor   = COLOR_WHITE;
    oled.SetTextProperties(&textProperties);
}