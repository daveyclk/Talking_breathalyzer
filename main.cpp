/****************************************************************************
* Title                 :   iBreathe Breathalyzer
* Filename              :   Talking_breathalyzer
* Author                :   Dave Clarke
* Origin Date           :   27/09/2016
* Notes                 :   Breathalyzer utilizing Hexiware, Alcohol click and 
                            Wolksense. Now with added voice thanks to the 
                            Text-To-Speech Click board
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials       Description
*  07/10/16       1.0.0            DC        Interface Created.
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
 * <li><b> Ext. Modules  </b> :      TTS Click on mikroBUS 2      </li>
 * <li><b> SW            </b> :      mBed OS5    </li>
 * </ul>
 */

/**
 * @mainpage
 * <h3> Breathalyser created with HEXIWEAR and mBed OS 5 </h3>
 * @par This will show you how much you've drunk and tell you with an Emoticon if
 * you're too hammered to even consider driving. Using the Hexiware app the readings
 * are transmitted to the cloud via bluetooth. Is it time to give up drinking yet?!
 * It now talks to you! 
 
 * <h3> Alcohol Features </h3>
 * @par Text to Speech Click, Alcohol click, Hexiwear docking station, Hexiware
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
#include "text_to_speech.h"
#include "text_to_speech_hal.h"
#include "text_to_speech_hw.h"
#include "text_to_speech_img.h"


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
              

/* Indication Flags */
static volatile bool        _tts_rdy_f;
static volatile bool        _spc_rdy_f;
static volatile bool        _tts_fin_f;
static volatile bool        _spc_fin_f;

/* Error Buffers */
static uint16_t             _req_err;
static uint16_t             _err_code;

/* Default Configuration */
static ACONF_t              _audio_conf;
static TTSCONF_t            _tts_conf;
static PMANCONF_t           _pman_conf;

static bool                 _flush_enable;

/* Timer flag and counter */
static volatile bool        _ticker_f;
static volatile uint16_t    _ticker;

/* Input and output buffers */
static  ISC_REQ_t   _last_req;
static  ISC_RESP_t  _last_rsp;

static uint8_t test[ 8 ] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
int frequency = 750000;
VER_t *version;
/******************************************************************************
* Function Prototypes
*******************************************************************************/
/* Checks for indications */
static int _parse_ind( void );

/* Message block and error callback function pointers */
static void ( *_fatal_err_callback )( uint16_t *err_code );
static void ( *_err_callback )( uint16_t *err_code );
static void ( *_msg_block_callback )( uint16_t *msg_code,
                                      uint16_t *err_code ); 

void voiceinit(void);
void sysinit(void);
void ReadSensor();
void CalculatePPM(int times, bool amb);
void ambient(int times);
void StartHaptic(void);
void StopHaptic(void const *n);
void ButtonUp(void);
void txTask(void);
void fatal_err( uint16_t *err );
void msg_blk( uint16_t *req, uint16_t *err );

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

/* Instantiate the Click Text to Speech pinout */ 
DigitalOut TTS_RST(PTB19);
DigitalOut TTS_CS(PTC3);
DigitalOut TTS_MUTE(PTB3);
DigitalIn  TTS_RDY(PTB8);

//Instantiate SPI for comms with Speak module
SPI TextToSpeech(PTC6, PTC7, PTC5); // MOSI, MISO, SCK

// Debug Serial
Serial pc(USBTX, USBRX);

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
    
    /* initialise voice engine */  
     voiceinit();
     
    /* Welcome message on reset */    
     tts_speak( "Hello, welcome to the eye brethalizer " );
     wait(0.5);
     tts_speak( "please press start.  " ); 
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
    tts_speak( "initializing" );
    ambient(10);
    CalculatePPM(10,Ref);
    
    
    /* Fill 96px by 96px Screen with 96px by 96px Blowing Image starting at x=0,y=0 */  
    oled.DrawImage(blow,0,0);
    tts_speak( "please blow until buzzing stops" );
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
    tts_speak( "Test Complete" );
    Thread::wait(500);
    /* Show Calculated alcohol level in PPM and send data via bluetooth to the cloud */
    oled.SetTextProperties(&textProperties);
    oled.Label(ppmText,20,36);
    tts_speak( "you blew" );
    sprintf(text,"%.2f",ppm);
    Thread::wait(50);
    tts_speak( text );
    oled.Label((uint8_t *)text,50,36);
    Thread::wait(50);
    tts_speak( "pee pee emm" );
    Thread::wait(1000);
    
    /* Currently sending to the Pressure variable as a temp place holder */
    kw40z_device.SendiBreathe(ppm); // this is used for custom Hexiware app
    kw40z_device.SendPressure(ppm * 10); // using this to record on Wolksense Cloud
    
    
    /* You've got a Hangover coming!*/
    if ( ppm > 200)
    {
        tts_speak("you have got a hang over coming");
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
        tts_speak("you shouldn't drive");
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
        tts_speak("you have had a drinky");
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
        tts_speak("you are as sober as a judge");
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function void voiceinit(void)
* -------------------------------------------------------------------------------------------------
* Overview: initialise voice engine
* Input: None
* Output: None
**************************************************************************************************/
void voiceinit(void)
{
    
    /*setting up the SPI defaults*/
    TextToSpeech.lock();
    TextToSpeech.format(8,3);
    TextToSpeech.frequency(frequency);
    TextToSpeech.unlock();
    
    /* initialise voice */
    pc.printf("System Init Done!\r\n");
    tts_init();
    pc.printf("tts Init Done!\r\n");
    tts_setup();
    pc.printf("tts setup Done!\r\n");
    tts_msg_block_callback( msg_blk );
    tts_fatal_err_callback( fatal_err );
    tts_config( 0x10, false, TTSV_US, 0x0080 );
    tts_unmute();
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function msg_blk( uint16_t *req, uint16_t *err )
* -------------------------------------------------------------------------------------------------
* Overview: receive a blocked message from  S1V30120;
**************************************************************************************************/
void msg_blk( uint16_t *req, uint16_t *err )
{
    char txt[ 6 ];
    
    pc.printf( " MSG BLOCKED \r\n" );
    sprintf( txt, "%x\r\n", *req );
    pc.printf( txt );
    sprintf( txt, "%x\r\n", *err );
    pc.printf( txt );
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* Function fatal_err( uint16_t *err )
* -------------------------------------------------------------------------------------------------
* Overview: error detected
**************************************************************************************************/
void fatal_err( uint16_t *err )
{
    pc.printf( "Fatal Error Detected" );
    tts_init();
    tts_fatal_err_callback( fatal_err );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* text_to_speech_hal.c
* -------------------------------------------------------------------------------------------------
* Overview: Functions to control SPI interface.
**************************************************************************************************/

void tts_hal_cs_high()
{
    
    TTS_CS = 1;
}

void tts_hal_cs_low()
{
    TTS_CS = 0;
    
}

void tts_hal_mut_high()
{
    TTS_MUTE = 1;
}

void tts_hal_mut_low()
{
    TTS_MUTE = 0;
}

void tts_hal_reset( void )
{
    tts_hal_cs_high();
    TTS_RST = 0;
    tts_hal_write_pad(1);
    wait(0.01);
    TTS_RST = 1;
    wait(POR_TIME);
}

bool tts_hal_msg_rdy( void )
{
    return TTS_RDY;
}

void tts_hal_init()
{
    tts_hal_reset();
    tts_hal_cs_high();
    tts_hal_mut_low();
}

void tts_hal_write( uint8_t *buffer,
                    uint16_t count, bool boot )
{
    TextToSpeech.lock();
    while( count-- )
    {
        if(!boot)
            pc.printf("%02X\r\n", *buffer);
        
        TextToSpeech.write( *buffer++  );
        
     }   
    TextToSpeech.unlock();
}


void tts_hal_write_pad( int cnt )
{
    TextToSpeech.lock();
    tts_hal_cs_low();
    while(cnt--)
    {
         TextToSpeech.write( PADDING_BYTE );
    }  

    tts_hal_cs_high();
    TextToSpeech.unlock();
}

void tts_hal_read( uint8_t *buffer,
                   uint16_t count )
{
    TextToSpeech.lock();
    while(count--)
    {   
        *buffer++ = TextToSpeech.write( DUMMY_BYTE ); //read spi bus
      
       
        //pc.printf("buffer = %X\n\r", *buffer);
    }
    TextToSpeech.unlock();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* text_to_speech.c
* -------------------------------------------------------------------------------------------------
* Overview: Functions to control the setup and speech controls
**************************************************************************************************/

static int _parse_ind( void )
{
    uint16_t rsp_idx = tts_rsp_idx();
    if ( rsp_idx == ISC_MSG_BLOCKED_RESP )
    {
        uint8_t rsp_data[ 4 ] = { 0 };

        _req_err = 0;
        _err_code = 0;

        tts_rsp_data( rsp_data );

        _req_err |= rsp_data[ 0 ];
        _req_err |= rsp_data[ 1 ] << 8;
        _err_code |= rsp_data[ 2 ];
        _err_code |= rsp_data[ 3 ] << 8;

        if( _msg_block_callback != NULL )
            _msg_block_callback( &_req_err, &_err_code );

        return 1;

    } else if ( rsp_idx == ISC_ERROR_IND ) {

         uint8_t rsp_data[ 4 ] = { 0 };

        _req_err = 0;
        _err_code = 0;

        tts_rsp_data( rsp_data );

        _err_code |= rsp_data[ 0 ];
        _err_code |= rsp_data[ 1 ] << 8;

        if ( _err_code && _err_code < 0x8000 )
        {
            if( _err_callback != NULL )
                _err_callback( &_err_code );

        } else if ( _err_code && _err_code > 0x7FFF ) {

            if( _fatal_err_callback != NULL )
                _fatal_err_callback( &_err_code );
        }

        return 1;

    } else if ( rsp_idx == ISC_TTS_READY_IND ) {

        _tts_rdy_f = 1;

    } else if ( rsp_idx == ISC_SPCODEC_READY_IND ) {

        _spc_rdy_f = 1;

    } else if ( rsp_idx == ISC_TTS_FINISHED_IND ) {

        _tts_fin_f = 1;

    } else if ( rsp_idx == ISC_SPCODEC_FINISHED_IND ) {

        _spc_fin_f = 1;
    }

    return 0;
}

void tts_init()
{
    _req_err = 0;
    _err_code = 0;

    _tts_rdy_f = true;
    _spc_rdy_f = true;
    _tts_fin_f = true;
    _spc_fin_f = true;

    _audio_conf.as = 0x00;
    _audio_conf.ag = 0x43;
    _audio_conf.amp = 0x00;
    _audio_conf.asr = ASR_11KHZ;
    _audio_conf.ar = 0x00;
    _audio_conf.atc = 0x00;
    _audio_conf.acs = 0x00;
    _audio_conf.dc = 0x00;

    _tts_conf.sr = 0x01;
    _tts_conf.voice = 0x00;
    _tts_conf.ep = 0x00;
    _tts_conf.lang = TTSV_US;
    _tts_conf.sr_wpm_lsb = 0xc8;
    _tts_conf.sr_wpm_msb = 0x00;
    _tts_conf.ds = 0x00;
    _tts_conf.res = 0x00;

    _pman_conf.am_lsb = 0x01;
    _pman_conf.am_msb = 0x00;
    _pman_conf.spi_clk = 0x01;
    _pman_conf.pad = PADDING_BYTE;

    _flush_enable = false;

    _msg_block_callback = NULL;
    _fatal_err_callback = NULL;
    _err_callback = NULL;

    tts_hw_init();
}

void tts_msg_block_callback( void( *msg_blk_ptr )( uint16_t *req_ptr,
                                                   uint16_t *err_ptr ) )
{
    _msg_block_callback = msg_blk_ptr;
}

void tts_fatal_err_callback( void( *fatal_err_ptr )( uint16_t *err_ptr ) )
{
    _fatal_err_callback = fatal_err_ptr;
}

void tts_err_callback( void( *error_ptr )( uint16_t *err_ptr ) )
{
    _err_callback = error_ptr;
}

void tts_mute()
{
    tts_mute_cmd( true );
}

void tts_unmute()
{
    tts_mute_cmd( false );
}


void tts_setup()
{
    //check HW version
    tts_version_boot(); 
    
    int succ = tts_image_load( (uint8_t*)TTS_INIT_DATA, sizeof( TTS_INIT_DATA ) );
   
    if ( succ != 0x0001 )
    {
            // image load failed, try turning it off and on again
            pc.printf("tts init data failed!\n\r");
            pc.printf("returned value: %X\n\r", succ);
            while(1);
    }        
    pc.printf("tts image load done\n\r"); 
    succ =  tts_image_exec();
    
    if ( succ != 0x0001 )
    {
        // image boot failed, try turning it off and on again
        pc.printf("tts image exec failed!\n\r");
        pc.printf("returned value: %X\n\r", succ);
        while(1);
    } 
    pc.printf("tts image exec done\n\r");
    
    tts_interface_test();
    pc.printf("tts interface test done\n\r");
    
    tts_power_default_config();
    pc.printf("tts power default done\n\r");
    
    tts_audio_default_config();
    pc.printf("tts audio default done\n\r");
    
    tts_volume_set( 0 );
    pc.printf("tts volume set done\n\r");
    
    tts_default_config();
    pc.printf("tts default config done\n\r");
}

void tts_version_boot( void )
{
    
    uint8_t tmp_resp[ 16 ] = { 0 };

    wait( RESET_TO_BOOT_TIME );
   
    tts_parse_req( ISC_VERSION_REQ_BOOT, NULL, 0 );
   
    while( !tts_rsp_chk( ISC_VERSION_RESP_BOOT ) )
    {
        tts_get_resp();
    }
    tts_hal_write_pad( 16);
    tts_rsp_data( tmp_resp );
    pc.printf("hwver0 %X \n\r", tmp_resp[ 0 ]);
    pc.printf("hwver1 %X \n\r", tmp_resp[ 1 ]);
}

uint16_t tts_image_load(const uint8_t *image,
                         uint16_t count )
{
    uint16_t tmp_resp = 0;
    uint16_t index = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
   
    while ( ( count - index ) > ( BOOT_MESSAGE_MAX - 4  ) )
    {
       
       tts_parse_boot_img( image + index, BOOT_MESSAGE_MAX - 4  );
       wait(0.01);
       index += ( BOOT_MESSAGE_MAX - 4 );
    }
    tts_parse_boot_img( image + index, count - index );
    wait(0.01);           
    
    while( !tts_rsp_chk( ISC_BOOT_LOAD_RESP ) )
    {
        tts_get_resp();
          
        if( _parse_ind() )
            return _err_code;
    }
    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;
  
    return tmp_resp;
}

uint16_t tts_image_exec()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    tts_parse_req( ISC_BOOT_RUN_REQ, NULL, 0 );
     
    while( !tts_rsp_chk( ISC_BOOT_RUN_RESP ) )
    {
       tts_get_resp();
       
        if( _parse_ind() )
            return _err_code;
    }
    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return  tmp_resp;
}

uint16_t tts_interface_test()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    
    wait( BOOT_TO_MAIN_MODE );
    
    tts_parse_req( ISC_TEST_REQ, test, 8 );
           
    while( !tts_rsp_chk( ISC_TEST_RESP ) )
    {
        tts_get_resp();
      
        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

 uint16_t tts_version_main( VER_t *buffer )
{
    char tmp_char[ 3 ] = { 0 };
    uint32_t tmp_fwf = 0;
    uint32_t tmp_fwef = 0;
    uint8_t tmp_resp[ 20 ] = { 0 };

    tts_parse_req( ISC_VERSION_REQ_MAIN, NULL, 0 );

    while( !tts_rsp_chk( ISC_VERSION_RESP_MAIN ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( tmp_resp );
    sprintf(tmp_char, "%c", tmp_resp[ 0 ]);
    strcpy( buffer->hwver, tmp_char );
    strcat( buffer->hwver, "." );
    sprintf(tmp_char, "%c", tmp_resp[ 1 ]);
    strcat( buffer->hwver, tmp_char );
    sprintf(tmp_char, "%c", tmp_resp[ 2 ]);
    strcpy( buffer->fwver, tmp_char );
    strcat( buffer->fwver, "." );
    sprintf(tmp_char, "%c", tmp_resp[ 3 ]);
    strcat( buffer->fwver, tmp_char );
    strcat( buffer->fwver, "." );
    sprintf(tmp_char, "%c", tmp_resp[ 12 ]);
    strcat( buffer->fwver, tmp_char );

    tmp_fwf |= tmp_resp[ 4 ];
    tmp_fwf |= tmp_resp[ 5 ] << 8;
    tmp_fwf |= tmp_resp[ 6 ] << 16;
    tmp_fwf |= tmp_resp[ 7 ] << 24;
    buffer->fwf = (FF_t)tmp_fwf;
    tmp_fwef |= tmp_resp[ 8 ];
    tmp_fwef |= tmp_resp[ 9 ] << 8;
    tmp_fwef |= tmp_resp[ 10 ] << 16;
    tmp_fwef |= tmp_resp[ 11 ] << 24;
    buffer->fwef = (EFF_t)tmp_fwef;

    return 0x0000;
}

uint16_t tts_power_default_config()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    tts_parse_req( ISC_PMAN_CONFIG_REQ, ( uint8_t* )&_pman_conf, 4 );

    while( !tts_rsp_chk( ISC_PMAN_CONFIG_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_standby_enter()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    tts_parse_req( ISC_PMAN_STANDBY_ENTRY_REQ, NULL, 0 );

    while( !tts_rsp_chk( ISC_PMAN_STANDBY_ENTRY_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_standby_exit()
{
    wait( STBY_MODE_ENTERY );
    tts_parse_req( ISC_PMAN_STANDBY_EXIT_IND, NULL, 0 );

    while( !tts_rsp_chk( ISC_PMAN_STANDBY_EXIT_IND ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    return 0x0000;
}

uint16_t tts_audio_default_config()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    tts_parse_req( ISC_AUDIO_CONFIG_REQ, ( uint8_t* )&_audio_conf, 8 );

    while( !tts_rsp_chk( ISC_AUDIO_CONFIG_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_audio_config( int8_t audio_gain,
                           ASR_t sample_rate,
                           bool dac_control )
{
    ACONF_t audio_conf;
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    if ( audio_gain < -48 || audio_gain > 18 )
        return 0xFFFF;

    if ( sample_rate != 0 || sample_rate != 1 || sample_rate != 3 )
        return 0xFFFF;

    audio_conf.ag = ( uint8_t )audio_gain;
    audio_conf.asr = sample_rate;
    audio_conf.dc = dac_control;

    tts_parse_req( ISC_AUDIO_CONFIG_REQ, ( uint8_t* )&audio_conf, 8 );

    while( !tts_rsp_chk( ISC_AUDIO_CONFIG_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_volume_set( int16_t gain )
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_gain[ 2 ] = { 0 };

    tmp_gain[ 0 ] = gain & 0x00FF;
    tmp_gain[ 1 ] = ( gain & 0xFF00 ) >> 8;

    tts_parse_req( ISC_AUDIO_VOULME_REQ, tmp_gain, 2 );

    while( !tts_rsp_chk( ISC_AUDIO_VOLUME_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_audio_mute()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_mute[ 2 ] = { 1, 0 };

    tts_parse_req( ISC_AUDIO_MUTE_REQ, tmp_mute, 2 );

    while( !tts_rsp_chk( ISC_AUDIO_MUTE_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_audio_unmute()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_mute[ 2 ] = { 0, 0 };

    tts_parse_req( ISC_AUDIO_MUTE_REQ, tmp_mute, 2 );

    while( !tts_rsp_chk( ISC_AUDIO_MUTE_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_default_config()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    tts_parse_req( ISC_TTS_CONFIG_REQ, ( uint8_t* )&_tts_conf, 8 );

    while( !tts_rsp_chk( ISC_TTS_CONFIG_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_config( uint8_t voice_type,
                     bool epson_parse,
                     TTSV_t language,
                     uint16_t speaking_rate )
{
    TTSCONF_t tts_conf;
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    if ( voice_type > 8 )
        return 0xFFFF;

    if ( language > 4 )
        return 0xFFFF;

    if ( speaking_rate < 0x004B || speaking_rate > 0x0258 )
        return 0xFFFF;

    tts_conf.voice = voice_type;
    tts_conf.ep = epson_parse;
    tts_conf.lang = language;
    tts_conf.sr_wpm_lsb = ( speaking_rate & 0x00FF );
    tts_conf.sr_wpm_msb = ( speaking_rate & 0xFF00 ) >> 8;

    tts_parse_req( ISC_TTS_CONFIG_REQ, ( uint8_t* )&tts_conf, 8 );

    while( !tts_rsp_chk( ISC_TTS_CONFIG_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_speak( char *word )
{
    bool tmp_f              = false;
    char *wptr              = word;
    uint8_t raw_resp[ 2 ]   = { 0 };
    uint16_t tmp_resp       = 0;
    uint32_t wlen           = strlen( wptr );

    tts_parse_speak_req( ISC_TTS_SPEAK_REQ, _flush_enable, wptr, wlen );

    _tts_rdy_f = 0;
    _tts_fin_f = 0;

    while( !( tmp_f && _tts_rdy_f ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;

        if( tts_rsp_chk( ISC_TTS_SPEAK_RESP ) )
        {
            tts_rsp_data( raw_resp );

            tmp_resp |= raw_resp[ 0 ];
            tmp_resp |= raw_resp[ 1 ] << 8;
            tmp_f = true;
        }
    }

    return tmp_resp;
}

uint16_t tts_pause( void )
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_pause[ 2 ] = { 1, 0 };

    tts_parse_req( ISC_TTS_PAUSE_REQ, tmp_pause, 2 );

    while( !tts_rsp_chk( ISC_TTS_PAUSE_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_unpause( void )
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_pause[ 2 ] = { 0 };

    tts_parse_req( ISC_TTS_PAUSE_REQ, tmp_pause, 2 );

    while( !tts_rsp_chk( ISC_TTS_PAUSE_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_stop( bool reset )
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_reset[ 2 ] = { 0 };

    if( reset )
        tmp_reset[ 0 ] = 0x01;

    tts_parse_req( ISC_TTS_STOP_REQ, tmp_reset, 2 );

    while( !tts_rsp_chk( ISC_TTS_STOP_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_user_dict( bool erase,
                        uint8_t *udict_data,
                        uint16_t count )
{
    uint16_t cnt = 2;
    uint16_t tmp_rsp = 0;
    uint8_t rsp_data[ 2 ] = { 0 };
    uint8_t tmp_data[ BOOT_MESSAGE_MAX ] = { 0 };

    if ( erase )
        tmp_data[ 0 ] = 1;

    while ( count-- )
        tmp_data[ cnt ++ ] = *( udict_data++ );

    tts_parse_req( ISC_TTS_UDICT_DATA_REQ, tmp_data, count + 2 );

    while( !tts_rsp_chk( ISC_TTS_UDICT_DATA_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( rsp_data );

    tmp_rsp |= rsp_data[ 0 ];
    tmp_rsp |= rsp_data[ 1 ] << 8;

    return tmp_rsp;
}

uint16_t tts_codec_configure()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_codec[ 32 ] = { 0 };

    tmp_codec[ 0 ] = 0x01;
    tmp_codec[ 1 ] = 0x01;
    tmp_codec[ 24 ] = 0x02;

    tts_parse_req( ISC_SPCODEC_CONFIG_REQ , tmp_codec, 32 );

    while( !tts_rsp_chk( ISC_SPCODEC_CONFIG_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_codec_start( uint8_t *codec_data,
                          uint16_t count )
{
    bool tmp_f = false;
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };

    if ( count != 512 || count != 1024 || count != 2048 )
        return 0xFFFF;

    while( !_spc_fin_f )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_parse_req( ISC_SPCODEC_START_REQ , codec_data, count );

    _spc_rdy_f = 0;
    _spc_fin_f = 0;

    while( !( tmp_f && _spc_rdy_f ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;

        if( tts_rsp_chk( ISC_TTS_SPEAK_RESP ) )
        {
            tts_rsp_data( raw_resp );

            tmp_resp |= raw_resp[ 0 ];
            tmp_resp |= raw_resp[ 1 ] << 8;
            tmp_f = true;
        }
    }

    return tmp_resp;
}

uint16_t tts_codec_pause()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_data[ 2 ] = { 1, 0 };

    tts_parse_req( ISC_SPCODEC_PAUSE_REQ , tmp_data, 2 );

    while( !tts_rsp_chk( ISC_SPCODEC_PAUSE_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_codec_unpause()
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_data[ 2 ] = { 0 };

    tts_parse_req( ISC_SPCODEC_PAUSE_REQ , tmp_data, 2 );

    while( !tts_rsp_chk( ISC_SPCODEC_PAUSE_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}

uint16_t tts_codec_stop( bool reset )
{
    uint16_t tmp_resp = 0;
    uint8_t raw_resp[ 2 ] = { 0 };
    uint8_t tmp_data[ 2 ] = { 0 };

    if( reset )
        tmp_data[ 0 ] = 1;

    tts_parse_req( ISC_SPCODEC_STOP_REQ, tmp_data, 2 );

    while( !tts_rsp_chk( ISC_SPCODEC_PAUSE_RESP ) )
    {
        tts_get_resp();

        if( _parse_ind() )
            return _err_code;
    }

    tts_rsp_data( raw_resp );

    tmp_resp |= raw_resp[ 0 ];
    tmp_resp |= raw_resp[ 1 ] << 8;

    return tmp_resp;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
* text_to_speech_hw.c
* -------------------------------------------------------------------------------------------------
* Overview: Mainly read and write functions with parsing
**************************************************************************************************/

void _read_rsp()
{
    uint8_t tmp_byte = 0;
    uint16_t tmp_len = 0;

    tts_hal_cs_low();
    
    tts_hal_read( &tmp_byte, 1 );

    if( tmp_byte == START_MESSAGE )
    {
        tts_hal_read( ( uint8_t* )&_last_rsp, 2 );
        tmp_len |= _last_rsp.len[ 0 ];
        tmp_len |= _last_rsp.len[ 1 ] << 8;
        tts_hal_read( ( uint8_t* )&_last_rsp + 2, tmp_len );
        
    } else {

        wait( 0.005 );
    }
   
    tts_hal_cs_high();
}

void _write_req(bool boot)
{
    uint16_t cnt = 0;
    uint8_t start = START_MESSAGE;

    cnt |= _last_req.len[ 0 ];
    cnt |= _last_req.len[ 1 ] << 8;
    
    tts_hal_cs_low();
    
    
    if(boot) // use for debug - displays the current buffer on serial
    {
        tts_hal_write( &start, 1, true );
        tts_hal_write( ( uint8_t* )&_last_req, cnt, true);
    }
    else
    {
        tts_hal_write( &start, 1, false );
        tts_hal_write( ( uint8_t* ) &_last_req, cnt, false );
    }    
   
    tts_hal_cs_high();
    
}

/******************************************************************************
* Public Function Definitions
*******************************************************************************/
void tts_hw_init( void )
{
    tts_hal_init();

    _ticker_f = false;
    _ticker = 0;

    _last_req.idx[ 0 ] = 0;
    _last_req.idx[ 1 ] = 0;
    _last_req.len[ 0 ] = 0;
    _last_req.len[ 1 ] = 0;
    memset( _last_req.payload, 0, MAIN_MESSAGE_MAX );

    _last_rsp.idx[ 0 ] = 255;
    _last_rsp.idx[ 1 ] = 255;
    _last_rsp.len[ 0 ] = 0;
    _last_rsp.len[ 1 ] = 0;
    memset( _last_rsp.payload, 0, RESP_MESSAGE_MAX );
}

void tts_tick_isr()
{
    _ticker++;

    if( _ticker > 500 )
        _ticker_f == true;
}

void tts_mute_cmd( bool cmd )
{
    if( cmd )
        tts_hal_mut_high();
    else
        tts_hal_mut_low();
}

void tts_parse_req( uint16_t req,
                    uint8_t *payload,
                    uint16_t pl_len )
{
    uint8_t *pl = payload;
    uint16_t i = 0;
    uint16_t tmp = pl_len + 4;
     
    _last_req.len[ 0 ] = tmp & 0x00FF;
    _last_req.len[ 1 ] = ( tmp & 0xFF00 ) >> 8;
    _last_req.idx[ 0 ] = req & 0x00FF;
    _last_req.idx[ 1 ] = ( req & 0xFF00 ) >> 8;
    _last_rsp.idx[ 0 ] = 0xFF;
    _last_rsp.idx[ 1 ] = 0xFF;
    
    if ( payload != NULL )
    {
        while ( pl_len-- )
            _last_req.payload[ i++ ] = *( pl++ );
           
    }
    
    _write_req(true);
}

void tts_parse_boot_img( const uint8_t *payload,
                         uint16_t pl_len )
{
    uint16_t i = 0;
    uint16_t tmp = pl_len + 0x04;
    
    _last_req.len[ 0 ] = tmp & 0x00FF;
    _last_req.len[ 1 ] = ( tmp & 0xFF00 ) >> 8;
    _last_req.idx[ 0 ] = 0x00;
    _last_req.idx[ 1 ] = 0x10;
    _last_rsp.idx[ 0 ] = 0xFF;
    _last_rsp.idx[ 1 ] = 0xFF;

    if ( payload != NULL )
    {
        while ( pl_len-- )
            _last_req.payload[ i++ ] = payload[ i ];  
    }

    _write_req(true);
}

void tts_parse_speak_req( uint16_t req,
                          uint8_t flush_en,
                          char *word,
                          uint16_t word_len )
{
    char *ptr = word;
    uint16_t i = 1;
    uint16_t tmp = word_len;

    word_len += 7;

    _last_req.len[ 0 ] = word_len & 0x00FF;
    _last_req.len[ 1 ] = ( word_len & 0xFF00 ) >> 8;
    _last_req.idx[ 0 ] = req & 0x00FF;
    _last_req.idx[ 1 ] = ( req & 0xFF00 ) >> 8;
    _last_rsp.idx[ 0 ] = 0xFF;
    _last_rsp.idx[ 1 ] = 0xFF;

    if( flush_en )
    {
        _last_req.payload[ 0 ] = 1;

    } else {

        _last_req.payload[ 0 ] = 0;
    }

    while( tmp-- )
        _last_req.payload[ i++ ] = *( ptr++ );

    _last_req.payload[ i++ ] = 0x20;
    _last_req.payload[ i ] =   0x00;

    _write_req(true);
}

void tts_get_resp()
{
    // only read if ready
    if( tts_hal_msg_rdy() )
    {
        _read_rsp();
    }
}

bool tts_rsp_chk( uint16_t idx )
{
    uint16_t tmp = 0;

    tmp |= _last_rsp.idx[ 0 ];
    tmp |= _last_rsp.idx[ 1 ] << 8;
 
   return ( idx == tmp ) ? true : false;
}

uint16_t tts_rsp_idx()
{
    uint16_t tmp = 0;

    tmp |= _last_rsp.idx[ 0 ];
    tmp |= _last_rsp.idx[ 1 ] << 8;

    return tmp;
}

void tts_rsp_data( uint8_t *buffer )
{
    uint8_t *bfr = buffer;
    uint16_t cnt = 0;
    uint8_t *ptr = _last_rsp.payload;

    cnt |= _last_rsp.len[ 0 ];
    cnt |= _last_rsp.len[ 1 ] << 8;
    cnt -= 4;

    while( cnt-- )
        *( bfr++ ) = *( ptr++ );
}

/*************** END OF FUNCTIONS *********************************************/