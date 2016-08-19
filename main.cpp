#include "mbed.h"
#include "Hexi_OLED_SSD1351.h"
#include "OLED_types.h"
#include "OLED_fonts.h"
#include "string.h"

int main() {
    
    //Instantiate Time    
    Timer time;

    //Instantiate the SSD1351 OLED Driver    
    SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15); // (MOSI,SCLK,POWER,CS,RST,DC)
    //Turn on the backlight of the OLED Display 
    oled.DimScreenON();
    //Fills the screen with solid black         
    oled.FillScreen(COLOR_BLACK);
    
    //Set the parameters for desired text properties
     oled_text_properties_t
            textProperties =
            {
            .font       = oledFont_Tahoma_8_Regular,    
            .fontColor  = COLOR_WHITE,
            .alignParam = OLED_TEXT_ALIGN_LEFT, //currently does nothing in this example
            .background = NULL
            };

    oled.SetTextProperties(&textProperties);

    //Approximately 15 characters can fit on one line for this font size. 
    //Extra element for null terminator of string. 
    char text[15+1];
    
    //Display Text at (x=13,y=0). Need to manually center. 
    strcpy((char *) text,"TEXT EXAMPLE");
    oled.DrawText((uint8_t *)text,13,0);
    
    //Change Font to Blue 
    textProperties.fontColor = COLOR_BLUE;
    oled.SetTextProperties(&textProperties);

    //Display text at (x=5,y=24)
    strcpy(text,"Timer(s):");
    oled.DrawText((uint8_t *)text,5,24);
        
    //Set text color to white
    textProperties.fontColor = COLOR_WHITE;
    oled.SetTextProperties(&textProperties);    

    //start timer
    time.start();

    while (true) {

        //Dynamic Text
        
        //Display Time Reading at (x=50, y=24)
        sprintf(text,"%.2f",time.read());
        oled.DrawText((uint8_t *)text,50,24);

        Thread::wait(1000);
    }
}


    
    