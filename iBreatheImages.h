/*
* Bitmaps need to be formatted as 16bppRgb565, flipped vertically
* and a 6 byte header needs to be appended at the start of the array.
* Use the following tool to create arrays for images. 
* https://github.com/MikroElektronika/HEXIWEAR/tree/master/SW/ResourceCollectionTool
* It takes an image and outputs the array. It handles the flipping and the 6 byte header.  
* Image needs to be converted outside the tool to fit the screen (96px by 96px).
*/

#include "stdint.h"

extern const uint8_t iBreatheWS_bmp[18438];
extern const uint8_t iBreatheBlank_bmp[18438];
extern const uint8_t iBreatheBlow_bmp[18438];
extern const uint8_t iBreatheDrink_bmp[18438];
extern const uint8_t iBreatheDrive_bmp[18438];
extern const uint8_t iBreatheHang_bmp[18438];
extern const uint8_t iBreatheini_bmp[18438];
extern const uint8_t iBreatheSober_bmp[18438];
