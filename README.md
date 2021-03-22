# MPU6050-Library-CMSIS

This is my first ever library for the ARM CMSIS Platform.

This lib is work in progress so stay tuned.

Right now it only supports the **STM32F10xx**-boards like the
STM32F103C8T6 BluePill Dev Board.
  
I did not test it on other boards but will try to provide it for as many boards as possible.  

Feel free to contribute to the library and tell me if something does not work.  

## Usage
When using Eclipse+Plugins as an IDE just put the downloaded folder into the `src` folder.
<br>
Put the 2 files inside the `src` folder into a Folder called `MPU6050` or similar and put that folder into the `src` folder of your project.
The `delay.h` and `delay.c` can be put directly into the `src` folder of your project.  
<br>
```c
#define "MPU6050/mpu.h"
#define "delay.h"
```
First call the `DelayInit()` function before the while-loop.

```
MPU_init(I2C_TypeDef *I2Cx, uint8_t mpu_address);
```
To Initialize the MPU on the I2C port it is sitting on with the corresponding Addresses:
`MPU_ADDR_DEFAULT` or `MPU_ADDR_SCND`.
<br>
Then you can call all the other functions

## Planned Features

- [ ] add support for more board
- [ ] add more configuration options
- [x] rewrite it to use it directly with the ARM registers without functions
- [ ] add self-test routine
- [ ] enhance the calibration options
- [ ] make more stable
- [ ] add timeout for I2C read

Contact: **sl7@flate.io**
