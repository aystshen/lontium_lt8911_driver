# Lontium LT8911 Driver
This is the Linux driver for Lontium LT8911, which is used to convert LVDS to edp output.  

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be a reference to you, when you are integrating the Lontium's LT8911 IC into your system, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

## Usage
1. Copy lt8911 to kernel/drivers/video/ directory.
2. Modify the kernel/drivers/video/Kconfig as follows
```
source "drivers/video/lt8911/Kconfig"
```
3. Modify the kernel/drivers/video/Makefile as follows
```
obj-$(CONFIG_LT8911_DRIVER) += lt8911/
```
4. Modify the dts as follows
```
	&i2c2 {  
		status = "okay";
		lt8911@29 {
			compatible = "lontium,lt8911";
			reg = <0x29>;
			power-gpio = <&gpio1 GPIO_B4 GPIO_ACTIVE_HIGH>;
			reset-gpio = <&gpio0 GPIO_A2 GPIO_ACTIVE_HIGH>;
		};
	};
```

## Notes:
1. IIC address of LT8911:  
  A) If LT8911's number 13 (S_ADR) is low, the I2C address of LT8911 is 0x52, and bit0 is the read-write mark bit.  
  If it is a Linux system, the I2C address of LT8911 is 0x29, and bit7 is the read and write flag bit.  
  B) If LT8911's 13th foot (S_ADR) is high, then LT8911's I2C address is 0x5a, and bit0 is the read-write mark bit.  
  If it is a Linux system, the I2C address of LT8911 is 0x2d, and bit7 is the read and write flag bit.
2. The IIC rate should not exceed 100KHz. 
3. To ensure that LVDS signal is given to LT8911, then initialize LT8911. 
4. The front-end master control GPIO is required to reply to LT8911. Before the register, the LT8911 is reset. Use GPIO to lower LT8911's reset foot 100ms, then pull up and maintain 100ms.

## Developed By
* ayst.shen@foxmail.com

## License
	Copyright 2012-2014 Jeremy Feinstein

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
