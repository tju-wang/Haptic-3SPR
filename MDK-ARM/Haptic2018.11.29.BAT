SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\Python37-32\Scripts\;C:\Program Files (x86)\Python37-32\;C:\Program Files\Python36\Scripts\;C:\Program Files\Python36\;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files\Microsoft SQL Server\Client SDK\ODBC\110\Tools\Binn\;C:\Program Files (x86)\Microsoft SQL Server\120\Tools\Binn\;C:\Program Files\Microsoft SQL Server\120\Tools\Binn\;C:\Program Files\Microsoft SQL Server\120\DTS\Binn\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Windows\system32\config\systemprofile\.dnx\bin;C:\Program Files\Microsoft DNX\Dnvm\;C:\Program Files\MATLAB\R2016b\runtime\win64;C:\Program Files\MATLAB\R2016b\bin;C:\Program Files\MATLAB\R2016b\polyspace\bin;C:\Program Files\Git\cmd;D:\m_py_model;C:\Users\Mrwang\AppData\Local\Microsoft\WindowsApps;C:\Program Files (x86)\;
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32H743ZITx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Haptic2018.11.29
SET CPU_CLOCK=0x0BEBC200
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\haptic2018.11.29\startup_stm32h743xx._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\usart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_it.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_msp.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_cortex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_tim_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_uart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_uart_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_rcc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_flash.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_flash_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_hsem.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_dma_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_mdma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_pwr.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_pwr_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_i2c.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\stm32h7xx_hal_i2c_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\haptic2018.11.29\system_stm32h7xx.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via ".\Haptic2018.11.29\Haptic2018.11.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" ".\Haptic2018.11.29\Haptic2018.11.29" --i32combined --output ".\Haptic2018.11.29\Haptic2018.11.hex"
