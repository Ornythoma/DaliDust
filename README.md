# DaliDust
Workshop for PM2.5 mapping in Dali, Yunnan, China

Work in Progress :)

BOM:
- Bluepill
- BME280
- SD breakout board
- SDS011/SDS018/SDS021
- MT3333/MT3339/SIM28...
- External battery, USB micro cable

Steps: 
- If not done, flash bootloader on Bluepill as per:
  https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki/Flashing-Bootloader-for-BluePill-Boards
- Install Arduino and the STM32 addendum as per:
  https://wiki.stm32duino.com/index.php?title=Installation
  
- Install the libraries: NeoGPS, SdFat, BME280

- Compile DaliDust.ino
- Upload to Bluepill

- Assemble device as per DaliDust.pdf instructions :)
