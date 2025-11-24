# ad5420
driver for linux-4.9.37

## Тестовый драйвер для hi3518 && linux-4.9.37
### ТЗ:
ЦАП токовой петли AD5420AREZ управляется с помощью следующих выводов:
- iocfg_reg33: сигнал SPI1_SCLK стоит в режиме SPI1_SCLK (Вывод SCLK ЦАПа)
- iocfg_reg35: сигнал SPI1_SDO стоит в режиме SPI1_SDO (Вывод SDIN ЦАПа)
- iocfg_reg40: сигнал GPIO7_3 стоит в режиме GPIO7_3 (Вывод LATCH ЦАПа)
- iocfg_reg33: сигнал GPIO8_5 стоит в режиме GPIO8_5 (Вывод CLEAR ЦАПа)

