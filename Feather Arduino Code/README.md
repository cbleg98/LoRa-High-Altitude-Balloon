# GUI Code

The following code is what is needed to flash on a LoRa enabled Featherboard M0 with adafruit display.
This is **based on code from Jonathan Chen** and is modified to recieve/send data as we need it

## RadioHead
This library is included for quick reference, it is used in both of the firmware versions

## feather_datalogger_RX
This code receives and the data from the MSGC LoRa board then sends it on the UART port in a way our python GUI script will understand.

## feather_datalogger_RX_wscreen
This code does the same as the above (receives the data form our board and sends to UART for our GUI script) but it also displays the data
on the adafruit screen attached to the featherboard. If you don't have a screen, probably use the one without the screen.
