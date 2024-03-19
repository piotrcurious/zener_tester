# zener_tester
Simple esp32 zener diode testing tool, dreamed by copilot . It allows to quickly identify zener diode or any other diode like LED 

Basic tester looks ok , except read voltage analog calibrated function should be used with esp32. 
Bluedisplay tester looks ok
Both need some tweaks for PWM resolution and other variable consistency fixes.

Tester plotting the voltage drop graphs in function of temperature needs some work. I have bashed copilot to create all the extra bits needed like pid and pid auto tune but spit it out as a puzzle instead of integrating it, so that's it, i have no more time to bash and improve it.

Again it all needs some refactoring and adding more features like ability to set delays by user, and more formatting of the output for external plotting tools 

Temperature plotting system is designed with using adt75 with 12 bit resolution as it's cheap factory calibrated sensor.
Heating and cooling (by f.e. peltier cell ) outputs so one can test semiconductors in wider temperature range.

Have fun
