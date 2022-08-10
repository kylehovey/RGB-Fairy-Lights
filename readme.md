# RGB Fairy Lights

This project includes all of the firmware and 3D files to create a driver for the RGB fairy lights sold on Adafruit's store. In order to control this light, you will need to be running [Home Assistant](https://www.home-assistant.io/) on a server at your house along with a [MQTT](https://mqtt.org/) broker and Home Assistant [MQTT Integration](https://www.home-assistant.io/integrations/mqtt/).

## Materials

* [Adafruit QT Py ESP32-S3](https://www.adafruit.com/product/5426) (other boards will most likely work)
* [Neopixel Fairy Lights](https://www.adafruit.com/product/4917)
* [5V/2A Power Supply](https://www.adafruit.com/product/276)
* [3 Pin JST SM Plug](https://www.adafruit.com/product/1663)
* [Panel Mount 2.1mm Barrel Jack](https://www.adafruit.com/product/610)
* Hookup wire
* Heat Shrink Tubing (highly recommended, but not strictly required)
* USB C Cable
* Some way of mounting your fairy lights where you want them (I used tacks)

## Tools

* Soldering Iron
* Wire Strippers/Cutters
* Solder
* Double-Sided Adhesive (keeps ESP32 in place, I used thick double-sided tape)
* 3D Printer
* Filament

## Printing the Case

I used PLA to print the case. The models are located in the `models` directory of this project, and you can slice them using whatever software you prefer. I used [Cura](https://ultimaker.com/software/ultimaker-cura).

## Assembly

1. Strip the ends of two short wires and solder them to positive and ground on the barrel jack.
2. Add heat shrink for the barrel jack connection.
3. Insert the barrel jack into the case and tighten the nut to fasten it in place.
4. Solder the positive wire to 5V on the ESP32, and ground to ground on the ESP32.
5. Insert the bare wires from the male end of the JST connector into the upper slot on the case (above the USB slot where the ESP32 sits).
6. Plug in the LED strip to the connector and match up the wires to find out which one is:
  * Ground - Black on the strand
  * Positive Voltage - Red on the strand
  * Data - Green on the strand
7. Solder positive to positive on the ESP32 (shares the same positive connection with the power jack wire you just added).
8. Solder ground to ground on the ESP32 (shares the same ground connection with the power jack wire you just added).
9. Add the double-sided tape to the bottom of the ESP32 and insert the USB C port into the slot in the case, then fold the ESP32 into the walled area in the case (it should be snug).
10. Snap the lid onto the case.
11. If using leaded solder, wash your hands and then wipe down the case with a paper towel and throw the towel away afterwards.

## Flashing

Follow the instructions to get [PlatformIO](https://platformio.org/) set up on your system and open this project using it.

1. Open `main.cpp` in the `src` directory and add your wifi network name and password, as well as the IP address of the MQTT broker.
2. Update the IP address settings to match your subnet/router at home.
3. You may need to comment out the `local_IP` and allow the board to get an IP using DHCP, which you will be able to see over the serial monitor. Once you see the IP address, you can uncomment that line and update the IP to the one it received. Alternatively, just comment out this line entirely and always use DHCP (you won't ever need to know the IP address of your light).

Once this is done, connect the ESP32 driver box to your computer via USB C and compile/upload this code to your board.

## Adding To Home Assistant

Ensure that you have installed [mqtt-cli](https://www.npmjs.com/package/mqtt-cli) and run the script `src/publish_discovery.sh`. This will publish and retain a configuration payload for your device. Ostensibly, this could be sent from the ESP32, but I ran into payload size limitations with the PubSub library I used for MQTT on the ESP32 (and you only need to do this once).


## Controlling Your Strand

Now go into Home Assistant and you should see a new device called "Rainbow Fairy Lights". Add it to a dashboard wherever you would like, and control the color/brightness/patterns using Home Assistant's UI. Enjoy!

## Known Problems

* The settable aurora pattern doesn't work yet... I need to find a good formula to convert RGB to hue.
