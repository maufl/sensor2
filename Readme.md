# A ESP based sensor platform
This is a learning project for me. The goal is to build sensors (and maybe eventually actors) using ESP32 micro-controllers that can run on battery for a long time (a year and more).
I started it to learn PCB design and embedded programming.

## Climate sensor
The hardware design for the climate sensor does work now the way I hoped. 
Most of all this means that the linear voltage regulator provides enough power even when sending data over WiFi or BLE 
and that I can put the ESP into deep sleep and the whole board will only consume around 11µA.

Software wise I do have a firmware written in Rust that periodically sends measurements via BLE using the [BTHome](https://bthome.io/) standard and goes into deep sleep during the rest of the time.

Using an small LiPo battery with 700mAh it should last at least a year.

There is an (even more) experimental feature of doing the measurements using the low power core that is not working yet. 
The idea would be to measure with a higher frequency than send measurements with a lower frequency, thus saving power.