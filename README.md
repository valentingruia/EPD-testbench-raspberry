# EPD testbench raspberry
The project was created to become familiar and test the Electronic Paper Displays (EPD) in a quick, python based environment.

# Hardware setup:
* I got a development kit with GDEY0266F52H + DESPI-C02 [from aliexpress](www.aliexpress.com/item/1005009549160716.html)
* raspberry pi 2w

![raspi](https://github.com/[username]/[reponame]/blob/[branch]/Doc/Images/setup_raspi.jpg?raw=true)
![display](https://github.com/[username]/[reponame]/blob/[branch]/Doc/Images/setup_display.jpg?raw=true)

## Wire connections:
|EPD pin name|Epd pin #|wire color|Rasp pin #|Rasp pin name|
|:---|:---|:---|:---|:---|
|busy|8|white|11|13|
|reset|7|gray |13|27|
|d/c|6|violet|15|22|
|cs|5|blue |24|8|
|vcc|1|red|17|3v3|
|sdi|3|yellow|19|10/SPI_MOSI|
|Gnd|2|orange|20|GND|
|scl|4|green|23|11/SPI_SCLK|


# Software stack
The python driver is modified from [Inky repository](https://github.com/pimoroni/inky/blob/main/inky/inky_jd79668.py)


I used the environment from pimorony but didn't use the inky library. This dependency can be replaced in the future:
```bash
git clone https://github.com/pimoroni/inky
cd inky
./install.sh
```

**Note** Libraries will be installed in the "pimoroni" virtual environment, you will need to activate it to run examples:

```bash
source ~/.virtualenvs/pimoroni/bin/activate
cd ~/pimoroni
git clone https://github.com/valentingruia/EPD-testbench-raspberry
cd EPD-testbench-raspberry
```

How to start the script:
```bash
source ~/.virtualenvs/pimoroni/bin/activate

cd ~/pimoroni/EPD-testbench-raspberry
python name-badge.py --name "Sherlock Vader"
```
