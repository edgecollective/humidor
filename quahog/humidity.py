import ujson as json
import urequests as requests
import time
import gc
import machine
from machine import Pin
from machine import SPI
from upy_rfm9x import RFM9x
import ssd1306
from machine import I2C

TIMEOUT = 1.
DISPLAY = True
OLED_LINESKIP=18
OLED_CURRENTLINE=0

feet_per_meter=3.28084

# set up the display
i2c = I2C(-1, Pin(14), Pin(2))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

# set up the 'done' pin
done_pin=Pin(22,Pin.OUT)
done_pin.value(0)

# indicate that we're starting up
oled.fill(0)
oled.text("Listening ...",0,0)
oled.show()
time.sleep(2)


# radio test
sck=Pin(25)
mosi=Pin(33)
miso=Pin(32)
cs = Pin(26, Pin.OUT)
#reset=Pin(13)
led = Pin(13,Pin.OUT)

resetNum=27

spi=SPI(1,baudrate=5000000,sck=sck,mosi=mosi,miso=miso)

rfm9x = RFM9x(spi, cs, resetNum, 915.0)


# set up FARMOS params
url='http://64.227.0.108:9000/posts'

headers = {'Content-type':'application/json', 'Accept':'application/json'}

# wifi parameters
#WIFI_NET = 'Artisan\'s Asylum'
#WIFI_PASSWORD = 'I won\'t download stuff that will get us in legal trouble.'

WIFI_NET = 'InmanSquareOasis'
WIFI_PASSWORD = 'portauprince'

#WIFI_NET = 'Verizon-MiFi8800L-4AA2'
#WIFI_PASSWORD = '640b8aaf'

#WIFI_NET = 'Chappy house'
#WIFI_PASSWORD = 'george5656'

# function for posting data
def post_data():
    print('posting...')
    try:
    	r = requests.post(url,data=json.dumps(payload),headers=headers)
    except Exception as e:
	print(e)
	#r.close()
	return "timeout"
    else:
	r.close()
	print('Status', r.status_code)
   	return "posted"

# function for connecting to wifi
def do_connect():
    import network
    sta_if = network.WLAN(network.STA_IF)	
    if not sta_if.isconnected():
        print('connecting to network...')
	sta_if.active(False)
        sta_if.active(True)
        sta_if.connect(WIFI_NET, WIFI_PASSWORD)
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())

index=0

# main loop
while True:
    rfm9x.receive(timeout=TIMEOUT)
    if rfm9x.packet is not None:
        try:
            packet_text = str(rfm9x.packet, 'ascii')
            rssi=str(rfm9x.rssi)
            oled.fill(0)
            #oled.text("<--",0,0)
            #oled.text(packet_text,0,30)
            oled.text(" received:"+str(index),0,40)
            index = index + 1
            #oled.text(" index:"+ index,0,10)
            oled.show() 
            print('Received: {}'.format(packet_text))
            print("RSSI: {}".format(rssi))
            s=packet_text.replace('\x00','')
            print('full:',s)
            payload=json.loads(s)
            print('payload:',payload)
            #s=s.split(',')
            #devEUI=s[0].split(':')[1].strip().strip('}');
            #print(s)
            #print('devEUI',devEUI)
            #deviceName=s[1].split(':')[1].strip().strip('}');
            #distance=s[2].split(':')[1].strip().strip('}');
            #BatV=s[3].split(':')[1].strip().strip('}');
            #payload ={"devEUI": devEUI,"deviceName":deviceName,"distance":distance,"BatV":BatV,"rssi":rssi}
            print(payload)
            do_connect()
            post_data()
            time.delay(30)
    
        except:
            print("some error?")
        #display_text = "[{}]: (garbled msg)".format(i)
        #update_display(display_text)
	        
	        
    gc.collect()
    
