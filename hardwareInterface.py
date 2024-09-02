from threading import Thread

# For serial (including USB-to-serial) interfaces:
# https://pyserial.readthedocs.io/en/latest/pyserial.html
# Install pyserial library:
#   python -m pip install pyserial
# List ports:
#   python -m serial.tools.list_ports

import paho.mqtt.client as mqtt
import serial  # the pyserial
from paho.mqtt import subscribe
from serial.tools.list_ports import comports
from time import sleep, time
from configmodule import getConfigValue, getConfigValueBool
import sys  # For exit_on_session_end hack

PinCp = "P8_18"
PinPowerRelay = "P8_16"

if (getConfigValue("digital_output_device")=="beaglebone"):
    # In case we run on beaglebone, we want to use GPIO ports.
    import Adafruit_BBIO.GPIO as GPIO

if (getConfigValue("charge_parameter_backend")=="chademo"):
    # In case we use the CHAdeMO backend, we want to use CAN
    import can

class hardwareInterface():
    def needsSerial(self):
        # Find out, whether we need a serial port. This depends on several configuration items.
        if (getConfigValueBool("display_via_serial")):
            return True # a display is expected to be connected to serial port.
        if (getConfigValue("digital_output_device")=="dieter"):
            return True # a "dieter" output device is expected to be connected on serial port.
        if (getConfigValue("analog_input_device")=="dieter"):
            return True # a "dieter" input device is expected to be connected on serial port.
        if (getConfigValue("digital_output_device")=="celeron55device"):
            pass # return True
        if (getConfigValue("analog_input_device")=="celeron55device"):
            pass # return True
        return False # non of the functionalities need a serial port.
        
    def findSerialPort(self):
        baud = int(getConfigValue("serial_baud"))
        if (getConfigValue("serial_port")!="auto"):
            port = getConfigValue("serial_port")
            try:
                self.addToTrace("Using serial port " + port)
                self.ser = serial.Serial(port, baud, timeout=0)
                self.isSerialInterfaceOk = True
            except:
                if (self.needsSerial()):
                    self.addToTrace("ERROR: Could not open serial port.")
                else:
                    self.addToTrace("Could not open serial port, but also do not need it. Ok.")
                self.ser = None
                self.isSerialInterfaceOk = False
            return

        ports = []
        self.addToTrace('Auto detection of serial ports. Available serial ports:')
        for n, (port, desc, hwid) in enumerate(sorted(comports()), 1):
            if (port=="/dev/ttyAMA0"):
                self.addToTrace("ignoring /dev/ttyAMA0, because this is not an USB serial port")
            else:
                self.addToTrace('{:2}: {:20} {!r}'.format(n, port, desc))
                ports.append(port)
        if (len(ports)<1):
            if (self.needsSerial()):
                self.addToTrace("ERROR: No serial ports found. No hardware interaction possible.")
                self.ser = None
                self.isSerialInterfaceOk = False
            else:
                self.addToTrace("We found no serial port, but also do not need it. No problem.")
                self.ser = None
                self.isSerialInterfaceOk = False
        else:
            self.addToTrace("ok, we take the first port, " + ports[0])
            try:
                self.ser = serial.Serial(ports[0], baud, timeout=0)
                self.isSerialInterfaceOk = True
            except:
                self.addToTrace("ERROR: Could not open serial port.")
                self.ser = None
                self.isSerialInterfaceOk = False

    def addToTrace(self, s):
        self.callbackAddToTrace("[HARDWAREINTERFACE] " + s)            

    def setStateB(self):
        self.addToTrace("Setting CP line into state B.")
        self.mqttc.publish("plc/cp_line", "B")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinCp, GPIO.LOW)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            pass # self.ser.write(bytes("cp=0\n", "utf-8"))
        self.outvalue &= ~1
        
    def setStateC(self):
        self.addToTrace("Setting CP line into state C.")
        self.mqttc.publish("plc/cp_line", "C")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinCp, GPIO.HIGH)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            pass # self.ser.write(bytes("cp=1\n", "utf-8"))
        self.outvalue |= 1
        
    def setPowerRelayOn(self):
        self.addToTrace("Switching PowerRelay ON.")
        self.mqttc.publish("plc/contactor/main", "on")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinPowerRelay, GPIO.HIGH)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            pass # self.ser.write(bytes("contactor=1\n", "utf-8"))
        self.outvalue |= 2

    def setPowerRelayOff(self):
        self.addToTrace("Switching PowerRelay OFF.")
        self.mqttc.publish("plc/contactor/main", "off")
        if (getConfigValue("digital_output_device")=="beaglebone"):
            GPIO.output(PinPowerRelay, GPIO.LOW)
        if (getConfigValue("digital_output_device")=="celeron55device"):
            pass # self.ser.write(bytes("contactor=0\n", "utf-8"))
        self.outvalue &= ~2

    def setRelay2On(self):
        self.addToTrace("Switching Relay2 ON.")
        self.mqttc.publish("plc/contactor/secondary", "on")
        self.outvalue |= 4

    def setRelay2Off(self):
        self.addToTrace("Switching Relay2 OFF.")
        self.mqttc.publish("plc/contactor/secondary", "off")
        self.outvalue &= ~4
        
    def getPowerRelayConfirmation(self):
        return self.contactor_confirmed
        
    def triggerConnectorLocking(self):
        self.addToTrace("Locking the connector")
        self.mqttc.publish("plc/pluglock", "locked")
        if (getConfigValue("digital_output_device")=="celeron55device"):
            pass # self.ser.write(bytes("lock\n", "utf-8"))
        # todo control the lock motor into lock direction until the end (time based or current based stopping?)

    def triggerConnectorUnlocking(self):
        self.addToTrace("Unocking the connector")
        self.mqttc.publish("plc/pluglock", "unlocked")
        if (getConfigValue("digital_output_device")=="celeron55device"):
            pass # self.ser.write(bytes("unlock\n", "utf-8"))
        # todo control the lock motor into unlock direction until the end (time based or current based stopping?)

    def isConnectorLocked(self):
        # TODO: Read the lock= value from the hardware so that this works
        return 1
        
    def setChargerParameters(self, maxVoltage, maxCurrent):
        self.maxChargerVoltage = int(maxVoltage)
        self.maxChargerCurrent = int(maxCurrent)
        
    def setChargerVoltageAndCurrent(self, voltageNow, currentNow):
        self.chargerVoltage = int(voltageNow)
        self.chargerCurrent = int(currentNow)
        
    def setPowerSupplyVoltageAndCurrent(self, targetVoltage, targetCurrent):
        # if we are the charger, and have a real power supply which we want to control, we do it here
        self.homeplughandler.sendSpecialMessageToControlThePowerSupply(targetVoltage, targetCurrent)

    def getInletVoltage(self):
        # uncomment this line, to take the simulated inlet voltage instead of the really measured
        # self.inletVoltage = self.simulatedInletVoltage
        return self.inletVoltage
        
    def getAccuVoltage(self):
        return self.accuVoltage

    def getAccuMaxCurrent(self):
        if (getConfigValue("digital_output_device")=="celeron55device"):
            # The overall current limit is currently hardcoded in
            # OpenV2Gx/src/test/main_commandlineinterface.c
            EVMaximumCurrentLimit = 250
            if self.accuMaxCurrent >= EVMaximumCurrentLimit:
                return EVMaximumCurrentLimit
            return self.accuMaxCurrent
        elif getConfigValue("charge_parameter_backend")=="chademo":
            return self.accuMaxCurrent #set by CAN        
        #todo: get max charging current from the BMS
        self.accuMaxCurrent = 10
        return self.accuMaxCurrent

    def getAccuMaxVoltage(self):
        if getConfigValue("charge_parameter_backend")=="chademo":
            return self.accuMaxVoltage #set by CAN
        elif getConfigValue("charge_target_voltage"):
            self.accuMaxVoltage = getConfigValue("charge_target_voltage")
        return self.accuMaxVoltage

    def getIsAccuFull(self):
        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.IsAccuFull = (self.soc_percent >= 98)
        else:
            #todo: get "full" indication from the BMS
            self.IsAccuFull = (self.simulatedSoc >= 98)
        return self.IsAccuFull

    def getSoc(self):
        if self.callbackShowStatus:
            self.callbackShowStatus(format(self.soc_percent,".1f"), "soc")
        if (getConfigValue("digital_output_device")=="celeron55device"):
            return self.soc_percent
        #todo: get SOC from the BMS
        self.callbackShowStatus(format(self.simulatedSoc,".1f"), "soc")
        return self.simulatedSoc

    def isUserAuthenticated(self):
        # If the user needs to authorize, fill this function in a way that it returns False as long as
        # we shall wait for the users authorization, and returns True if the authentication was successfull.
        # Discussing here: https://github.com/uhi22/pyPLC/issues/28#issuecomment-2230656379
        # For testing purposes, we just use a counter to decide that we return
        # once "ongoing" and then "finished".
        if (self.demoAuthenticationCounter<1):
            self.demoAuthenticationCounter += 1
            return False
        else:
            return True

    def initPorts(self):
        if (getConfigValue("charge_parameter_backend") == "chademo"):
            filters = [
               {"can_id": 0x100, "can_mask": 0x7FF, "extended": False},
               {"can_id": 0x101, "can_mask": 0x7FF, "extended": False},
               {"can_id": 0x102, "can_mask": 0x7FF, "extended": False}]
            self.canbus = can.interface.Bus(bustype='socketcan', channel="can0", can_filters = filters)
    
        if (getConfigValue("digital_output_device") == "beaglebone"):
            # Port configuration according to https://github.com/jsphuebner/pyPLC/commit/475f7fe9f3a67da3d4bd9e6e16dfb668d0ddb1d6
            GPIO.setup(PinPowerRelay, GPIO.OUT) #output for port relays
            GPIO.setup(PinCp, GPIO.OUT) #output for CP

    def mqttsub(self, topics: list[str], callback: callable, qos: int = 0):
        t = Thread(
            target=subscribe.callback,
            args=(callback, topics,),
            kwargs={
                "auth":{'username': "elpis", 'password': "adventure"},
                "hostname":'100.69.1.105',
                "userdata": self}
        )
        t.start()

    def __init__(self, mqttc: mqtt.Client, callbackAddToTrace=None, callbackShowStatus=None, homeplughandler=None):
        self.callbackAddToTrace = callbackAddToTrace
        self.callbackShowStatus = callbackShowStatus
        self.homeplughandler = homeplughandler
        self.mqttc = mqttc
        self.loopcounter = 0
        self.outvalue = 0
        self.simulatedSoc = 20.0 # percent
        self.demoAuthenticationCounter = 0

        self.inletVoltage = 0.0 # volts
        self.accuVoltage = 0.0
        self.lock_confirmed = False  # Confirmation from hardware
        self.cp_pwm = 0.0
        self.soc_percent = 0.0
        self.capacity = 0.0
        self.accuMaxVoltage = 0.0
        self.accuMaxCurrent = 0.0
        self.contactor_confirmed = False  # Confirmation from hardware
        self.plugged_in = None  # None means "not known yet"
        self.lastReceptionTime = 0

        self.maxChargerVoltage = 0
        self.maxChargerCurrent = 10
        self.chargerVoltage = 0
        self.chargerCurrent = 0

        self.logged_inlet_voltage = None
        self.logged_dc_link_voltage = None
        self.logged_cp_pwm = None
        self.logged_max_charge_a = None
        self.logged_soc_percent = None
        self.logged_contactor_confirmed = None
        self.logged_plugged_in = None

        self.rxbuffer = ""

        self.findSerialPort()
        self.initPorts()

        self.mqttsub([
            "plc/params/accuvoltage",
            "plc/params/dc_link_v",
            "plc/params/accuMaxCurrent", "plc/params/max_charge_a",
            "plc/params/accuMaxVoltage", "plc/params/charge_target_voltage",
            "plc/params/inlet_voltage",
            "plc/params/soc_percent",
            "plc/params/cp_pwm",
            "plc/params/cp_output_state",
            "plc/params/contactor_confirmed",
            "plc/params/plugged_in",
        ], mqttcallback)



    def resetSimulation(self):
        self.simulatedInletVoltage = 0.0 # volts
        self.simulatedSoc = 20.0 # percent
        self.demoAuthenticationCounter = 0
        
    def simulatePreCharge(self):
        if (self.simulatedInletVoltage<230):
            self.simulatedInletVoltage = self.simulatedInletVoltage + 1.0 # simulate increasing voltage during PreCharge

    def close(self):
        if (self.isSerialInterfaceOk):        
            self.ser.close()

    def evaluateReceivedData_dieter(self, s):
        self.rxbuffer += s
        x=self.rxbuffer.find("A0=")
        if (x>=0):
            s = self.rxbuffer[x+3:x+7]
            if (len(s)==4):
                try:
                    self.inletVoltage = int(s) / 1024.0 * 1.08 * (6250) / (4.7+4.7)
                    if (getConfigValue("analog_input_device")=="dieter"):
                        self.callbackShowStatus(format(self.inletVoltage,".1f"), "uInlet")
                except:
                    # keep last known value, if nothing new valid was received.
                    pass
                #self.addToTrace("RX data ok " + s)
                self.rxbuffer = self.rxbuffer[x+3:] # consume the receive buffer entry

    def evaluateReceivedData_celeron55device(self, s):
        pass

    def showOnDisplay(self, s1, s2, s3):
        pass
        
    def mainfunction(self):
        # print(self.soc_percent)
        if (getConfigValueBool("soc_simulation")):
            if (self.simulatedSoc<100):
                if ((self.outvalue & 2)!=0):
                    # while the relay is closed, simulate increasing SOC
                    deltaSoc = 0.5 # how fast the simulated SOC shall rise.
                    # Examples:
                    #  0.01 charging needs some minutes, good for light bulb tests
                    #  0.5 charging needs ~8s, good for automatic test case runs.
                    self.simulatedSoc = self.simulatedSoc + deltaSoc
                
        if (getConfigValue("charge_parameter_backend")=="chademo"):
           self.mainfunction_chademo()
        
        if (getConfigValue("digital_output_device")=="dieter"):
            self.mainfunction_dieter()

        if (getConfigValue("digital_output_device")=="celeron55device"):
            self.mainfunction_celeron55device()

        if getConfigValueBool("exit_on_session_end"):
            # TODO: This is a hack. Do this in fsmPev instead and publish some
            # of these values into there if needed.
            if (self.plugged_in is not None and self.plugged_in == False and
                    self.inletVoltage < 50):
                sys.exit(0)

    def mainfunction_dieter(self):
        self.loopcounter+=1
        if (self.isSerialInterfaceOk):
            if (self.loopcounter>15):
                self.loopcounter=0
                # self.ser.write(b'hello world\n')
                s = "000" + str(self.outvalue)
                self.ser.write(bytes("do"+s+"\n", "utf-8")) # set outputs of dieter, see https://github.com/uhi22/dieter
            s = self.ser.read(100)
            if (len(s)>0):
                try:
                    s = str(s, 'utf-8').strip()
                except:
                    s = "" # for the case we received corrupted data (not convertable as utf-8)
                self.addToTrace(str(len(s)) + " bytes received: " + s)
                self.evaluateReceivedData_dieter(s)

    def mainfunction_celeron55device(self):
        pass
                
    def mainfunction_chademo(self):
       message = self.canbus.recv(0)
       
       if message:
          if message.arbitration_id == 0x100:
             vtg = (message.data[1] << 8) + message.data[0]
             if self.accuVoltage != vtg:
                 self.addToTrace("CHAdeMO: Set battery voltage to %d V" % vtg)
             self.accuVoltage = vtg
             if self.capacity != message.data[6]:
                 self.addToTrace("CHAdeMO: Set capacity to %d" % message.data[6])
             self.capacity = message.data[6]
             
             msg = can.Message(arbitration_id=0x108, data=[ 0, self.maxChargerVoltage & 0xFF, self.maxChargerVoltage >> 8, self.maxChargerCurrent, 0, 0, 0, 0], is_extended_id=False)
             self.canbus.send(msg)
             #Report unspecified version 10, this makes our custom implementation send the momentary
             #battery voltage in 0x100 bytes 0 and 1
             status = 4 if self.maxChargerVoltage > 0 else 0  #report connector locked
             msg = can.Message(arbitration_id=0x109, data=[ 10, self.chargerVoltage & 0xFF, self.chargerVoltage >> 8, self.chargerCurrent, 0, status, 0, 0], is_extended_id=False)
             self.canbus.send(msg)
             
          if message.arbitration_id == 0x102:
             vtg = (message.data[2] << 8) + message.data[1]
             if self.accuMaxVoltage != vtg:
                 self.addToTrace("CHAdeMO: Set target voltage to %d V" % vtg)
             self.accuMaxVoltage = vtg
             
             if self.accuMaxCurrent != message.data[3]:
                 self.addToTrace("CHAdeMO: Set current request to %d A" % message.data[3])
             self.accuMaxCurrent = message.data[3]
             self.lastReceptionTime = time()
             
             if self.capacity > 0:
                 soc = message.data[6] / self.capacity * 100
                 if self.simulatedSoc != soc:
                     self.addToTrace("CHAdeMO: Set SoC to %d %%" % soc)
                 self.simulatedSoc = soc
       #if nothing was received for over a second, time out        
       if self.lastReceptionTime < (time() - 1):
           if self.accuMaxCurrent != 0:
              self.addToTrace("CHAdeMO: No current limit update for over 1s, setting current to 0")
           self.accuMaxCurrent = 0


def myPrintfunction(s):
    print("myprint " + s)


def mqttcallback(client: mqtt.Client, cls: hardwareInterface, message):
    cls.callbackAddToTrace("%s %s" % (message.topic, message.payload))
    print("%s %s" % (message.topic, message.payload))
    if message.topic in ["plc/params/accuVoltage", "plc/params/dc_link_v"]:
        cls.accuVoltage = int(message.payload)
    if message.topic in ["plc/params/accuMaxCurrent", "plc/params/max_charge_a"]:
        cls.accuMaxCurrent = int(message.payload)
    if message.topic in ["plc/params/accuMaxVoltage", "plc/params/charge_target_voltage"]:
        cls.accuMaxVoltage = int(message.payload)
    if message.topic == "plc/params/inlet_voltage":
        cls.inletVoltage = float(message.payload)
    if message.topic == "plc/params/soc_percent":
        cls.soc_percent = int(message.payload)
    if message.topic == "plc/params/cp_pwm":
        pass
    if message.topic == "plc/params/cp_output_state":
        pass
    if message.topic == "plc/params/contactor_confirmed":
        cls.contactor_confirmed = bool(message.payload)
    if message.topic == "plc/params/plugged_in":
        cls.plugged_in = bool(message.payload)



if __name__ == "__main__":
    print("Testing hardwareInterface...")
    hw = hardwareInterface(myPrintfunction)
    for i in range(0, 350):
        hw.mainfunction()
        if (i==20):
            hw.showOnDisplay("Hello", "A DEMO", "321.0V")
        if (i==50):
            hw.setStateC()
        if (i==100):
            hw.setStateB()
        if (i==150):
            hw.setStateC()
            hw.setPowerRelayOn()
            hw.showOnDisplay("", "..middle..", "")
        if (i==200):
            hw.setStateB()
            hw.setPowerRelayOff()
        if (i==250):
            hw.setRelay2On()
        if (i==300):
            hw.setRelay2Off()
        if (i==320):
            hw.showOnDisplay("This", "...is...", "DONE :-)")
        sleep(0.03)
    hw.close()    
    print("finished.")
