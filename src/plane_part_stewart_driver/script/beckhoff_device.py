import threading
import time
from ctypes import sizeof

import pyads


class beckhoff_device(object):
    def __init__(self, pyads_addr=None, ip_addr=None):
        if pyads_addr is None:
            pyads_addr = "192.168.1.5.1.1"

        if ip_addr is None:
            ip_addr = "192.168.1.5"

        # plc = pyads.Connection('192.168.1.5.1.1',pyads.PORT_SPS1)

        self.plc = pyads.Connection(pyads_addr, pyads.PORT_TC3PLC1, ip_addr)
        self.plc.open()
        self.basicInterface()
        self.lock = threading.Lock()

    def basicInterface(self):
        override = self.plc.read_by_name(
            "MC_FIFO.stFifoInterface.rOverride", pyads.PLCTYPE_REAL
        )
        print("override=", override)

        override_flag = self.plc.read_by_name(
            "MC_FIFO.stFifoInterface.bSetOverride", pyads.PLCTYPE_BOOL
        )
        print("override=", override_flag)

        self.adsCallStatusNames = [
            ".g_sAxisArr[1].NcToPlc.ActPos",
            ".g_sAxisArr[2].NcToPlc.ActPos",
            ".g_sAxisArr[3].NcToPlc.ActPos",
            ".g_sAxisArr[4].NcToPlc.ActPos",
            ".g_sAxisArr[5].NcToPlc.ActPos",
            ".g_sAxisArr[6].NcToPlc.ActPos",
        ]

    def enableBeckhoffMotor(self, isEnable=True):
        self.lock.acquire()
        self.plc.write_by_name("MC_FIFO.externalEnable", isEnable, pyads.PLCTYPE_BOOL)
        self.lock.release()
        return isEnable

    def readLegControlLength(self):
        self.lock.acquire()
        adsCallStatus = self.plc.read_list_by_name(self.adsCallStatusNames)
        self.lock.release()
        ret = [adsCallStatus[i] / 1000.0 for i in self.adsCallStatusNames]
        return ret

    def close(self):
        self.plc.close()


class beckhoff_device_nofity(beckhoff_device):
    def __init__(self, pyads_addr=None, ip_addr=None):
        super(beckhoff_device_nofity, self).__init__(pyads_addr, ip_addr)
        self.controlInterface()

    def controlInterface(self):
        self.adsCallParaListNames = [
            "MC_FIFO.stADSSharedMem.bReturnSignal",
            "MC_FIFO.stADSSharedMem.nFuntionCode",
            "MC_FIFO.stADSSharedMem.nReturnCode",
            "MC_FIFO.stADSSharedMem.nInputLen",
        ]
        adsCallParaList = self.plc.read_list_by_name(self.adsCallParaListNames)
        arrInput = self.plc.read_by_name(
            "MC_FIFO.stADSSharedMem.arrInput",
            pyads.PLCTYPE_LREAL * adsCallParaList["MC_FIFO.stADSSharedMem.nInputLen"],
        )

        self.AdsCallEvent = threading.Event()

        self.adsCallSignal = self.plc.get_symbol(
            "MC_FIFO.stADSSharedMem.bADSCallSignal"
        )
        self.adsCallSignal.auto_update = True
        atr = pyads.NotificationAttrib(sizeof(pyads.PLCTYPE_BOOL))
        self.adsCallSignal.add_device_notification(self.adsCallback, atr)

    def adsCallback(self, notification, data):
        contents = notification.contents
        # value = next(map(bool,bytearray(contents.data)[0:contents.cbSampleSize]))

        value = contents.data
        # print("recv:",contents.data)
        if value:
            self.AdsCallEvent.set()

    def restartFifo(self):
        self.lock.acquire()
        self.plc.write_by_name(
            "MC_FIFO.stFifoInterface.bStart", True, pyads.PLCTYPE_BOOL
        )
        self.lock.release()
        time.sleep(0.1)

    def startFifoRunning(self):
        self.lock.acquire()
        self.plc.write_by_name(
            "MC_FIFO.stFifoInterface.bChannelStart", True, pyads.PLCTYPE_BOOL
        )
        self.plc.write_by_name(
            "MC_FIFO.stFifoInterface.bSetOverride", True, pyads.PLCTYPE_BOOL
        )
        self.lock.release()

    def enableBeckhoffMotor(self, isEnable=True):
        self.lock.acquire()
        self.plc.write_by_name("MC_FIFO.externalEnable", isEnable, pyads.PLCTYPE_BOOL)
        self.lock.release()
        return isEnable

    def isBeckhoffMotorEnabled(self):
        self.lock.acquire()
        ret = self.plc.read_by_name("MC_FIFO.externalEnable", pyads.PLCTYPE_BOOL)
        self.lock.release()
        return ret

    def disableFifo(self):
        self.lock.acquire()
        self.plc.write_by_name(
            "MC_FIFO.stFifoInterface.bStart", False, pyads.PLCTYPE_BOOL
        )
        self.plc.write_by_name(
            "MC_FIFO.stFifoInterface.bChannelStart", False, pyads.PLCTYPE_BOOL
        )
        self.plc.write_by_name(
            "MC_FIFO.stFifoInterface.bSetOverride", False, pyads.PLCTYPE_BOOL
        )
        self.lock.release()
        return

    def echoTrajectoryProc(self, controllers, wait=0.1):
        directWrite = False
        if not wait > 1e-6:
            directWrite = True

        if directWrite or (self.AdsCallEvent.wait(wait) and self.AdsCallEvent.is_set()):
            self.AdsCallEvent.clear()

            self.lock.acquire()

            adsCallParaList = self.plc.read_list_by_name(self.adsCallParaListNames)
            # print(adsCallParaList)
            paraNum = adsCallParaList["MC_FIFO.stADSSharedMem.nInputLen"]

            if paraNum > 0:
                arrInput = self.plc.read_by_name(
                    "MC_FIFO.stADSSharedMem.arrInput",
                    pyads.PLCTYPE_LREAL
                    * adsCallParaList["MC_FIFO.stADSSharedMem.nInputLen"],
                )
                print(arrInput)

            col = int(arrInput[0])
            row = int(arrInput[1])

            for i in controllers:
                memData, dataLen = i.RunControlStep(col, row)
                memRet = [i * 1000.0 for i in memData]
                if dataLen != 0:
                    break

            if dataLen > 0:
                self.plc.write_by_name(
                    "MC_FIFO.stADSSharedMem.arrOutput",
                    memRet,
                    pyads.PLCTYPE_LREAL * dataLen,
                )
            adsCallParaList["MC_FIFO.stADSSharedMem.bReturnSignal"] = 1
            adsCallParaList["MC_FIFO.stADSSharedMem.nReturnCode"] = dataLen
            self.plc.write_list_by_name(adsCallParaList)

            self.lock.release()

        else:
            return -1
        return dataLen


if __name__ == "__main__":
    print("starting plane part stewart calibration process")

    device = beckhoff_device("192.168.1.103.1.1", "192.168.1.100")
    result = device.readLegControlLength()
    print(result)

    device.close()
