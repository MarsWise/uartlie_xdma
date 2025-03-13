# Import necessary libraries
import XdmaAccess
import numpy as np
import struct
import time
import ctypes

device = XdmaAccess.XdmaAccess(0)

# Define constants
XUL_RX_FIFO_OFFSET = 0
XUL_TX_FIFO_OFFSET = 4
XUL_STATUS_REG_OFFSET = 8
XUL_CONTROL_REG_OFFSET = 12

XUL_CR_ENABLE_INTR = 0x10
XUL_CR_FIFO_RX_RESET = 0x02
XUL_CR_FIFO_TX_RESET = 0x01

XUL_SR_PARITY_ERROR = 0x80
XUL_SR_FRAMING_ERROR = 0x40
XUL_SR_OVERRUN_ERROR = 0x20
XUL_SR_INTR_ENABLED = 0x10
XUL_SR_TX_FIFO_FULL = 0x08
XUL_SR_TX_FIFO_EMPTY = 0x04
XUL_SR_RX_FIFO_FULL = 0x02
XUL_SR_RX_FIFO_VALID_DATA = 0x01

XUL_FIFO_SIZE = 16
XUL_STOP_BITS = 1

XUL_PARITY_NONE = 0
XUL_PARITY_ODD = 1
XUL_PARITY_EVEN = 2

# Define structures
class XUartLite_Stats():
    def __init__(self):
        self.TransmitInterrupts =0
        self.ReceiveInterrupts =0
        self.CharactersTransmitted =0
        self.CharactersReceived =0
        self.ReceiveOverrunErrors =0
        self.ReceiveParityErrors =0
        self.ReceiveFramingErrors =0



class XUartLite():
    def __init__(self):
        self.Stats = XUartLite_Stats()
        self.RegBaseAddress =0
        self.IsReady =0
        self.SendBuffer = []
        self.ReceiveBuffer = []
        self.RecvHandler = None
        self.RecvCallBackRef = None
        self.SendHandler = None
        self.SendCallBackRef = None




# Define functions
def XUartLite_WriteReg(BaseAddress, RegOffset, RegisterValue):
    device.write(BaseAddress + RegOffset, RegisterValue)

def XUartLite_ReadReg(BaseAddress, RegOffset):
    return device.read(BaseAddress + RegOffset)

def XUartLite_SetControlReg(BaseAddress, Mask):
    XUartLite_WriteReg(BaseAddress, XUL_CONTROL_REG_OFFSET, Mask)

def XUartLite_GetStatusReg(BaseAddress):
    return XUartLite_ReadReg(BaseAddress, XUL_STATUS_REG_OFFSET)

def XUartLite_IsReceiveEmpty(BaseAddress):
    return (XUartLite_GetStatusReg(BaseAddress) & XUL_SR_RX_FIFO_VALID_DATA) != XUL_SR_RX_FIFO_VALID_DATA

def XUartLite_IsTransmitFull(BaseAddress):
    return (XUartLite_GetStatusReg(BaseAddress) & XUL_SR_TX_FIFO_FULL) == XUL_SR_TX_FIFO_FULL

def XUartLite_IsIntrEnabled(BaseAddress):
    return (XUartLite_GetStatusReg(BaseAddress) & XUL_SR_INTR_ENABLED) == XUL_SR_INTR_ENABLED

def XUartLite_EnableIntr(BaseAddress):
    XUartLite_SetControlReg(BaseAddress, XUL_CR_ENABLE_INTR)

def XUartLite_DisableIntr(BaseAddress):
    XUartLite_SetControlReg(BaseAddress, 0)

def XUartLite_SendByte(BaseAddress, Data):
    while XUartLite_IsTransmitFull(BaseAddress):
        pass
    XUartLite_WriteReg(BaseAddress, XUL_TX_FIFO_OFFSET, Data)

def XUartLite_RecvByte(BaseAddress):
    while XUartLite_IsReceiveEmpty(BaseAddress):
        pass
    return XUartLite_ReadReg(BaseAddress, XUL_RX_FIFO_OFFSET)


#BaudRate=115200,UseParity=0, ParityOdd=0, DataBits=8,
def XUartLite_Initialize(InstancePtr, BaseAddress, callbackfunc=None):
    InstancePtr.RecvCallBackRef = callbackfunc
    return XUartLite_CfgInitialize(InstancePtr, BaseAddress)

def XUartLite_CfgInitialize(InstancePtr, EffectiveAddr):
    
    InstancePtr.IsReady = 1
    InstancePtr.RegBaseAddress = EffectiveAddr

    InstancePtr.RecvHandler = None
    InstancePtr.SendHandler = None

    XUartLite_WriteReg(InstancePtr.RegBaseAddress, XUL_CONTROL_REG_OFFSET, 0)
    XUartLite_ClearStats(InstancePtr)

    return 0

def XUartLite_ClearStats(InstancePtr):
    InstancePtr.Stats.TransmitInterrupts = 0
    InstancePtr.Stats.ReceiveInterrupts = 0
    InstancePtr.Stats.CharactersTransmitted = 0
    InstancePtr.Stats.CharactersReceived = 0
    InstancePtr.Stats.ReceiveOverrunErrors = 0
    InstancePtr.Stats.ReceiveParityErrors = 0
    InstancePtr.Stats.ReceiveFramingErrors = 0

# Define functions for sending and receiving data
def XUartLite_Send(InstancePtr, DataBufferPtr, NumBytes):
    return XUartLite_SendBuffer(InstancePtr,DataBufferPtr, NumBytes)

def XUartLite_Recv(InstancePtr, NumBytes):
    return XUartLite_ReceiveBuffer(InstancePtr,NumBytes)

def XUartLite_SendBuffer(InstancePtr,DataBufferPtr, NumBytes):
    SentCount = 0
    InstancePtr.SendBuffer = DataBufferPtr 
    while (XUartLite_GetStatusReg(InstancePtr.RegBaseAddress) & XUL_SR_TX_FIFO_FULL) == 0 and SentCount < NumBytes:
        XUartLite_WriteReg(InstancePtr.RegBaseAddress, XUL_TX_FIFO_OFFSET, InstancePtr.SendBuffer[SentCount])
        SentCount += 1
    return InstancePtr.SendBuffer

def XUartLite_ReceiveBuffer(InstancePtr,NumBytes):
    InstancePtr.ReceiveBuffer = []
    for ReceivedCount in range(NumBytes):
        while XUartLite_GetStatusReg(InstancePtr.RegBaseAddress) & XUL_SR_RX_FIFO_FULL == 0:
            pass
        c = XUartLite_ReadReg(InstancePtr.RegBaseAddress, XUL_RX_FIFO_OFFSET)
        if c != 0xffffffff:
            InstancePtr.ReceiveBuffer.append(c)
    return bytearray(InstancePtr.ReceiveBuffer)

# Define functions for enabling and disabling interrupts
def XUartLite_EnableInterrupt(InstancePtr):
    XUartLite_WriteReg(InstancePtr.RegBaseAddress, XUL_CONTROL_REG_OFFSET, XUL_CR_ENABLE_INTR)

def XUartLite_DisableInterrupt(InstancePtr):
    XUartLite_WriteReg(InstancePtr.RegBaseAddress, XUL_CONTROL_REG_OFFSET, 0)

# Define functions for setting handlers
def XUartLite_SetRecvHandler(InstancePtr, FuncPtr, CallBackRef):
    InstancePtr.RecvHandler = FuncPtr
    InstancePtr.RecvCallBackRef = CallBackRef

def XUartLite_SetSendHandler(InstancePtr, FuncPtr, CallBackRef):
    InstancePtr.SendHandler = FuncPtr
    InstancePtr.SendCallBackRef = CallBackRef

# Define interrupt handler
def XUartLite_InterruptHandler(InstancePtr):
    IsrStatus = XUartLite_GetStatusReg(InstancePtr.RegBaseAddress)

    if (IsrStatus & (XUL_SR_RX_FIFO_FULL | XUL_SR_RX_FIFO_VALID_DATA)) != 0:
        ReceiveDataHandler(InstancePtr)

    if (IsrStatus & XUL_SR_TX_FIFO_EMPTY) != 0 and InstancePtr.SendBuffer.RequestedBytes > 0:
        SendDataHandler(InstancePtr)

# Define data handlers
def ReceiveDataHandler(InstancePtr):
    if InstancePtr.ReceiveBuffer.RemainingBytes != 0:
        XUartLite_ReceiveBuffer(InstancePtr)

    if InstancePtr.ReceiveBuffer.RemainingBytes == 0:
        InstancePtr.RecvHandler(InstancePtr.RecvCallBackRef, InstancePtr.ReceiveBuffer.RequestedBytes - InstancePtr.ReceiveBuffer.RemainingBytes)

    InstancePtr.Stats.ReceiveInterrupts += 1

def SendDataHandler(InstancePtr):
    if InstancePtr.SendBuffer.RemainingBytes == 0:
        SaveReq = InstancePtr.SendBuffer.RequestedBytes
        InstancePtr.SendBuffer.RequestedBytes = 0
        InstancePtr.SendHandler(InstancePtr.SendCallBackRef, SaveReq)
    else:
        XUartLite_SendBuffer(InstancePtr)

    InstancePtr.Stats.TransmitInterrupts += 1

# Define function for resetting FIFOs
def XUartLite_ResetFifos(InstancePtr):
    Register = XUartLite_GetStatusReg(InstancePtr.RegBaseAddress)
    Register &= XUL_SR_INTR_ENABLED
    XUartLite_WriteReg(InstancePtr.RegBaseAddress, XUL_CONTROL_REG_OFFSET, Register | XUL_CR_FIFO_TX_RESET | XUL_CR_FIFO_RX_RESET)

# Define function for checking if sending
def XUartLite_IsSending(InstancePtr):
    StatusRegister = XUartLite_GetStatusReg(InstancePtr.RegBaseAddress)
    return (StatusRegister & XUL_SR_TX_FIFO_EMPTY) == 0

# Define function for getting status register
def XUartLite_GetSR(InstancePtr):
    StatusRegister = XUartLite_GetStatusReg(InstancePtr.RegBaseAddress)
    XUartLite_UpdateStats(InstancePtr, StatusRegister)
    return StatusRegister

# Define function for updating stats
def XUartLite_UpdateStats(InstancePtr, StatusRegister):
    if StatusRegister & XUL_SR_OVERRUN_ERROR:
        InstancePtr.Stats.ReceiveOverrunErrors += 1
    if StatusRegister & XUL_SR_PARITY_ERROR:
        InstancePtr.Stats.ReceiveParityErrors += 1
    if StatusRegister & XUL_SR_FRAMING_ERROR:
        InstancePtr.Stats.ReceiveFramingErrors += 1

# Define stub handler
def StubHandler(CallBackRef, ByteCount):
    raise AssertionError("Stub handler called")

# Define function for self-test
def XUartLite_SelfTest(InstancePtr):
    XUartLite_ResetFifos(InstancePtr)
    StatusRegister = XUartLite_GetSR(InstancePtr)
    if StatusRegister != XUL_SR_TX_FIFO_EMPTY:
        return -1
    return 0

# Define functions for getting and clearing stats
def XUartLite_GetStats(InstancePtr, StatsPtr):
    StatsPtr.TransmitInterrupts = InstancePtr.Stats.TransmitInterrupts
    StatsPtr.ReceiveInterrupts = InstancePtr.Stats.ReceiveInterrupts
    StatsPtr.CharactersTransmitted = InstancePtr.Stats.CharactersTransmitted
    StatsPtr.CharactersReceived = InstancePtr.Stats.CharactersReceived
    StatsPtr.ReceiveOverrunErrors = InstancePtr.Stats.ReceiveOverrunErrors
    StatsPtr.ReceiveFramingErrors = InstancePtr.Stats.ReceiveFramingErrors
    StatsPtr.ReceiveParityErrors = InstancePtr.Stats.ReceiveParityErrors


if __name__ == '__main__':
    #baudrate 9600
    # SIM68
    BRAM_ADDR = 0x0
    BRAM_SIZE = 0x1000000 #32bitsx16364depth

    TEST_BUFFER_SIZE = 64
    
    
    XUARTLITE_AXI_BASEADDRESS = 0x40000
    

    SendBuffer = [0] * TEST_BUFFER_SIZE

    UartLite  = XUartLite()
    Status = XUartLite_Initialize(UartLite, XUARTLITE_AXI_BASEADDRESS);

    while True:
        ReceiveBuffer = XUartLite_Recv(UartLite, TEST_BUFFER_SIZE)    
        if(len(ReceiveBuffer)) > 0:
            print(''.join(chr(b) if b != 0xFF else '' for b in ReceiveBuffer))
    
    #Status = SetupInterruptSystem(IntcInstancePtr, UartLiteInstPtr,UartLiteIntrId)
    #XUartLite_SetSendHandler(UartLite, SendHandler, SendCallBackRef)
	#XUartLite_SetRecvHandler(UartLite, RecvHandler, SendCallBackRef)
    #XUartLite_EnableInterrupt(UartLite)
    
    
    # XUartLite_Send(UartLite, SendBuffer, TEST_BUFFER_SIZE);
    
