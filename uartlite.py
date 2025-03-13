import os
import time
import numpy as np
import mmap
import asyncio

# ============================ #
#  XDMA + UARTLite (Объединено)
# ============================ #
class XdmaUartLite:
    BASE_ADDR = 0x40000  # Базовый адрес UARTLite в XDMA

    # Оффсеты регистров UARTLite
    RX_FIFO = 0x00
    TX_FIFO = 0x04
    STATUS = 0x08
    CONTROL = 0x0C

    # Флаги
    TX_FULL = 0x08
    RX_VALID = 0x01
    TX_RESET = 0x01
    RX_RESET = 0x02

    def __init__(self, device_index=0):
        """Инициализация XDMA и UARTLite"""
        base = f"/dev/xdma{device_index}"
        
        # Открываем файлы для работы с XDMA
        self.fd_user = os.open(f"{base}_user", os.O_RDWR)

        # Создаём отображение памяти для быстрого доступа
        self.m_rmap = np.frombuffer(mmap.mmap(self.fd_user, int(1e6)), np.uint32)
        
        # Сбрасываем FIFO UARTLite
        self.reset_fifos()

    def close(self):
        """Закрываем файлы XDMA при завершении"""
        os.close(self.fd_user)


    # ============================ #
    #  Чтение/Запись через XDMA
    # ============================ #
    def read_reg(self, addr):
        """Читаем 32-битное значение из регистра"""
        return self.m_rmap[addr >> 2] & 0xFFFF

    def write_reg(self, addr, data):
        """Записываем 32-битное значение в регистр"""
        self.m_rmap[addr >> 2] = np.uint32(data)

    # ============================ #
    #  Работа с UARTLite
    # ============================ #
    def reset_fifos(self):
        """Сбрасываем FIFO передатчика и приёмника"""
        self.write_reg(self.BASE_ADDR + self.CONTROL, self.TX_RESET | self.RX_RESET)

    def send_byte(self, data):
        """Отправляем 1 байт через UARTLite"""
        while self.read_reg(self.BASE_ADDR + self.STATUS) & self.TX_FULL:
            pass  # Ждём, пока FIFO не освободится
        self.write_reg(self.BASE_ADDR + self.TX_FIFO, data)

    def recv_byte(self):
        """Получаем 1 байт через UARTLite"""
        while not (self.read_reg(self.BASE_ADDR + self.STATUS) & self.RX_VALID):
            pass  # Ждём, пока появятся данные
        return self.read_reg(self.BASE_ADDR + self.RX_FIFO)

    def send_data(self, data):
        """Отправляем массив байтов через UARTLite"""
        for byte in data:
            self.send_byte(byte)

    def recv_data(self, size):
        """Получаем массив байтов через UARTLite"""
        return bytearray([self.recv_byte() for _ in range(size)])

    async def recv_byte_async(self):
        """Асинхронное чтение 1 байта."""
        while not (self.read_reg(self.BASE_ADDR + self.STATUS) & self.RX_VALID):
            await asyncio.sleep(0.001)  # Дадим CPU отдохнуть, чтобы не нагружать 100%
        return self.read_reg(self.BASE_ADDR + self.RX_FIFO)

    async def recv_data_async(self, size):
        """Асинхронное чтение массива байтов."""
        return bytearray([await self.recv_byte_async() for _ in range(size)])

# ============================ #
#  Тестирование XDMA + UARTLite
# ============================ #



### Простой вариант 
# if __name__ == "__main__":
#     # Инициализируем XDMA+UARTLite
#     xdma_uart = XdmaUartLite(device_index=0)

#     print("Ожидание данных с UARTLite...")
#     while True:
#         data = xdma_uart.recv_data(128)  # Читаем 128 байта
#         if data:
#             print(data.decode(errors="ignore").strip())  # Убираем пустые строки и пробелы


### Вариант с async
async def main():
    uart = XdmaUartLite()
    while True:
        data = await uart.recv_data_async(128)
        print(data.decode(errors="ignore"))

asyncio.run(main())