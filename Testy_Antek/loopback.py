import serial  
import time  
ser = serial.Serial(  
 port='/dev/ttyAMA3',  
 baudrate=115200,  
 bytesize=serial.EIGHTBITS,  
 parity=serial.PARITY_NONE,  
 stopbits=serial.STOPBITS_ONE,  
 timeout=1  
)  
test_msg = b'Loopback test RPi5!\n'  
ser.write(test_msg)  
time.sleep(0.1)  
received = ser.read(ser.in_waiting)  
print(f'Wysłano: {test_msg}')  
print(f'Odebrano: {received}')
if received == test_msg:  
 print('OK - UART działa poprawnie!')  
else:  
 print('BŁĄD - sprawdź połączenie i konfigurację')  
ser.close()  
