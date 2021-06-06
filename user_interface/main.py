import serial
import threading
import logging

serialConnection = serial.Serial(port='COM4', baudrate=9600, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,
                                 timeout=0)


def receiving_data(serial_connection):
    receivedData = []
    line = []
    while True:
        for c in serial_connection.read(100):
            if chr(c) != '\n':
                line.append(chr(c))
            else:
                f = open("data.txt", "a")
                data = ''.join(line)
                print(data)
                f.write(data + '\n')
                f.close()
                receivedData.append(data)
                line = []

if __name__ == "__main__":
    logging.info("Main    :  Connection is opened.")
    x = threading.Thread(target=receiving_data, args=(serialConnection,))
    logging.info("Main    : Connection is ready.")
    x.start()
    logging.info("Main    : Connected!")


    while True:
        inp = input("Send something from the console! \n")
        if str(inp) == 'R':
            serialConnection.write('<R>'.encode())
            print("sent\n")

    x.join()
