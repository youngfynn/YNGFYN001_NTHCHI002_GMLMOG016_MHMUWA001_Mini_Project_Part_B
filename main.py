from tkinter import *
from tkinter import messagebox
import serial
import threading
import time

root = Tk()
root.title("EEE Mini Project")

# Used to terminate the thread displaying the ADC values
terminateThread = False
# Open file to save ADC data into csv file
file = open("ADC_Log.csv","w")
# Frame to hold listBox with scrollbar

displayFrame = Frame(root)
verticalScroll = Scrollbar(displayFrame, orient = VERTICAL)

# Frame to hold buttons

buttonFrame = Frame(root)
buttonFrame.pack(side = BOTTOM)

# Create a listbox to display information

displayBox = Listbox(displayFrame, width = 100, yscrollcommand = verticalScroll.set)

# Configure scrollbar in listbox

verticalScroll.config(command = displayBox.yview)
verticalScroll.pack(side = RIGHT, fill = Y)
displayFrame.pack(side = TOP)
displayBox.pack()

# Create actions for each button

def EchoTest():
    global t1
    global terminateThread
    displayBox.delete(0, END)
    # Terminates display of ADC value thread if it is active
    if(t1.is_alive()):
        terminateThread = True
        displayAndLog["state"] = NORMAL
        displayBox.see(END)
        displayBox.insert(END, "Closing previous connection. Please wait.")
        time.sleep(0.1)
    terminateThread = False
    displayBox.insert(END, 'Connection closed')
    sentString = ""
    receivedString = ""

    # Checks if device is connected before attempting to perform echo test
    try:
        serialPort1 = serial.Serial(port = "COM4", baudrate=9600, bytesize=8, timeout=2, write_timeout=2, stopbits=serial.STOPBITS_ONE)
        displayBox.see(END)
        displayBox.insert(END, "Performing Echo Test")
        displayBox.see(END)
        displayBox.insert(END, "Sending data to STM")
        echoButton["state"] = DISABLED
        s_string = 'ABCD1234!@#$'
        sentString = serialPort1.write(s_string.encode(encoding="ascii", errors="backslashreplace"))
        currentTime = time.time()
        
        # Allows 5 seconds for the echo test to complete
        while(time.time() < (currentTime + 5)):
        # Wait until there is data waiting in the serial buffer
            if(serialPort1.in_waiting > 0):
                # Read data out of the buffer until a carriage return / new line is found
                receivedString = serialPort1.readline()
                displayBox.see(END)
                displayBox.insert(END, "Received data")
        # Verify if echo test was successful and inform user
        if (len(receivedString) != 0):
            receivedString = (receivedString.decode("utf-8"))[0:12]   # convert bytes to string
            if (s_string == receivedString):
                displayBox.see(END)
                displayBox.insert(END, "Echo test successful")
            else:
                displayBox.see(END)
                displayBox.insert(END, "Echo test failed")
        else:   
            displayBox.see(END)
            displayBox.insert(END, "Echo test failed")
        # Re-enable button once echo test is finished
        echoButton["state"] = NORMAL
    except (serial.serialutil.SerialException):
        messagebox.showwarning("warning", "Device is not connected!")
        echoButton["state"] = NORMAL

def transmitLoT():
    global t1
    global terminateThread
    displayBox.delete(0, END)
    # Terminates display of ADC value thread if it is active
    if(t1.is_alive()):
        terminateThread = True
        displayAndLog["state"] = NORMAL
        displayBox.see(END)
        displayBox.insert(END, "Closing previous connection. Please wait.")
        time.sleep(0.1)
    terminateThread = False
    displayBox.insert(END, 'Connection closed')
    try:
        serialPort2 = serial.Serial(port = "COM4", baudrate=9600, bytesize=8, timeout=2, write_timeout=2, stopbits=serial.STOPBITS_ONE)
        displayBox.see(END)
        displayBox.insert(END, "Transmitting via LoT")
        transmitButton["state"] = DISABLED
        sentString = serialPort2.write("B".encode(encoding="ascii",errors="replace"))
        transmitButton["state"] = NORMAL
        displayBox.see(END)
        displayBox.insert(END, "Transmitted!")
    except (serial.serialutil.SerialException):
         messagebox.showwarning("warning", "Device is not connected!")
         transmitButton["state"] = NORMAL
    

def on_close():
    global t1
    global terminateThread
    displayBox.delete(0, END)
    # Terminates display of ADC value thread if it is active
    if(t1.is_alive()):
        terminateThread = True
        displayAndLog["state"] = NORMAL
        displayBox.see(END)
        displayBox.insert(END, "Closing previous connection. Please wait.")
        time.sleep(0.1)
    terminateThread = False
    displayBox.insert(END, 'Connection closed')
    response = messagebox.askyesno('Exit','Are you sure you want to exit?')
    if response:
        file.close()
        root.destroy()   

# Function to display data received from UART in listbox and save that in a .csv file
def displayUART(): 
    # Checks if device is connected before attempting read data from it
    try:
        serialPort = serial.Serial(port = "COM4", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        displayAndLog["state"] = DISABLED
        displayBox.see(END)
        displayBox.insert(END, "Connecting STM and preparing to receive data...")
        # Used to hold data coming over UART
        serialString = ""

        global terminateThread
        
        while(terminateThread != True):
            # Wait until there is data waiting in the serial buffer
            if(serialPort.in_waiting > 0):
                displayBox.insert(END, "Done")
                # Read data out of the buffer until a carraige return / new line is found
                serialString = serialPort.readline()
                receivedString = serialString.decode('Ascii')
                # Print the contents of the serial data
                displayBox.see(END)
                # ignore checkpoint messages
                if receivedString[1] != '0':
                    displayBox.insert(END, "Binary packet: " + receivedString)
                    adc_val = receivedString[2:14]
                    adc_val = int(adc_val[::-1], 2)
                    displayBox.insert(END, "ADC Value: " + str(adc_val))
                    # Write contents of the serial data into csv file
                    file.write("{time:.0f},{binary},{adc}\n".format(time=time.time(), binary=receivedString, adc=adc_val))
    except (serial.serialutil.SerialException):
        messagebox.showwarning("warning", "Device is not connected!")
        displayAndLog["state"] = NORMAL

t1 = threading.Thread(target = displayUART)

def ADCLogButton():
    displayBox.delete(0, END)
    global t1
    t1 = threading.Thread(target = displayUART)
    t1.start()

# Associate action to root using protocol  
root.protocol('WM_DELETE_WINDOW',on_close)

# Create required buttons

displayAndLog = Button(buttonFrame, text = "Display ADC reading and record log", width =  50, command = ADCLogButton)

echoButton = Button(buttonFrame, text = "Echo Test", width =  50, command = EchoTest)

transmitButton = Button(buttonFrame, text = "Transmit via LoT", width =  50, command = transmitLoT)

exitButton = Button(buttonFrame, fg='red', text = "Exit", width =  50, command = on_close)

# Position buttons on menu using grid layout

displayAndLog.grid(row = 0,column = 0)
echoButton.grid(row = 0,column = 1)
transmitButton.grid(row = 1, column = 0)
exitButton.grid(row = 1, column = 1)

root.mainloop()