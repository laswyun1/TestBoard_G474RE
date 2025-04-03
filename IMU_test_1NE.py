import serial
import csv
import matplotlib.pyplot as plt
import pandas as pd
import os

# Use LaTeX font 
# plt.rc('text', usetex=True)
plt.rc('font', family='serif')

serialPort = serial.Serial('COM7', 128000, timeout=1)
base_filename = 'IMU_test'
fileFormat = '.csv'
counter = 1
csv_filename = f"{base_filename}_{counter}{fileFormat}"
while os.path.exists(csv_filename):
    counter += 1
    csv_filename = f"{base_filename}_{counter}{fileFormat}"


headerData = ['Time[ms]', 'AccX']   # Should modify this 
headerNum = len(headerData) 
terminateDataValue = 78         # Should be matched with STM32 FW


with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(headerData)

    try:
        while True:
            if serialPort.in_waiting > 0:
                temp = serialPort.readline().decode('utf-8').strip()  
                temp = temp.replace('\0', '') 
                print(f"Received line: {temp}") 

                if temp:
                    try:
                        data = temp.split(',')
                        if len(data) >= headerNum:
                            time_data = float(data[0].strip())
                            AccX_data = float(data[1].strip())
                            # AccY_data = float(data[2].strip())
                            # AccZ_data = float(data[3].strip())
                            # GyrX_data = float(data[4].strip())
                            # GyrY_data = float(data[5].strip())
                            # GyrZ_data = float(data[6].strip())

                            # writer.writerow([time_data, AccX_data, AccY_data, AccZ_data, GyrX_data, GyrY_data, GyrZ_data])
                            writer.writerow([time_data, AccX_data])
                        
                        elif len(data) == 1:
                            terminate_data = int(data[0].strip())
                            if (terminate_data == terminateDataValue):
                                print("Exit the program. Close the CSV file")
                                serialPort.close()
                                file.close()
                                break
                            else:
                                print("Wrong data is received:", temp)
                        else:
                            print("Wrong data is received:", temp)
                    except IndexError:
                        print(f"IndexError: This data cannot be read. Received Data: {temp}")
                    except ValueError:
                        print(f"ValueError: This data cannot be converted. Received Data: {temp}")
                else:
                    print("Error: Received Empty data")

    except KeyboardInterrupt:
        print("Exit the program. Close the CSV file")
    
    except serial.SerialException as e:
        print(f"SerialException: {e}")
    
    finally:
        serialPort.close()
        file.close()


data = pd.read_csv(csv_filename, header=0)

plt.figure(figsize=(14,8), dpi=120)
plt.plot(data['Time[ms]'], data['AccX'], label='AccX', color='b')
# plt.plot(data['Time[ms]'], data['AccY'], label='AccY', color='r')
# plt.plot(data['Time[ms]'], data['GyrZ'], label='GyrZ', color='b')
plt.xlabel('Time[ms]', fontsize=15)
plt.ylabel('Acceleration[m/s^2]', fontsize=15)
plt.ylim(-2, 2)
plt.title('Result of IMU test', fontsize=25)
plt.grid(alpha=0.4)
plt.legend(loc=1, fontsize=15)
plt.savefig('IMU_test.png')  
plt.show()


