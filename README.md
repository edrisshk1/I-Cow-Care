
# I Cow Care

This project presents an IoT solution for cow health monitoring. The IoT device (Wearable Collar for cows) will help cow breeders detect cow health problems, and cow reproduction cycle in order to increase milk productivity and help prevent sickness and diseases among cows. This also gives the user easy access to the cow's vital indicators and helps predict its needs.

We used an ESP32, MLX90614 temperature sensor, LIS2DH accelerometer, and Lithium Polymer battery for power.

The steps below presenting the work done to establish the IoT network and prepare the hardware and software environment are going to be tackled with more details later.

1/ Install MQTT: MQTT will be our data transfer protocol using Wifi. We can use local or online MQTT servers. In our case we use Mosquitto MQTT locally and HiveMQTT as an online server.

2/ Prepare the script to connect to Wifi and MQTT, read data from sensors, and send data to the database/dashboard using MQTT protocol...(includes testing sensors, importing libraries, timestamp, etc.)

3/ Prepare the database for data storage and management using Firebase.

4/ Prepare a dashboard for data visualization and analysis using Node-RED.

5/Train machine learning models using Edge Impulse Platform for Embedded ML.

6/ Hardware design: 3D mechanical and electronic design.

# 1/ Installation and environment preparation

### Install Mosquitto MQTT 

in our case, the implementation was on Windows 10, and the procedure is similar for Linux, and MacOS

1/ Download link: https://mosquitto.org/download/
2/ Modify the configuration file:
To ensure the proper functioning of the Mosquitto broker, manually modify the listener configuration for port 1883 in the configuration file in Mosquitto root folder.
3/ Start MQTT server by running this command in the mosquitto root folder :
  
```bash
mosquitto -c mosquitto.conf -v
```
4/ To get the IP address, run this command (IPV4 in our case) :
```bash
ip config
```
5/ Configure the ESP32 code to connect to the MQTT server.

### Install Node-RED

1/ Download link: https://nodered.org/docs/getting-started/. You need to install node.js.

2/ Run Node-RED using this command :
  
```bash
node-red
```
3/ A local webpage will open in your browser to create, modify and connect your devices and different functions. You can connect to the database / MQTT server...

### Write script for ESP32 :

1/ We used Arduino IDE to program the ESP32  You'll find the whole script and library in the repository.

### Machine Learning :

1/ We saved data locally in a .csv file and then used Edge Impulse to train our model using different algorithms: KNN, Neural Networks, SVM, Logistic regression...

Edge impulse link: https://edgeimpulse.com/

### 3D Design :
We used SolidWorks to design different parts of the device.

## Next Steps :
The project is far from being perfect or completed. The following steps may include enlarging and building a bigger database to ensure our sufficiency of data for model training. It may also include building a mobile application to facilitate access to data and different stats for different categories of users.
## Authors

This contribution to the project wasn't possible without the participation of the outstanding team :
- Edriss Hadj Khelil
- Majd Karoui
- Niam Oueslati
- Elyes Khechine

## Demo

MQTT

Firebase

Dashboard

Machine Learning

Mechanical Design

Electronic Scheme
