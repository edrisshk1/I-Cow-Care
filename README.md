
# I Cow Care

This project presents an IoT solution for cow health monitoring. The IoT device (Wearable Collar for cows) will help cow breeders detect cow health problems, and cow reproduction cycle in order to increase milk productivity and help prevent sickness and diseases among cows. This also gives the user easy access to the cow's vital indicators and helps predict its needs.

We used an ESP32, MLX90614 temperature sensor, LIS2DH accelerometer, and Lithium Polymer battery for power.

The steps below presenting the work done to establish the IoT network and prepare the hardware and software environment are going to be tackled with more details later.

1/ Install MQTT: MQTT will be our data transfer protocol using Wifi. We can use local or online MQTT servers. In our case we use Mosquitto MQTT locally and HiveMQTT as an online server.

2/ Prepare the script to connect to Wifi and MQTT, read data from sensors, and send data to the database/dashboard using MQTT protocol...(includes testing sensors, importing libraries, timestamp, etc.)

3/ Prepare the database for data storage and management using Firebase.

4/ Prepare a dashboard for data visualization and analysis using Node-RED.

5/ Hardware design: 3D mechanical and electronic design.

6/Train machine learning models using Edge Impulse Platform for Embedded ML.

# 1/ Installation and environment preparation

### Install Mosquitto MQTT 

in our case, the implementation was on Windows 10, and the procedure is similar for Linux, and MacOS

1/ Download link: https://mosquitto.org/download/
2/ Modify the configuration file:
To ensure the proper functioning of the Mosquitto broker, you need to manually modify the listener configuration for port 1883 in the configuration file in Mosquitto root folder.
3/ Start the MQTT server by running this command in the mosquitto root folder :
  
```bash
mosquitto -c mosquitto. conf -v
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
  
And the supervision of our professors :
- Imen Harbaoui
- Rabaa Youssef

## Demo

### MQTT

![MQTT1](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/cf390cda-7481-468f-8e86-b82f5a506e1b)
![MQTT2](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/14a777ec-289d-47e4-926f-7f040c07e3de)
![MQTT3](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/8a1264de-80f0-4ef1-8ca4-a51d47563105)

### Firebase

![Firebase realtime data](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/7f58e825-9b98-46b6-acf6-1c721add272f)

### Node-RED / Dashboard

![begin cmd](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/4eddd70f-5a01-42e2-8b92-e93d33fb475f)
![dashboard nodes](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/068f30c1-7a90-4830-901c-28de313ee544)
![dashboard1](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/5bd83df8-dd71-4aed-9f6b-209fa93fde47)
![dashboard2](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/3846327b-40c2-43c7-9012-6f8b625694f6)
![dashboard3](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/86035161-ae1a-4ba9-9531-1bc948bd641d)

### Mechanical Design

![CollarRender5](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/efd6ec51-9f1b-4ec9-a791-f0fd087f769b)
![CollarRender4](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/9d79aaff-963f-4bdc-8732-f107cb7a6991)
![CollarRender3](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/55be4b72-5e29-4745-8696-ed2330d8376a)
![CollarRender2](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/30b85e70-892b-42fa-91f8-97f52771fe2e)
![CollarRender1](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/aa9f8014-60b8-4d13-8808-8ebeacdf2bdb)
![CollarNeck4](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/a9f02aee-d4ad-4e49-9d8a-047b5d79b3dc)
![CollarNeck3](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/6fa0380c-957e-46f6-bbc6-142354aa5223)

### Electronic Scheme

![SCL](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/1bdf43de-b45b-4655-911c-91a4d0970367)

### Flow Diagram

![SCL (1)](https://github.com/edrisshk1/I-Cow-Care/assets/122979130/8d827da0-4d0c-4716-9b6d-ed8d183c2d42)
