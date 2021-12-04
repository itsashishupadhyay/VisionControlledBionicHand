# IoT based Bionic Arm, Powered By Computer Vision

Idea is to create a novel solution to control 5 fingers of a bionic arm, Wirelessly over the web while mimicking a real hand using Computer vision  
To achieve that we'll use the best of   
    ESP32, 5 Servo Motors, and IIC based PWM generator  
    AWS IoT (MQTT)  
    Media Pipe Library  
## Hardware Used
    1. ESP32  
    2. I2C based PWM Driver [PCA9685] (https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
    3. Servo x 6

## Folder contents
```
├── CMakeLists.txt
├── HandRecognition.py
├── components
|   ├── aws-iot-device-sdk-embedded-C
├── main
|   ├── certs                  AWS Certificates
|   ├── frozen                 Library to parse JSON
│   ├── CMakeLists.txt
│   └── main.c                 Entry point
└── README.md                  This is the file you are currently reading
```

## How to use
In the Home folder, you'll find the `HandRecognition.py` Script, which uses the [mediaPipe hand library](https://google.github.io/mediapipe/solutions/hands.html) to calculate multiple points on your hand (20 to be exact). I use a simple Distance formula from the tip of each finger to the start of the hand to determine the distance.
This distance is mapped to 0 t0 100 (as power percentage) and sent to AWS IoT via MQTT.

Our ESP32 is subscribed to the channel and is expecting a JSON   
```  
{
"Data": [
{
    "finger": 0,
    "PowerPercent": 39
},
{
    "finger": 1,
    "PowerPercent": 55
},
{
    "finger": 2,
    "PowerPercent": 8
},
{
    "finger": 3,
    "PowerPercent": 60
},
{
    "finger": 4,
    "PowerPercent": 58
}
]
}
```  
which is basically the five fingers and their respective PWM duty cycle

## Setup

AWS IoT setup is required there is amazing Documentation by Amazon with a Video library, I'll suggest referring to it for setup, refer to `Kconfig` in main to set up your variable. The certificates files need to be copied and renamed as per `CmakeLists.txt`.

Also before you compile please ensure that you have the lastest version of ESP-IDF.

For servo control, I wrote the Library for PCA9685, if there is interest I'll be willing to write the Library for universal use. For now, it works for 5 channels from 0 to 100% duty cycle (refer `Drivers.c`).


## Contact  
Please fell free to reach out to me at `itsashishupadhyay@gmail.com` for any query or concern

## Demo
Please fell free to check out the [Demo Video](https://youtu.be/dBysV4OpSVE) at `https://youtu.be/dBysV4OpSVE`
![AWS Screenshot](https://github.com/itsashishupadhyay/VisionControlledBionicHand/blob/master/images/AwsTest.png)
![Working Screenshot](https://github.com/itsashishupadhyay/VisionControlledBionicHand/blob/master/images/Demo.jpeg)
