# OpenCM

소논문 탐구내용에 대한 성과( Kriswiner 와 Jeff Rowberg 의 코드를 바탕으로 작성 ) 

OpenCM IDE에서 작동하는 프로그램

DMP의 경우에는 I2Cdev를 OpenCM IDE에 설치해야 올바르게 작동 

AHRS는 #define magcalibration 을 활성화하면 지자기 보정을 활성화할 수 있다. 

자세한 지자기 센서 보정은 다음 링크를 참조 : https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

AHRS와 DMP(MPU9250_master)모두 동일한 핀구성을 가진다. 

자세한 외형은 사진파일을 참고

둘다 바퀴 구동 및 RC-100 리모컨 구동 코드가 있으며, 필요에 따라서는 지워도 무방하다.


참고해볼 링크
https://github.com/kriswiner/MPU9250
https://github.com/jrowberg/i2cdevlib
https://github.com/ROBOTIS-GIT/OpenCM9.04
https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
