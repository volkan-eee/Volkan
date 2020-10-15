#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Kutuphaneler
import cv2
import numpy as np
import RPi.GPIO as GPIO
from dronekit import connect
import time
# Sabit Degiskenler

lat = 0
lng = 0
lower_red =np.array([161,155,84])
upper_red =np.array([179,255,255])

def readRedArea():
    # print("area")

    (ret, frame) = camera.read()  # Kamerayı okuyor.

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Kırmızı renk için maskeleme yapıcam o yüzden color türünü değiştirdim.

    mask = cv2.inRange(hsv, lower_red, upper_red)  # Bu renk skalasına sahip görüntü kırmızı olarak çıkıyor.
    mask = cv2.dilate(mask, None, iterations=1)  # Kenarların belli olması için dilation yaptım.
    eq_frame = cv2.equalizeHist(hsv[:, :, 2])  # Contrast ı ayarlaması için histogram eşitleme yaptım.
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # Contourunu buluyor.
    return cnts, mask, eq_frame

def calculateCircle(cnts):
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    return center, radius

def tur1():
    '''
    Kirmizi alan lokasyonunu belirler ve global lat lng degiskenlerinde tutar.
    :return: None
    '''
    print("Tur 1")

    # Sabit Degiskenler

    MIN_RADIUS = 90  # minimum radius degeri: px irtifaya gore ayarlanacak

    # Diger Degiskenler

    isRedFinished = False
    firstRedRead = False

    max_x = -10000
    min_x = 10000
    max_y = -10000
    min_y = 10000

    while (not isRedFinished):

        cnts2, mask2, eq_frame2 = readRedArea()

        while (len(cnts2) > 0 ):

            center, radius = calculateCircle(cnts2)
            if radius > MIN_RADIUS:
                isRedFinished = True

                cv2.circle(eq_frame2, (int(center[0]), int(center[1])), int(radius), (0, 255, 255),2)
                cv2.circle(eq_frame2, center, 5, (0, 0, 255), -1)

                # Gercek ucusta buradaki x-y degerleri gpsden alinan lat lng degerleri olacak
                loc=vehicle.location.global_frame
                x = loc.lat
                y = loc.lon

                if x < min_x:
                    min_x = x
                if y < min_y:
                    min_y = y
                if x > max_x:
                    max_x = x
                if y > max_y:
                    max_y = y

            cv2.imshow('frame', eq_frame2)
            cv2.imshow('mask', mask2)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
            cnts2, mask2, eq_frame2 = readRedArea()


        cv2.imshow('frame', eq_frame2)
        cv2.imshow('mask', mask2)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

        global lat,lng
        # gelen verilere gore round yapilabilir
        lat = (max_x + min_x) / 2
        lng = (max_y + min_y) / 2

def tur2():
    '''
    Tur1 de belirlenen lokasyona dusecek sekilde hesaplanmis birakma noktasinda Birinci Servo motorunu calistirir.
    :return:None
    '''
    print("Tur 2")

    print(lat," ",lng)
    while(True):
        cnts, mask, eq_frame = readRedArea()
        cv2.imshow('frame', eq_frame)
        cv2.imshow('mask', mask)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
        
        
        # TODO
        # gps'den verileri surekli oku
        loc2=vehicle.location.global_frame
        #print(loc2)
        dist_lat=abs(loc2.lat-lat)
        dist_lng=abs(loc2.lon-lng)
        #print("Lat: %s" %dist_lat)
        #print("lon : %s" % dist_lng)
        # eger gps lat lng degerleri global lat lng degerine belli bir oranda yaklasirsa servoyu calistir
        
        if cnts>0 and dist_lat < 0.0001 and dist_lng< 0.0001:
            ilk_servo.start(5)
            ilk_servo.ChangeDutyCycle(12.5)
            print("ILK TOP BIRAKILDI...")
            break
        # servo calisinca breakle
        
        


def tur3():
    '''
    Tur1 de belirlenen lokasyona dusecek sekilde hesaplanmis birakma noktasinda Ikinci Servo motorunu calistirir.
    :return:None
    '''
    print("Tur 3")

    while(True):

          # TODO
        # gps'den verileri surekli oku
        loc2=vehicle.location.global_frame
        #print(loc2)
        dist_lat=abs(loc2.lat-lat)
        dist_lng=abs(loc2.lon-lng)
        print("Lat: %s" %dist_lat)
        print("lon : %s" % dist_lng)
        # eger gps lat lng degerleri global lat lng degerine belli bir oranda yaklasirsa servoyu calistir
        
        if dist_lat < 0.00001 and dist_lng< 0.00001:
            ikinci_servo.start(5)
            ikinci_servo.ChangeDutyCycle(12.5)
            print("İKinci TOP BIRAKILDI...")
            break
        # servo calisinca breakle
        break


if __name__ == "__main__":
    camera = cv2.VideoCapture(0)
    #Connect to the Vehicle (in this case a UDP endpoint)
    vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup (12, GPIO.OUT)
    ilk_servo= GPIO.PWM(12,50)
    
    GPIO.setup (16, GPIO.OUT)
    ikinci_servo= GPIO.PWM(16,50)
    GPIO.setwarnings(False)
    tur1()
    time.sleep(5)
    tur2()
    #time.sleep(4)
    #tur3()
    camera.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()