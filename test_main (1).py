##################################################
## AeroTED Kod v1 Test Kodu
##################################################
## Description #TODO
##################################################
## Author: Volkan Akbucak, Akdeniz Kutay Ocal
## Copyright: Copyright 2020, AeroTED
## Credits: #TODO #Kirmizi alan filtreleme kodunun kaynaginin gosterimi gerekiyor
## License: MIT License
## Version: v1
## Email: {contact_email}
##################################################

# Kutuphaneler
import cv2
from numpy import sin, cos, arccos, array
import RPi.GPIO as GPIO
from dronekit import connect
from time import sleep

# Sabit Degiskenler
rad = 0.0174532925
lat = 0
lng = 0
lower_red = array([161, 155, 84])
upper_red = array([179, 255, 255])


def readRedArea():
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


def spotDropZone():
    '''
    Kirmizi alan lokasyonunu belirler ve global lat lng degiskenlerinde tutar.

    Kirmizi alan buyuklugu belirli bir miktardan fazla olunca algilamaya baslar ve gps uzerinden
    lokasyon verilerini alir. Kirmizi alan bitene kadar devam eder ve en sonunda aldigi lokasyon verilerinin
    ortalamasini alir.

    :return: None
    '''

    # Sabit Degiskenler

    MIN_RADIUS = 50  # minimum radius degeri (px cinsinden) (irtifaya gore ayarlanacak)

    # Diger Degiskenler

    isRedFinished = False  # Kirmizi bolgenin okunup okunmadigini kontrol icin kullaniliyor

    max_x = -10000
    min_x = 10000
    max_y = -10000
    min_y = 10000

    print("Tur 1")
    while not isRedFinished:

        cnts2, mask2, eq_frame2 = readRedArea()

        while len(cnts2) > 0:  # Kirmizi alan algilaniyorsa

            center, radius = calculateCircle(cnts2)  # alanin buyuklugunu hesapla

            if radius > MIN_RADIUS:  # eger belirtilen yaricaptan buyukse
                isRedFinished = True

                cv2.circle(eq_frame2, (int(center[0]), int(center[1])), int(radius), (0, 255, 255), 2)
                cv2.circle(eq_frame2, center, 5, (0, 0, 255), -1)

                loc = vehicle.location.global_frame  # lokasyon verisini al (latitude, longitude)

                # lokasyon verisinin ortalamasinin alinabilmesi icin min max hesaplamasi
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

        global lat, lng

        lat = (max_x + min_x) / 2
        lng = (max_y + min_y) / 2
        print("Kirmizi alan merkezi: lat: ",lat," long: ", lng)

def dropBall(servo):
    '''
    Ilk turda belirlenen noktaya belirli oranda yaklasinca servolari calistirir
    :param servo: Servo motor objesi (Tur 2 icin ilk_servo, Tur 3 icin ikinci_servo)
    :return: None
    '''

    # Sabit Degiskenler
    MIN_DIST = 1  # Kirmizi noktaya ne kadar yaklasinca atis yapilacagini belirten degisken (metre cinsiden)

    while True:

        cnts, mask, eq_frame = readRedArea()
        cv2.imshow('frame', eq_frame)
        cv2.imshow('mask', mask)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

        loc2 = vehicle.location.global_frame
        # Anlik gps verisi ile kirmizi alanin merkezi arasindaki mesafe hesaplamasi (metre cinsinden)

        dist = arccos(
            sin(lat * rad) * sin(loc2.lat * rad) + cos(lat * rad) * cos(loc2.lat * rad) * cos(
                (lng * rad) - (loc2.lon * rad))) * 6371 * 1000

        #print(dist)

        if dist < MIN_DIST:  # eger 1m den yakinsa atesleme yap
            servo.ChangeDutyCycle(12.5)
            print("ATES!")
            break


if __name__ == "__main__":

    # Goruntu yakalama acildi
    camera = cv2.VideoCapture(0)

    # Connect to the Vehicle (in this case a UDP endpoint)
    try:
        vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)
    except Exception as e:
        print("Cihazla baglanti kurulamadi.")
        print(e)

    # Servo motor pinleri ayarlandi
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(16, GPIO.OUT)
    ilk_servo = GPIO.PWM(12, 50)
    ikinci_servo = GPIO.PWM(16, 50)
    GPIO.setwarnings(False)

    # Tur1 - Kirmizi alan tespiti icin kod calistirildi
    try:
        spotDropZone()
        sleep(5)
    except Exception as e:
        print("(Tur 1) - Kirmizi alan tespiti sirasinda hata meydana geldi.")
        print(e)

    # Tur 2 - Birinci topu birakmak icin kod calistirildi (12. pindeki servo)
    try:
        print("Tur 2")
        dropBall(ilk_servo)
        sleep(5)
    except Exception as e:
        print("(Tur 2) - Ilk topun atilmasi sirasinda hata meydana geldi.")
        print(e)

    # Tur 3 - Ikinci topu birakmak icin kod calistirildi (16. pindeki servo)
    try:
        print("Tur 3")
        dropBall(ikinci_servo)
        camera.release()
    except Exception as e:
        print("(Tur 3) - Ikinci topun atilmasi sirasinda hata meydana geldi.")
        print(e)

    cv2.destroyAllWindows()
    GPIO.cleanup()
