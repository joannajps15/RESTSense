import serial
from pygame import mixer

ser = serial.Serial("COM7", 9600)
audio = ["audioOne.mp3", 
            "audioTwo.mp3",
            "audioThree.mp3"]
index = 0


mixer.init()

while True:
    try:
        line = ser.readline().decode().strip()
    except UnicodeDecodeError: 
        print("ok")
    print(line)
    if (line == "PLAY MUSIC"):
        print("playing music")
        mixer.music.load(audio[index])
        mixer.music.set_volume(0.7)
        mixer.music.play()
        while (mixer.music.get_busy()): 
            continue
        index = 0 if (index == 2) else index+1
        ser.write("1\n".encode())   # send as line of text

    if (line == "END"):
        with open("RESTSense_data.txt", "w") as file:
            print("writing to file")
            while True: 
                line = ser.readline().decode().strip()
                if (line == "COMPLETE"):
                    break
                file.write(line + "\n")