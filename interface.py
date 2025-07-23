import cv2
import numpy as np
import time
import imutils
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
from math import *

# Utility function to find an external USB camera
def guess_usb_camera_index():
    # Attempt to find a camera index that is likely a USB camera (skip 0, usually the laptop camera)
    cam = cv2.VideoCapture(0)
    return 0  # Fallback to 0

# Load and parse calibration data
lines = open("data.txt").read().splitlines()
lines = lines[:-11]     # Remove last 11 lines
lines = lines[1:]       # Remove the first line

dataDict = {}

camHeight = 480
camWidth = 640

# Use guessed USB camera index, fallback to 0 if not found
usb_cam_index = guess_usb_camera_index()
cam = cv2.VideoCapture(usb_cam_index)
if not cam.isOpened():
    print("Error: Could not open USB camera (tried index {}).".format(usb_cam_index))
cam.set(3, camWidth)
cam.set(4, camHeight)

getPixelColor = False
H, S, V = 15, 10 , 75

mouseX, mouseY = 0, 0

for i in range(0, len(lines)):
    line = lines[i].strip()
    if not line or "#" not in line:
        print(f"[Warning] Skipping invalid or empty line {i}: {repr(line)}")
        continue
    try:
        key, value = line.split("#")
        alpha, beta = key.split("|")
        angleA, angleB, angleC = value.split("|")
        dataDict[(float(alpha), float(beta))] = (float(angleA), float(angleB), float(angleC))
    except Exception as e:
        print(f"[Error] Skipping line {i} due to parsing error: {e}")
        continue

controllerWindow = tk.Tk()
controllerWindow.title("Control Window")
controllerWindow.geometry("820x500")
controllerWindow["bg"] = "white"
controllerWindow.resizable(0, 0)

videoWindow = tk.Toplevel(controllerWindow)
videoWindow.title("Camera Feed")
videoWindow.resizable(0, 0)
lmain = tk.Label(videoWindow)
lmain.pack()
videoWindow.withdraw()

graphWindow = tk.Toplevel(controllerWindow)
graphWindow.title("Position over Time")
graphWindow.resizable(0, 0)
graphCanvas = tk.Canvas(graphWindow, width=camHeight + 210, height=camHeight)
graphCanvas.pack()
graphWindow.withdraw()

pointsListCircle = []
def createPointsListCircle(radius):
    global pointsListCircle
    for angle in range(0, 360):
        angle_ = angle - 90
        pointsListCircle.append([radius * cos(radians(angle_)) + 240, radius * sin(radians(angle_)) + 240])
createPointsListCircle(150)

pointsListEight = []
def createPointsListEight(radius):
    global pointsListEight
    for angle in range(270, 270 + 360):
        pointsListEight.append([radius * cos(radians(angle)) + 240, radius * sin(radians(angle)) + 240 + radius])
    for angle in range(360, 0, -1):
        angle_ = angle + 90
        pointsListEight.append([radius * cos(radians(angle_)) + 240, radius * sin(radians(angle_)) + 240 - radius])
createPointsListEight(80)

drawCircleBool = False
def startDrawCircle():
    global drawCircleBool, drawEightBool, consigneX, consigneY
    if not drawCircleBool:
        drawCircleBool = True
        BballDrawCircle["text"] = "Center the Ball"
    else:
        drawCircleBool = False
        consigneX, consigneY = 240, 240
        sliderCoefP.set(sliderCoefPDefault)
        BballDrawCircle["text"] = "Move Ball in Circle"

drawEightBool = False
def startDrawEight():
    global drawEightBool, drawCircleBool, consigneX, consigneY
    if not drawEightBool:
        drawEightBool = True
        BballDrawEight["text"] = "Center the Ball"
    else:
        drawEightBool = False
        consigneX, consigneY = 240, 240
        sliderCoefP.set(sliderCoefPDefault)
        BballDrawEight["text"] = "Move Ball in Eight"

pointCounter = 0
def drawWithBall():
    global pointCounter, consigneX, consigneY
    if drawCircleBool:
        sliderCoefP.set(15)
        if pointCounter >= len(pointsListCircle):
            pointCounter = 0
        point = pointsListCircle[pointCounter]
        consigneX, consigneY = point[0], point[1]
        pointCounter += 7
    if drawEightBool:
        sliderCoefP.set(15)
        if pointCounter >= len(pointsListEight):
            pointCounter = 0
        point = pointsListEight[pointCounter]
        consigneX, consigneY = point[0], point[1]
        pointCounter += 7

def setConsigneWithMouse(mousePosition):
    global consigneX, consigneY
    if mousePosition.y > 10:
        refreshGraph()
        consigneX, consigneY = mousePosition.x, mousePosition.y

def getMouseClickPosition(mousePosition):
    global mouseX, mouseY
    global getPixelColor
    mouseX, mouseY = mousePosition.x, mousePosition.y
    getPixelColor = True

showVideoWindow = False
def showCameraFrameWindow():
    global showVideoWindow, showGraph
    if not showVideoWindow:
        if showGraph:
            graphWindow.withdraw()
            showGraph = False
            BafficherGraph["text"] = "Show Graph"
        videoWindow.deiconify()
        showVideoWindow = True
        BRetourVideo["text"] = "Hide Camera Feed"
    else:
        videoWindow.withdraw()
        showVideoWindow = False
        BRetourVideo["text"] = "Show Camera Feed"

showCalibrationOverlay = False
def showCalibrationOverlayFunc():
    global showCalibrationOverlay
    showCalibrationOverlay = not showCalibrationOverlay

showGraph = False
def showGraphWindow():
    global showGraph, showVideoWindow
    if not showGraph:
        if showVideoWindow:
            videoWindow.withdraw()
            showVideoWindow = False
            BRetourVideo["text"] = "Show Camera Feed"
        showGraph = True
        BafficherGraph["text"] = "Hide Graph"
    else:
        showGraph = False
        BafficherGraph["text"] = "Show Graph"

t = 480
consigneY = 240
consigneX = 240
def paintGraph():
    global t, consigneY, x, y, prevX, prevY, alpha, prevAlpha
    global showGraphPositionX, showGraphPositionY, showGraphAlpha
    if showGraph:
        graphWindow.deiconify()
        if showGraphPositionX.get() == 1:
            graphCanvas.create_line(t - 3, prevX, t, x, fill="#b20000", width=2)
        if showGraphPositionY.get() == 1:
            graphCanvas.create_line(t - 3, prevY, t, y, fill="#0069b5", width=2)
        if showGraphAlpha.get() == 1:
            graphCanvas.create_line(t - 3, 240 - prevAlpha * 3, t, 240 - alpha * 3, fill="#8f0caf", width=2)
        if t >= 480:
            t = 0
            graphCanvas.delete("all")
            graphCanvas.create_line(3, 3, 480, 3, fill="black", width=3)
            graphCanvas.create_line(3, 480, 480, 480, fill="black", width=3)
            graphCanvas.create_line(3, 3, 3, 480, fill="black", width=3)
            graphCanvas.create_line(480, 3, 480, 480, fill="black", width=3)
            graphCanvas.create_line(550, 32, 740, 32, fill="#b20000", width=5)
            graphCanvas.create_line(550, 53, 740, 53, fill="#0069b5", width=5)
            graphCanvas.create_line(550, 73, 740, 73, fill="#8f0caf", width=5)
            if showGraphPositionX.get() == 1:
                graphCanvas.create_line(3, consigneX, 480, consigneX, fill="#ff7777", width=2)
            if showGraphPositionY.get() == 1:
                graphCanvas.create_line(3, consigneY, 480, consigneY, fill="#6f91f7", width=2)
        t += 3
    else:
        graphWindow.withdraw()

def refreshGraph():
    global t
    t = 480

def endProgram():
    controllerWindow.destroy()

sliderHDefault = 20   # Larger to capture wider Hue range for dark colors
sliderSDefault = 50
sliderVDefault = 30
sliderCoefPDefault = 10
sliderCoefIDefault = 0.1
sliderCoefDDefault = 5.7

def resetSlider():
    sliderH.set(sliderHDefault)
    sliderS.set(sliderSDefault)
    sliderV.set(sliderVDefault)
    sliderCoefP.set(sliderCoefPDefault)
    sliderCoefI.set(sliderCoefIDefault)
    sliderCoefD.set(sliderCoefDDefault)

def donothing():
    pass

def lowerPlatform():
    if arduinoIsConnected:
        if messagebox.askokcancel("Warning", "Please remove the platform."):
            print("Lowering arms")
            ser.write(("descendArms\n").encode())
    else:
        if messagebox.askokcancel("Warning", "Arduino is not connected"):
            donothing()

def raisePlatform():
    global alpha
    if arduinoIsConnected:
        if messagebox.askokcancel("Warning", "Please remove the platform."):
            print("Raising arms")
            ser.write((str(dataDict[(0, 0)]) + "\n").encode())
            alpha = 0
    else:
        if messagebox.askokcancel("Warning", "Arduino is not connected"):
            donothing()

def servosTest():
    if arduinoIsConnected:
        if messagebox.askokcancel("Warning", "Platform must be in place."):
            for i in range(2):
                beta = 0
                alpha = 35
                while beta < 360:
                    ser.write((str(dataDict[(alpha, beta)]) + "\n").encode())
                    ser.flush()
                    time.sleep(0.002)
                    beta = round(beta + 0.2, 2)
                    print(alpha, beta)
            time.sleep(1)
            ser.write((str(dataDict[(0, 0)]) + "\n").encode())
    else:
        if messagebox.askokcancel("Warning", "Arduino is not connected"):
            donothing()

arduinoIsConnected = False
def connectArduino():
    global ser
    global label
    global arduinoIsConnected
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in p.description:
            print(p)
            ser = serial.Serial(p[0], 19200, timeout=1)
            time.sleep(1)
            label.configure(text="Arduino connected", fg="#36db8b")
            arduinoIsConnected = True

startBalanceBall = False
def startBalance():
    global startBalanceBall
    if arduinoIsConnected:
        if not startBalanceBall:
            startBalanceBall = True
            BStartBalance["text"] = "Stop"
        else:
            startBalanceBall = False
            BStartBalance["text"] = "Start"
    else:
        if messagebox.askokcancel("Warning", "Arduino is not connected"):
            donothing()

sommeErreurX = 1
sommeErreurY = 1
timeInterval = 1
alpha, beta, prevAlpha, prevBeta = 0, 0, 0, 0
omega = 0.2
def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, consigneX, consigneY):
    global omega
    global sommeErreurX, sommeErreurY
    global alpha, beta, prevAlpha, prevBeta
    global startBalanceBall, arduinoIsConnected

    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()

    Ix = Kp * (consigneX - ballPosX) + Ki * sommeErreurX + Kd * ((prevBallPosX - ballPosX) / 0.0333)
    Iy = Kp * (consigneY - ballPosY) + Ki * sommeErreurY + Kd * ((prevBallPosY - ballPosY) / 0.0333)

    Ix = round(Ix / 10000, 4)
    Iy = round(Iy / 10000, 4)

    if Ix == 0 and Iy == 0:
        alpha = 0
        beta = 0

    elif Ix != 0 and sqrt(Ix**2 + Iy**2) < 1:
        beta = atan(Iy / Ix)
        alpha = asin(sqrt(Ix**2 + Iy**2))
        beta = degrees(beta)
        alpha = degrees(alpha)
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180 - abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180 + abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360 - abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) < 1:
        if Iy > 0:
            beta = 90
            alpha = asin(sqrt(Ix**2 + Iy**2))
        elif Iy < 0:
            beta = 270
            alpha = asin(sqrt(Ix**2 + Iy**2))
        alpha = degrees(alpha)

    elif Ix != 0 and sqrt(Ix**2 + Iy**2) > 1:
        beta = degrees(atan(Iy / Ix))
        alpha = 35
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180 - abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180 + abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360 - abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) > 1:
        alpha = 35
        if Iy > 0:
            beta = 90
        elif Iy < 0:
            beta = 270

    if alpha > 35:
        alpha = 35

    alpha = prevAlpha * omega + (1 - omega) * alpha
    beta = prevBeta * omega + (1 - omega) * beta

    alpha = round(round(alpha / 0.2) * 0.2, -int(floor(log10(0.2))))
    beta = round(round(beta / 0.2) * 0.2, -int(floor(log10(0.2))))

    if alpha <= 35 and beta <= 360:
        prevAlpha = alpha
        prevBeta = beta

        if arduinoIsConnected and startBalanceBall:
            key = (round(alpha, 1), round(beta, 1))
            if key in dataDict:
                servo_values = dataDict[key]
                command_str = f"{servo_values[0]}|{servo_values[1]}|{servo_values[2]}\n"
                ser.write(command_str.encode())
            else:
                print(f"[Warning] Angle ({key}) not found in dataDict.")
    else:
        print(f"[Warning] Computed angles out of range: alpha={alpha}, beta={beta}")

prevX, prevY = 240, 240
prevConsigneX, prevConsigneY = 240, 240
x, y = 240, 240
start_time = time.time()

def main():
    start_timeFPS = time.time()
    global H, S, V
    global getPixelColor
    global x, y, alpha, beta
    global prevX, prevY, prevAlpha, prevBeta, prevConsigneX, prevConsigneY
    global consigneX, consigneY, sommeErreurX, sommeErreurY
    global camWidth, camHeight
    global timeInterval, start_time
    global showVideoWindow

    # Read from camera
    ret, img = cam.read()
    if not ret or img is None:
        print("Failed to capture from USB camera.")
        lmain.after(100, main)
        return

    img = img[0:int(camHeight), int((camWidth - camHeight) / 2):int(camWidth - ((camWidth - camHeight) / 2))]
    imgCircle = np.zeros(img.shape, dtype=np.uint8)
    cv2.circle(imgCircle, (240, 240), 270, (255, 255, 255), -1, 8, 0)
    img = img & imgCircle
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Color thresholds for black/grey ball detection
    # Tune if target ball is extremely black (may require further lowering V max and H spread)
    lowerBound = np.array([0, 0, 0])    # Minimum HSV for "blackish grey"
    upperBound = np.array([180, 65, 65])  # Max HSV for dark grey (adjust as needed for your lighting)

    mask = cv2.inRange(imgHSV, lowerBound, upperBound)
    mask = cv2.blur(mask, (6, 6))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)


    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    center = None

    cv2.circle(img, (int(consigneX), int(consigneY)), 4, (255, 0, 0), 2)
    if showCalibrationOverlay:
        cv2.circle(img, (240, 240), 220, (255, 0, 0), 2)
        cv2.circle(img, (240, 240), 160, (255, 0, 0), 2)
        cv2.line(img, (240, 240), (240, 240 + 160), (255, 0, 0), 2)
        cv2.line(img, (240, 240), (240 + 138, 240 - 80), (255, 0, 0), 2)
        cv2.line(img, (240, 240), (240 - 138, 240 - 80), (255, 0, 0), 2)
    if len(cnts) > 0:
        valid_cnts = [cnt for cnt in cnts if cnt is not None and len(cnt) > 0 and cnt.dtype in [np.int32, np.float32]]
        if valid_cnts:
            c = max(valid_cnts, key=cv2.contourArea)
        else:
            print("No valid contours with correct format found.")
            c = None
        timeInterval = time.time() - start_time
        if c is not None:
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 10:
                cv2.putText(img, str(int(x)) + ";" + str(int(y)), (int(x) - 50, int(y) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                PIDcontrol(int(x), int(y), prevX, prevY, consigneX, consigneY)
                start_time = time.time()
    else:
        sommeErreurX, sommeErreurY = 0, 0

    if showVideoWindow:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=img)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)
    lmain.after(5, main)

    drawWithBall()
    if prevConsigneX != consigneX or prevConsigneY != consigneY:
        sommeErreurX, sommeErreurY = 0, 0

    paintGraph()
    prevX, prevY = int(x), int(y)
    prevConsigneX, prevConsigneY = consigneX, consigneY
    prevAlpha = alpha
    prevBeta = beta

    print("FPS: ", 1.0 / (time.time() - start_timeFPS))

# ==== UI Layout ====

FrameVideoControl = tk.LabelFrame(controllerWindow, text="Video Control")
FrameVideoControl.place(x=20, y=20, width=380)
BRetourVideo = tk.Button(FrameVideoControl, text="Show Camera Feed", command=showCameraFrameWindow)
BRetourVideo.pack()
BPositionCalibration = tk.Button(FrameVideoControl, text="Overlay", command=showCalibrationOverlayFunc)
BPositionCalibration.place(x=290, y=0)

sliderH = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Hue Sensitivity", length=350, tickinterval=10)
sliderH.set(sliderHDefault)
sliderH.pack()
sliderS = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Saturation Sensitivity", length=350, tickinterval=10)
sliderS.set(sliderSDefault)
sliderS.pack()
sliderV = tk.Scale(FrameVideoControl, from_=0, to=100, orient="horizontal", label="Value Sensitivity", length=350, tickinterval=10)
sliderV.set(sliderVDefault)
sliderV.pack()

FrameServosControl = tk.LabelFrame(controllerWindow, text="Servos Control")
FrameServosControl.place(x=20, y=315, width=380)
BAbaisserPlateau = tk.Button(FrameServosControl, text="Lower Arms", command=lowerPlatform)
BAbaisserPlateau.pack()
BElevationBras = tk.Button(FrameServosControl, text="Raise Platform", command=raisePlatform)
BElevationBras.pack()
BTesterServos = tk.Button(FrameServosControl, text="Test Servo Motors", command=servosTest)
BTesterServos.pack()
BStartBalance = tk.Button(FrameServosControl, text="Start", command=startBalance, highlightbackground="#36db8b")
BStartBalance.pack()

FramePIDCoef = tk.LabelFrame(controllerWindow, text="PID Coefficients")
FramePIDCoef.place(x=420, y=20, width=380)
BafficherGraph = tk.Button(FramePIDCoef, text="Show Graph", command=showGraphWindow)
BafficherGraph.pack()
sliderCoefP = tk.Scale(FramePIDCoef, from_=0, to=15, orient="horizontal", label="P", length=350, tickinterval=3, resolution=0.01)
sliderCoefP.set(sliderCoefPDefault)
sliderCoefP.pack()
sliderCoefI = tk.Scale(FramePIDCoef, from_=0, to=1, orient="horizontal", label="I", length=350, tickinterval=0.2, resolution=0.001)
sliderCoefI.set(sliderCoefIDefault)
sliderCoefI.pack()
sliderCoefD = tk.Scale(FramePIDCoef, from_=0, to=10, orient="horizontal", label="D", length=350, tickinterval=2, resolution=0.01)
sliderCoefD.set(sliderCoefDDefault)
sliderCoefD.pack()

FrameBallControl = tk.LabelFrame(controllerWindow, text="Ball Control")
FrameBallControl.place(x=420, y=315, width=380, height=132)
BballDrawCircle = tk.Button(FrameBallControl, text="Move Ball in Circle Trajectory", command=startDrawCircle)
BballDrawCircle.pack()
BballDrawEight = tk.Button(FrameBallControl, text="Move Ball in Eight Trajectory", command=startDrawEight)
BballDrawEight.pack()

label = tk.Label(controllerWindow, text="Arduino disconnected", fg="red", anchor="ne")
label.pack(fill="both")
BReset = tk.Button(controllerWindow, text="Reset", command=resetSlider)
BReset.place(x=20, y=460)
BConnect = tk.Button(controllerWindow, text="Connect", command=connectArduino, background="black")
BConnect.place(x=100, y=460)
BQuit = tk.Button(controllerWindow, text="Quit", command=endProgram)
BQuit.place(x=730, y=460)

showGraphPositionX = tk.IntVar()
showGraphPositionX.set(1)
CheckbuttonPositionX = tk.Checkbutton(graphWindow, text="X Position", variable=showGraphPositionX, command=refreshGraph)
CheckbuttonPositionX.place(x=500, y=20)
showGraphPositionY = tk.IntVar()
showGraphPositionY.set(1)
CheckbuttonPositionY = tk.Checkbutton(graphWindow, text="Y Position", variable=showGraphPositionY, command=refreshGraph)
CheckbuttonPositionY.place(x=500, y=40)
showGraphAlpha = tk.IntVar()
CheckbuttonAlpha = tk.Checkbutton(graphWindow, text="Plate Inclination", variable=showGraphAlpha, command=refreshGraph)
CheckbuttonAlpha.place(x=500, y=60)

videoWindow.protocol("WM_DELETE_WINDOW", donothing)
videoWindow.bind("<Button-2>", getMouseClickPosition)
videoWindow.bind("<Button-1>", setConsigneWithMouse)

main()
tk.mainloop()
