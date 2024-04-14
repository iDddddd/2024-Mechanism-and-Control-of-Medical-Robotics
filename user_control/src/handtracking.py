import cv2
import mediapipe as mp
import time
import math


if __name__ == '__main__':

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    mpHands = mp.solutions.hands
    hands = mpHands.Hands()
    mpDraw = mp.solutions.drawing_utils
    handLMsStyle = mpDraw.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2)
    handConStyle = mpDraw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
    pTime = 0
    cTime = 0

    while True:
        ret, img = cap.read()
        img = cv2.flip(img, 1)
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            continue
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        result = hands.process(imgRGB)
        # print(result.multi_hand_landmarks)

        if result.multi_hand_landmarks:
            count = 0 # 记录手指伸出的个数
            for handLms in result.multi_hand_landmarks:
                mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS, handLMsStyle, handConStyle)
                finger = []
                finger_point = []
                #这里我将每一根手指的四个坐标整合到一个列表中，大家可以打印finger，进行详细的查看
                for id,lm in enumerate(handLms.landmark):
                    h, w, c = img.shape
                    x,y = int(lm.x*w),int(lm.y*h)
                    if id == 0:
                        pass
                    elif id % 4 == 0:
                        finger_point.append([x,y])
                        finger.append(finger_point)
                        finger_point = []
                    else:
                        finger_point.append([x,y])
                    if id == 4:
                        cv2.circle(img, (x, y), 15, (255, 0, 0), cv2.FILLED)
                        print(id, x, y)
                        # 发布消息
                        msg = [x, y]

                #遍历每一根手指列表，计算其构成的三角形的三边长，这里使用2，6，10，14，18所对应的角进行判断
                for id,point in enumerate(finger):
                    a = math.hypot((point[0][0]-point[1][0]),(point[0][1]-point[1][1]))
                    b = math.hypot((point[1][0]-point[2][0]),(point[1][1]-point[2][1]))
                    c = math.hypot((point[0][0]-point[2][0]),(point[0][1]-point[2][1]))
                    try:
                        value = (a**2+b**2-c**2)/(2*a*b)
                        #这里的value为弧度制，乘上57转换为角度制，当然你也可以选择直接使用弧度制
                        angle = math.acos(value)*57
                    except ValueError:
                        angle = 180
                    except ZeroDivisionError:
                        angle = 0

                    #若为大拇指，当角度大于160的时候记为手指伸出
                    if id == 0:
                        if angle >= 160:
                            count += 1
                        else:
                            pass
                    else:
                        #当角度大于130的时候记为手指伸出
                        if angle >= 130:
                            count += 1
                        else:
                            pass
                    #检测是否握拳
                    if count == 0:
                        cv2.putText(img, "Fist", (10, 90), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
                    else:
                        pass

            cv2.putText(img, f"Count: {count}", (10, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime
            cv2.putText(img, f"FPS: {int(fps)}", (400,50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
            cv2.imshow('img', img)

            if cv2.waitKey(1) == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
