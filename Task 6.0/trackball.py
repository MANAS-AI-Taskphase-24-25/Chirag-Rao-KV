import cv2 as cv
import numpy as np
vid  = cv.VideoCapture('Task 6.0/volleyball_match.mp4')
frame_width = int(vid.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(vid.get(cv.CAP_PROP_FRAME_HEIGHT))
fps = int(vid.get(cv.CAP_PROP_FPS))
fourcc = cv.VideoWriter_fourcc(*'XVID')  
out = cv.VideoWriter("Task 6.0/output_video.avi", fourcc, fps, (frame_width, frame_height))

def circularity(area,perimeter):
   
    if area > 0:
        score =  (4 * np.pi * area) / (perimeter ** 2)
        return score
    else:
        return 0
    
    


def FindYellowball(image):
    
    hsv = cv.cvtColor(image,cv.COLOR_BGR2HSV)
    
    #defing the range of yellow
    lower_yellow = np.array([10, 60, 80])  
    upper_yellow = np.array([30, 255, 255])     
    blur = cv.GaussianBlur(hsv,(7,7),0)
    mask = cv.inRange(blur,lower_yellow, upper_yellow)
    mask = cv.dilate(mask,(5,5),iterations=15)
    mask = cv.erode(mask,(5,5),iterations= 10)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    max_score = 0
    if(len(contours )!= 0):
        ball_contour = contours[0]
        for c in contours:
            
            area = cv.contourArea(c)
            peri = cv.arcLength(c,closed= False)
            score = circularity(area,peri)
            if 0 <= score <= 1.5 and 300 <= area <= 500:
                print(f" area  ={area}, score: {score}")

                if(score > max_score):
                    max_score = score
                    ball_contour = c

        if ball_contour is not None:
                x, y, w, h = cv.boundingRect(ball_contour)
                center = (x + w // 2, y + h // 2)  
                print(f"Ball detected at {center}")
                ball_contour = contours[0]
                return [center[0], center[1]]
                
        print("No ball detected.")
        return []
    
while(True):
    _,frame = vid.read()
    coordinates= np.array(FindYellowball(frame))
    if coordinates.all():  # Ensure x and y are not empty
        print(f"coordinates are {coordinates}")
        cv.circle(frame, coordinates, 5, (0, 0, 255), 20)  
        cv.circle(frame, coordinates, 5, (0, 255, 0), 10)  
        cv.imshow("circled",frame)
        out.write(frame)

    
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break
    
vid.release()
cv.destroyAllWindows()