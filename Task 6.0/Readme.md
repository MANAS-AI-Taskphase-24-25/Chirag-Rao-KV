# Task 6
## Description

Timothy being a reptile has eyes on either side of his face and his having trouble keeping track of the Ball in a match between two teams. Can you help him out?

Use opencv to help Tim consitently track the ball.

Implementation

The task is to use opencv functionalities and  track the trajectory of the ball in the given video of a volleyball match. You may choose to either trace the ball's path, or highlight its current location while it moves. Also find the number of players in each team.

## Documentation
## Approach and Challenges
I tried using Houge gradient for detecting circles is not reliable as it uses greyscale input which causes other circular objects to be detected as the ball.

After trying few other methdods, best results were seen in using Gaussian Blur to blur the image and mask the frames based on color (yellow here). 

One more challenge faced here is, the ball in not completely yellow, it has blue stripes. But the colors of the ball are not seen as they are due to camera angle and lighting. To make up for this, I decided to pass the masked frame through Dialution and erosion.

![Mask](mask.png)

I think the range of yellow can be optimised to NOT include the players who are also wearing jersey of color yellow.


To distinguish between other contour and the ball to be tracked, I use the circularity of the ball. This however this has its own disadvantage as the score is capped between values when the ball moves very far from the camera or is very near the camera, it is not detected.
```
def circularity(area,perimeter):
    #score based on area and perimeter as real life objects are not perfecty circular or may have color variations.
    if area > 0:
        score =  (4 * np.pi * area) / (perimeter ** 2)
        return score
    else:
        return 0
```

Almost the same logic can be used for players, masking based on colour and finding the contours of a certain area range. But in the current case the one of the team has jersey colored same as the floor, making it difficult to mask out the players.  