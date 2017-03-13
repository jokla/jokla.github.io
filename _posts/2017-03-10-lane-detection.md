---
title:  "Finding Lane Lines on the Road"
excerpt: "First project of the Udacity Self Driving Car NanoDegree"
modified: 2017-03-10
categories: 
  - Robotics
tags:
  - Computer vision
  - OpenCV
---


## Overview:

The goal of this project is to make a pipeline that finds lane lines on the road using Python and OpenCV. See an example:

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/solidWhiteRight.jpg" width="360" alt="Combined Image" />    <img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/laneLines_thirdPass.jpg" width="360" alt="Combined Image" />

The pipeline will be tested on some images and videos provided by Udacity. The following assumptions are made:
* The camera always has the same position with respect to the road
* There is always a visible white or yellow line on the road
* We don't have any vehicle in front of us 
* We consider highway scenario with good weather conditions

---

## Reflection

###1. Describe your pipeline.

I will use the following picture to show you all the steps:  

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/original.png" width="360" alt="Combined Image" />

#### Color selection   
Firstly, I applied a color filtering to suppress non-yellow and non-white colors. The pixels that were above the thresholds have been retained, and pixels below the threshold have been blacked out.  This is the result:

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/mask_color.png" width="360" alt="Combined Image" />

I will keep aside this mask and use it later.

#### Convert the color image in grayscale  
The original image is converted in grayscale. In this way we have only one channel:

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/gray.png" width="360" alt="Combined Image" />


#### Use Canny for edge detection 
Before running the [Canny detector](http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html), I applied a [Gaussian smoothing](http://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html?highlight=gaussianblur#gaussianblur) which is essentially a way of suppressing noise and spurious gradients by averaging. The Canny allows detecting the edges in the images. To improve the result, I also used the OpenCV function `dilate` and `erode`.

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/canny.png" width="360" alt="Combined Image" />

#### Merge Canny and Color Selection
In some cases, the Canny edge detector fails to find the lines. For example, when there is not enough contrast between the asphalt and the line, as in the challenge video (see section `Optional challenge`). The color selection, on the other hand, doesn't have this problem. For this reason, I decided to merge the result of the Canny detector and the color selection:   

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/merge.png" width="360" alt="Combined Image" />

#### Region Of Interest Mask
I defined a left and right trapezoidal Region Of Interest (ROI) based on the image size. Since that the front facing camera is mounted in a fix position, we supposed here that the lane lines will always appear in the same region of the image. 
 
<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/roi.png" width="360" alt="Combined Image" />

#### Run Hough transform to detect lines  
The Hough transform is used to detect lines in the images. At this step, I applied a slope filter to get rid of horizontal lines. This is the result:   
<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/hough.png" width="360" alt="Combined Image" />


#### Compute lines
Now I need to average/extrapolate the result of the Hough transform and draw the two lines onto the image. I used the function  [`fitLine`](http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#fitline), after having extrapolated the points from the Hough tranform result with the OpenCV function `findNonZero`. I did this two times, once for the right line and another time for the left line. As a result, I got the slopes of the lines, and I could draw them onto the original picture:   

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/final.png" width="360" alt="Combined Image" />

## Results:

### Pictures
Here some results on test images provided by Udacity:   
<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/final.png" width="360" alt="Combined Image" /> <img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/result1.png" width="360" alt="Combined Image" />    
<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/result2.png" width="360" alt="Combined Image" /> <img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/result3.png" width="360" alt="Combined Image" />   
<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/result4.png" width="360" alt="Combined Image" />     <img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/result5.png" width="360" alt="Combined Image" />        

You can find the original pictures and the results in the folder `test_images`.


### Videos
Here some results on test videos provided by Udacity:   

![](https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/white.gif) 
![](https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/yellow.gif)

You can find the video files here: [video1](https://github.com/jokla/CarND-LaneLines-P1/blob/master/yellow.mp4), [video2](https://github.com/jokla/CarND-LaneLines-P1/blob/master/white.mp4).

#### Optional challenge:

While I got a satisfactory result on the first two videos provided by Udacity, it was not the case for the challenge video. In the challenge video we can identify more difficulties:
* The color of the asphalt became lighter at a certain point. The Canny edge detector is not able to find the line using the grayscale image (where we lose information about the color)
* The car is driving on a curving road
* There are some shadows due to some trees  

To overcome theses problems, I introduced the color mask and resized the ROI. This is the result, using only the color mask (without the canny detection): 

![](./img_doc/extra.gif)  

You can find the video file here: [video_challenge](https://github.com/jokla/CarND-LaneLines-P1/blob/master/extra.mp4)   

The right line is a little jumpy mainly because of the curve: the function `fitline` is trying to fit a line on a curvy lane. It would be useful to shrink the ROI in this case, but I preferred to keep the same ROI size used in the first two videos.

If we analyze the steps using a snapshot from the challenge video, we can notice that the Canny detector is not very useful:   

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/original_challenge.png" width="360" alt="Combined Image" /> <img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/canny_challenge.png" width="360" alt="Combined Image" />

while the color mask is able to detect the lines:   

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/color_challenge.png" width="360" alt="Combined Image" />
  
Indeed, as you can see in the following picture, we lose valuable color information when we convert the image in grayscale. Moreover, the Canny operator find a lot of edges when we have shadows on the road.   

<img src="https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/gray_challenge.png" width="360" alt="Combined Image" />


#### Testing the pipeline on a YouTube Video:

Just out of curiosity, I wanted to test the pipeline on a video extracted from Youtube (see the original video [here](https://www.youtube.com/watch?v=jwBaGY67olI) ).

I noticed that the color selection was not working properly in this case, so I had to tune a little bit the thresholds values. This is the new result using both color selection and Canny:

![](https://github.com/jokla/jokla.github.io/tree/master/images/post/2017-03-10-lane-detection/extra_test.gif)

You can find the video file here: [video_extra](https://github.com/jokla/CarND-LaneLines-P1/blob/master/extra_test_result.mp4)   

It would be wiser to transform the image in the HSV space and to apply the color selection, instead of doing it on the RGB images.

###2. Identify potential shortcomings with your current pipeline

* This approach could not work properly:
    * if the camera is placed at a different position
    * if other vehicles in front are occluding the view
    * if one or more lines are missing
    * at different weather and light condition (fog, rain, or at night)


###3. Suggest possible improvements to your pipeline

Some possible improvements:

* Perform a color selection in the HSV space, instead of doing it in the RGB images
* Update the ROI mask dynamically
* Perform a segmentation of the road
* Using a better filter to smooth the current estimation, using the previous ones
* If a line is not detected, we could estimate the current slope using the previous estimations and/or the other line detection
* Use a moving-edges tracker for the continuous lines
