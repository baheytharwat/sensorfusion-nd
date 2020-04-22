# **Track an Object in 3D Space** 

## Writeup

---

**Track an Object in 3D Space Project**


[//]: # (Image References)

[image1]: ./images/lidar.png "Visualization"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/2550/view) individually and describe how I addressed each point in my implementation.  

---
### FP.0 Final Report

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it!

### FP.1 Match 3D Objects

#### 1. Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.
This function `matchBoundingBoxes` can be found in `camFusion_Student.cpp` Code lines `267-325`.

### FP.2 Compute Lidar-based TTC

#### Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame. 
This function `computeTTCLidar` can be found in `camFusion_Student.cpp` Code lines `228-264`.


### FP.3 Associate Keypoint Correspondences with Bounding Boxes

#### Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.
This function ` clusterKptMatchesWithROI` can be found in `camFusion_Student.cpp` Code lines `134-174`.

### FP.4 Compute Camera-based TTC

#### Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame..
This function ` computeTTCCamera` can be found in `camFusion_Student.cpp` Code lines `178-225`.


### FP.5 Performance Evaluation 1

#### Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

Most of the estimations from lidar seem plausible. They don't deviate a lot because I am dealing with the mean distance from current and previous frames. but If there are some errors, I think the car could not be detected well, The bounding box is not fitting the car and maybe there are some points that are not located on the car. Maybe increasing the shrinking factor can help in this problem.


### FP.6 Performance Evaluation 2


#### Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

I chose the best descriptors for each detectors  ( From previous project) and tried these combinations. All the results can be found in `results.csv`. 

Camera-based TTC estimation has many fault results while using 'HARRIS' detector with 'FREAK' descriptor. Some of the estimations are negative and -infinity. Also, Using 'ORB' detector with 'BRIEF' descriptor has some huge number such as 188,  40 and 31. I think maybe the reasons are camera calibration, camera distorion and detection of keypoints and matching.