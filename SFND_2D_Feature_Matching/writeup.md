# **Camera Based 2D Feature Tracking** 


**Camera Based 2D Feature Tracking**

The goals / steps of this project are the following:
* Detect keypoints on image
* Extract descriptors for keypoints
* Match keypoints through sequence of images



[//]: # (Image References)

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/2549/view) individually and describe how I addressed each point in my implementation.  

---
### Mid-Term Report

#### 1. MP.0 Mid-Term Report

You are reading it now.

### Data Buffer


#### 1. MP.1 Data Buffer Optimization

I optimized the data buffer by storing just speciefic number of frames which is default to 2 Instead of storing all the frames. `MidTermProject_Camera_Student.cppMidTermProject_Camera_Student.cpp` (Code Lines: [68:78] ) 

#### Keypoints
#### 1. MP.2 Keypoint Detection
There are so many available detectors to use ( SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT ). Just pass its name to 'detectorType' `MidTermProject_Camera_Student.cppMidTermProject_Camera_Student.cpp` (Code Lines: [92:98] ).
Implementation of each of them can be found in `matching2D_Student.cpp` (detKeypointsShiTomasi[120], detKeypointsHarris[161], detKeypointsModern[223] ).


#### 2. MP.3 Keypoint Removal
The keypoint detectors returns many keypoints. most of them are unwanted because we only care about the keypoints the are placed on the front car. So, I specify a rectangle region to choose  keypoints inside it.
`MidTermProject_Camera_Student.cppMidTermProject_Camera_Student.cpp` (Code Lines: [134:154] ).


### Descriptors

#### 1. MP.4 Keypoint Descriptors

There are so many available descriptors to use ( BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT ). Just pass its name to 'descriptorType' `MidTermProject_Camera_Student.cppMidTermProject_Camera_Student.cpp` (Code Lines: [184:189] ).
Implementation of each of them can be found in `matching2D_Student.cpp` (descKeypoints[65] ).

#### 2. MP.5 Descriptor Matching

There are two available descriptor matchers to use ( MAT_BF, MAT_FLANN ). Just pass its name to 'matcherType' `MidTermProject_Camera_Student.cppMidTermProject_Camera_Student.cpp` (Code Lines: [208:209] ).
Implementation of each of them can be found in `matching2D_Student.cpp` (matchDescriptors[7] ).
Also, You can choose both ways when matching. either 'Gradient-based' or 'Binary-Based' using 'descriptorType2' Line[210:211]. For matching, you can use either  Nearest neighbour or K-Nearest neighbour using 'selectorType' Line[212:213].


#### 3. MP.6 Descriptor Distance Ratio

I implemented K-Nearest-Neighbor matching and used k = 2 to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints. and used a threshold value = 0.8.

Its implementation can be found in `matching2D_Student.cpp` (Code Lines[47:57] ).
### Performance
#### 1. MP.7 Performance Evaluation 1


#### 2. MP.8 Performance Evaluation 2



#### 3. MP.9 Performance Evaluation 3

All these evaluations can be found in `MidTermProject_Camera_Student.cppMidTermProject_Camera_Student.cpp` (Code Lines: [239:251] ).
Type of detector used, Type of descriptor used, processing time for each of detector and descriptor, total processing time for an image, an Average processing time, Number of keypoints and Number of matched points.
All these informations are displayed in each iteration.
The results of the use of different combinations of detectors and descriptors can be found in `results.csv` file in the project folder.
According to the average processing time, The results  show that The TOP3 detectors/descriptors combination are:
# Detector / Descriptor :
* FAST / BRIEF
* FAST / ORB
* FAST / FREAK


'FAST' is the fastest one and the second fastest one is 'ORB'.
