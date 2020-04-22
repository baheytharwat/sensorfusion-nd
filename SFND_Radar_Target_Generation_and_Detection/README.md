# **Radar Target Generation and Detection** 

## Writeup

---

**Radar Target Generation and Detection**


[//]: # (Image References)

[image1]: ca_cfar_code.JPG "Ca-Cfar code"
[image2]: resize_output.JPG "resize the map code"
[image3]: frist_fft.JPG "range"
[image4]: second_fft.JPG "Range Doppler Map"
[image5]: ca-cfar_filter.JPG "final output"


---
### 2D CFAR


Slide the cell under test across the complete matrix.
For every iteration sum the signal level within all the training cells. 
Average the summed values for all of the training cells used. 
Further add the offset to it to determine the threshold.
Next, compare the signal under CUT against this threshold.
If the CUT level > threshold assign it a value of 1, else equate it to 0.

![alt text][image1]

To keep the map size same as it was before CFAR, equate all the non-thresholded cells to 0

![alt text][image2]

### results

![alt text][image3]
![alt text][image4]
![alt text][image5]
