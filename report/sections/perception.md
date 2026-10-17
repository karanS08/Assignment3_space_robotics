# Perception

Owner: Andrew Than.

## Perception 1: Image dataset
1 Group Member Details
Andrew Than	13223096

3 Statement of Individual Contributions
Perception 1 (dataset) – COMPLETED
Perception 2( detection) – Alien, rock completed
Perception 3 (localisation) – Partially completed
Recorded videos – not completed
4 Project Overview

5 Perception 
5.1 Perception 1
The objective of Perception Task 1 was to collect a dataset of artefact images from the rover's onboard RGB camera. An automated image collection approach was selected to reduce manual effort and ensure data was gathered continuously during exploration.
The rover's RGB camera feed was accessed through the ROS 2 camera/image topic. Images received by the image callback were converted to OpenCV format using the CvBridge library. A dataset directory is automatically created, and images were stored at regular intervals throughout exploration. To avoid saving duplicate frames and to reduce storage consumption, an interval of three seconds was chosen to capture an image. The filename for each image is generated to have its timestamp so that each image was in logical order with unique names.
The implementation of the dataset collection system is shown in Figure.1

<img width="940" height="201" alt="image" src="https://github.com/user-attachments/assets/a5d2cf4a-a34a-4b06-86cd-b79000524483" />
 
Figure 1. Dataset Collection System Code

The images were then saved into this dataset directory during the running of the image callback in Figure 2.

<img width="940" height="229" alt="image" src="https://github.com/user-attachments/assets/d1a13719-9553-494a-af3b-02bebe87cfa6" />

Figure 2. Image Save Directory Code

The dataset collection system functioned successfully and continuously captured images throughout the rover exploration. The automated approach ensured that images were collected from a variety of distances, orientations and viewpoints without human input. A few examples of each artefact captured by the rover can be seen.

Alien	
<img width="243" height="198" alt="image" src="https://github.com/user-attachments/assets/cabf4a82-7148-4e72-85d9-6220ba3beed5" />

Rock
<img width="241" height="244" alt="image" src="https://github.com/user-attachments/assets/59907b87-138c-4765-8740-ed266fd5c581" />


Ice Cap	 
<img width="244" height="121" alt="image" src="https://github.com/user-attachments/assets/7d34dc20-5541-47b7-abf2-729514147de6" />

Sphere	 
<img width="243" height="261" alt="image" src="https://github.com/user-attachments/assets/f7d15fa9-c348-4dcf-a777-2994c342fdca" />

Mushroom	 
<img width="243" height="245" alt="image" src="https://github.com/user-attachments/assets/59755c39-1817-4608-a07f-1b2d09cc846a" />

Stop Sign	 
<img width="347" height="425" alt="image" src="https://github.com/user-attachments/assets/6d00f453-f0eb-4b17-806b-3de6874cf115" />

Nothing	 
<img width="338" height="217" alt="image" src="https://github.com/user-attachments/assets/517bf3a6-1747-49da-b2c1-ec24de1f4313" />


Because image collection occurred during autonomous exploration, the number of samples obtained for each artefact type depended on the rover's navigation path. Artefacts that were observed more frequently contributed a larger number of images, resulting in an imbalanced dataset.
Table. 1 – Distribution of artefact images collected in 5 minutes

Table 1 - Distribution of artefact images collected in 5 minutes
| Artefact Type | Images Collected |
|--------------|-----------------:|
| Alien | 4 |
| Rock | 17 |
| Ice Cap | 23 |
| Sphere | 41 |
| Mushroom | 13 |
| Stop Sign | 73 |
| Nothing (Negative Example) | 1 |
| **Total** | **100** |


The results demonstrate that the dataset was not evenly distributed across artefact classes. Sphere and Stop Sign artefacts appeared more frequently within the rover's field of view, whereas Alien artefacts were encountered significantly less. This reflects the autonomous exploration behaviour of the rover rather than a deliberate sampling strategy. Overall, the dataset collection system successfully generated a collection of 100 artefact images without requiring manual intervention, demonstrating that the rover could continuously gather visual data while exploring the environment.
Although the dataset collection system successfully automated the image capture process, several limitations were identified. All captured images were stored within a single dataset directory, meaning artefact classes were not automatically separated during collection. Consequently, the system did not record the artefact class associated with each image, requiring manual inspection and sorting before the dataset could be used for analysis. Additionally, because images were collected during autonomous exploration, the frequency with which each artefact was observed depended on the rover's navigation path. As a result, some artefact classes were represented by more images than others, creating an imbalanced dataset. 
Future work could focus on automatically organising and labelling images during collection. For example, once an artefact has been detected by the perception system, the corresponding image could be automatically saved into a dedicated folder for that artefact class. This would significantly reduce the manual effort required to sort and label the dataset. Additional metadata, including artefact distance, rover position and timestamp information, could also be stored to provide greater context for each image. Finally, a more structured exploration strategy could be implemented to ensure a more balanced number of images are collected for each artefact type.


## Perception 2: Artefact detection

## Perception 3: Artefact localisation and display
