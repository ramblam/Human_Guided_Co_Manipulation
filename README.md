# Human_Guided_Co_Manipulation (IN PROGRESS)
This repository contains the codes utilized and example clips of the experiments performed for our paper Human-Guided Co-Manipultion of Carbon Fiber Plies. The implemetation consists of three main parts: control, (wrist) detection, and verbal communication (both speech recognition and TTS). The implementation of those can be found from the corresponding directories. To keep the main README manageable in size, more information and instructions on how to run those can be found within those subfolders.

## Experiments
As explained in more detail in the paper, Five different control methods for co-manipulation of carbon fiber ply were tested and compared both with and without an obstacle in the scene:

* Compliant control with zero stiffness
* Voice control using step-by-step commands
* Wrist tracking with robot following the user
* Robot has a predefined trajectory while user follows
* A hybrid approach combining wrist tracking, stepwise voice commands and compliance

The experimental procedure consists of the user and robot picking up a sheet of carbon fiber, maneuvering it over a mold while avoiding the possible obstacle, and placing the ply on the mold. The procedure is repeated three times for each control strategy for both cases. Example clips of the experiments performed can be found below. The data presented in the paper was collected from similar experiments.

### Case 1 (without obstacle):

#### Compliance:

https://github.com/user-attachments/assets/6fc10816-38c9-480f-bf1a-2791383a375f

#### Stepwise voice control:

https://github.com/user-attachments/assets/8d971a84-f133-4bca-8222-e102788d41a1

#### Wrist tracking:

https://github.com/user-attachments/assets/faa31b21-2f30-4656-ba65-3437d1e00da8

#### Predefined trajectory:

https://github.com/user-attachments/assets/327c3a32-a8e0-41aa-b5ae-9c595f7bf01d

#### Hybrid approach:
ADD THE VIDEO HERE (SLIGHTLY TOO BIG)

### Case 2 (with obstacle):

#### Compliance:

https://github.com/user-attachments/assets/57624fac-0203-4554-b694-5b84ee84a56b

#### Stepwise voice control:

https://github.com/user-attachments/assets/f0fc8e61-e9fe-4041-a263-1c3d7ef14d5c

#### Wrist tracking:

https://github.com/user-attachments/assets/0df83344-fa62-4df5-916f-94d217cef49a

#### Predefined trajectory:

https://github.com/user-attachments/assets/414e820e-7abf-4602-ba6f-6160bb374969

#### Hybrid approach:

ADD THE VIDEO HERE (SLIGHTLY TOO BIG)

## Credits & Acknowledgements
**Core research, analysis, experiments etc.:** Rami Ojanen [(@ramblam)](https://github.com/ramblam)

**The Cartesian Impedance controller utilized was originally implemented (in ROS1) by Matthias Mayr, see: https://github.com/matthias-mayr/Cartesian-Impedance-Controller/tree/master**

**Thanks and credits for the ROS2 port of the Cartesian Impedance Controller and many control-related clients and functionalities**: Ossi Parikka [(@Ozzyuni)](https://github.com/ozzyuni)



