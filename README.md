About
------

[1] adapts one of the most influential models in the field of collective behaviour [2], and projects virtual prey in the 
tank of a bluegill sunfish (_Lepomis macrochirus_). Based on predatory interactions of the sunfish with virtual prey, 
virtual agents evolve social interaction rules. The experiment depicts the importance of both alignment with and attraction
to conspecifics to avoid predators (even in a condition where prey are not responding to the predator). Here, we present
this simulation with the evolvable parameters set to their optimum.

The simulation was written in C++ and uses OpenCV for visualisation.

Use
---

The above code is written using C++11 and tested on MacOS Sierra 10.12.5. The visualisation is executed using OpenCV 3.2.0. In case of troubles installing OpenCV / linking it to the C++ project, visualisation can be turned off by silencing OpenCV headers and the graphics function in main.cpp. The rest of the code should still work on naked C++.

Reference
---------

1. Christos C. Ioannou, Vishwesha Guttal, and Iain D. Couzin, Predatory fish select for coordinated
collective motion in virtual prey, _Science_ **337**, 1212 (2017).

2. Iain D. Couzin, Jens Krause, Nigel R. Franks, and Simon A. Levin, Effective leadership and decision-making
in animal groups on the move, _Nature_ **433**, 513 (2005).
