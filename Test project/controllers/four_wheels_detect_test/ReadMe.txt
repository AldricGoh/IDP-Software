Four_wheels_detect_test.py gives the software used before we were given access to the actual robot

Robot has 2 wheels and one central distance sensor
It also has an emitter receiver for comms between robots
#The robot is based on a coordiiante system where x points North, z points East and angles are taken with respect to the poitive z axis to make calculations easier


Block finding algorithm is done by comparing the measured value to an expected value
Discrepancies are divided into several cases,
It is a new block
It is an existing block
It is a sorted block
It is the other robot

These are broadcasted to the other robot so theyc an coordinate

Sometimes, if two blocks are close together only one will be detected.

However, once one block is sorted, the second box will be scanned again at a later time.

Very little navigation logic has been implemented as colour detection and comms have to be sorted first.

Graphics.py is an optional extra model for making a GUI if we ever need to
