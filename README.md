# csci5611pj2   
# Name: Bob Zhou
 

### control the simulation:
   arrow keys: move the root of the skeleton,
   c: switch the arm being controlled.


# demo image:
![alt text](https://github.com/bobhansky/5611_PA3/blob/main/show.png)

### demo video:

[https://www.youtube.com/watch?v=kgArrB1H1Wc](https://www.youtube.com/watch?v=Ydjn56p6J38)


# Timestamp
## (also available under the youtube video description )
<pre>

Two arms skeleton  0:10
User interaction   0:15
3D Rendering 0:31
Two arms skeleton explaination  1:08
Moving IK    1:22
Joints limit 1:56


</pre>



## • List of the tools/library you used && All code for your project with a clear indication of what code you wrote
     Vec2.pde is provided by Dr Stephen J Guy.

     
     All other files are written by me.


## Brief write-up explaining difficulties you encountered
   Implementing two arms shareing one segment is a work requires patience and careness: 
   conflict when updating two arms trying to reach the same endpoint may happen, so I only decide
   to allow user to control one arm. 

   For a resonable simulation (or looks resonable), I need to tune the angle limits carefully.
