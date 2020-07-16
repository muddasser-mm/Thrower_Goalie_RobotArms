world { }

# Define the boundary

fence1c (world)	{  
    shape:ssBox, Q:<t(0.5 -3.575 0.05)>, size:[1 0.15 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence1r1 (world)	{  
    shape:ssBox, Q:<t(-0.5 -3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence1r2 (world)	{  
    shape:ssBox, Q:<t(-1.5 -3.575 0.05)>, size:[1 0.15 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence1r3 (world)	{  
    shape:ssBox, Q:<t(-2.5 -3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence1l1 (world)	{  
    shape:ssBox, Q:<t(1.5 -3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence1l2 (world)	{  
    shape:ssBox, Q:<t(2.5 -3.575 0.05)>, size:[1 0.15 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence1l3 (world)	{  
    shape:ssBox, Q:<t(3.5 -3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence2c (world)	{  
    shape:ssBox, Q:<t(0.5 3.575 0.05)>, size:[1 0.15 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence2r1 (world)	{  
    shape:ssBox, Q:<t(-0.5 3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence2r2 (world)	{  
    shape:ssBox, Q:<t(-1.5 3.575 0.05)>, size:[1 0.15 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence2r3 (world)	{  
    shape:ssBox, Q:<t(-2.5 3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence2l1 (world)	{  
    shape:ssBox, Q:<t(1.5 3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence2l2 (world)	{  
    shape:ssBox, Q:<t(2.5 3.575 0.05)>, size:[1 0.15 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence2l3 (world)	{  
    shape:ssBox, Q:<t(3.5 3.575 0.05)>, size:[1 0.15 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence3c (world)	{  
    shape:ssBox, Q:<t(4.075 0 0.05)>, size:[0.15 1 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence3r1 (world)	{  
    shape:ssBox, Q:<t(4.075 -1 0.05)>, size:[0.15 1 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence3r2 (world)	{  
    shape:ssBox, Q:<t(4.075 -2 0.05)>, size:[0.15 1 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence3r3 (world)	{  
    shape:ssBox, Q:<t(4.075 -3 0.05)>, size:[0.15 1 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence3l1 (world)	{  
    shape:ssBox, Q:<t(4.075 1 0.05)>, size:[0.15 1 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence3l2 (world)	{  
    shape:ssBox, Q:<t(4.075 2 0.05)>, size:[0.15 1 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence3l3 (world)	{  
    shape:ssBox, Q:<t(4.075 3 0.05)>, size:[0.15 1 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence4r1 (world)	{  
    shape:ssBox, Q:<t(-3.075 -1.2 0.05)>, size:[0.15 0.6 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence4r2 (world)	{  
    shape:ssBox, Q:<t(-3.075 -2 0.05)>, size:[0.15 1 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence4r3 (world)	{  
    shape:ssBox, Q:<t(-3.075 -3 0.05)>, size:[0.15 1 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence4l1 (world)	{  
    shape:ssBox, Q:<t(-3.075 1.2 0.05)>, size:[0.15 0.6 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

fence4l2 (world)	{  
    shape:ssBox, Q:<t(-3.075 2 0.05)>, size:[0.15 1 0.05.02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fence4l3 (world)	{  
    shape:ssBox, Q:<t(-3.075 3 0.05)>, size:[0.15 1 0.05.02], color:[1 1 1]
    contact, logical:{ }, mass = 100
}

# Corners stones
fencecl1 (world)	{  
    shape:ssBox, Q:<t(4.075 -3.575 0.05)>, size:[0.15 0.15 0.25 .02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fencecl2 (world)	{  
    shape:ssBox, Q:<t(4.075 3.575 0.05)>, size:[0.15 0.15 0.25 .02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}


fencecl3 (world)	{  
    shape:ssBox, Q:<t(-3.075 -3.575 0.05)>, size:[0.15 0.15 0.25 .02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}

fencecl4 (world)	{  
    shape:ssBox, Q:<t(-3.075 3.575 0.05)>, size:[0.15 0.15 0.25 .02], color:[0 0 0]
    contact, logical:{ }, mass = 100
}


# Import the goal mesh
goal (world) {
    X:< t(-3.2 0.055 0) d(90 0 0 1) d(90 1 0 0)>,
    mesh:'meshes/goal/goal.obj'
    #reduce the size of goal mesh
    meshscale: 1 
}
    
Prefix: "Goalee_"
Include: 'panda_paddle.g'

Prefix: "Thrower1_"
Include: 'panda_moveGripper.g'

#MARK: Uncomment this for two throwers.
#Prefix: "Thrower2_"
#Include: 'panda_moveGripper.g'

Prefix!
        
Edit Goalee_panda_link0 (world) { Q:<t(-2.5 0 0) d(0 0 0 0)> }
Edit Thrower1_panda_link0 (world)  { Q:<t( 2.6 0 0) d(0 0 0 1)> }
#MARK: Uncomment this for two throwers.
#Edit Thrower2_panda_link0 (world)  { Q:<t( 2.6 1 0) d(0 0 0 1)> }

camera(world){
    Q:<t(0 0 5) d(90 1 0 0)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}
