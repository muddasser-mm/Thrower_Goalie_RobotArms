world { }

goal (world) {
    X:< t(-2.7 0.055 0) d(90 0 0 1) d(90 1 0 0)>,
    #mesh:'/home/muddasser/git/robotics-course/scenarios/meshes/goal/open_goal.obj'
    mesh:'/home/muddasser/git/robotics-course/scenarios/meshes/goal/goal.obj'
    meshscale: 0.8 #make it a bit smaller
}

fence1 (world)	{  
    shape:ssBox, Q:<t(0.6 -2.5 0.3)>, size:[6.5 0.15 0.8 .02], color:[1 1 1]
    contact, logical:{ }, mass = 100
    #friction:.1
}

#fencetest (world)	{  
#    X:< t(0.6 -2.5 -0.01) d(90 1 0 0) >,
#    mesh:'/home/muddasser/git/robotics-course/scenarios/meshes/goal/fence.obj'
#    #friction:.1
#}

fence2 (world)	{  
    shape:ssBox, Q:<t(0.6 2.5 0.3)>, size:[6.5 0.15 0.8 .02], color:[1 1 1]
    contact, logical:{ }, mass = 100
    #friction:.1
}

fence3 (world)	{  
    shape:ssBox, Q:<t(3.91 0 0.3)>, size:[0.15 5.2 0.8 .02], color:[1 1 1]
    contact, logical:{ }, mass = 100
    #friction:.1
}

fence4a (world)	{  
    shape:ssBox, Q:<t(-2.71 1.68 0.3)>, size:[0.15 1.8 0.8 .02], color:[1 1 1]
    contact, logical:{ }, mass = 60
    #friction:.1
}

fence4b (world)	{  
    shape:ssBox, Q:<t(-2.71 -1.68 0.3)>, size:[0.15 1.8 0.8 .02], color:[1 1 1]
    contact, logical:{ }, mass = 60
    #friction:.1
}
    
Prefix: "Goalee_"
Include: 'panda_paddle.g'

Prefix: "Thrower1_"
Include: 'panda_moveGripper.g'

#Prefix: "Thrower2_"
#Include: 'panda_moveGripper.g'

Prefix!
        
Edit Goalee_panda_link0 (world) { Q:<t(-2 0 0) d(0 0 0 0)> }
Edit Thrower1_panda_link0 (world)  { Q:<t( 2.6 0 0) d(180 0 0 1)> }

#camera(world){
#    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
#    shape:marker, size:[.1],
#    focalLength:0.895, width:640, height:360, zRange:[.5 100]
#}
