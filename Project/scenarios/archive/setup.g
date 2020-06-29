world {}

Prefix: "Goalee_"
Include: 'panda_paddle.g'

Prefix: "Thrower1_"
Include: 'panda_moveGripper.g'

#Prefix: "Thrower2_"
#Include: 'panda_moveGripper.g'

Prefix!
        
Edit Goalee_panda_link0 (world) { Q:<t(-4 0 0) d(0 0 0 0)> }
Edit Thrower1_panda_link0 (world)  { Q:<t( 4 0 0) d(180 0 0 1)> }

#camera(world){
#    Q:<t(-0.01 -.2 1.8) d(30 1 0 0)>,
#    shape:marker, size:[.1],
#    focalLength:0.895, width:640, height:360, zRange:[.5 100]
#}
