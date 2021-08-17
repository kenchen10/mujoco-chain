union() {
    
    translate([7.3 + 2.9,0,0]) 
        cylinder(h=64.3 - 26.2, r=2.9, center=true);
    translate([-7.3 - 2.9   ,0,0]) 
        cylinder(h=64.3 - 26.2, r=2.9, center=true);

    translate([0,0,32.15 - 2.9 - 10.2]) 
    rotate(a=90,v=[1,0, 0]) 
    rotate_extrude(angle=90, convexity=10)
        translate([10.2, 0]) circle(2.9);
    
    translate([0,0,32.15 - 2.9 - 10.2]) 
    rotate(a=270,v=[1,0, 0]) 
    rotate_extrude(angle=90, convexity=10)
        translate([-10.2, 0]) circle(2.9);
    
    translate([0,0,-32.15 + 2.9 + 10.2]) 
    rotate(a=90,v=[1,0, 0]) 
    rotate_extrude(angle=90, convexity=10)
        translate([-10.2, 0]) circle(2.9);
    
    translate([0,0,-32.15 + 2.9 + 10.2]) 
    rotate(a=270,v=[1,0, 0]) 
    rotate_extrude(angle=90, convexity=10)
        translate([10.2, 0]) circle(2.9);
}
