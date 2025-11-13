$fn=21;
r = 20.1275367222;
ball_radius = 4.6/2; //6.05/2; //4.45/2;
h = ball_radius * 2 * (1 + .25 + .2 );
echo(h);
n_teeth=21;
link_radius=.5;
cover_height=2;
shaft_radius = 12/2 + 0.25; // 12mm diameter D-shaft
d_shaft_flat_depth = 0.5; // Adjust this value to match your motor shaft's flat depth

module make_sphere_ring(ring_radius, sphere_radius, num_spheres) {
  union(){
    for (i = [0:num_spheres]){
      rotate([0,0,i*360/num_spheres])
        translate([ring_radius,0,0])hull() {
          sphere(sphere_radius);
          translate([-.5 * sphere_radius,0,0])sphere(sphere_radius);
          translate([0,0,.25 * sphere_radius])sphere(sphere_radius);
          translate([0,0,-.25 * sphere_radius])sphere(sphere_radius);
        }
    }
  }
}

module make_torus(ring_radius, inner_radius) {
  // openscad apparently can't handle rotate_extrude in a difference, so this isn't a real cylinder
  /*rotate_extrude(convexity=10)translate([ring_radius, 0, 0])circle(inner_radius);*/
  difference() {
    cylinder (
        h = inner_radius*2,
        r = ring_radius,
        center = true);
    cylinder (
        h = inner_radius * 2,
        r = ring_radius - 2*inner_radius,
        center = true);
  }
}

module bead_ring() {
  union() {
    make_sphere_ring(r, ball_radius, n_teeth);
    make_torus(r, link_radius);
  }
}

module d_shaft_hole(shaft_r, flat_depth, shaft_height) {
  difference() {
    cylinder(r=shaft_r, h=shaft_height, center=true);
    translate([shaft_r - flat_depth, 0, 0])
      cube([shaft_r, shaft_r * 2, shaft_height + 1], center=true);
  }
}

module make_gear() {
  difference() {
    union() {
      cylinder(h=h, r=r, center=true);
      translate([0,0,-.9*(h-cover_height)])
        cylinder(h=cover_height, r=r+ball_radius, center=true);
      translate([0,0,.9*(h-cover_height)])
        cylinder(h=cover_height, r=r+ball_radius, center=true);
    }
    /*cylinder(h=h, r=r, center=true);*/
    # d_shaft_hole(shaft_radius, d_shaft_flat_depth, h + 4 * cover_height);
    # bead_ring();
  }
}

make_gear();
/*bead_ring() ;*/
/*make_torus(r, link_radius);*/