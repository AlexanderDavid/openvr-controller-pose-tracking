// All measurements are in imperial :(

hole_spacing = 12/16;
hole_diameter = 3/16;
y_hole_num = 11;
x_hole_num = 7;
base_width = 4.5;
base_length = 7.5;
base_height = 0.25;
rim_size = 0;

// Turtlebot Top Board-ish
//color([51 / 255, 51 / 255, 51 / 255]) difference()
//{
//    cube(size = [base_width + rim_size * 2, base_length + rim_size * 2, base_height]);
//
//    union()
//    {
//        for (x = [0: x_hole_num - 1])
//            for (y = [0: y_hole_num - 1])
//                translate([x * hole_spacing + rim_size,
//                           y * hole_spacing + rim_size, -1])
//                    cylinder(base_height + 2, hole_diameter / 2, hole_diameter / 2, $fn=100);
//    }
//}
union()
{
difference()
{
color("blue") translate([hole_diameter, hole_spacing + hole_diameter - hole_spacing / 2, base_height]) difference()
{
    cube(size = [2 + 7/16 + hole_spacing, 2 + 5/8 + hole_spacing, 0.25]);
    
    
    translate([-hole_diameter, -hole_spacing - hole_diameter + hole_spacing / 2, -base_height]) union()
    {
        for (x = [1: 3: x_hole_num - 1])
            for (y = [1: 4: y_hole_num - 1])
                translate([x * hole_spacing + rim_size,
                           y * hole_spacing + rim_size, -1])
                    cylinder(base_height + 2, hole_diameter / 2, hole_diameter / 2, $fn=100);
    }
}

color("purple") translate([hole_diameter, hole_spacing + hole_diameter, -0.25]) union()
{
    translate([0.75, 5/16, -0.125]) cylinder(1.5, 3/32, 3/32, $fn=100);
    translate([0.75, 2 + 5/16, -0.125]) cylinder(1.5, 3/32, 3/32, $fn=100);
    translate([1.75 + 5/16, 5/16, -0.125]) cylinder(1.5, 3/32, 3/32, $fn=100);
    translate([1.75 + 5/16, 2 + 5/16, -0.125]) cylinder(1.5, 3/32, 3/32, $fn=100);
}
}

color("white") translate([hole_diameter, hole_spacing + hole_diameter, base_height * 2]) difference()
{
    cube(size = [2 + 7/16, 2 + 5/8, 0.25]);

    translate([0.75, 5/16, -0.125]) cylinder(0.5, 3/32, 3/32, $fn=100);
    translate([0.75, 2 + 5/16, -0.125]) cylinder(0.5, 3/32, 3/32, $fn=100);
    translate([1.75 + 5/16, 5/16, -0.125]) cylinder(0.5, 3/32, 3/32, $fn=100);
    translate([1.75 + 5/16, 2 + 5/16, -0.125]) cylinder(0.5, 3/32, 3/32, $fn=100);
}
}