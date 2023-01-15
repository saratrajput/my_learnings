import cadquery as cq
from cadquery import exporters


def make_box_model(height, width, depth, thickness):
    # Create a box using the parameters
    box = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))

    # Thin the walls of the box to create a thickness of 2.0
    box = box.faces(">Z").edges().chamfer(thickness)

    # Set the unit as mm.
    box = box.set_units("mm")

    # Export the model to an STL file.
    exporters.exportShape(box, "box_model.stl")

    return "box_model.stl"


# Test the function.
file_name = make_box_model(100, 200, 50, 2)
print(f"3D model of box saved as {file_name}.")
