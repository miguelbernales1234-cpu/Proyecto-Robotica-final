import os

grid = [
    list("###########"),
    list("#S        #"),
    list("#         #"),
    list("#   ###   #"),
    list("#   ###   #"),
    list("#   ###   #"),
    list("#   ###   #"),
    list("#   ###   #"),
    list("#         #"),
    list("#        E#"),
    list("###########")
]

cell_size = 0.25
wall_height = 0.1

nodes = []
nodes.append('#VRML_SIM R2025a utf8\n')
nodes.append('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"\n')
nodes.append('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"\n')
nodes.append('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"\n')
nodes.append('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"\n')

rows = len(grid)
cols = len(grid[0])
arena_w = cols * cell_size
arena_h = rows * cell_size

nodes.append(f"""
WorldInfo {{
}}
Viewpoint {{
  orientation -0.5773 0.5773 0.5773 2.0944
  position 0 0 {max(arena_w, arena_h) * 1.5}
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
RectangleArena {{
  floorSize {arena_w + 0.2} {arena_h + 0.2}
}}
""")

offset_x = -cols * cell_size / 2.0 + cell_size / 2.0
offset_y = -rows * cell_size / 2.0 + cell_size / 2.0

for r in range(rows):
    for c in range(cols):
        char = grid[r][c]
        
        x = offset_x + c * cell_size
        y = - (offset_y + r * cell_size)
        
        if char == '#':
            wall = f"""
Solid {{
  translation {x:.3f} {y:.3f} {wall_height/2}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.8 0.2 0.2
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {cell_size:.3f} {cell_size:.3f} {wall_height}
      }}
    }}
  ]
  boundingObject Box {{
    size {cell_size:.3f} {cell_size:.3f} {wall_height}
  }}
}}
"""
            nodes.append(wall)
        elif char == 'S':
            epuck = f"""
E-puck {{
  translation {x:.3f} {y:.3f} 0.05
  controller "controlador"
}}
"""
            nodes.append(epuck)
        elif char == 'E':
            goal = f"""
Solid {{
  translation {x:.3f} {y:.3f} 0.01
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.2 0.8 0.2
        roughness 1
        metalness 0
      }}
      geometry Cylinder {{
        radius {cell_size*0.4:.3f}
        height 0.02
      }}
    }}
  ]
  name "goal"
}}
"""
            nodes.append(goal)

wbt_path = os.path.join(os.path.dirname(__file__), "simple.wbt")
with open(wbt_path, "w", encoding="utf-8") as f:
    f.write("".join(nodes))

# Export the grid for the controller
grid_str = "\n".join(["".join(row) for row in grid])
map_path = os.path.join(os.path.dirname(__file__), "..", "controllers", "controlador", "maze_map_simple.txt")
os.makedirs(os.path.dirname(map_path), exist_ok=True)
with open(map_path, "w", encoding="utf-8") as f:
    f.write(grid_str)

print("Simple maze generated successfully and map exported.")
