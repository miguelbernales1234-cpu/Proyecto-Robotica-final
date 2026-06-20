import random
import os

width = 6
height = 6

# Initialize grid
grid = [['#' for _ in range(width * 2 + 1)] for _ in range(height * 2 + 1)]

def carve_passages_from(cx, cy):
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    random.shuffle(directions)
    
    for dx, dy in directions:
        nx, ny = cx + dx, cy + dy
        if 0 <= nx < width and 0 <= ny < height and grid[ny * 2 + 1][nx * 2 + 1] == '#':
            grid[cy * 2 + 1 + dy][cx * 2 + 1 + dx] = ' '
            grid[ny * 2 + 1][nx * 2 + 1] = ' '
            carve_passages_from(nx, ny)

grid[1][1] = ' '
carve_passages_from(0, 0)

# Set start and end
# Bottom left
grid[height * 2 - 1][1] = 'S'
# Top right
grid[1][width * 2 - 1] = 'E'

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

wbt_path = os.path.join(os.path.dirname(__file__), "complejo.wbt")
with open(wbt_path, "w") as f:
    f.write("".join(nodes))

# Also export the grid for the controller
grid_str = "\n".join(["".join(row) for row in grid])
map_path = os.path.join(os.path.dirname(__file__), "..", "controllers", "controlador", "maze_map.txt")
os.makedirs(os.path.dirname(map_path), exist_ok=True)
with open(map_path, "w") as f:
    f.write(grid_str)

print("Maze generated successfully and map exported.")
