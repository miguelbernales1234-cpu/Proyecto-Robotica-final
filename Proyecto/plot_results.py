import os
import csv

def generate_svg(map_path, csv_path, cell_pixels=32):
    if not os.path.exists(map_path):
        return f'<div class="error-msg">No se encontró el mapa en: {os.path.basename(map_path)}</div>'
    
    with open(map_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    grid = [list(line.strip()) for line in lines if line.strip()]
    
    rows = len(grid)
    cols = len(grid[0])
    
    CELL = 0.25
    off_x = -cols * CELL / 2.0 + CELL / 2.0
    off_y = -rows * CELL / 2.0 + CELL / 2.0
    
    svg_w = cols * cell_pixels
    svg_h = rows * cell_pixels
    
    elements = []
    
    # 1. Dibujar celdas de la grilla (paredes y pasillos)
    for r in range(rows):
        for c in range(cols):
            char = grid[r][c]
            x = c * cell_pixels
            y = r * cell_pixels
            
            if char == '#':
                # Paredes (rojo oscuro elegante)
                elements.append(f'  <rect x="{x}" y="{y}" width="{cell_pixels}" height="{cell_pixels}" fill="#ef4444" fill-opacity="0.15" stroke="#ef4444" stroke-width="1.5" stroke-opacity="0.4" rx="2" />')
            else:
                # Pasillos
                elements.append(f'  <rect x="{x}" y="{y}" width="{cell_pixels}" height="{cell_pixels}" fill="#0f172a" stroke="#1e293b" stroke-width="0.5" />')
                
                # Indicadores de inicio y fin dentro del laberinto
                if char == 'S':
                    elements.append(f'  <circle cx="{x + cell_pixels/2}" cy="{y + cell_pixels/2}" r="{cell_pixels*0.3}" fill="#3b82f6" fill-opacity="0.2" stroke="#3b82f6" stroke-width="2" />')
                    elements.append(f'  <circle cx="{x + cell_pixels/2}" cy="{y + cell_pixels/2}" r="3" fill="#3b82f6" />')
                elif char == 'E':
                    elements.append(f'  <circle cx="{x + cell_pixels/2}" cy="{y + cell_pixels/2}" r="{cell_pixels*0.35}" fill="#10b981" fill-opacity="0.2" stroke="#10b981" stroke-width="2" />')
                    elements.append(f'  <circle cx="{x + cell_pixels/2}" cy="{y + cell_pixels/2}" r="4" fill="#10b981" />')

    # 2. Leer trayectorias reales y planificadas
    executed_points = []
    planned_points = []
    
    if os.path.exists(csv_path):
        try:
            with open(csv_path, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                last_tx, last_ty = None, None
                
                for i, row in enumerate(reader):
                    rx = float(row["x_m"])
                    ry = float(row["y_m"])
                    tx = float(row["target_x_m"])
                    ty = float(row["target_y_m"])
                    
                    # Convertir pose real a coordenadas SVG
                    c_val = (rx - off_x) / CELL
                    r_val = (-ry - off_y) / CELL
                    px = c_val * cell_pixels + cell_pixels / 2.0
                    py = r_val * cell_pixels + cell_pixels / 2.0
                    executed_points.append(f"{px:.1f},{py:.1f}")
                    
                    # Si es el primer step, añadir la pose inicial como primer punto de ruta
                    if i == 0:
                        planned_points.append(f"{px:.1f},{py:.1f}")
                    
                    # Registrar waypoints únicos de la ruta planificada
                    if (tx, ty) != (last_tx, last_ty):
                        tc_val = (tx - off_x) / CELL
                        tr_val = (-ty - off_y) / CELL
                        tpx = tc_val * cell_pixels + cell_pixels / 2.0
                        tpy = tr_val * cell_pixels + cell_pixels / 2.0
                        planned_points.append(f"{tpx:.1f},{tpy:.1f}")
                        last_tx, last_ty = tx, ty
                        
        except Exception as e:
            print(f"Error procesando CSV {csv_path}: {e}")

    # 3. Dibujar la ruta planificada (línea punteada verde esmeralda)
    if len(planned_points) > 1:
        pts_str = " ".join(planned_points)
        elements.append(f'  <polyline points="{pts_str}" fill="none" stroke="#10b981" stroke-width="2" stroke-dasharray="6,4" stroke-linecap="round" stroke-linejoin="round" />')
        
        # Puntos clave de waypoints
        for pt in planned_points:
            x_coord, y_coord = map(float, pt.split(","))
            elements.append(f'  <circle cx="{x_coord}" cy="{y_coord}" r="3" fill="#0f172a" stroke="#10b981" stroke-width="1.5" />')

    # 4. Dibujar la trayectoria ejecutada real (línea sólida ámbar)
    if len(executed_points) > 1:
        pts_str = " ".join(executed_points)
        elements.append(f'  <polyline points="{pts_str}" fill="none" stroke="#f59e0b" stroke-width="3.5" stroke-linecap="round" stroke-linejoin="round" />')
        
        # Posición final del robot (triángulo o flecha indicando orientación)
        # Tomamos el último punto
        last_pt = executed_points[-1]
        lx, ly = map(float, last_pt.split(","))
        elements.append(f'  <circle cx="{lx}" cy="{ly}" r="5" fill="#f59e0b" stroke="#ffffff" stroke-width="1.5" />')

    svg_content = f"""
    <svg width="100%" height="auto" viewBox="0 0 {svg_w} {svg_h}" style="background-color: #0b0f19; border-radius: 12px; box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.3);">
        <defs>
            <pattern id="grid" width="{cell_pixels}" height="{cell_pixels}" patternUnits="userSpaceOnUse">
                <rect width="{cell_pixels}" height="{cell_pixels}" fill="none" stroke="#1e293b" stroke-width="0.2" />
            </pattern>
        </defs>
        <rect width="{svg_w}" height="{svg_h}" fill="url(#grid)" />
        {os.linesep.join(elements)}
    </svg>
    """
    return svg_content

def load_summary(summary_path):
    if not os.path.exists(summary_path):
        return "<p class='error-msg'>Aún no hay datos de resumen. Ejecuta la simulación primero.</p>"
    
    html = []
    with open(summary_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
        
    for line in lines:
        if line.startswith("==="):
            continue
        parts = line.split(":")
        if len(parts) == 2:
            name = parts[0].strip()
            val = parts[1].strip()
            html.append(f"""
            <div class="metric-item">
                <span class="metric-name">{name}</span>
                <span class="metric-value">{val}</span>
            </div>
            """)
    return "".join(html)

def main():
    root_dir = os.path.dirname(__file__)
    controller_dir = os.path.join(root_dir, "controllers", "controlador")
    
    # Rutas
    simple_map = os.path.join(controller_dir, "maze_map_simple.txt")
    simple_csv = os.path.join(controller_dir, "trajectory_log_simple.csv")
    simple_summary = os.path.join(controller_dir, "summary_simple.txt")
    
    complex_map = os.path.join(controller_dir, "maze_map_complejo.txt")
    complex_csv = os.path.join(controller_dir, "trajectory_log_complejo.csv")
    complex_summary = os.path.join(controller_dir, "summary_complejo.txt")
    
    # Generar SVGs
    svg_simple = generate_svg(simple_map, simple_csv)
    svg_complex = generate_svg(complex_map, complex_csv)
    
    # Generar Resúmenes
    summary_html_simple = load_summary(simple_summary)
    summary_html_complex = load_summary(complex_summary)
    
    # HTML template
    html_content = f"""<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Visualización de Trayectorias - Proyecto Webots</title>
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700&display=swap" rel="stylesheet">
    <style>
        :root {{
            --bg-color: #0b0f19;
            --card-bg: #111827;
            --border-color: #1f2937;
            --text-main: #f3f4f6;
            --text-muted: #9ca3af;
            --accent-green: #10b981;
            --accent-amber: #f59e0b;
            --accent-blue: #3b82f6;
        }}
        
        * {{
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }}
        
        body {{
            background-color: var(--bg-color);
            color: var(--text-main);
            font-family: 'Outfit', sans-serif;
            line-height: 1.6;
            padding: 2rem;
            max-width: 1200px;
            margin: 0 auto;
        }}
        
        header {{
            text-align: center;
            margin-bottom: 3rem;
        }}
        
        h1 {{
            font-size: 2.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, #60a5fa, #34d399);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 0.5rem;
        }}
        
        header p {{
            color: var(--text-muted);
            font-size: 1.1rem;
        }}
        
        .grid-container {{
            display: grid;
            grid-template-columns: 1fr;
            gap: 2.5rem;
        }}
        
        @media (min-width: 900px) {{
            .grid-container {{
                grid-template-columns: 1fr 1fr;
            }}
        }}
        
        .card {{
            background-color: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 16px;
            padding: 1.5rem;
            display: flex;
            flex-direction: column;
            gap: 1.5rem;
            transition: transform 0.2s ease, box-shadow 0.2s ease;
        }}
        
        .card:hover {{
            transform: translateY(-4px);
            box-shadow: 0 15px 30px -10px rgba(0, 0, 0, 0.4);
        }}
        
        .card-header {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid var(--border-color);
            padding-bottom: 0.75rem;
        }}
        
        .card-title {{
            font-size: 1.5rem;
            font-weight: 600;
        }}
        
        .tag {{
            font-size: 0.8rem;
            font-weight: 600;
            text-transform: uppercase;
            padding: 0.25rem 0.75rem;
            border-radius: 20px;
            background-opacity: 0.1;
        }}
        
        .tag-simple {{
            color: var(--accent-blue);
            background-color: rgba(59, 130, 246, 0.15);
            border: 1px solid rgba(59, 130, 246, 0.3);
        }}
        
        .tag-complex {{
            color: #ec4899;
            background-color: rgba(236, 72, 153, 0.15);
            border: 1px solid rgba(236, 72, 153, 0.3);
        }}
        
        .visualization {{
            width: 100%;
            display: flex;
            justify-content: center;
        }}
        
        .metrics {{
            display: flex;
            flex-direction: column;
            gap: 0.75rem;
            background-color: rgba(0, 0, 0, 0.2);
            padding: 1rem;
            border-radius: 12px;
            border: 1px solid rgba(255, 255, 255, 0.05);
        }}
        
        .metric-item {{
            display: flex;
            justify-content: space-between;
            font-size: 0.95rem;
        }}
        
        .metric-name {{
            color: var(--text-muted);
        }}
        
        .metric-value {{
            font-weight: 600;
            color: var(--text-main);
        }}
        
        .legend {{
            display: flex;
            gap: 1.5rem;
            justify-content: center;
            margin-bottom: 2rem;
            background: rgba(255, 255, 255, 0.02);
            padding: 1rem;
            border-radius: 12px;
            border: 1px solid var(--border-color);
        }}
        
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 0.5rem;
            font-size: 0.9rem;
        }}
        
        .legend-line {{
            width: 24px;
            height: 4px;
            border-radius: 2px;
        }}
        
        .legend-dashed {{
            border-bottom: 2.5px dashed var(--accent-green);
            background: none;
            height: 0;
        }}
        
        .legend-solid {{
            background-color: var(--accent-amber);
        }}
        
        .error-msg {{
            color: #f87171;
            font-size: 0.9rem;
            font-style: italic;
            text-align: center;
            width: 100%;
            padding: 2rem;
            border: 1px dashed rgba(248, 113, 113, 0.2);
            border-radius: 8px;
            background-color: rgba(248, 113, 113, 0.02);
        }}
        
        footer {{
            text-align: center;
            margin-top: 4rem;
            color: var(--text-muted);
            font-size: 0.9rem;
            border-top: 1px solid var(--border-color);
            padding-top: 2rem;
        }}
    </style>
</head>
<body>

    <header>
        <h1>Análisis de Trayectorias y Odometría</h1>
        <p>Proyecto Final de Robótica y Sistemas Autónomos (ICI 4150)</p>
    </header>
    
    <div class="legend">
        <div class="legend-item">
            <span class="legend-line legend-dashed"></span>
            <span>Ruta Planificada (A* Simplificada)</span>
        </div>
        <div class="legend-item">
            <span class="legend-line legend-solid"></span>
            <span>Trayectoria Real Recorrida (Odometría)</span>
        </div>
    </div>
    
    <div class="grid-container">
        
        <!-- Tarjeta Escenario Simple -->
        <div class="card" id="card-simple">
            <div class="card-header">
                <span class="card-title">Escenario Simple</span>
                <span class="tag tag-simple">Simple</span>
            </div>
            
            <div class="visualization">
                {svg_simple}
            </div>
            
            <div class="metrics">
                <h3 style="font-size: 1.1rem; font-weight: 600; margin-bottom: 0.25rem;">Resultados Obtenidos</h3>
                {summary_html_simple}
            </div>
        </div>
        
        <!-- Tarjeta Escenario Complejo -->
        <div class="card" id="card-complex">
            <div class="card-header">
                <span class="card-title">Escenario Complejo</span>
                <span class="tag tag-complex">Complejo</span>
            </div>
            
            <div class="visualization">
                {svg_complex}
            </div>
            
            <div class="metrics">
                <h3 style="font-size: 1.1rem; font-weight: 600; margin-bottom: 0.25rem;">Resultados Obtenidos</h3>
                {summary_html_complex}
            </div>
        </div>
        
    </div>
    
    <footer>
        <p>Generado automáticamente a partir de logs de odometría de Webots.</p>
    </footer>

</body>
</html>
"""
    output_html_path = os.path.join(root_dir, "trajectory_plots.html")
    with open(output_html_path, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"Página de gráficos generada exitosamente en: {output_html_path}")

if __name__ == "__main__":
    main()
