# A* en Python — 10 ciudades

Este proyecto implementa el **Problema del Agente Viajero (TSP)** usando el algoritmo **A\*** en Python, encontrando la ruta de menor costo que:
1) inicia en una ciudad origen,  
2) visita **las 10 ciudades exactamente una vez**, y  
3) regresa al origen.

La implementación usa:
- **Estados**: (ciudad_actual, conjunto_de_visitadas)
- **Frontera**: cola de prioridad (`heapq`)
- **Heurística admisible**: combinación de conexión mínima + **MST** (árbol de expansión mínima) + retorno mínimo al origen.

---

## Requisitos

- Python **3.9+** (recomendado)
- (Opcional) `matplotlib` para visualizar el recorrido

---

## Instalación

1. Coloca el script en una carpeta, por ejemplo:

```
a_estrella.py
README.md
```

2. (Opcional) Crea y activa un entorno virtual:

```bash
python -m venv .venv
# Windows:
# .venv\Scripts\activate
# macOS/Linux:
source .venv/bin/activate
```

3. (Opcional) Instala dependencias para la gráfica:

```bash
pip install matplotlib
```

> Si no instalas `matplotlib`, el programa funcionará igual y solo omitirá la visualización.

---

## Ejecución

Desde la terminal:

```bash
python tsp_astar.py
```

Salida esperada (ejemplo):
- Ciudad origen: `A`
- Ruta: `A -> ... -> A`
- Costo total (distancia): `XX.XXXX`

---

## Estructura del algoritmo

### Representación del estado (nodo)
Cada estado es una tupla:
- `current_city` (int): índice de la ciudad actual (0..9)
- `visited_mask` (int): máscara de bits con las ciudades visitadas

### Función de costo
- `g(n)`: costo real acumulado
- `f(n) = g(n) + h(n)`

### Heurística admisible (h)
Si faltan ciudades por visitar (U = no visitadas):
```
h = min(current -> U) + MST(U) + min(U -> start)
```

Si ya visitó todas:
```
h = dist(current, start)
```

---

## Personalización

### Cambiar coordenadas de ciudades
Dentro de `if __name__ == "__main__":`, modifica `coords` (manteniendo n=10).

### Cambiar la ciudad origen
Modifica `start_city`.

### Cambiar nombres de ciudades
Edita `city_names`.

---

## Visualización (opcional)
Si `matplotlib` está instalado, se mostrará una gráfica del tour.

---

## Notas de rendimiento
El espacio de estados crece como `O(n * 2^n)`. Para 10 ciudades, A\* suele ser viable con esta heurística.

---

## Licencia
Uso académico/educativo. Puedes adaptar el código libremente.
