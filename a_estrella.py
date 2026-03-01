
import math
import heapq
from typing import Dict, List, Tuple, Optional

try:
	import matplotlib.pyplot as plt
	HAS_MPL = True
except ImportError:
	HAS_MPL = False


City = int
State = Tuple[City, int]  # (current_city, visited_mask)


def euclidean(a: Tuple[float, float], b: Tuple[float, float]) -> float:
	return math.hypot(a[0] - b[0], a[1] - b[1])


def build_distance_matrix(coords: List[Tuple[float, float]]) -> List[List[float]]:
	n = len(coords)
	dist = [[0.0] * n for _ in range(n)]
	for i in range(n):
		for j in range(n):
			dist[i][j] = euclidean(coords[i], coords[j])
	return dist


def mst_cost(nodes: List[int], dist: List[List[float]]) -> float:
	"""
	Costo del Árbol de Expansión Mínima (MST) sobre 'nodes' (Prim).
	Si nodes tiene 0 o 1 nodo, el costo es 0.
	"""
	if len(nodes) <= 1:
		return 0.0

	in_mst = set()
	start = nodes[0]
	in_mst.add(start)

	# min_edge[v] = menor costo para conectar v al MST actual
	min_edge: Dict[int, float] = {v: float("inf") for v in nodes}
	for v in nodes:
		min_edge[v] = dist[start][v]

	total = 0.0
	while len(in_mst) < len(nodes):
		# elegir el nodo fuera del MST con menor conexión
		candidate = None
		best = float("inf")
		for v in nodes:
			if v not in in_mst and min_edge[v] < best:
				best = min_edge[v]
				candidate = v

		# agregarlo al MST
		in_mst.add(candidate)
		total += best

		# actualizar conexiones mínimas
		for v in nodes:
			if v not in in_mst:
				min_edge[v] = min(min_edge[v], dist[candidate][v])

	return total


def heuristic_admissible(current: int, visited_mask: int, start: int,
                         dist: List[List[float]], n: int) -> float:
	"""
	Heurística admisible para TSP (distancias métricas):
	h = (min current->unvisited) + MST(unvisited) + (min unvisited->start)

	Intuición: cualquier tour que complete debe:
	- salir de current hacia alguna ciudad no visitada (si faltan)
	- conectar todas las no visitadas (al menos un MST)
	- volver finalmente al start desde alguna no visitada

	Si ya visitó todas, h = dist(current, start) para cerrar el ciclo.
	"""
	all_visited = (1 << n) - 1
	if visited_mask == all_visited:
		return dist[current][start]

	unvisited = [i for i in range(n) if not (visited_mask & (1 << i))]

	min_from_current = min(dist[current][u] for u in unvisited)
	min_to_start = min(dist[u][start] for u in unvisited)
	connect_unvisited = mst_cost(unvisited, dist)

	return min_from_current + connect_unvisited + min_to_start


def astar_tsp(coords: List[Tuple[float, float]],
						city_names: Optional[List[str]] = None,
						start: int = 0) -> Tuple[float, List[int]]:
	"""
	Resuelve TSP (Hamiltoniano + regreso al origen) con A*.
	Retorna: (costo_optimo, ruta_completa_en_indices) donde la ruta incluye start al final.
	"""
	n = len(coords)
	if n != 10:
		raise ValueError("Este script está configurado para 10 ciudades (n=10).")

	dist = build_distance_matrix(coords)
	all_visited = (1 << n) - 1

	start_state: State = (start, 1 << start)

	# best_g guarda el mejor costo conocido para cada estado
	best_g: Dict[State, float] = {start_state: 0.0}

	# parent para reconstrucción
	parent: Dict[State, Optional[State]] = {start_state: None}

	# frontera = heap de (f, g, state)
	h0 = heuristic_admissible(start, 1 << start, start, dist, n)
	frontier = [(h0, 0.0, start_state)]

	def is_goal(state: State) -> bool:
		cur, mask = state
		return (cur == start) and (mask == all_visited)

	while frontier:
		f, g, state = heapq.heappop(frontier)
		cur, mask = state

		# entrada obsoleta
		if g != best_g.get(state, float("inf")):
			continue

		if is_goal(state):
			# reconstruir ruta
			route = []
			s = state
			while s is not None:
				route.append(s[0])
				s = parent[s]
			route.reverse()
			return g, route

		# generar sucesores
		if mask == all_visited:
			# ya visitó todas: solo falta regresar al inicio (si no está ahí)
			if cur != start:
				nxt = start
				nxt_mask = mask
				step = dist[cur][nxt]
				nxt_state = (nxt, nxt_mask)
				new_g = g + step

				if new_g < best_g.get(nxt_state, float("inf")):
					best_g[nxt_state] = new_g
					parent[nxt_state] = state
					h = heuristic_admissible(nxt, nxt_mask, start, dist, n)
					heapq.heappush(frontier, (new_g + h, new_g, nxt_state))
		else:
			# aún faltan ciudades por visitar: ir a cualquier no visitada
			for nxt in range(n):
				if mask & (1 << nxt):
					continue  # ya visitada
				nxt_mask = mask | (1 << nxt)
				step = dist[cur][nxt]
				nxt_state = (nxt, nxt_mask)
				new_g = g + step

				if new_g < best_g.get(nxt_state, float("inf")):
					best_g[nxt_state] = new_g
					parent[nxt_state] = state
					h = heuristic_admissible(nxt, nxt_mask, start, dist, n)
					heapq.heappush(frontier, (new_g + h, new_g, nxt_state))

	raise RuntimeError("No se encontró solución (esto no debería ocurrir con grafo completo).")


def plot_route(coords: List[Tuple[float, float]], route: List[int], names: List[str]) -> None:
	if not HAS_MPL:
		print("matplotlib no está instalado; omitiendo gráfica.")
		return

	xs = [coords[i][0] for i in route]
	ys = [coords[i][1] for i in route]

	plt.figure()
	plt.plot(xs, ys, marker="o")
	for i, (x, y) in enumerate(coords):
		plt.text(x, y, f" {names[i]}", fontsize=10)

	plt.title("TSP con A* (ruta óptima)")
	plt.xlabel("X")
	plt.ylabel("Y")
	plt.axis("equal")
	plt.grid(True)
	plt.show()


if __name__ == "__main__":
	# 10 ciudades (puedes cambiar coords, pero mantén n=10)
	city_names = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]

	# Coordenadas fijas (distancias Euclidianas => métrica; útil para la heurística)
	coords = [
		(2, 3),   # A
		(5, 7),   # B
		(1, 9),   # C
		(8, 2),   # D
		(7, 9),   # E
		(3, 6),   # F
		(9, 6),   # G
		(4, 1),   # H
		(6, 4),   # I
		(2, 8),   # J
	]

	start_city = 0  # A
	cost, route = astar_tsp(coords, city_names=city_names, start=start_city)

	print("=== Resultado A* TSP (10 ciudades) ===")
	print(f"Ciudad origen: {city_names[start_city]}")
	print("Ruta (índices):", route)
	print("Ruta (nombres):", " -> ".join(city_names[i] for i in route))
	print(f"Costo total (distancia): {cost:.4f}")

	# Visualización (opcional)
	if HAS_MPL:
		plot_route(coords, route, city_names)
