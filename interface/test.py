import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt

def csv_to_graph(csv_file):
    # Lecture du fichier CSV dans un DataFrame pandas
    df = pd.read_csv(csv_file)

    # Création d'un graphe dirigé
    G = nx.DiGraph()

    # Ajout des arêtes avec les attributs de direction
    for _, row in df.iterrows():
        start_point = row['Origine']
        direction = row['Direction']
        end_point = row['Destination']

        # Ajout de l'arête avec l'attribut de direction
        G.add_edge(start_point, end_point, direction=direction, weight=1)  # poids unitaire

    return G

# Nom du fichier CSV contenant les données
csv_file = 'topo_gros_cacouna.csv'

# Appel de la fonction pour créer le graphe à partir du fichier CSV
graph = csv_to_graph(csv_file)

# Calcul du plus court chemin depuis le nœud source jusqu'au nœud cible
source_node = 'A1'  
target_node = 'ZC1' 

shortest_path = nx.shortest_path(graph, source=source_node, target=target_node)

print(f"Le plus court chemin de '{source_node}' à '{target_node}' est:", shortest_path)

# Tracé du graphe avec l'algorithme kamada_kawai_layout
pos = nx.kamada_kawai_layout(graph)

# Dessin du graphe
nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')

# Ajout des étiquettes de direction sur toutes les arêtes
edge_labels = {(n1, n2): d['direction'] for n1, n2, d in graph.edges(data=True)}
nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_color='black')

# Ajout des étiquettes aux boucles
for node in graph.nodes():
    if graph.has_edge(node, node):
        direction = graph[node][node]['direction']
        plt.text(pos[node][0], pos[node][1] + 0.215, f"{direction}", fontsize=10, color='black', ha='center')

# Affichage du graphe
plt.show()
