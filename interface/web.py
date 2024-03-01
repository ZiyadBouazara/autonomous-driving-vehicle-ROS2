# Import des bibliothèques nécessaires
from flask import Flask, render_template, request, redirect, url_for
from werkzeug.utils import secure_filename
import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import io
import base64

# Initialisation de l'application Flask
app = Flask(__name__)

UPLOAD_FOLDER = 'uploads'
ALLOWED_EXTENSIONS = {'csv'}

app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER


# Fonction pour vérifier les extensions de fichier autorisées
def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


# Fonction pour créer le graphe à partir du fichier CSV
def csv_to_graph(csv_file):
    df = pd.read_csv(csv_file)

    G = nx.DiGraph()

    dic_edge_labels = {}

    for _, row in df.iterrows():
        start_point = row['Origine']
        direction = row['Direction']
        end_point = row['Destination']

        dic_edge_labels[(start_point, end_point)] = direction

        G.add_edge(start_point, end_point, edge_label=direction, weight=1)

    return G, dic_edge_labels


@app.route('/', methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        # Vérification si le fichier a été téléversé correctement
        if 'file' not in request.files:
            return redirect(request.url)
        file = request.files['file']
        if file.filename == '':
            return redirect(request.url)
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
            return redirect(url_for('transform_graph',
                                    filename=filename))
    return render_template('upload.html')


@app.route('/transform/<filename>', methods=['GET', 'POST'])
def transform_graph(filename):
    csv_file = os.path.join(app.config['UPLOAD_FOLDER'], filename)
    graph, dic_edge_labels = csv_to_graph(csv_file)

    # Génération de l'image du graphe dès l'accès à la page
    pos = nx.spring_layout(graph)
    plt.figure(figsize=(8, 6))
    nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')

    nx.draw_networkx_edge_labels(graph, pos, edge_labels=dic_edge_labels, font_color='red')

    plt.title('Graph')
    # Conversion de l'image en base64
    img = io.BytesIO()
    plt.savefig(img, format='png')
    img.seek(0)
    graph_img = base64.b64encode(img.getvalue()).decode()
    plt.close()

    shortest_path = None
    error_message = None

    if request.method == 'POST':
        source_node = request.form['source']
        target_node = request.form['target']

        try:
            shortest_path = nx.shortest_path(graph, source=source_node, target=target_node)
        except nx.NetworkXNoPath:
            error_message = f"No path found between {source_node} and {target_node}."

    return render_template('result.html', shortest_path=shortest_path, graph_img=graph_img, dic_edge_labels=dic_edge_labels, error_message=error_message)


if __name__ == '__main__':
    app.run(debug=True)
