import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from datetime import datetime
import os
import glob

# Configuration du style
plt.style.use('seaborn')
sns.set_palette("husl")

def load_results():
    """Charge les résultats depuis le fichier CSV"""
    try:
        # Chercher le fichier CSV le plus récent
        csv_files = glob.glob('parameter_test_results_*.csv')
        if not csv_files:
            print("Erreur: Aucun fichier de résultats trouvé.")
            return None
        
        # Prendre le fichier le plus récent
        latest_file = max(csv_files, key=os.path.getctime)
        print(f"Chargement du fichier : {latest_file}")
        
        # Lire le fichier pour trouver où commencent les données
        with open(latest_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # Trouver la première ligne qui correspond au format attendu
        # Format: float,float,float,float,float,float,float,float,bool,bool,str,float
        start_idx = 0
        header = ['max_speed', 'max_steer', 'min_front_dist', 'safety_margin', 'front_angle', 
                 'side_angle', 'total_time', 'distance', 'collision', 'tour_complete', 'temps_tour', 'score']
        
        # Créer un DataFrame vide avec les colonnes attendues
        df = pd.DataFrame(columns=header)
        
        # Parser les lignes manuellement
        data = []
        for line in lines:
            # Ignorer les lignes de commentaires et les lignes vides
            if line.strip() and not line.startswith('#'):
                try:
                    # Séparer les valeurs
                    values = line.strip().split(',')
                    if len(values) == 12:  # Vérifier qu'on a le bon nombre de colonnes
                        # Convertir les valeurs
                        row = {
                            'max_speed': float(values[0]),
                            'max_steer': float(values[1]),
                            'min_front_dist': float(values[2]),
                            'safety_margin': float(values[3]),
                            'front_angle': float(values[4]),
                            'side_angle': float(values[5]),
                            'total_time': float(values[6]),
                            'distance': float(values[7]),
                            'collision': values[8].lower() == 'true',
                            'tour_complete': values[9].lower() == 'true',
                            'temps_tour': -1 if values[10] == 'N/A' else float(values[10]),
                            'score': float(values[11])
                        }
                        data.append(row)
                except (ValueError, IndexError):
                    continue
        
        # Créer le DataFrame à partir des données collectées
        df = pd.DataFrame(data)
        
        if len(df) == 0:
            print("Erreur: Aucune donnée valide trouvée dans le fichier.")
            return None
        
        print(f"\nDonnées chargées avec succès: {len(df)} lignes")
        print("\nAperçu des données:")
        print(df.head())
        print("\nStatistiques:")
        print(df.describe())
        
        return df
        
    except Exception as e:
        print(f"Erreur lors du chargement du fichier : {str(e)}")
        return None

def plot_score_evolution(df):
    """Affiche l'évolution du score au fil du temps"""
    plt.figure(figsize=(12, 6))
    plt.plot(df.index, df['score'], 'b-', label='Score')
    plt.scatter(df.index, df['score'], c='blue', s=30)
    
    # Marquer les meilleurs scores
    best_scores = df[df['score'] == df['score'].max()]
    plt.scatter(best_scores.index, best_scores['score'], 
               c='red', s=100, label='Meilleurs scores')
    
    plt.title('Évolution du Score au Fil du Temps')
    plt.xlabel('Numéro du Test')
    plt.ylabel('Score')
    plt.legend()
    plt.grid(True)
    plt.savefig('docs/score_evolution.png')
    plt.close()

def plot_parameter_distribution(df):
    """Affiche la distribution des paramètres"""
    param_cols = ['max_speed', 'max_steer', 'min_front_dist', 
                  'safety_margin', 'front_angle', 'side_angle']
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.ravel()
    
    for idx, param in enumerate(param_cols):
        sns.histplot(data=df, x=param, ax=axes[idx])
        axes[idx].set_title(f'Distribution de {param}')
    
    plt.tight_layout()
    plt.savefig('docs/parameter_distribution.png')
    plt.close()

def plot_correlation_matrix(df):
    """Affiche la matrice de corrélation entre les paramètres"""
    param_cols = ['max_speed', 'max_steer', 'min_front_dist', 
                  'safety_margin', 'front_angle', 'side_angle', 'score']
    
    plt.figure(figsize=(10, 8))
    sns.heatmap(df[param_cols].corr(), annot=True, cmap='coolwarm', center=0)
    plt.title('Matrice de Corrélation des Paramètres')
    plt.tight_layout()
    plt.savefig('docs/correlation_matrix.png')
    plt.close()

def plot_success_rate(df):
    """Affiche le taux de succès (tours complétés)"""
    success_rate = df['tour_complete'].mean() * 100
    collision_rate = df['collision'].mean() * 100
    
    plt.figure(figsize=(8, 6))
    plt.pie([success_rate, 100-success_rate], 
            labels=['Tours Complétés', 'Tours Incomplets'],
            colors=['green', 'red'],
            autopct='%1.1f%%')
    plt.title('Taux de Succès des Tours')
    plt.savefig('docs/success_rate.png')
    plt.close()

def plot_best_parameters(df):
    """Affiche les meilleurs paramètres trouvés"""
    best_result = df.loc[df['score'].idxmax()]
    
    plt.figure(figsize=(12, 6))
    params = ['max_speed', 'max_steer', 'min_front_dist', 
              'safety_margin', 'front_angle', 'side_angle']
    
    x = np.arange(len(params))
    plt.bar(x, best_result[params])
    
    plt.xticks(x, params, rotation=45)
    plt.title('Meilleurs Paramètres Trouvés')
    plt.ylabel('Valeur')
    plt.tight_layout()
    plt.savefig('docs/best_parameters.png')
    plt.close()

def generate_report(df):
    """Génère un rapport texte avec les statistiques principales"""
    with open('docs/analysis_report.txt', 'w') as f:
        f.write("Rapport d'Analyse des Résultats\n")
        f.write("=" * 50 + "\n\n")
        
        # Statistiques générales
        f.write("Statistiques Générales:\n")
        f.write("-" * 20 + "\n")
        f.write(f"Nombre total de tests: {len(df)}\n")
        f.write(f"Meilleur score: {df['score'].max():.2f}\n")
        f.write(f"Score moyen: {df['score'].mean():.2f}\n")
        f.write(f"Taux de succès: {df['tour_complete'].mean()*100:.1f}%\n")
        f.write(f"Taux de collision: {df['collision'].mean()*100:.1f}%\n\n")
        
        # Meilleurs paramètres
        best_result = df.loc[df['score'].idxmax()]
        f.write("Meilleurs Paramètres:\n")
        f.write("-" * 20 + "\n")
        for param in ['max_speed', 'max_steer', 'min_front_dist', 
                     'safety_margin', 'front_angle', 'side_angle']:
            f.write(f"{param}: {best_result[param]:.3f}\n")
        
        # Temps moyen
        f.write("\nTemps Moyen:\n")
        f.write("-" * 20 + "\n")
        f.write(f"Temps moyen par tour: {df['temps_tour'].mean():.2f}s\n")
        f.write(f"Meilleur temps: {df['temps_tour'].min():.2f}s\n")

def main():
    # Créer le dossier docs s'il n'existe pas
    if not os.path.exists('docs'):
        os.makedirs('docs')
    
    # Charger les données
    df = load_results()
    if df is None:
        return
    
    # Générer les visualisations
    plot_score_evolution(df)
    plot_parameter_distribution(df)
    plot_correlation_matrix(df)
    plot_success_rate(df)
    plot_best_parameters(df)
    
    # Générer le rapport
    generate_report(df)
    
    print("Visualisations générées dans le dossier 'docs':")
    print("- score_evolution.png : Évolution du score")
    print("- parameter_distribution.png : Distribution des paramètres")
    print("- correlation_matrix.png : Corrélation entre paramètres")
    print("- success_rate.png : Taux de succès")
    print("- best_parameters.png : Meilleurs paramètres")
    print("- analysis_report.txt : Rapport détaillé")

if __name__ == '__main__':
    main() 