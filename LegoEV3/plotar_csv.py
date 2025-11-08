import pandas as pd
import matplotlib.pyplot as plt

# Função para ler arquivos CSV e retornar um DataFrame
def ler_csv(caminho):
    return pd.read_csv(caminho)

# Leitura dos três arquivos CSV
arquivo1 = ler_csv('E:\\UFJF\\TCC\\Seguidor_P3.csv')
arquivo2 = ler_csv('E:\\UFJF\\TCC\\Seguidor_PI3.csv')
arquivo3 = ler_csv('E:\\UFJF\\TCC\\Seguidor_PID3.csv')

# Plotando os dados no mesmo gráfico
plt.figure(figsize=(10, 6))

# Ajuste a coluna correta de acordo com o nome das colunas nos seus arquivos CSV
plt.plot(arquivo1['Tempo'],arquivo1['Erro'],label='P')
plt.plot(arquivo2['Tempo'],arquivo2['Erro'], label='PI')
plt.plot(arquivo3['Tempo'],arquivo3['Erro'], label='PID')

# Adicionando linhas horizontais em 0.5 e -0.5
a = 0.05*255/2
plt.axhline(y=a, color='red', linestyle='--', linewidth=1)
plt.axhline(y=-a, color='red', linestyle='--', linewidth=1)

# Configurações do gráfico
plt.xlabel('Tempo')
plt.ylabel('Erro')
plt.title('Erro vs Tempo')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Ajusta o layout para melhor exibição
plt.show()
