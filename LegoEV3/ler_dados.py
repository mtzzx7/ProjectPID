import random
import matplotlib.pyplot as plt
import paramiko
import pandas as pd
import time

hostname = 'ev3dev.local'
port = 22
username = 'robot'
password = 'maker'  # Se necessário

diretorio_ev3 = '/home/robot/'

# Função para conectar ao EV3
def conectar_ev3():
    global cliente_ssh
    cliente_ssh = paramiko.SSHClient()
    cliente_ssh.load_system_host_keys()
    cliente_ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    while True:
        try:
            cliente_ssh.connect(hostname, port, username, password)
            print("Conexão estabelecida com sucesso!")
            break
        except (paramiko.ssh_exception.SSHException, OSError):
            print("Erro na conexão. Tentando reconectar...")
            time.sleep(5)  # Aguarda 5 segundos antes de tentar reconectar

# Função para obter a lista de arquivos no diretório EV3
def obter_arquivos_ev3():
    stdin, stdout, stderr = cliente_ssh.exec_command(f'ls {diretorio_ev3}')
    return stdout.read().decode('utf-8').splitlines()

# Função para exibir o menu de arquivos
def exibir_menu(arquivos):
    print("\nEscolha um arquivo:")
    for i, arquivo in enumerate(arquivos):
        print(f"{i+1}. {arquivo}")

# Função para ler e plotar o arquivo escolhido
def ler_e_plotar(arquivo):
    path = diretorio_ev3 + arquivo
    print(f"Lendo o arquivo: {path}")
    with cliente_ssh.open_sftp() as sftp:
        with sftp.open(path) as file:
            df = pd.read_csv(file)
    
    print(df)
    # Definindo a coluna "tempo" como o índice do DataFrame
    df.set_index('Tempo', inplace=True)
    fig, ax = plt.subplots()
    # Plotando os dados do DataFrame
    df.plot(ax=ax)
    plt.xlabel('Tempo')
    plt.ylabel('Valores')
    plt.title('Erro vs Tempo')
    plt.legend(title='Amostras')
    plt.grid(True)
    plt.show()
    
# Função para baixar um arquivo
def baixar_arquivo(arquivo_ev3, destino_pc):
    with cliente_ssh.open_sftp() as sftp:
        sftp.get(arquivo_ev3, destino_pc)
    print(f"Arquivo '{arquivo_ev3}' foi baixado para '{destino_pc}'")

# Função para deletar um arquivo
def deletar_arquivo(arquivo_ev3):
    try:
        stdin, stdout, stderr = cliente_ssh.exec_command(f'rm {arquivo_ev3}')
        print(f"Arquivo '{arquivo_ev3}' foi deletado.")
    except Exception as e:
        print(f"Erro ao deletar o arquivo: {e}")

# Iniciando a conexão
conectar_ev3()

try:
    # Inicializando a lista de arquivos
    arquivos = obter_arquivos_ev3()
    arquivos_anteriores = arquivos[:]

    while True:
        try:
            # Verifica se a lista de arquivos mudou
            arquivos = obter_arquivos_ev3()
            
            if arquivos != arquivos_anteriores:
                print("\n--- Lista de arquivos atualizada ---")
                arquivos_anteriores = arquivos[:]
            exibir_menu(arquivos)
            
            # Pergunta ao usuário se deseja ler, baixar ou deletar um arquivo
            opcao = input("Digite o número do arquivo que deseja ler (ou 'q' para sair, 'b' para baixar, 'd' para deletar): ")
            
            if opcao.lower() == 'q':
                break
            elif opcao.lower() == 'b':
                try:
                    opcao_baixar = int(input("Digite o número do arquivo que deseja baixar: "))
                    if 1 <= opcao_baixar <= len(arquivos):
                        destino = input("Digite o caminho completo no PC para salvar o arquivo: ")
                        baixar_arquivo(diretorio_ev3 + arquivos[opcao_baixar - 1], destino)
                    else:
                        print("Opção inválida.")
                except ValueError:
                    print("Favor, digite um número válido.")
            elif opcao.lower() == 'd':
                try:
                    opcao_deletar = int(input("Digite o número do arquivo que deseja deletar: "))
                    if 1 <= opcao_deletar <= len(arquivos):
                        deletar_arquivo(diretorio_ev3 + arquivos[opcao_deletar - 1])
                    else:
                        print("Opção inválida.")
                except ValueError:
                    print("Por favor, digite um número válido.")
            else:
                try:
                    opcao = int(opcao)
                    if 1 <= opcao <= len(arquivos):
                        ler_e_plotar(arquivos[opcao - 1])
                    else:
                        print("Opção inválida.")
                except ValueError:
                    print("Por favor, digite um número válido.")
        except (paramiko.ssh_exception.SSHException, OSError):
            print("Conexão perdida. Tentando reconectar...")
            conectar_ev3()  # Tenta reconectar caso a conexão tenha sido perdida
        
except (paramiko.ssh_exception.SSHException, OSError):
    conectar_ev3()  # Tenta reconectar caso a conexão tenha sido perdida

# Fechando a conexão SSH
cliente_ssh.close()