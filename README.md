# ModuBot - ROS2 Workspace

Sistema de controle e navegação para robô móvel diferencial ModuBot, desenvolvido em ROS2.

## 📋 Sumário

- [Descrição](#descrição)
- [Pré-requisitos](#pré-requisitos)
- [Estrutura do Projeto](#estrutura-do-projeto)
- [Instalação](#instalação)
- [Uso](#uso)
- [Pacotes](#pacotes)
- [Configuração](#configuração)
- [Contribuindo](#contribuindo)

## 📝 Descrição

O ModuBot é um robô móvel diferencial desenvolvido para navegação autônoma e teleoperação. Este workspace ROS2 contém todos os pacotes necessários para:

- Controle por joystick ou teclado
- Odometria via comunicação serial
- Simulação em Gazebo
- Visualização em RViz2
- Navegação autônoma com Nav2

## 🔧 Pré-requisitos

- **Sistema Operacional:** Ubuntu 22.04 (Jammy) ou compatível
- **ROS2:** Humble Hawksbill (ou versão compatível)
- **Python:** 3.10+
- **Dependências:**
  - `ros-humble-desktop`
  - `ros-humble-navigation2`
  - `ros-humble-nav2-bringup`
  - `python3-colcon-common-extensions`
  - `python3-serial`

### Instalação de Dependências

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install python3-colcon-common-extensions python3-serial
```

## 📁 Estrutura do Projeto

```
modubot_ws/
├── src/
│   ├── modubot_joystick/         # Controle por joystick
│   ├── modubot_model_description/ # Modelo URDF e visualização
│   ├── modubot_odometry/          # Odometria via serial
│   ├── modubot_teleop/            # Conversão cmd_vel para serial
│   ├── urdf_description/          # Descrição URDF adicional
│   └── teleop_twist_keyboard/     # Controle por teclado
└── teleop_twist_keyboard/         # Teleoperação por teclado (standalone)
```

## 🚀 Instalação

### 1. Clone o repositório

```bash
cd ~/
git clone <URL_DO_REPOSITORIO> ufscar/robodc-ros2
cd ufscar/robodc-ros2/modubot_ws
```

### 2. Instale as dependências ROS2

```bash
cd ~/ufscar/robodc-ros2/modubot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Compile o workspace

```bash
colcon build
```

### 4. Configure o ambiente

```bash
source install/setup.bash
```

**Dica:** Adicione ao `~/.bashrc` para carregar automaticamente:
```bash
echo "source ~/ufscar/robodc-ros2/modubot_ws/install/setup.bash" >> ~/.bashrc
```

## 💻 Uso

### Visualização no RViz2

Para visualizar o modelo do robô:

```bash
ros2 launch modubot_model_description display.launch.py
```

### Simulação no Gazebo

Para iniciar a simulação completa:

```bash
ros2 launch modubot_model_description gazebo.launch.py
```

### Teleoperação

#### Por Teclado

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Por Joystick

```bash
ros2 launch modubot_joystick joystick_teleop.launch.py
```

### Odometria (Hardware Real)

Para iniciar a leitura de odometria via serial:

```bash
ros2 launch modubot_odometry odom_only.launch.py
```

### Sistema Completo

Para iniciar todo o sistema (modelo + Gazebo + Nav2):

```bash
ros2 launch modubot_model_description full.launch.py
```

## 📦 Pacotes

### modubot_joystick

Pacote para teleoperação usando joystick/gamepad.

- **Configuração:** `config/teleop.yaml`
- **Launch:** `joystick_teleop.launch.py`

### modubot_model_description

Modelo URDF do robô e arquivos de visualização.

- **URDF:** `urdf/modubot_model.xacro`
- **Meshes:** `meshes/`
- **Mundos Gazebo:** `worlds/EnvDC_MakerSpace.world`
- **Mapas:** `maps/PisoInferior.pgm`

**Launches disponíveis:**
- `display.launch.py` - Visualização no RViz2
- `gazebo.launch.py` - Simulação no Gazebo
- `full.launch.py` - Sistema completo com Nav2

### modubot_odometry

Nó para leitura de odometria via comunicação serial com o hardware do robô.

**Parâmetros principais:**
- `port`: Porta serial (default: `/dev/ttyUSB0`)
- `baud`: Taxa de transmissão (default: `115200`)
- `wheel_radius`: Raio das rodas em metros (default: `0.05`)
- `wheel_separation`: Distância entre rodas em metros (default: `0.28`)
- `ticks_per_rev`: Pulsos por revolução dos encoders (default: `90.0`)

### modubot_teleop

Converte comandos `cmd_vel` (Twist) em comandos seriais para o hardware.

**Parâmetros principais:**
- `port`: Porta serial (default: `/dev/ttyUSB0`)
- `baud`: Taxa de transmissão (default: `115200`)
- `wheel_separation`: Distância entre rodas em metros (default: `0.28`)
- `wheel_radius`: Raio das rodas em metros (default: `0.05`)
- `v_wheel_max`: Velocidade máxima das rodas em m/s (default: `0.6`)

### teleop_twist_keyboard

Pacote padrão para teleoperação via teclado.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controles:**
- `i` / `k` - Frente / Trás
- `j` / `l` - Rotação esquerda / direita
- `q` / `z` - Aumentar / Diminuir velocidade

## ⚙️ Configuração

### Configuração de Portas Seriais

Para uso com hardware real, configure as permissões da porta serial:

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

**Nota:** Faça logout e login novamente após adicionar ao grupo dialout.

### Parâmetros Nav2

Os parâmetros de navegação estão em:
- `modubot_model_description/config/nav2_params.yaml`
- `urdf_description/config/nav2_params.yaml`

### Parâmetros do Robô

Configurações físicas do robô (modificar conforme necessário):

```yaml
wheel_radius: 0.05        # Raio das rodas [m]
wheel_separation: 0.28    # Distância entre rodas [m]
ticks_per_rev: 90.0      # Pulsos por revolução dos encoders
```

## 🔍 Troubleshooting

### Erro de porta serial

```
Não abriu serial: [Errno 13] Permission denied: '/dev/ttyUSB0'
```

**Solução:**
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Pacote não encontrado

```
Package 'modubot_*' not found
```

**Solução:**
```bash
cd ~/ufscar/robodc-ros2/modubot_ws
source install/setup.bash
```

### Erro de compilação

```bash
# Limpar build e reinstalar
rm -rf build install log
colcon build
```

## 🤝 Contribuindo

1. Faça um fork do projeto
2. Crie uma branch para sua feature (`git checkout -b feature/NovaFeature`)
3. Commit suas mudanças (`git commit -m 'Adiciona nova feature'`)
4. Push para a branch (`git push origin feature/NovaFeature`)
5. Abra um Pull Request

## 📄 Licença

Este projeto está sob a licença especificada em cada pacote individual. Consulte os arquivos LICENSE em cada diretório.

## 👥 Autores

- UFSCar - Universidade Federal de São Carlos
- RoboDC - Laboratório de Robótica

## 📞 Contato

Para dúvidas ou sugestões, entre em contato através dos canais do laboratório RoboDC.

---

**Última atualização:** Dezembro 2025
